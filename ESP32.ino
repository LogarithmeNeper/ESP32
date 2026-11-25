#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <time.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, HOME_LAT, HOME_LON
#include "types.h"     // Flight, RouteEntry, Backoff, Provider

// ================= CONFIG (non-secret) =================
const int    RADIUS_NM = 15;                // search radius, nautical miles
const int    MAX_ALT_FT = 15000;            // ignore aircraft at or above this altitude
const bool   SHOW_GROUND = false;
const bool   SHOW_ROUTE = true;
const unsigned long POLL_MS = 30000;        // update every 30 seconds

const int    SLEEP_START_HOUR = 1;          // deep sleep from 01:00...
const int    WAKE_HOUR = 9;                 // ...until 09:00
const char*  TZ_PARIS = "CET-1CEST,M3.5.0,M10.5.0/3";
const int    BACKLIGHT_PIN = 21;
// =======================================================

// Position feeds, all ADSBExchange-v2-compatible. dailyBudget is OUR self-imposed
// limit per day (kept under each provider's real limit). Edit freely.
Provider providers[] = {
  {"adsb.lol",       "https://api.adsb.lol/v2",          0, 4000, 0, {0, 0}},
  {"airplanes.live", "https://api.airplanes.live/v2",    0,  450, 0, {0, 0}},
  {"adsb.fi",        "https://opendata.adsb.fi/api/v2",  1, 2000, 0, {0, 0}},
};
const int PROV_N = sizeof(providers) / sizeof(providers[0]);

TFT_eSPI tft = TFT_eSPI();

uint16_t C_BG, C_CARD, C_MUTED, C_ACCENT, C_CYAN, C_GREEN, C_RED, C_WHITE;
uint16_t statusColor;

// ---------- back-off after errors / rate limiting ----------
Backoff routeBackoff;

void backoffFail(Backoff &b, unsigned long retryAfterMs = 0) {
  if (retryAfterMs) {
    b.delayMs = (retryAfterMs > 600000UL) ? 600000UL : retryAfterMs;
  } else {
    unsigned long next = b.delayMs ? b.delayMs * 2 : 30000UL;   // 30 s, 60 s, 120 s...
    b.delayMs = (next > 600000UL) ? 600000UL : next;            // max 10 min
  }
  b.until = millis() + b.delayMs;
  Serial.printf("Backing off for %lu s\n", b.delayMs / 1000);
}
void backoffOk(Backoff &b) { b.delayMs = 0; b.until = 0; }
bool backoffActive(const Backoff &b) { return b.until != 0 && (long)(b.until - millis()) > 0; }

// ---------- provider rotation ----------
int lastYday = -1;
unsigned long lastResetMs = 0;

void resetBudgetsIfNewDay() {
  struct tm t;
  bool reset = false;
  if (getLocalTime(&t, 0)) {
    if (t.tm_yday != lastYday) {
      reset = (lastYday != -1);
      lastYday = t.tm_yday;
    }
  } else if (millis() - lastResetMs > 86400000UL) {
    reset = true;                                  // no clock: reset every 24 h
  }
  if (reset) {
    for (int i = 0; i < PROV_N; i++) providers[i].used = 0;
    lastResetMs = millis();
    Serial.println("Daily request budgets reset");
  }
}

// provider with the lowest share of its budget used, skipping those backing off
int pickProvider() {
  int best = -1;
  float bestRatio = 1e9;
  for (int i = 0; i < PROV_N; i++) {
    Provider &p = providers[i];
    if (backoffActive(p.bo)) continue;
    if (p.used >= p.dailyBudget) continue;
    float ratio = (float)p.used / p.dailyBudget;
    if (ratio < bestRatio) { bestRatio = ratio; best = i; }
  }
  return best;
}

// ---------- route cache ----------
const int ROUTE_CACHE_N = 30;
RouteEntry routeCache[ROUTE_CACHE_N];
int routeCacheNext = 0;

RouteEntry* findRoute(const char* cs) {
  for (int i = 0; i < ROUTE_CACHE_N; i++)
    if (routeCache[i].callsign[0] && strcmp(routeCache[i].callsign, cs) == 0)
      return &routeCache[i];
  return nullptr;
}

// keep only plain ASCII (built-in fonts can't draw accents)
void copyAscii(char* dst, size_t n, const char* src) {
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j < n - 1; i++)
    if ((uint8_t)src[i] < 128) dst[j++] = src[i];
  dst[j] = 0;
}

void shorten(char* s, size_t maxLen) {
  if (strlen(s) > maxLen) s[maxLen] = 0;
}

// ---------- geometry ----------
float haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = radians(lat2 - lat1), dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return (float)(R * 2 * atan2(sqrt(a), sqrt(1 - a)));
}

float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) -
             sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  return (float)fmod(degrees(atan2(y, x)) + 360.0, 360.0);
}

const char* compass(float b) {
  static const char* d[] = {"N","NE","E","SE","S","SW","W","NW"};
  return d[(int)((b + 22.5f) / 45.0f) % 8];
}

// ---------- sleep schedule ----------
bool inSleepWindow(int hour) {
  if (SLEEP_START_HOUR < WAKE_HOUR) return hour >= SLEEP_START_HOUR && hour < WAKE_HOUR;
  return hour >= SLEEP_START_HOUR || hour < WAKE_HOUR;          // window crosses midnight
}

long secondsUntilWake(const struct tm &t) {
  long nowS = t.tm_hour * 3600L + t.tm_min * 60L + t.tm_sec;
  long d = WAKE_HOUR * 3600L - nowS;
  if (d <= 0) d += 86400L;
  return d;
}

// The ESP32 sleep timer drifts by a few percent, so sleep ~90% of the remaining
// time, wake, re-check the clock, and repeat. This keeps wake-up near the target.
long sleepChunk(long remaining) {
  return (remaining > 300) ? remaining * 9 / 10 : remaining;
}

void goToSleep(long seconds, bool showMessage) {
  if (showMessage) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Sleeping until %02d:00", WAKE_HOUR);
    tft.fillScreen(C_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_MUTED, C_BG);
    tft.drawString(buf, 160, 120, 4);
    delay(3000);
  }
  digitalWrite(BACKLIGHT_PIN, LOW);
  tft.writecommand(0x28);                          // display off
  gpio_hold_en((gpio_num_t)BACKLIGHT_PIN);         // keep backlight off during sleep
  gpio_deep_sleep_hold_en();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.printf("Deep sleep for %ld s\n", seconds);
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// ---------- network ----------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
}

// One attempt against one provider. Returns aircraft count, or -1 on failure
// (in which case that provider is put in back-off).
int fetchFrom(Provider &p, Flight &best) {
  best = Flight();

  String url = String(p.base);
  if (p.style == 0) {
    url += "/point/" + String(HOME_LAT, 5) + "/" + String(HOME_LON, 5) + "/" + String(RADIUS_NM);
  } else {
    url += "/lat/" + String(HOME_LAT, 5) + "/lon/" + String(HOME_LON, 5) + "/dist/" + String(RADIUS_NM);
  }

  p.used++;
  Serial.printf("Using %s (%d/%d today)\n", p.name, p.used, p.dailyBudget);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  http.setUserAgent("esp32-flight-panel/1.0");
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) { backoffFail(p.bo); return -1; }

  const char* keys[] = {"Retry-After"};
  http.collectHeaders(keys, 1);

  int code = http.GET();
  if (code != 200) {
    Serial.printf("HTTP error %d (%s) from %s\n", code, http.errorToString(code).c_str(), p.name);
    unsigned long ra = (code == 429) ? (unsigned long)http.header("Retry-After").toInt() * 1000UL : 0;
    http.end();
    backoffFail(p.bo, ra);
    return -1;
  }

  JsonDocument filter;
  JsonObject f = filter["ac"][0].to<JsonObject>();
  f["flight"] = true; f["t"] = true; f["r"] = true; f["alt_baro"] = true;
  f["gs"] = true; f["lat"] = true; f["lon"] = true;

  String payload = http.getString();
  http.end();
  Serial.printf("Payload: %u bytes, free heap: %u\n", payload.length(), ESP.getFreeHeap());

  JsonDocument doc;
  DeserializationError err =
      deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  payload = "";
  if (err) {
    Serial.printf("JSON error from %s: %s\n", p.name, err.c_str());
    backoffFail(p.bo);
    return -1;
  }
  backoffOk(p.bo);

  JsonArray ac = doc["ac"].as<JsonArray>();
  int count = 0;
  float bestDist = 1e9;

  for (JsonObject a : ac) {
    if (a["lat"].isNull() || a["lon"].isNull()) continue;

    JsonVariant v = a["alt_baro"];
    if (v.isNull()) continue;                      // unknown altitude: skip

    bool ground = false;
    int alt = 0;
    if (v.is<const char*>()) ground = true;        // value is "ground"
    else alt = v.as<int>();

    if (ground && !SHOW_GROUND) continue;
    if (!ground && alt >= MAX_ALT_FT) continue;    // too high to see or hear

    count++;
    float d = haversineKm(HOME_LAT, HOME_LON, a["lat"], a["lon"]);
    if (d < bestDist) {
      bestDist = d;
      best.valid = true;
      best.onGround = ground;
      best.altFt = alt;
      best.speedKt = (int)lroundf(a["gs"].as<float>());
      best.distKm = d;
      best.bearing = bearingDeg(HOME_LAT, HOME_LON, a["lat"], a["lon"]);
      strlcpy(best.callsign, a["flight"] | "", sizeof(best.callsign));
      strlcpy(best.type, a["t"] | "", sizeof(best.type));
      strlcpy(best.reg, a["r"] | "", sizeof(best.reg));
      for (int i = strlen(best.callsign) - 1; i >= 0 && best.callsign[i] == ' '; i--)
        best.callsign[i] = 0;
    }
  }
  return count;
}

// Tries providers in order of least-used until one works.
// Returns aircraft count, -1 on error, -2 if every provider is waiting/exhausted.
int fetchNearest(Flight &best) {
  best = Flight();
  for (int attempt = 0; attempt < PROV_N; attempt++) {
    int i = pickProvider();
    if (i < 0) return (attempt == 0) ? -2 : -1;
    int r = fetchFrom(providers[i], best);
    if (r >= 0) return r;
  }
  return -1;
}

// Looks up origin/destination/airline for a callsign and caches it.
// Returns false only on a transient failure (so we retry later).
bool lookupRoute(const char* callsign) {
  if (findRoute(callsign)) return true;
  if (backoffActive(routeBackoff)) return false;

  String url = String("https://api.adsbdb.com/v0/callsign/") + callsign;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  http.setUserAgent("esp32-flight-panel/1.0");
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;

  int code = http.GET();
  String payload;
  if (code == 200) payload = http.getString();
  http.end();

  if (code != 200 && code != 404) {
    Serial.printf("Route lookup error %d\n", code);
    if (code == 429 || code >= 500) backoffFail(routeBackoff);
    return false;
  }
  backoffOk(routeBackoff);

  RouteEntry e = {};
  strlcpy(e.callsign, callsign, sizeof(e.callsign));
  e.known = false;

  if (code == 200) {
    JsonDocument filter;
    for (const char* side : {"origin", "destination"}) {
      JsonObject o = filter["response"]["flightroute"][side].to<JsonObject>();
      o["iata_code"] = true; o["icao_code"] = true; o["municipality"] = true;
    }
    filter["response"]["flightroute"]["airline"]["name"] = true;

    JsonDocument doc;
    if (!deserializeJson(doc, payload, DeserializationOption::Filter(filter))) {
      JsonVariant r = doc["response"]["flightroute"];

      const char* o = r["origin"]["iata_code"] | "";
      if (!o[0]) o = r["origin"]["icao_code"] | "";
      const char* d = r["destination"]["iata_code"] | "";
      if (!d[0]) d = r["destination"]["icao_code"] | "";

      if (o[0] && d[0]) {
        e.known = true;
        strlcpy(e.orig, o, sizeof(e.orig));
        strlcpy(e.dest, d, sizeof(e.dest));
        copyAscii(e.origCity, sizeof(e.origCity), r["origin"]["municipality"] | "");
        copyAscii(e.destCity, sizeof(e.destCity), r["destination"]["municipality"] | "");
        copyAscii(e.airline, sizeof(e.airline), r["airline"]["name"] | "");
      }
    }
  }

  routeCache[routeCacheNext] = e;
  routeCacheNext = (routeCacheNext + 1) % ROUTE_CACHE_N;
  return true;
}

// ---------- display helpers ----------
void initColors() {
  C_BG     = tft.color565(8, 12, 24);
  C_CARD   = tft.color565(22, 30, 52);
  C_MUTED  = tft.color565(125, 138, 168);
  C_ACCENT = tft.color565(255, 190, 40);
  C_CYAN   = tft.color565(60, 200, 255);
  C_GREEN  = tft.color565(70, 220, 130);
  C_RED    = tft.color565(255, 90, 90);
  C_WHITE  = TFT_WHITE;
}

void drawHeader(uint16_t dotColor) {
  statusColor = dotColor;
  tft.fillRect(0, 0, 320, 24, C_CARD);
  tft.fillTriangle(9, 16, 9, 8, 21, 12, C_ACCENT);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_ACCENT, C_CARD);
  tft.drawString("SKYWATCH", 28, 5, 2);

  char buf[8] = "--:--";
  struct tm t;
  if (getLocalTime(&t, 0)) strftime(buf, sizeof(buf), "%H:%M", &t);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(C_MUTED, C_CARD);
  tft.drawString(buf, 288, 5, 2);
  tft.fillCircle(305, 12, 5, dotColor);
}

void drawCompass(int cx, int cy, int r, float bearing, const char* label) {
  tft.fillCircle(cx, cy, r, C_CARD);
  tft.drawCircle(cx, cy, r, C_MUTED);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_MUTED, C_CARD);
  tft.drawString("N", cx, cy - r + 7, 1);
  float a = radians(bearing);
  int px = cx + (int)((r - 6) * sin(a));
  int py = cy - (int)((r - 6) * cos(a));
  tft.fillCircle(px, py, 4, C_ACCENT);
  tft.setTextColor(C_WHITE, C_CARD);
  tft.drawString(label, cx, cy, 2);
}

void drawTile(int x, int y, const char* label, const String &value, uint16_t color) {
  tft.fillRoundRect(x, y, 98, 42, 6, C_CARD);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_MUTED, C_CARD);
  tft.drawString(label, x + 8, y + 5, 1);
  tft.setTextColor(color, C_CARD);
  tft.drawString(value, x + 8, y + 15, 4);
}

void drawRoute(const Flight &fl) {
  const int y = 104;
  tft.setTextDatum(TL_DATUM);
  if (!fl.routeKnown) {
    tft.setTextColor(C_MUTED, C_BG);
    tft.drawString("route unknown", 10, 130, 2);
    return;
  }
  int sz = (strlen(fl.origin) > 3 || strlen(fl.dest) > 3) ? 1 : 2;
  tft.setTextSize(sz);
  tft.setTextColor(C_WHITE, C_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(fl.origin, 10, y, 4);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(fl.dest, 310, y, 4);
  tft.setTextSize(1);

  int ly = y + (sz == 2 ? 26 : 13);
  for (int x = 112; x < 208; x += 8) tft.drawFastHLine(x, ly, 4, C_MUTED);
  tft.fillTriangle(150, ly - 7, 150, ly + 7, 170, ly, C_ACCENT);

  int cityY = y + (sz == 2 ? 54 : 28);
  char oc[24], dc[24];
  strlcpy(oc, fl.originCity, sizeof(oc)); shorten(oc, 14);
  strlcpy(dc, fl.destCity, sizeof(dc));   shorten(dc, 14);
  tft.setTextColor(C_MUTED, C_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(oc, 10, cityY, 2);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(dc, 310, cityY, 2);
}

void drawFlight(const Flight &fl, int count, bool error) {
  tft.fillRect(0, 24, 320, 216, C_BG);

  if (error) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_RED, C_BG);
    tft.drawString("Connection problem", 160, 105, 4);
    tft.setTextColor(C_MUTED, C_BG);
    tft.drawString("retrying shortly...", 160, 140, 2);
    return;
  }
  if (!fl.valid) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_WHITE, C_BG);
    tft.drawString("Clear skies", 160, 105, 4);
    tft.setTextColor(C_MUTED, C_BG);
    tft.drawString("nothing below " + String(MAX_ALT_FT) + " ft within " + String(RADIUS_NM) + " nm", 160, 140, 2);
    return;
  }

  // callsign (shrinks if too wide)
  const char* cs = strlen(fl.callsign) ? fl.callsign : "-------";
  tft.setTextSize(1);
  int w = tft.textWidth(cs, 4);
  tft.setTextSize(w * 2 > 240 ? 1 : 2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString(cs, 10, 28, 4);
  tft.setTextSize(1);

  // airline
  if (fl.routeKnown && fl.airline[0]) {
    tft.setTextColor(C_MUTED, C_BG);
    tft.drawString(fl.airline, 10, 84, 2);
  }

  // compass
  drawCompass(282, 62, 26, fl.bearing, compass(fl.bearing));

  // route
  if (SHOW_ROUTE) drawRoute(fl);

  // stat tiles
  String alt;
  if (fl.onGround) alt = "GND";
  else alt = String(fl.altFt);                     // always below 18,000 ft here
  drawTile(8,   178, "ALTITUDE (ft)", alt, C_GREEN);
  drawTile(111, 178, "SPEED (kt)", String(fl.speedKt), C_CYAN);
  drawTile(214, 178, "DISTANCE (km)", String(fl.distKm, 1), C_WHITE);

  // footer
  String left = strlen(fl.type) ? String(fl.type) : String("?");
  if (strlen(fl.reg)) left += " - " + String(fl.reg);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_MUTED, C_BG);
  tft.drawString(left, 10, 224, 2);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(String(count) + " below " + String(MAX_ALT_FT / 1000) + "k ft", 310, 224, 2);
}

// ---------- main ----------
unsigned long lastPoll = 0;
int lastMinute = -1;

void setup() {
  Serial.begin(115200);

  // wake-up handling: after a timer wake-up keep the screen dark until we know it's morning
  bool timerWake = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER);
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)BACKLIGHT_PIN);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, timerWake ? LOW : HIGH);

  tft.init();
  tft.setRotation(1);
  initColors();
  tft.fillScreen(C_BG);
  if (!timerWake) drawHeader(TFT_ORANGE);

  connectWiFi();
  configTzTime(TZ_PARIS, "pool.ntp.org", "time.google.com");

  struct tm t;
  bool haveTime = getLocalTime(&t, 10000);         // wait up to 10 s for NTP
  if (haveTime && inSleepWindow(t.tm_hour)) {
    goToSleep(sleepChunk(secondsUntilWake(t)), !timerWake);
  } else if (!haveTime && timerWake) {
    goToSleep(600, false);                         // no clock yet: retry in 10 min, screen stays off
  }

  if (timerWake) {                                 // it's morning: turn the screen on
    digitalWrite(BACKLIGHT_PIN, HIGH);
    drawHeader(TFT_ORANGE);
  }

  lastResetMs = millis();
  lastPoll = millis() - POLL_MS;                   // poll immediately
}

void loop() {
  struct tm t;
  if (getLocalTime(&t, 0) && t.tm_min != lastMinute) {
    lastMinute = t.tm_min;
    if (inSleepWindow(t.tm_hour)) {
      goToSleep(sleepChunk(secondsUntilWake(t)), true);
    }
    drawHeader(statusColor);                       // refresh the clock
  }

  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();

  resetBudgetsIfNewDay();

  if (WiFi.status() != WL_CONNECTED) {
    drawHeader(C_RED);
    connectWiFi();
  }
  if (WiFi.status() != WL_CONNECTED) return;

  drawHeader(TFT_ORANGE);
  Flight fl;
  int n = fetchNearest(fl);

  if (n == -2) {                                   // all providers waiting: keep last screen
    drawHeader(C_ACCENT);
    return;
  }
  if (n < 0) {
    drawHeader(C_RED);
    drawFlight(fl, 0, true);
    return;
  }

  if (SHOW_ROUTE && fl.valid && strlen(fl.callsign) >= 3) {
    lookupRoute(fl.callsign);
    RouteEntry* r = findRoute(fl.callsign);
    if (r && r->known) {
      fl.routeKnown = true;
      strlcpy(fl.origin, r->orig, sizeof(fl.origin));
      strlcpy(fl.dest, r->dest, sizeof(fl.dest));
      strlcpy(fl.originCity, r->origCity, sizeof(fl.originCity));
      strlcpy(fl.destCity, r->destCity, sizeof(fl.destCity));
      strlcpy(fl.airline, r->airline, sizeof(fl.airline));
    }
  }

  drawHeader(C_GREEN);
  drawFlight(fl, n, false);
}