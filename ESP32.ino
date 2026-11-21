#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <time.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, HOME_LAT, HOME_LON

// ================= CONFIG (non-secret) =================
const int    RADIUS_NM = 15;
const bool   SHOW_GROUND = false;
const bool   SHOW_ROUTE = true;
const unsigned long POLL_MS = 10000;

const bool   SOUND_ON = true;
const int    SOUND_VOLUME = 40;        // 1-100
const int    QUIET_START = 22;         // no sound from this hour...
const int    QUIET_END = 8;            // ...until this hour
const float  ALERT_KM = 3.0;           // "fly-over" alert distance
const int    ALERT_MAX_ALT_FT = 10000; // ...and maximum altitude
const char*  TZ_PARIS = "CET-1CEST,M3.5.0,M10.5.0/3";
const int    SPEAKER_PIN = 26;

const char* RARE_TYPES[] = {"A388", "A225", "A124", "B748", "B744",
                            "BLCF", "C5M", "C17", "A400", "B52"};
// =======================================================

TFT_eSPI tft = TFT_eSPI();

uint16_t C_BG, C_CARD, C_MUTED, C_ACCENT, C_CYAN, C_GREEN, C_RED, C_WHITE;
uint16_t statusColor;

struct Flight {
  bool  valid = false;
  char  callsign[10] = "";
  char  type[8] = "";
  char  reg[12] = "";
  int   altFt = 0;
  bool  onGround = false;
  int   speedKt = 0;
  float distKm = 0;
  float bearing = 0;
  bool  routeKnown = false;
  char  origin[8] = "";
  char  dest[8] = "";
  char  originCity[24] = "";
  char  destCity[24] = "";
  char  airline[24] = "";
};

// ---------- route cache ----------
const int ROUTE_CACHE_N = 10;
struct RouteEntry {
  char callsign[10];
  bool known;
  char orig[8];
  char dest[8];
  char origCity[24];
  char destCity[24];
  char airline[24];
};
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

// ---------- sound ----------
void playTone(int freq, int ms) {
  if (!SOUND_ON) return;
  if (freq <= 0) { delay(ms); return; }
  int amp = constrain(SOUND_VOLUME, 1, 100) * 127 / 100;
  unsigned long half = 500000UL / freq;
  unsigned long end = millis() + ms;
  while (millis() < end) {
    dacWrite(SPEAKER_PIN, 128 + amp);
    delayMicroseconds(half);
    dacWrite(SPEAKER_PIN, 128 - amp);
    delayMicroseconds(half);
  }
  dacWrite(SPEAKER_PIN, 128);
}

void soundStartup()   { playTone(523, 90); playTone(659, 90); playTone(784, 90); playTone(1047, 220); }
void soundNewFlight() { playTone(880, 90); playTone(0, 30); playTone(1175, 170); }
void soundOverhead()  { for (int i = 0; i < 3; i++) { playTone(1000, 70); playTone(0, 60); } }
void soundRare() {
  int n[] = {523, 659, 784, 1047, 1319};
  for (int i = 0; i < 5; i++) playTone(n[i], 80);
  playTone(1568, 320);
}

bool isRare(const char* t) {
  for (const char* r : RARE_TYPES) if (strcmp(t, r) == 0) return true;
  return false;
}

bool isQuiet() {
  struct tm t;
  if (!getLocalTime(&t, 0)) return false;      // time unknown: don't mute
  int h = t.tm_hour;
  if (QUIET_START > QUIET_END) return h >= QUIET_START || h < QUIET_END;
  return h >= QUIET_START && h < QUIET_END;
}

// ---------- network ----------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
}

// returns number of aircraft found, or -1 on error
int fetchNearest(Flight &best) {
  best = Flight();

  String url = "https://api.adsb.lol/v2/point/" + String(HOME_LAT, 5) + "/" +
               String(HOME_LON, 5) + "/" + String(RADIUS_NM);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  http.setUserAgent("esp32-flight-panel/1.0");
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return -1;

  int code = http.GET();
  if (code != 200) {
    Serial.printf("HTTP error %d (%s)\n", code, http.errorToString(code).c_str());
    Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
    http.end();
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
    Serial.printf("JSON error: %s\n", err.c_str());
    return -1;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  int count = 0;
  float bestDist = 1e9;

  for (JsonObject a : ac) {
    if (a["lat"].isNull() || a["lon"].isNull()) continue;

    bool ground = false;
    int alt = 0;
    JsonVariant v = a["alt_baro"];
    if (v.is<const char*>()) ground = true;
    else alt = v.as<int>();
    if (ground && !SHOW_GROUND) continue;

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

// Looks up origin/destination/airline for a callsign and caches it.
// Returns false only on a transient failure (so we retry next time).
bool lookupRoute(const char* callsign) {
  if (findRoute(callsign)) return true;

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
    return false;
  }

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
    tft.drawString("no aircraft within " + String(RADIUS_NM) + " nm", 160, 140, 2);
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
  else if (fl.altFt >= 18000) alt = "FL" + String(fl.altFt / 100);
  else alt = String(fl.altFt);
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
  tft.drawString(String(count) + " in range", 310, 224, 2);
}

// ---------- main ----------
unsigned long lastPoll = 0;
int lastMinute = -1;
char lastId[12] = "";

void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);              // backlight
  tft.init();
  tft.setRotation(1);
  initColors();
  tft.fillScreen(C_BG);

  drawHeader(TFT_ORANGE);
  connectWiFi();
  configTzTime(TZ_PARIS, "pool.ntp.org", "time.google.com");

  soundStartup();                      // doubles as a speaker test
  lastPoll = millis() - POLL_MS;
}

void loop() {
  // refresh the clock when the minute changes
  struct tm t;
  if (getLocalTime(&t, 0) && t.tm_min != lastMinute) {
    lastMinute = t.tm_min;
    drawHeader(statusColor);
  }

  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();

  if (WiFi.status() != WL_CONNECTED) {
    drawHeader(C_RED);
    connectWiFi();
  }
  if (WiFi.status() != WL_CONNECTED) return;

  drawHeader(TFT_ORANGE);
  Flight fl;
  int n = fetchNearest(fl);

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

  // sound when a new nearest flight appears
  if (fl.valid) {
    const char* id = fl.callsign[0] ? fl.callsign : fl.reg;
    if (id[0] && strcmp(id, lastId) != 0) {
      strlcpy(lastId, id, sizeof(lastId));
      if (!isQuiet()) {
        if (isRare(fl.type)) soundRare();
        else if (!fl.onGround && fl.distKm < ALERT_KM && fl.altFt < ALERT_MAX_ALT_FT) soundOverhead();
        else soundNewFlight();
      }
    }
  }
}