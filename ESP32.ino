#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <math.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, HOME_LAT, HOME_LON

// ================= CONFIG (non-secret) =================
const int    RADIUS_NM = 15;
const bool   SHOW_GROUND = false;
const bool   SHOW_ROUTE = true;
const unsigned long POLL_MS = 10000;
// =======================================================

TFT_eSPI tft = TFT_eSPI();

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
  // route (filled by lookupRoute)
  bool  routeKnown = false;
  char  origin[8] = "";
  char  dest[8] = "";
  char  originCity[24] = "";
  char  destCity[24] = "";
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
};
RouteEntry routeCache[ROUTE_CACHE_N];
int routeCacheNext = 0;

RouteEntry* findRoute(const char* cs) {
  for (int i = 0; i < ROUTE_CACHE_N; i++)
    if (routeCache[i].callsign[0] && strcmp(routeCache[i].callsign, cs) == 0)
      return &routeCache[i];
  return nullptr;
}

// keep only plain ASCII (the built-in fonts can't draw accents)
void copyAscii(char* dst, size_t n, const char* src) {
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j < n - 1; i++)
    if ((uint8_t)src[i] < 128) dst[j++] = src[i];
  dst[j] = 0;
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
      best.speedKt = (int)lroundf(a["gs"].as<float>());   // fix: speed is a decimal
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

// Looks up origin/destination for a callsign and stores it in the cache.
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
      }
    }
  }

  routeCache[routeCacheNext] = e;
  routeCacheNext = (routeCacheNext + 1) % ROUTE_CACHE_N;
  return true;
}

// ---------- display ----------
void drawFlight(const Flight &fl, int count, bool error) {
  tft.fillRect(0, 22, 320, 218, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  if (error) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Network / API error", 10, 100, 4);
    return;
  }
  if (!fl.valid) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("No aircraft nearby", 10, 90, 4);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("within " + String(RADIUS_NM) + " nm", 10, 125, 2);
    return;
  }

  // callsign
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString(strlen(fl.callsign) ? fl.callsign : "-------", 10, 26, 4);
  tft.setTextSize(1);

  // route
  if (SHOW_ROUTE) {
    if (fl.routeKnown) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString(String(fl.origin) + " > " + fl.dest, 10, 82, 4);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.drawString(String(fl.originCity) + " > " + fl.destCity, 10, 108, 2);
    } else {
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      tft.drawString("route unknown", 10, 88, 2);
    }
  }

  // type + registration
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String line = String(strlen(fl.type) ? fl.type : "?");
  if (strlen(fl.reg)) line += "  " + String(fl.reg);
  tft.drawString(line, 10, 128, 4);

  // altitude + speed
  String alt;
  if (fl.onGround) alt = "On ground";
  else if (fl.altFt >= 18000) alt = "FL" + String(fl.altFt / 100);
  else alt = String(fl.altFt) + " ft";
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(alt + "  " + String(fl.speedKt) + " kt", 10, 156, 4);

  // distance + direction
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(String(fl.distKm, 1) + " km " + compass(fl.bearing), 10, 184, 4);

  // footer
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(String(count) + " aircraft within " + String(RADIUS_NM) + " nm", 10, 220, 2);
}

void drawHeader(const char* status, uint16_t color) {
  tft.fillRect(0, 0, 320, 22, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("NEAREST FLIGHT", 8, 3, 2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(status, 312, 3, 2);
}

// ---------- main ----------
unsigned long lastPoll = 0;

void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  drawHeader("connecting...", TFT_ORANGE);
  connectWiFi();
  lastPoll = millis() - POLL_MS;
}

void loop() {
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();

    if (WiFi.status() != WL_CONNECTED) {
      drawHeader("wifi lost", TFT_RED);
      connectWiFi();
    }

    if (WiFi.status() == WL_CONNECTED) {
      drawHeader("updating", TFT_ORANGE);
      Flight fl;
      int n = fetchNearest(fl);

      if (n < 0) {
        drawHeader("error", TFT_RED);
        drawFlight(fl, 0, true);
      } else {
        if (SHOW_ROUTE && fl.valid && strlen(fl.callsign) >= 3) {
          lookupRoute(fl.callsign);
          RouteEntry* r = findRoute(fl.callsign);
          if (r && r->known) {
            fl.routeKnown = true;
            strlcpy(fl.origin, r->orig, sizeof(fl.origin));
            strlcpy(fl.dest, r->dest, sizeof(fl.dest));
            strlcpy(fl.originCity, r->origCity, sizeof(fl.originCity));
            strlcpy(fl.destCity, r->destCity, sizeof(fl.destCity));
          }
        }
        drawHeader("live", TFT_GREEN);
        drawFlight(fl, n, false);
      }
    }
  }
}