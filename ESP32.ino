#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <math.h>

#include "secrets.h"

// ================= CONFIG (non-secret) =================
const int    RADIUS_NM = 15;
const bool   SHOW_GROUND = false;
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
};

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
  double b = degrees(atan2(y, x));
  return (float)fmod(b + 360.0, 360.0);
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
  client.setInsecure();               // skips certificate check (see notes)
  HTTPClient http;
  http.setUserAgent("esp32-flight-panel/1.0");
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(8000);
  if (!http.begin(client, url)) return -1;

  int code = http.GET();
  if (code != 200) {
    Serial.printf("HTTP error %d\n", code);
    http.end();
    return -1;
  }

  // keep only the fields we need, to save RAM
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
  payload = "";                        // free the memory straight away
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
    if (v.is<const char*>()) ground = true;      // value is "ground"
    else alt = v.as<int>();
    if (ground && !SHOW_GROUND) continue;

    count++;
    float d = haversineKm(HOME_LAT, HOME_LON, a["lat"], a["lon"]);
    if (d < bestDist) {
      bestDist = d;
      best.valid = true;
      best.onGround = ground;
      best.altFt = alt;
      best.speedKt = (int)(a["gs"] | 0);
      best.distKm = d;
      best.bearing = bearingDeg(HOME_LAT, HOME_LON, a["lat"], a["lon"]);
      strlcpy(best.callsign, a["flight"] | "", sizeof(best.callsign));
      strlcpy(best.type, a["t"] | "", sizeof(best.type));
      strlcpy(best.reg, a["r"] | "", sizeof(best.reg));
      // callsigns come padded with trailing spaces
      for (int i = strlen(best.callsign) - 1; i >= 0 && best.callsign[i] == ' '; i--)
        best.callsign[i] = 0;
    }
  }
  return count;
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
  tft.drawString(strlen(fl.callsign) ? fl.callsign : "-------", 10, 28, 4);
  tft.setTextSize(1);

  // type + registration
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  String line = String(strlen(fl.type) ? fl.type : "?");
  if (strlen(fl.reg)) line += "  " + String(fl.reg);
  tft.drawString(line, 10, 92, 4);

  // altitude + speed
  String alt;
  if (fl.onGround) alt = "On ground";
  else if (fl.altFt >= 18000) alt = "FL" + String(fl.altFt / 100);
  else alt = String(fl.altFt) + " ft";
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(alt + "   " + String(fl.speedKt) + " kt", 10, 132, 4);

  // distance + direction
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(String(fl.distKm, 1) + " km " + compass(fl.bearing), 10, 172, 4);

  // footer
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(String(count) + " aircraft within " + String(RADIUS_NM) + " nm", 10, 218, 2);
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
  digitalWrite(21, HIGH);            // backlight
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  drawHeader("connecting...", TFT_ORANGE);
  connectWiFi();
  lastPoll = millis() - POLL_MS;     // poll immediately
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
        drawHeader("live", TFT_GREEN);
        drawFlight(fl, n, false);
      }
    }
  }
}