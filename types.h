#pragma once

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

struct RouteEntry {
  char callsign[10];
  bool known;
  char orig[8];
  char dest[8];
  char origCity[24];
  char destCity[24];
  char airline[24];
};

struct Backoff {
  unsigned long until;
  unsigned long delayMs;
};

struct Provider {
  const char* name;
  const char* base;      // base URL including API version
  int         style;     // 0 = /point/{lat}/{lon}/{nm}, 1 = /lat/{lat}/lon/{lon}/dist/{nm}
  int         dailyBudget;
  int         used;
  Backoff     bo;
};