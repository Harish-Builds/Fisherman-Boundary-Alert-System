#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3);
SoftwareSerial gsm(7, 8);
LiquidCrystal lcd(9, 10, 11, 12, 13, A0);

// ─── Pin Definitions ──────────────────────────────────────────
#define SOS_BUTTON        2    // D2 → INPUT_PULLUP (push button)
#define BUZZER            5    // D5 → OUTPUT
#define MOTOR             6    // D6 → OUTPUT

// ─── Settings ─────────────────────────────────────────────────
#define RADIUS_LIMIT_M    50.0
#define ALERT_COOLDOWN_MS 30000
#define CHECK_INTERVAL_MS 2000
#define BOAT_SAMPLES      10
#define SMOOTH_FACTOR     0.3

// ─── State Variables ──────────────────────────────────────────
bool          BOATSet         = false;
double        BOATLat         = 0.0;
double        BOATLon         = 0.0;
double        smoothedDist    = 0.0;
bool          wasBeyond       = false;
unsigned long lastAlertTime   = 0;
unsigned long lastDiagTime    = 0;
unsigned long lastCheckTime   = 0;
unsigned long lastLCDTime     = 0;
unsigned long lastBuzzTime    = 0;
bool          buzzState       = false;
int           lcdPage         = 0;
int           BOATSampleCount = 0;
double        BOATSumLat      = 0.0;
double        BOATSumLon      = 0.0;
bool          collectingBOAT  = false;

// ─── Forward Declarations ─────────────────────────────────────
void printGPSDiag();
void displayLocation(double lat, double lon);
void checkRadius(double lat, double lon);
void triggerSOS(double lat, double lon);
void sendAlert(double lat, double lon, double dist);
void switchToGSM();
void switchToGPS();
void collectBOATSample(double lat, double lon);
void updateNormalLCD(double lat, double lon);
void showAlertLCD(double dist);
void showLCDMessage(const __FlashStringHelper* line0,
                    const __FlashStringHelper* line1,
                    unsigned long holdMs);

// ── Haversine ─────────────────────────────────────────────────
double haversineDistance(double lat1, double lon1,
                         double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2.0)*sin(dLat/2.0) +
             cos(radians(lat1))*cos(radians(lat2))*
             sin(dLon/2.0)*sin(dLon/2.0);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0-a));
}

void switchToGSM() { gsm.listen();       delay(10); }
void switchToGPS() { gpsSerial.listen(); delay(10); }

// ── LCD message helper ────────────────────────────────────────
void showLCDMessage(const __FlashStringHelper* line0,
                    const __FlashStringHelper* line1,
                    unsigned long holdMs) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line0);
  lcd.setCursor(0,1); lcd.print(line1);
  if (holdMs > 0) delay(holdMs);
}

// ══════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsm.begin(9600);

  // ── Pin modes — clearly listed ────────────────────────────
  pinMode(SOS_BUTTON, INPUT_PULLUP); // D2 — push button, LOW when pressed
  pinMode(BUZZER,     OUTPUT);       // D5 — buzzer
  pinMode(MOTOR,      OUTPUT);       // D6 — motor relay

  // ── Safe initial output states ────────────────────────────
  digitalWrite(BUZZER, LOW);         // buzzer OFF
  digitalWrite(MOTOR,  HIGH);        // motor ON

  lcd.begin(16, 2);
  showLCDMessage(F("FISHERMAN"), F("RADAR"), 1500);

  switchToGSM();
  gsm.println(F("AT"));
  delay(1000);
  switchToGPS();

  // ── Startup serial log ────────────────────────────────────
  Serial.println(F("================================"));
  Serial.println(F("  Fisherman GPS  v5.3"));
  Serial.println(F("  Boundary Alert System"));
  Serial.println(F("================================"));
  Serial.println(F("  Pin Map:"));
  Serial.println(F("  D2  = SOS Button (INPUT_PULLUP)"));
  Serial.println(F("  D5  = Buzzer     (OUTPUT)"));
  Serial.println(F("  D6  = Motor      (OUTPUT)"));
  Serial.println(F("  D4  = GPS RX     (SoftSerial)"));
  Serial.println(F("  D3  = GPS TX     (SoftSerial)"));
  Serial.println(F("  D7  = GSM RX     (SoftSerial)"));
  Serial.println(F("  D8  = GSM TX     (SoftSerial)"));
  Serial.println(F("================================"));
  Serial.print  (F("  Radius : ")); Serial.print(RADIUS_LIMIT_M);
  Serial.println(F(" m"));
  Serial.println(F("  Button : PUSH ALERT + SMS"));
  Serial.println(F("  Border : AUTO ALERT + SMS"));
  Serial.println(F("================================"));
}

// ══════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════
void loop() {
  if (!gpsSerial.isListening()) switchToGPS();

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Diagnostics every 1 second
  if (millis() - lastDiagTime >= 1000) {
    lastDiagTime = millis();
    printGPSDiag();
  }

  // No GPS fix yet
  if (!gps.location.isValid()) {
    lcd.setCursor(0,0); lcd.print(F("Searching GPS.. "));
    lcd.setCursor(0,1);
    lcd.print(F("Sats:"));
    lcd.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    lcd.print(F("  Fix:NO   "));
    return;
  }

  double curLat = gps.location.lat();
  double curLon = gps.location.lng();

  // Collect BOAT average
  if (!BOATSet) {
    collectBOATSample(curLat, curLon);
    return;
  }

  // Boundary check every 2 seconds
  if (millis() - lastCheckTime >= CHECK_INTERVAL_MS) {
    lastCheckTime = millis();
    displayLocation(curLat, curLon);
    checkRadius(curLat, curLon);
  }

  // Buzzer beep every 500ms when boundary exceeded
  if (wasBeyond && millis() - lastBuzzTime >= 500) {
    lastBuzzTime = millis();
    buzzState    = !buzzState;
    digitalWrite(BUZZER, buzzState ? HIGH : LOW);
  }

  // LCD page flip every 3 sec ONLY when safe
  if (!wasBeyond && millis() - lastLCDTime >= 3000) {
    lastLCDTime = millis();
    lcdPage     = 1 - lcdPage;
    updateNormalLCD(curLat, curLon);
  }

  // ── D2 SOS Push Button ────────────────────────────────────
  // INPUT_PULLUP means pin is HIGH normally
  // When button pressed it connects D2 to GND → reads LOW
  if (digitalRead(SOS_BUTTON) == LOW) {
    Serial.println(F("[D2]   SOS button triggered!"));
    triggerSOS(curLat, curLon);
    delay(5000);  // debounce — prevent double trigger
  }
}

// ══════════════════════════════════════════════════════════════
//  Normal LCD — flips every 3 sec
//
//  Page 0:              Page 1:
//  ┌────────────────┐   ┌────────────────┐
//  │Lat: 11.020456  │   │Dist:  32.5 m   │
//  │Lon: 76.966392  │   │Bound: 50m  OK  │
//  └────────────────┘   └────────────────┘
// ══════════════════════════════════════════════════════════════
void updateNormalLCD(double lat, double lon) {
  lcd.clear();
  if (lcdPage == 0) {
    lcd.setCursor(0,0);
    lcd.print(F("Lat:"));
    lcd.print(lat, 6);
    lcd.setCursor(0,1);
    lcd.print(F("Lon:"));
    lcd.print(lon, 6);
  } else {
    lcd.setCursor(0,0);
    lcd.print(F("Dist: "));
    lcd.print(smoothedDist, 1);
    lcd.print(F(" m"));
    lcd.setCursor(0,1);
    lcd.print(F("Bound:"));
    lcd.print((int)RADIUS_LIMIT_M);
    lcd.print(F("m  OK"));
  }
}

// ══════════════════════════════════════════════════════════════
//  Alert LCD — flips every 2 sec when boundary exceeded
//
//  Screen A:            Screen B:
//  ┌────────────────┐   ┌────────────────┐
//  │***  ALERT!  ***│   │Dist: 52.3 m    │
//  │Boundary Crossed│   │BOUND:  50m     │
//  └────────────────┘   └────────────────┘
// ══════════════════════════════════════════════════════════════
void showAlertLCD(double dist) {
  static unsigned long lastFlip  = 0;
  static bool          flipState = false;

  if (millis() - lastFlip >= 2000) {
    lastFlip  = millis();
    flipState = !flipState;
    lcd.clear();
    if (flipState) {
      lcd.setCursor(0,0); lcd.print(F("***  ALERT!  ***"));
      lcd.setCursor(0,1); lcd.print(F("Boundary Crossed"));
    } else {
      lcd.setCursor(0,0);
      lcd.print(F("Dist: "));
      lcd.print(dist, 1);
      lcd.print(F(" m"));
      lcd.setCursor(0,1);
      lcd.print(F("BOUND:  "));
      lcd.print((int)RADIUS_LIMIT_M);
      lcd.print(F(" m"));
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  Collect 10 averaged BOAT samples
// ══════════════════════════════════════════════════════════════
void collectBOATSample(double lat, double lon) {
  if (!collectingBOAT) {
    collectingBOAT = true;
    Serial.println(F("[BOAT] Collecting 10 samples"));
    Serial.println(F("       Stay still ~20 sec..."));
    showLCDMessage(F("Calibrating..."), F("Stay still!"), 0);
  }

  static unsigned long lastSampleTime = 0;
  if (millis() - lastSampleTime < 2000) return;
  lastSampleTime = millis();

  BOATSumLat += lat;
  BOATSumLon += lon;
  BOATSampleCount++;

  Serial.print(F("[BOAT] Sample "));
  Serial.print(BOATSampleCount);
  Serial.print(F("/10 Lat:"));
  Serial.print(lat, 6);
  Serial.print(F(" Lon:"));
  Serial.println(lon, 6);

  lcd.setCursor(0,0); lcd.print(F("Calibrating...  "));
  lcd.setCursor(0,1);
  lcd.print(F("Sample "));
  lcd.print(BOATSampleCount);
  lcd.print(F("/10     "));

  if (BOATSampleCount >= BOAT_SAMPLES) {
    BOATLat      = BOATSumLat / BOAT_SAMPLES;
    BOATLon      = BOATSumLon / BOAT_SAMPLES;
    BOATSet      = true;
    smoothedDist = 0.0;
    lcdPage      = 0;

    Serial.println(F("================================"));
    Serial.println(F("[BOAT] Starting point locked!"));
    Serial.print  (F("  Lat   : ")); Serial.println(BOATLat, 6);
    Serial.print  (F("  Lon   : ")); Serial.println(BOATLon, 6);
    Serial.print  (F("  Bound : ")); Serial.print(RADIUS_LIMIT_M);
    Serial.println(F(" m"));
    Serial.println(F("[INFO]  System ready!"));
    Serial.println(F("[INFO]  D2 button = PUSH ALERT"));
    Serial.println(F("================================"));

    lcd.clear();
    lcd.setCursor(0,0); lcd.print(F("BOAT LOCKED!"));
    lcd.setCursor(0,1);
    lcd.print(BOATLat, 4);
    lcd.print(F(","));
    lcd.print(BOATLon, 4);
    delay(2500);
    lcd.clear();
  }
}

// ══════════════════════════════════════════════════════════════
//  GPS Diagnostics — every second
// ══════════════════════════════════════════════════════════════
void printGPSDiag() {
  Serial.print(gps.charsProcessed());
  Serial.print(F(" | "));
  Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print(F(" | "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" | "));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(" | "));
    Serial.print(BOATSet ? smoothedDist : 0.0, 1);
    Serial.println(F(" m"));
  } else {
    Serial.println(F("NO FIX"));
  }
  if (gps.charsProcessed() < 10) {
    Serial.println(F("!!! NO DATA — Check GPS wiring !!!"));
  }
}

// ══════════════════════════════════════════════════════════════
//  Display location — Serial + smoothedDist update
// ══════════════════════════════════════════════════════════════
void displayLocation(double lat, double lon) {
  double raw   = haversineDistance(BOATLat, BOATLon, lat, lon);
  smoothedDist = (SMOOTH_FACTOR * raw) +
                 ((1.0 - SMOOTH_FACTOR) * smoothedDist);

  Serial.println(F("--------------------------------"));
  Serial.print(F("[LAT]  ")); Serial.println(lat, 6);
  Serial.print(F("[LON]  ")); Serial.println(lon, 6);
  Serial.print(F("[SATS] "));
  Serial.println(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print(F("[RAW]  ")); Serial.print(raw,          2); Serial.println(F(" m"));
  Serial.print(F("[DIST] ")); Serial.print(smoothedDist, 2); Serial.println(F(" m"));
  Serial.print(F("[LIM]  ")); Serial.print(RADIUS_LIMIT_M);  Serial.println(F(" m"));
}

// ══════════════════════════════════════════════════════════════
//  Boundary Check — all 5 outputs fire here
// ══════════════════════════════════════════════════════════════
void checkRadius(double lat, double lon) {
  bool beyond = (smoothedDist > RADIUS_LIMIT_M);

  if (beyond) {
    digitalWrite(MOTOR, LOW);
    showAlertLCD(smoothedDist);

    Serial.println(F("================================"));
    Serial.println(F("****  BOUNDARY ALERT!  ****"));
    Serial.print  (F("[DIST]  ")); Serial.print(smoothedDist, 2); Serial.println(F(" m"));
    Serial.print  (F("[OVER]  ")); Serial.print(smoothedDist - RADIUS_LIMIT_M, 2);
    Serial.println(F(" m beyond limit"));
    Serial.println(F("[INFO]  Buzzer beeping — Motor OFF"));
    Serial.println(F("================================"));

    // Auto GSM with 30s cooldown
    unsigned long now = millis();
    if (now - lastAlertTime > ALERT_COOLDOWN_MS) {
      lastAlertTime = now;
      showLCDMessage(F("Boundary Alert!"), F("Sending MSG..."), 1500);
      sendAlert(lat, lon, smoothedDist);
      showLCDMessage(F("ALERT MSG SENT!"), F("To:+918667326128"), 2500);
      Serial.println(F("[LCD]   Shown: ALERT MSG SENT!"));
    } else {
      Serial.print(F("[GSM]   Cooldown "));
      Serial.print((ALERT_COOLDOWN_MS-(millis()-lastAlertTime))/1000);
      Serial.println(F("s remaining"));
    }

  } else {
    digitalWrite(BUZZER, LOW);
    digitalWrite(MOTOR,  HIGH);
    buzzState = false;

    if (wasBeyond) {
      Serial.println(F("================================"));
      Serial.println(F("[SAFE]  Returned inside boundary"));
      Serial.print  (F("[DIST]  ")); Serial.print(smoothedDist, 2);
      Serial.println(F(" m — OK"));
      Serial.println(F("================================"));
      showLCDMessage(F("** SAFE ZONE **"), F("Back in range!"), 2000);
      lcdPage     = 0;
      lastLCDTime = millis();
    } else {
      Serial.print(F("[OK]    "));
      Serial.print(smoothedDist, 2);
      Serial.println(F(" m — inside boundary"));
    }
  }

  wasBeyond = beyond;
}

// ══════════════════════════════════════════════════════════════
//  SOS Push Button — D2 triggered
//
//  LCD sequence:
//  ┌────────────────┐   ┌────────────────┐   ┌────────────────┐
//  │** PUSH ALERT **│   │Sending MSG...  │   │PUSH MSG SENT!  │
//  │SOS Triggered!  │   │Please wait...  │   │To:+9186673261  │
//  └────────────────┘   └────────────────┘   └────────────────┘
// ══════════════════════════════════════════════════════════════
void triggerSOS(double lat, double lon) {

  // 3 rapid beeps
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER,HIGH); delay(150);
    digitalWrite(BUZZER,LOW);  delay(150);
  }

  // Step 1 — PUSH ALERT on LCD
  showLCDMessage(F("** PUSH ALERT **"), F("SOS Triggered!"), 1500);

  Serial.println(F("================================"));
  Serial.println(F("[D2]    Button pressed!"));
  Serial.println(F("[LCD]   Shown: PUSH ALERT"));
  Serial.println(F("[GSM]   Sending help message..."));
  Serial.println(F("================================"));

  // Step 2 — Sending on LCD
  showLCDMessage(F("Sending MSG..."), F("Please wait..."), 0);

  // Step 3 — Send GSM SMS
  switchToGSM();
  gsm.println(F("AT+CMGF=1")); delay(1000);
  gsm.println(F("AT+CMGS=\"+918667326128\"")); delay(1000);
  gsm.print(F("PUSH ALERT! I need help!\n"));
  gsm.print(F("Fisherman SOS triggered!\n"));
  gsm.print(F("My Location:\n"));
  gsm.print(F("https://maps.google.com/?q="));
  gsm.print(lat, 6);
  gsm.print(F(","));
  gsm.print(lon, 6);
  gsm.write(26);
  delay(3000);
  switchToGPS();

  // Step 4 — PUSH MSG SENT on LCD
  showLCDMessage(F("PUSH MSG SENT!"), F("To:+918667326128"), 2500);

  Serial.println(F("================================"));
  Serial.println(F("[LCD]   Shown: PUSH MSG SENT!"));
  Serial.println(F("[GSM]   Help message delivered!"));
  Serial.println(F("================================"));

  lcdPage     = 0;
  lastLCDTime = millis();
}

// ══════════════════════════════════════════════════════════════
//  Boundary Alert SMS
// ══════════════════════════════════════════════════════════════
void sendAlert(double lat, double lon, double dist) {
  Serial.println(F("[GSM]   Sending boundary alert..."));
  switchToGSM();
  gsm.println(F("AT+CMGF=1")); delay(1000);
  gsm.println(F("AT+CMGS=\"+918667326128\"")); delay(1000);
  gsm.print(F("BOUNDARY ALERT!\n"));
  gsm.print(F("Fisherman crossed limit!\n"));
  gsm.print(F("Distance: "));
  gsm.print(dist, 1);
  gsm.print(F("m (Bound: "));
  gsm.print((int)RADIUS_LIMIT_M);
  gsm.print(F("m)\n"));
  gsm.print(F("Location:\n"));
  gsm.print(F("https://maps.google.com/?q="));
  gsm.print(lat, 6);
  gsm.print(F(","));
  gsm.print(lon, 6);
  gsm.write(26);
  delay(3000);
  switchToGPS();
  Serial.println(F("[GSM]   Boundary alert sent!"));
}
