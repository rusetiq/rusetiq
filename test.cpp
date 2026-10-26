// AquaSense - ARDUINO UNO sketch (updated for the final wiring)
// Reads sensors, runs the pump, drives the LCD,
// and talks to the Wi-Fi module (ESP-01) over 2 wires.
// Also provides full telemetry and debug logging over USB Serial at 9600 baud.
//
// WIRING (signal wires)
//   Soil sensor AO      -> A0
//   Light sensor AO     -> A1   (optional)
//   DHT11 S pin         -> D4
//   Pump driver gate    -> D7   (through a 220 ohm resistor, MOSFET/transistor
//   driver) Wi-Fi adapter TX    -> D2 Wi-Fi adapter RX    -> D3 LCD SDA -> A4
//   LCD SCK (SCL)       -> A5
//   USB Serial Monitor  -> Hardware USB (9600 baud)
// All parts also need 5V and GND (see the breadboard power diagram).

#include <U8g2lib.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// ---------- 1. EDIT THESE ----------
const int SOIL_PIN = A0;
const int LIGHT_PIN = A1; // optional light sensor
const int DHT_PIN = 4;
const int PUMP_PIN = 7;
const int LAMP_PIN = 8;          // not used: there is no lamp in this build
const int LED_PIN = LED_BUILTIN; // blinks when a command is processed

const bool HAS_LAMP = false; // keep false: nothing is connected to pin 8

const int ESP_RX = 2; // Uno pin 2 <- Wi-Fi adapter TX
const int ESP_TX =
    3; // Uno pin 3 -> Wi-Fi adapter RX
       // (use a voltage divider here only if your adapter has no level shifter)

const int SOIL_DRY = 600; // reading with sensor in dry air
const int SOIL_WET = 300; // reading with sensor in a glass of water
const bool RELAY_ACTIVE_LOW =
    false; // false for a MOSFET or transistor driver (HIGH = pump on)

const unsigned long PUMP_MAX_MS = 60000UL; // pump never runs longer than this
const unsigned long PUMP_COOLDOWN_MS =
    300000UL; // then auto mode waits this long
// -----------------------------------

DHT dht(DHT_PIN, DHT11); // blue DHT11 sensor
// U8g2 page-buffer mode: Uses only ~128 bytes of RAM!
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
SoftwareSerial esp(ESP_RX, ESP_TX);

bool oledReady = false;

bool pumpAuto = true, lampAuto = HAS_LAMP;
int moThreshold = 35, lightThreshold = 300;
unsigned long pumpOnSince = 0, pumpBlockedUntil = 0;
bool pumpBlocked = false;

float lastT = 0, lastH = 0;
unsigned long lastDht = 0;

char note[17] = "";
unsigned long noteUntil = 0;
char ip[20] = "";
unsigned long ipUntil = 0;

char espBuf[32];
byte espBufLen = 0;

char serialBuf[32];
byte serialBufLen = 0;

bool sendNow = false;
unsigned long lastAuto = 0, lastSend = 0;

void setRelay(int pin, bool on) {
  digitalWrite(pin, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

bool relayOn(int pin) { return (digitalRead(pin) == HIGH) != RELAY_ACTIVE_LOW; }

bool lampOn() { // always false when there is no lamp
  return HAS_LAMP && relayOn(LAMP_PIN);
}

void setPump(bool on) {
  if (on && !relayOn(PUMP_PIN))
    pumpOnSince = millis();
  setRelay(PUMP_PIN, on);
  Serial.print(F("[RELAY] Pump state -> "));
  Serial.println(on ? F("ON") : F("OFF"));
}

// ---------- 2. Sensor reading functions ----------
int readMoisture() {
  return constrain(map(analogRead(SOIL_PIN), SOIL_DRY, SOIL_WET, 0, 100), 0,
                   100);
}

int readLight() { // the app expects roughly 0 to 1300
  return map(analogRead(LIGHT_PIN), 0, 1023, 0, 1300);
}

void readDht() { // read the DHT11 at most every 2 seconds
  if (lastDht != 0 && millis() - lastDht < 2000)
    return;
  lastDht = millis();
  float t = dht.readTemperature(), h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println(F("[DEBUG] Warning: Failed to read from DHT11 sensor!"));
  } else {
    lastT = t;
    lastH = h;
  }
}

void blinkLed() {
  digitalWrite(LED_PIN, HIGH);
  delay(80);
  digitalWrite(LED_PIN, LOW);
}

// ---------- 0.96-inch OLED (U8g2) ----------
void showOledLines(const char *line1, const char *line2) {
  if (!oledReady)
    return;
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_ncenB08_tr); // Clean, readable font
    oled.drawStr(0, 18, line1);
    oled.drawStr(0, 42, line2);
  } while (oled.nextPage());
}

void updateLcd() {
  readDht();
  if (!oledReady)
    return;
  char a[24];
  char b[24];
  if ((long)(ipUntil - millis()) > 0) {
    showOledLines("Open in browser:", ip);
    return;
  }
  snprintf(a, sizeof(a), "M%3d%% T%2d H%2d%%", readMoisture(), (int)lastT,
           (int)lastH);
  if ((long)(noteUntil - millis()) > 0) {
    snprintf(b, sizeof(b), "%s", note);
  } else if (ip[0] == 0 && millis() < 40000UL) {
    snprintf(b, sizeof(b), "Wi-Fi: waiting..");
  } else if (HAS_LAMP) {
    snprintf(b, sizeof(b), "P:%s%c L:%s%c", relayOn(PUMP_PIN) ? "ON" : "OFF",
             pumpAuto ? 'A' : 'M', lampOn() ? "ON" : "OFF",
             lampAuto ? 'A' : 'M');
  } else {
    snprintf(b, sizeof(b), "P:%s%c Lt:%d", relayOn(PUMP_PIN) ? "ON" : "OFF",
             pumpAuto ? 'A' : 'M', readLight());
  }
  showOledLines(a, b);
}

void setNote(const char *text) {
  snprintf(note, sizeof(note), "%s", text);
  noteUntil = millis() + 3000;
  if (oledReady)
    updateLcd();
}

// ---------- Telemetry (ESP-01 & USB Serial Monitor at 9600 baud) ----------
void sendData() {
  int mo = readMoisture();
  int li = readLight();
  bool pOn = relayOn(PUMP_PIN);
  bool lOn = lampOn();

  // 1. Human-readable telemetry log for Serial Monitor (9600 baud)
  Serial.print(F("[TELEMETRY] Moisture: "));
  Serial.print(mo);
  Serial.print(F("% | Temp: "));
  Serial.print(lastT, 1);
  Serial.print(F(" C | Humidity: "));
  Serial.print((int)lastH);
  Serial.print(F("% | Light: "));
  Serial.print(li);
  Serial.print(F(" | Pump: "));
  Serial.print(pOn ? F("ON") : F("OFF"));
  Serial.print(F(" ("));
  Serial.print(pumpAuto ? F("auto") : F("manual"));
  Serial.println(F(")"));

  // 2. Transmit JSON packet to Wi-Fi module
  esp.print(F("{\"mo\":"));
  esp.print(mo);
  esp.print(F(",\"te\":"));
  esp.print(lastT, 1);
  esp.print(F(",\"hu\":"));
  esp.print((int)lastH);
  esp.print(F(",\"li\":"));
  esp.print(li);
  esp.print(F(",\"pump\":"));
  esp.print(pOn ? 1 : 0);
  esp.print(F(",\"lamp\":"));
  esp.print(lOn ? 1 : 0);
  esp.print(F(",\"pm\":\""));
  esp.print(pumpAuto ? F("auto") : F("manual"));
  esp.print(F("\",\"lm\":\""));
  esp.print(lampAuto ? F("auto") : F("manual"));
  esp.print(F("\",\"th\":"));
  esp.print(moThreshold);
  esp.print(F(",\"lt\":"));
  esp.print(lightThreshold);
  esp.println('}');

  // 3. Mirror JSON packet to Serial Monitor
  Serial.print(F("[JSON] "));
  Serial.print(F("{\"mo\":"));
  Serial.print(mo);
  Serial.print(F(",\"te\":"));
  Serial.print(lastT, 1);
  Serial.print(F(",\"hu\":"));
  Serial.print((int)lastH);
  Serial.print(F(",\"li\":"));
  Serial.print(li);
  Serial.print(F(",\"pump\":"));
  Serial.print(pOn ? 1 : 0);
  Serial.print(F(",\"lamp\":"));
  Serial.print(lOn ? 1 : 0);
  Serial.print(F(",\"pm\":\""));
  Serial.print(pumpAuto ? F("auto") : F("manual"));
  Serial.print(F("\",\"lm\":\""));
  Serial.print(lampAuto ? F("auto") : F("manual"));
  Serial.print(F("\",\"th\":"));
  Serial.print(moThreshold);
  Serial.print(F(",\"lt\":"));
  Serial.print(lightThreshold);
  Serial.println('}');
}

// ---------- Command Parsing & Debugging ----------
void handleCommand(char *commandLine, const char *source) {
  char *eq = strchr(commandLine, '=');
  if (!eq)
    return;
  *eq = 0;
  const char *k = commandLine;
  const char *v = eq + 1;
  char msg[24];

  Serial.print(F("[DEBUG] Received cmd from "));
  Serial.print(source);
  Serial.print(F(": "));
  Serial.print(k);
  Serial.print(F("="));
  Serial.println(v);

  if (!strcmp(k, "ip")) {
    if (strcmp(ip, v)) {
      snprintf(ip, sizeof(ip), "%s", v);
      ipUntil = millis() + 10000UL;
      Serial.print(F("[DEBUG] ESP reported IP: "));
      Serial.println(ip);
    }
    return;
  }
  if (!HAS_LAMP && (!strcmp(k, "lm") || !strcmp(k, "lamp"))) {
    Serial.println(F("[DEBUG] Lamp command ignored (no lamp attached)."));
    return;
  }

  if (!strcmp(k, "pm")) {
    pumpAuto = !strcmp(v, "auto");
    setNote(pumpAuto ? "App: Pump AUTO" : "App: Pump MANUAL");
    Serial.print(F("[DEBUG] Pump Mode: "));
    Serial.println(pumpAuto ? F("AUTO") : F("MANUAL"));
  } else if (!strcmp(k, "lm")) {
    lampAuto = !strcmp(v, "auto");
    setNote(lampAuto ? "App: Light AUTO" : "App: Light MANUAL");
    Serial.print(F("[DEBUG] Light Mode: "));
    Serial.println(lampAuto ? F("AUTO") : F("MANUAL"));
  } else if (!strcmp(k, "th")) {
    moThreshold = constrain(atoi(v), 0, 100);
    snprintf(msg, sizeof(msg), "App: Water <%d%%", moThreshold);
    setNote(msg);
    Serial.print(F("[DEBUG] Moisture Threshold updated: <"));
    Serial.print(moThreshold);
    Serial.println(F("%"));
  } else if (!strcmp(k, "lt")) {
    lightThreshold = constrain(atoi(v), 0, 1300);
    snprintf(msg, sizeof(msg), "App: Light <%d", lightThreshold);
    setNote(msg);
    Serial.print(F("[DEBUG] Light Threshold updated: <"));
    Serial.println(lightThreshold);
  } else if (!strcmp(k, "pump")) {
    pumpBlocked = false;
    setPump(atoi(v) == 1);
    setNote(atoi(v) == 1 ? "App: Pump ON" : "App: Pump OFF");
    Serial.print(F("[DEBUG] Manual Pump override: "));
    Serial.println(atoi(v) == 1 ? F("ON") : F("OFF"));
  } else if (!strcmp(k, "lamp")) {
    setRelay(LAMP_PIN, atoi(v) == 1);
    setNote(atoi(v) == 1 ? "App: Light ON" : "App: Light OFF");
    Serial.print(F("[DEBUG] Manual Lamp override: "));
    Serial.println(atoi(v) == 1 ? F("ON") : F("OFF"));
  } else {
    Serial.print(F("[DEBUG] Unknown command: "));
    Serial.println(k);
    return;
  }
  sendNow = true; // transmit updated state immediately
  blinkLed();
}

void readEsp() {
  while (esp.available()) {
    char c = esp.read();
    if (c == '\n') {
      espBuf[espBufLen] = 0;
      if (espBufLen)
        handleCommand(espBuf, "ESP");
      espBufLen = 0;
    } else if (c != '\r' && espBufLen < sizeof(espBuf) - 1) {
      espBuf[espBufLen++] = c;
    }
  }
}

// Allows testing and commands directly from the USB Serial Monitor
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufLen > 0) {
        serialBuf[serialBufLen] = 0;
        handleCommand(serialBuf, "USB_SERIAL");
        serialBufLen = 0;
      }
    } else if (serialBufLen < sizeof(serialBuf) - 1) {
      serialBuf[serialBufLen++] = c;
    }
  }
}

// Runs on the Arduino, so watering keeps working even if the Wi-Fi or app
// drops.
void runAutomation() {
  bool pumpIsOn = relayOn(PUMP_PIN);

  if (pumpIsOn && millis() - pumpOnSince > PUMP_MAX_MS) { // safety stop
    setPump(false);
    pumpBlocked = true;
    pumpBlockedUntil = millis() + PUMP_COOLDOWN_MS;
    pumpIsOn = false;
    Serial.println(F("[SAFETY] Max pump runtime (60s) reached! Pump halted & "
                     "cooling down."));
  }
  if (pumpBlocked && (long)(millis() - pumpBlockedUntil) >= 0) {
    pumpBlocked = false;
    Serial.println(
        F("[SAFETY] Pump cooldown period finished. Pump re-enabled."));
  }

  if (pumpAuto) {
    int mo = readMoisture();
    if (!pumpIsOn && !pumpBlocked && mo < moThreshold) {
      Serial.print(F("[AUTO] Moisture dry ("));
      Serial.print(mo);
      Serial.print(F("% < "));
      Serial.print(moThreshold);
      Serial.println(F("%). Turning pump ON."));
      setPump(true);
    } else if (pumpIsOn && mo >= moThreshold + 22) {
      Serial.print(F("[AUTO] Moisture target achieved ("));
      Serial.print(mo);
      Serial.print(F("% >= "));
      Serial.print(moThreshold + 22);
      Serial.println(F("%). Turning pump OFF."));
      setPump(false);
    }
  }
  if (HAS_LAMP && lampAuto) {
    int li = readLight();
    bool on = relayOn(LAMP_PIN);
    setRelay(LAMP_PIN, on ? li < lightThreshold + 300 : li < lightThreshold);
  }
}

void initOled() {
  Serial.println(F("[OLED] Initializing SSD1306 via U8g2 (page buffer mode)..."));
  oled.begin();
  oledReady = true;
  showOledLines("AquaSense", "Starting...");
  Serial.println(F("[OLED] Initialized successfully."));
}

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println(F("\n========================================"));
  Serial.println(F("       AquaSense Controller v1.0        "));
  Serial.println(F("   Telemetry & Debug Port: 9600 baud    "));
  Serial.println(F("========================================"));

  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  setRelay(PUMP_PIN, false);
  if (HAS_LAMP) {
    pinMode(LAMP_PIN, OUTPUT);
    setRelay(LAMP_PIN, false);
  }

  Serial.println(F("[BOOT] Initializing DHT11 on Pin 4..."));
  dht.begin();

  Serial.println(F("[BOOT] Initializing OLED..."));
  initOled();

  Serial.println(F("[BOOT] Initializing ESP-01 on Pins 2/3 (9600 baud)..."));
  esp.begin(9600);
  delay(500);

  Serial.println(F("[BOOT] System initialization complete!"));
  Serial.print(F("[BOOT] Moisture Threshold: <"));
  Serial.print(moThreshold);
  Serial.println(F("%"));
  Serial.println(F("[BOOT] Streaming sensor telemetry below...\n"));
}

void loop() {
  readSerial(); // listen for commands from USB Serial Monitor
  readEsp();    // listen for commands from Wi-Fi module
  unsigned long now = millis();
  if (now - lastAuto >= 1000) {
    lastAuto = now;
    runAutomation();
    updateLcd();
  }
  if (sendNow || now - lastSend >= 2000) {
    lastSend = now;
    sendNow = false;
    readDht();
    sendData();
  }
}
