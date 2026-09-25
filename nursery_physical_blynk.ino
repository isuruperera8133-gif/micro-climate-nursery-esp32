

#define BLYNK_TEMPLATE_ID   "TMPL6uMJeIuBy"        // <-- from Blynk.Console
#define BLYNK_TEMPLATE_NAME "MicroClimate Nursery"
#define BLYNK_AUTH_TOKEN    "5KV0LOJ3slE1etDEsmMjHCoZ9oiS0eR9" // <-- from Blynk.Console / email
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>

// ---------------- WiFi Credentials ----------------
char ssid[] = "Isuru";
char pass[] = "Isuru#075";

// ---------------- Pin Definitions (identical to Wokwi build) ----------------
#define DHTPIN        4
#define DHTTYPE       DHT22
#define LDR_PIN       34
#define POT_PIN       35
#define SERVO_PIN     13
#define GREEN_LED_PIN 25   // Grow light
#define RED_LED_PIN   26   // Warning light
#define BTN_VENT_PIN  27
#define BTN_MODE_PIN  14

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDR   0x3C

// ---------------- Objects ----------------
DHT dht(DHTPIN, DHTTYPE);
Servo ventServo;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BlynkTimer blynkTimer;

// ---------------- System State ----------------
enum SystemMode { AUTONOMOUS, MANUAL, SAFETY_FAULT };
volatile SystemMode currentMode = AUTONOMOUS;

volatile bool manualModeRequested = false; // Button2 OR Blynk V0
volatile bool manualVentOpen      = false; // Button1 OR Blynk V1

float  temperature = 0, humidity = 0;
int    ldrRaw = 0, potRaw = 0;
bool   sensorFault = false;
String faultReason = "";
bool   growLightOn = false;
bool   conflictActive = false;

int ventAngle   = 0;
int targetAngle = 0;

// ---------------- Design-challenge constants ----------------
const float HEAT_THRESHOLD     = 30.0;
const float COLD_THRESHOLD     = 15.0;
const float COLD_SAFETY_WEIGHT = 1.6;
const int VENT_CLOSED_ANGLE    = 0;
const int VENT_MIN_OPEN_ANGLE  = 20;
const int VENT_MAX_OPEN_ANGLE  = 120;

// ---------------- Timing ----------------
unsigned long lastSensorRead  = 0;
const unsigned long SENSOR_INTERVAL_MS = 2000;
unsigned long lastServoMove   = 0;
unsigned long lastOledUpdate  = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastBlynkPush   = 0;

volatile unsigned long lastVentBtnIsr = 0;
volatile unsigned long lastModeBtnIsr = 0;
const unsigned long DEBOUNCE_MS = 250;

// ---------------- ISRs ----------------

/**
 * @brief GPIO interrupt for the vent button. Toggles the manual vent
 *        request flag; debounced using a millis() timestamp inside the ISR.
 * @return void.
 */
void IRAM_ATTR onVentButton() {
  unsigned long now = millis();
  if (now - lastVentBtnIsr > DEBOUNCE_MS) {
    manualVentOpen = !manualVentOpen;
    lastVentBtnIsr = now;
  }
}

/**
 * @brief GPIO interrupt for the mode button. Toggles the manual mode
 *        request flag; debounced using a millis() timestamp inside the ISR.
 * @return void.
 */
void IRAM_ATTR onModeButton() {
  unsigned long now = millis();
  if (now - lastModeBtnIsr > DEBOUNCE_MS) {
    manualModeRequested = !manualModeRequested;
    lastModeBtnIsr = now;
  }
}

// ---------------- Blynk callbacks ----------------

/**
 * @brief Blynk V0 switch: sets/clears MANUAL mode request from the app.
 * @param param Blynk event payload, 1 = manual requested, 0 = auto.
 * @return void.
 */
BLYNK_WRITE(V0) {
  manualModeRequested = (bool)param.asInt();
}

/**
 * @brief Blynk V1 switch: Manual Vent ON/OFF. Only takes effect while the
 *        system is actually in MANUAL mode (priority hierarchy enforced
 *        centrally in loop(), not here).
 * @param param Blynk event payload, 1 = open, 0 = closed.
 * @return void.
 */
BLYNK_WRITE(V1) {
  manualVentOpen = (bool)param.asInt();
}

/**
 * @brief Called once when the device (re)connects to the Blynk server.
 *        Re-syncs V0/V1 so the app state matches the device state.
 * @return void.
 */
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V0, V1);
}

// ---------------- Setup ----------------

/**
 * @brief Initialises serial, sensors, pins, interrupts, the servo, the
 *        OLED, and starts the Blynk/WiFi connection.
 * @return void.
 */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("=== Micro-Climate Nursery Controller (PHYSICAL+BLYNK) ==="));

  dht.begin();

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BTN_VENT_PIN, INPUT_PULLUP);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BTN_VENT_PIN), onVentButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_MODE_PIN), onModeButton, FALLING);

  ventServo.setPeriodHertz(50);
  ventServo.attach(SERVO_PIN, 500, 2400);
  ventServo.write(VENT_CLOSED_ANGLE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    Serial.println(F("OLED init failed"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Connecting WiFi..."));
  display.display();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

/**
 * @brief Reads sensors and flags a fault if any value is missing/out of range.
 * @return void.
 */
void checkSensorFault() {
  sensorFault = false;
  faultReason = "";

  if (isnan(temperature) || isnan(humidity)) {
    sensorFault = true;
    faultReason = "DHT22 disconnected";
  } else if (temperature < -40 || temperature > 80) {
    sensorFault = true;
    faultReason = "DHT22 out-of-range";
  } else if (ldrRaw <= 0 || ldrRaw >= 4095) {
    sensorFault = true;
    faultReason = "LDR stuck/disconnected";
  }
}

/**
 * @brief Autonomous climate logic - severity-weighted conflict resolution.
 * @return void. Sets targetAngle, growLightOn, conflictActive.
 */
void applyAutonomousLogic() {
  int lightThreshold = map(potRaw, 0, 4095, 500, 3500);
  growLightOn = (ldrRaw < lightThreshold); // dark -> lights on

  float coldSeverity = COLD_THRESHOLD - temperature;
  float heatSeverity  = temperature - HEAT_THRESHOLD;

  conflictActive = false;

  if (coldSeverity > 0 && heatSeverity > 0) {
    conflictActive = true;
    float weightedCold = coldSeverity * COLD_SAFETY_WEIGHT;
    if (weightedCold >= heatSeverity) {
      targetAngle = VENT_CLOSED_ANGLE;
    } else {
      targetAngle = map(constrain((int)temperature, (int)HEAT_THRESHOLD, (int)HEAT_THRESHOLD + 10),
                         (int)HEAT_THRESHOLD, (int)HEAT_THRESHOLD + 10,
                         VENT_MIN_OPEN_ANGLE, VENT_MAX_OPEN_ANGLE);
    }
  } else if (coldSeverity > 0) {
    targetAngle = VENT_CLOSED_ANGLE;
  } else if (heatSeverity > 0) {
    targetAngle = map(constrain((int)temperature, (int)HEAT_THRESHOLD, (int)HEAT_THRESHOLD + 10),
                       (int)HEAT_THRESHOLD, (int)HEAT_THRESHOLD + 10,
                       VENT_MIN_OPEN_ANGLE, VENT_MAX_OPEN_ANGLE);
  } else {
    targetAngle = VENT_CLOSED_ANGLE;
  }
}

/**
 * @brief Manual override: vent responds only to button/app; automation suspended.
 * @return void.
 */
void applyManualState() {
  targetAngle    = manualVentOpen ? VENT_MAX_OPEN_ANGLE : VENT_CLOSED_ANGLE;
  growLightOn    = false;
  conflictActive = false;
}

/**
 * @brief Predefined safe posture used whenever a sensor fault is active.
 * @return void.
 */
void applySafetyPosture() {
  targetAngle    = VENT_CLOSED_ANGLE;
  growLightOn    = false;
  conflictActive = false;
}

/**
 * @brief Moves the servo one step toward targetAngle every 20ms, giving
 *        smooth non-blocking motion instead of jumping to position.
 * @return void.
 */
void updateServo() {
  if (millis() - lastServoMove >= 20) {
    lastServoMove = millis();
    if (ventAngle < targetAngle) ventAngle++;
    else if (ventAngle > targetAngle) ventAngle--;
    ventServo.write(ventAngle);
  }
}

/**
 * @brief Refreshes the OLED with mode-appropriate status information.
 * @return void.
 */
void updateOLED() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  if (currentMode == SAFETY_FAULT) {
    display.println(F("*** SENSOR FAULT ***"));
    display.println(faultReason);
    display.println(F("Vent forced CLOSED"));
  } else {
    display.print(F("Mode: "));
    display.println(currentMode == MANUAL ? F("MANUAL OVERRIDE") : F("AUTONOMOUS"));
    display.print(F("Temp: ")); display.print(temperature, 1); display.println(F(" C"));
    display.print(F("Hum : ")); display.print(humidity, 1); display.println(F(" %"));
    display.print(F("Light raw: ")); display.println(ldrRaw);
    display.print(F("Vent: ")); display.print(ventAngle); display.println(F(" deg"));
    display.print(F("Grow LED: ")); display.println(growLightOn ? F("ON") : F("OFF"));
    if (conflictActive) display.println(F("!! CLIMATE CONFLICT !!"));
    display.println(F("Blynk: connected"));
  }
  display.display();
}

/**
 * @brief Prints a single-line status snapshot to the serial terminal.
 * @return void.
 */
void printSerialStatus() {
  Serial.print(F("Mode="));
  Serial.print(currentMode == AUTONOMOUS ? "AUTO" : (currentMode == MANUAL ? "MANUAL" : "FAULT"));
  Serial.print(F(" | T=")); Serial.print(temperature);
  Serial.print(F(" | H=")); Serial.print(humidity);
  Serial.print(F(" | LDR=")); Serial.print(ldrRaw);
  Serial.print(F(" | POT=")); Serial.print(potRaw);
  Serial.print(F(" | Vent=")); Serial.print(ventAngle);
  Serial.print(F(" | Grow=")); Serial.print(growLightOn);
  Serial.print(F(" | Conflict=")); Serial.print(conflictActive);
  if (sensorFault) { Serial.print(F(" | FAULT=")); Serial.print(faultReason); }
  Serial.println();
}

/**
 * @brief Pushes live readings and a text status to the Blynk dashboard.
 * @return void.
 */
void pushToBlynk() {
  Blynk.virtualWrite(V2, temperature);
  Blynk.virtualWrite(V3, humidity);
  int lightPercent = map(ldrRaw, 0, 4095, 0, 100);
  Blynk.virtualWrite(V4, lightPercent);

  String status;
  if (currentMode == SAFETY_FAULT) status = "FAULT: " + faultReason;
  else if (currentMode == MANUAL)   status = "MANUAL OVERRIDE";
  else if (conflictActive)          status = "AUTO - CLIMATE CONFLICT";
  else                               status = "AUTO - NORMAL";
  Blynk.virtualWrite(V5, status);
}

// ---------------- Main Loop (no delay()) ----------------

/**
 * @brief Main non-blocking loop: services Blynk, samples sensors on
 *        schedule, resolves the mode priority hierarchy (fault > manual >
 *        autonomous), applies the resulting actuator state, and refreshes
 *        the OLED/serial/Blynk outputs on their own independent intervals.
 * @return void.
 */
void loop() {
  Blynk.run();

  if (millis() - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = millis();
    temperature = dht.readTemperature();
    humidity    = dht.readHumidity();
    ldrRaw      = analogRead(LDR_PIN);
    potRaw      = analogRead(POT_PIN);
  }

  // ---- Priority hierarchy: SENSOR FAULT > MANUAL > AUTONOMOUS ----
  // Applies identically whether MANUAL/AUTO was requested by the physical
  // button or by the Blynk V0 switch - the app cannot bypass a fault state.
  checkSensorFault();
  if (sensorFault) {
    currentMode = SAFETY_FAULT;
  } else if (manualModeRequested) {
    currentMode = MANUAL;
  } else {
    currentMode = AUTONOMOUS;
  }

  switch (currentMode) {
    case SAFETY_FAULT: applySafetyPosture();    break;
    case MANUAL:        applyManualState();     break;
    case AUTONOMOUS:    applyAutonomousLogic();  break;
  }

  updateServo();
  digitalWrite(GREEN_LED_PIN, growLightOn ? HIGH : LOW);
  digitalWrite(RED_LED_PIN, (currentMode == SAFETY_FAULT || conflictActive) ? HIGH : LOW);

  if (millis() - lastOledUpdate >= 500) {
    lastOledUpdate = millis();
    updateOLED();
  }

  if (millis() - lastSerialPrint >= 1000) {
    lastSerialPrint = millis();
    printSerialStatus();
  }

  if (millis() - lastBlynkPush >= 2000) {
    lastBlynkPush = millis();
    pushToBlynk();
  }
}
