
#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// LCD
// =====================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// ACTUATOR DRIVER - BTS7960 #1
// BOTH ACTUATORS CONNECTED TO THIS DRIVER
// =====================================================

#define ACT_RPWM 5
#define ACT_LPWM 6
#define ACT_REN  7
#define ACT_LEN  8

// =====================================================
// SOLENOID DRIVER - BTS7960 #2
// ON = UNLOCK
// OFF = LOCK
// =====================================================

#define SOL_RPWM 4
#define SOL_LPWM 3
#define SOL_REN  9
#define SOL_LEN  10

// =====================================================
// ULTRASONIC SENSOR - HC-SR04
// =====================================================

#define TRIG_PIN 11
#define ECHO_PIN 12

// =====================================================
// KEYPAD - 4x3
// =====================================================

const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {13, 2, 0};

Keypad keypad = Keypad(
  makeKeymap(keys),
  rowPins,
  colPins,
  ROWS,
  COLS
);

// =====================================================
// SETTINGS
// =====================================================

const char CORRECT_PIN[] = "2580";

// Parcel detection range in millimeters
const int PARCEL_MIN_DISTANCE = 50;
const int PARCEL_MAX_DISTANCE = 180;

// Empty-space detection
const int EMPTY_DISTANCE = 280;

// Parcel must remain detected for 20 seconds
const unsigned long PARCEL_DETECTION_DURATION = 5000UL;

// Required consecutive empty readings
const int REQUIRED_READINGS = 3;

// Motor speed
const int MOTOR_SPEED = 255;

// Extension safety timeout
const unsigned long MAX_EXTENSION_TIME = 50000UL;

// Sensor and LCD intervals
const unsigned long SENSOR_INTERVAL = 100UL;
const unsigned long LCD_INTERVAL = 300UL;

// Solenoid unlock duration: 5 seconds
const unsigned long SOLENOID_UNLOCK_DURATION = 1000UL;

// Wait 30 seconds before retraction
const unsigned long RETRACTION_DELAY = 30000UL;

// =====================================================
// MOVEMENT TIMING CALIBRATION
// =====================================================

// 1.00 = same extension and retraction duration
// 1.20 = retraction takes 120% of extension duration

const float RETRACTION_TIME_FACTOR = 1.20;

// =====================================================
// LOCKER STATES
// =====================================================

enum LockerState {
  RETRACTED,
  EXTENDING,
  EXTENDED,
  RETRACTING
};

LockerState lockerState = RETRACTED;

// =====================================================
// MOVEMENT TIMERS
// =====================================================

unsigned long extensionStartTime = 0;
unsigned long retractionStartTime = 0;

unsigned long totalExtensionTime = 0;
unsigned long retractDuration = 0;
unsigned long actualRetractionTime = 0;

// =====================================================
// SOLENOID TIMERS
// =====================================================

bool solenoidUnlocked = false;

unsigned long solenoidUnlockStartTime = 0;

// =====================================================
// DELAYED RETRACTION
// =====================================================

bool retractionPending = false;

unsigned long retractionRequestTime = 0;

// =====================================================
// SENSOR AND LCD TIMERS
// =====================================================

unsigned long lastSensorRead = 0;
unsigned long lastLCDUpdate = 0;

// =====================================================
// PARCEL DETECTION COUNTERS
// =====================================================

unsigned long parcelDetectionStartTime = 0;

bool parcelDetectionActive = false;

int emptyCount = 0;

int currentDistance = 0;

// =====================================================
// PIN INPUT
// =====================================================

char enteredPIN[5];

byte pinIndex = 0;

// =====================================================
// LCD MESSAGE
// =====================================================

String statusMessage = "";

unsigned long messageUntil = 0;

// =====================================================
// SERIAL DEBUGGING
// =====================================================

const char* getLockerStateName() {

  switch (lockerState) {

    case RETRACTED:
      return "RETRACTED";

    case EXTENDING:
      return "EXTENDING";

    case EXTENDED:
      return "EXTENDED";

    case RETRACTING:
      return "RETRACTING";

    default:
      return "UNKNOWN";
  }
}

void printSensorDebug() {

  Serial.print(F("[SENSOR] Distance: "));
  Serial.print(currentDistance);

  Serial.print(F(" mm | State: "));
  Serial.print(getLockerStateName());

  if (
    lockerState == RETRACTED ||
    lockerState == EXTENDED
  ) {

    Serial.print(F(" | Parcel range: "));
    Serial.print(PARCEL_MIN_DISTANCE);
    Serial.print(F("-"));
    Serial.print(PARCEL_MAX_DISTANCE);

    Serial.print(F(" mm | Detection: "));

    if (parcelDetectionActive) {

      unsigned long elapsed =
        millis() - parcelDetectionStartTime;

      Serial.print(F("ACTIVE | elapsed: "));
      Serial.print(elapsed / 1000.0, 1);

      Serial.print(F("/20.0 sec"));

    } else {

      Serial.print(F("NOT ACTIVE"));
    }
  }

  if (lockerState == EXTENDING) {

    Serial.print(F(" | Empty readings: "));
    Serial.print(emptyCount);
    Serial.print(F("/"));
    Serial.print(REQUIRED_READINGS);
  }

  if (retractionPending) {

    unsigned long elapsed =
      millis() - retractionRequestTime;

    Serial.print(F(" | Retraction countdown: "));

    if (elapsed < RETRACTION_DELAY) {

      Serial.print(
        (RETRACTION_DELAY - elapsed) / 1000
      );

      Serial.print(F(" sec"));
    }
  }

  Serial.println();
}

// =====================================================
// ACTUATOR FUNCTIONS
// =====================================================

void enableActuatorDriver() {

  digitalWrite(ACT_REN, HIGH);
  digitalWrite(ACT_LEN, HIGH);
}

void stopActuators() {

  analogWrite(ACT_RPWM, 0);
  analogWrite(ACT_LPWM, 0);
}

void extendActuators() {

  enableActuatorDriver();

  analogWrite(ACT_RPWM, MOTOR_SPEED);
  analogWrite(ACT_LPWM, 0);
}

void retractActuators() {

  enableActuatorDriver();

  analogWrite(ACT_RPWM, 0);
  analogWrite(ACT_LPWM, MOTOR_SPEED);
}

// =====================================================
// SOLENOID FUNCTIONS
// =====================================================

void unlockSolenoid() {

  digitalWrite(SOL_REN, HIGH);
  digitalWrite(SOL_LEN, HIGH);

  // ON = UNLOCK
  digitalWrite(SOL_RPWM, HIGH);
  digitalWrite(SOL_LPWM, LOW);
}

void lockSolenoid() {

  // OFF = LOCK
  digitalWrite(SOL_RPWM, LOW);
  digitalWrite(SOL_LPWM, LOW);
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

int readDistanceMM() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    25000UL
  );

  if (duration == 0) {
    return 0;
  }

  return (int)(duration * 0.1715);
}

// =====================================================
// LCD MESSAGE HELPER
// =====================================================

void showMessage(
  String message,
  unsigned long duration = 2000UL
) {

  statusMessage = message;

  messageUntil = millis() + duration;
}

// =====================================================
// EXTENSION TIMING HELPER
// =====================================================

void recordExtensionTime() {

  if (lockerState != EXTENDING) {
    return;
  }

  unsigned long elapsed =
    millis() - extensionStartTime;

  totalExtensionTime += elapsed;

  extensionStartTime = 0;
}

// =====================================================
// START EXTENSION
// =====================================================

void startExtension() {

  Serial.println(
    F("[ACTION] 20-second parcel detection completed.")
  );

  Serial.println(F("[MOTOR] Starting extension."));

  extendActuators();

  extensionStartTime = millis();

  emptyCount = 0;

  parcelDetectionActive = false;
  parcelDetectionStartTime = 0;

  lockerState = EXTENDING;

  Serial.println(
    F("[STATE] RETRACTED/EXTENDED -> EXTENDING")
  );

  showMessage("Extending...");
}

// =====================================================
// STOP EXTENSION AND RECORD TIME
// =====================================================

void stopExtensionSegment() {

  if (lockerState != EXTENDING) {
    return;
  }

  recordExtensionTime();

  stopActuators();

  emptyCount = 0;

  parcelDetectionActive = false;
  parcelDetectionStartTime = 0;

  lockerState = EXTENDED;

  Serial.print(
    F("[STATE] EXTENDING -> EXTENDED | Total extension: ")
  );

  Serial.print(totalExtensionTime);
  Serial.println(F(" ms"));

  showMessage("Space available");
}

// =====================================================
// CALCULATE RETRACTION DURATION
// =====================================================

void calculateRetractionTime() {

  retractDuration = (unsigned long)(
    totalExtensionTime * RETRACTION_TIME_FACTOR
  );

  if (totalExtensionTime > 0 && retractDuration == 0) {
    retractDuration = 1;
  }

  Serial.print(F("[MOTOR] Calculated retraction time: "));
  Serial.print(retractDuration);
  Serial.println(F(" ms"));
}

// =====================================================
// START PIN UNLOCK SEQUENCE
// =====================================================

void startPINUnlockSequence() {

  // Prevent another unlock request during retraction.
  if (
    lockerState == RETRACTING ||
    retractionPending
  ) {

    showMessage("Please wait");

    return;
  }

  // Clear any previous parcel detection timer.
  parcelDetectionActive = false;
  parcelDetectionStartTime = 0;

  // If actuator is extending, stop it first.
  if (lockerState == EXTENDING) {

    recordExtensionTime();

    stopActuators();

    lockerState = EXTENDED;

    emptyCount = 0;

    Serial.println(
      F("[MOTOR] Extension stopped by PIN request.")
    );
  }

  // Unlock solenoid immediately.
  unlockSolenoid();

  solenoidUnlocked = true;

  solenoidUnlockStartTime = millis();

  Serial.println(
    F("[SOLENOID] UNLOCKED. Will lock after 5 seconds.")
  );

  // Check whether actuator retraction is needed.
  if (totalExtensionTime > 0) {

    retractionPending = true;

    retractionRequestTime = millis();

    Serial.println(
      F("[RETRACTION] Scheduled after 30 seconds.")
    );

    showMessage("Unlocked! Wait 30s");

  } else {

    // Locker has never extended.
    // Do not move the actuators.

    retractionPending = false;

    Serial.println(
      F("[SYSTEM] No extension recorded.")
    );

    Serial.println(
      F("[SYSTEM] Solenoid unlock only.")
    );

    showMessage("Locker unlocked!");
  }
}

// =====================================================
// AUTOMATIC SOLENOID LOCKING
// =====================================================

void handleSolenoidAutoLock() {

  if (!solenoidUnlocked) {
    return;
  }

  unsigned long elapsed =
    millis() - solenoidUnlockStartTime;

  if (elapsed >= SOLENOID_UNLOCK_DURATION) {

    lockSolenoid();

    solenoidUnlocked = false;

    Serial.println(
      F("[SOLENOID] Automatically LOCKED after 5 seconds.")
    );

    showMessage("Solenoid locked!");
  }
}

// =====================================================
// DELAYED RETRACTION HANDLER
// =====================================================

void handleRetraction() {

  // ---------------------------------------------------
  // WAIT 30 SECONDS AFTER PIN ENTRY
  // ---------------------------------------------------

  if (retractionPending) {

    unsigned long elapsed =
      millis() - retractionRequestTime;

    if (elapsed >= RETRACTION_DELAY) {

      retractionPending = false;

      calculateRetractionTime();

      if (retractDuration > 0) {

        Serial.println(
          F("[RETRACTION] 30-second delay completed.")
        );

        Serial.println(
          F("[MOTOR] Starting actuator retraction.")
        );

        // Start both actuators together.
        retractActuators();

        retractionStartTime = millis();

        lockerState = RETRACTING;

        showMessage("Retracting...");

      } else {

        Serial.println(
          F("[SYSTEM] No retraction required.")
        );
      }
    }

    return;
  }

  // ---------------------------------------------------
  // MONITOR ACTIVE RETRACTION
  // ---------------------------------------------------

  if (lockerState == RETRACTING) {

    unsigned long elapsed =
      millis() - retractionStartTime;

    if (elapsed >= retractDuration) {

      finishRetraction();
    }
  }
}

// =====================================================
// COMPLETE RETRACTION
// =====================================================

void finishRetraction() {

  actualRetractionTime =
    millis() - retractionStartTime;

  stopActuators();

  // Ensure solenoid is locked.
  lockSolenoid();

  solenoidUnlocked = false;

  // Reset movement timers.
  extensionStartTime = 0;
  retractionStartTime = 0;

  totalExtensionTime = 0;
  retractDuration = 0;

  // Reset parcel detection.
  parcelDetectionActive = false;
  parcelDetectionStartTime = 0;

  emptyCount = 0;

  retractionPending = false;

  lockerState = RETRACTED;

  Serial.print(
    F("[STATE] RETRACTING -> RETRACTED | Actual time: ")
  );

  Serial.print(actualRetractionTime);

  Serial.println(F(" ms"));

  Serial.println(
    F("[SYSTEM] Locker is now locked.")
  );

  showMessage("Locker locked!");
}

// =====================================================
// PIN VALIDATION
// =====================================================

void submitPIN() {

  enteredPIN[pinIndex] = '\0';

  if (strcmp(enteredPIN, CORRECT_PIN) == 0) {

    Serial.println(
      F("[PIN] Correct PIN entered.")
    );

    if (
      lockerState != RETRACTING &&
      !retractionPending
    ) {

      showMessage("Correct PIN!");

      startPINUnlockSequence();

    } else {

      showMessage("Please wait");
    }

  } else {

    Serial.println(
      F("[PIN] Incorrect PIN.")
    );

    showMessage("Incorrect PIN!");
  }

  pinIndex = 0;

  memset(enteredPIN, 0, sizeof(enteredPIN));
}

// =====================================================
// KEYPAD HANDLING
// =====================================================

void handleKeypad() {

  char key = keypad.getKey();

  if (!key) {
    return;
  }

  Serial.print(F("[KEYPAD] Key pressed: "));
  Serial.println(key);

  // Clear PIN
  if (key == '*') {

    pinIndex = 0;

    memset(enteredPIN, 0, sizeof(enteredPIN));

    showMessage("PIN cleared");

    return;
  }

  // Submit PIN
  if (key == '#') {

    if (pinIndex > 0) {
      submitPIN();
    }

    return;
  }

  // Accept numbers only
  if (key >= '0' && key <= '9') {

    if (pinIndex < 4) {

      enteredPIN[pinIndex] = key;

      pinIndex++;

      // Automatically submit after 4 digits.
      if (pinIndex == 4) {
        submitPIN();
      }
    }
  }
}

// =====================================================
// PARCEL DETECTION
// =====================================================

void handleParcelDetection() {

  // Pause detection while unlocking or retracting.
  if (
    retractionPending ||
    solenoidUnlocked ||
    lockerState == RETRACTING
  ) {
    return;
  }

  if (
    millis() - lastSensorRead < SENSOR_INTERVAL
  ) {
    return;
  }

  lastSensorRead = millis();

  currentDistance = readDistanceMM();

  printSensorDebug();

  // ---------------------------------------------------
  // RETRACTED OR EXTENDED:
  // LOOK FOR A NEW PARCEL
  // ---------------------------------------------------

  if (
    lockerState == RETRACTED ||
    lockerState == EXTENDED
  ) {

    if (
      currentDistance >= PARCEL_MIN_DISTANCE &&
      currentDistance <= PARCEL_MAX_DISTANCE
    ) {

      // Start parcel detection timer.
      if (!parcelDetectionActive) {

        parcelDetectionActive = true;

        parcelDetectionStartTime = millis();

        Serial.println(
          F("[DETECTION] Parcel detected. 20-second timer started.")
        );
      }

      unsigned long detectionElapsed =
        millis() - parcelDetectionStartTime;

      // Extend only after continuous detection for 20 seconds.
      if (
        detectionElapsed >= PARCEL_DETECTION_DURATION
      ) {

        Serial.println(
          F("[DETECTION] 20 seconds reached. Starting extension.")
        );

        parcelDetectionActive = false;

        parcelDetectionStartTime = 0;

        startExtension();
      }

    } else {

      // Reset timer if parcel leaves detection range.
      if (parcelDetectionActive) {

        Serial.print(
          F("[DETECTION] Timer RESET. Distance: ")
        );

        Serial.print(currentDistance);

        Serial.println(
          F(" mm (outside detection range or invalid).")
        );
      }

      parcelDetectionActive = false;

      parcelDetectionStartTime = 0;
    }
  }

  // ---------------------------------------------------
  // EXTENDING:
  // STOP WHEN EMPTY SPACE IS DETECTED
  // ---------------------------------------------------

  else if (lockerState == EXTENDING) {

    if (currentDistance >= EMPTY_DISTANCE) {

      emptyCount++;

      Serial.print(
        F("[EXTENSION] Empty-space reading: ")
      );

      Serial.print(emptyCount);

      Serial.print(F("/"));

      Serial.println(REQUIRED_READINGS);

      if (emptyCount >= REQUIRED_READINGS) {

        stopExtensionSegment();
      }

    } else {

      emptyCount = 0;
    }
  }
}

// =====================================================
// EXTENSION SAFETY TIMEOUT
// =====================================================

void handleExtensionTimeout() {

  if (lockerState != EXTENDING) {
    return;
  }

  if (
    millis() - extensionStartTime >= MAX_EXTENSION_TIME
  ) {

    stopExtensionSegment();

    showMessage("Extension limit!");

    Serial.println(
      F("[SAFETY] Maximum extension time reached.")
    );
  }
}

// =====================================================
// LCD DISPLAY
// =====================================================

void updateLCD() {

  static String previousLine1 = "";
  static String previousLine2 = "";

  String line1 = "";
  String line2 = "";

  // Temporary messages
  if (
    millis() < messageUntil &&
    statusMessage.length() > 0
  ) {

    line1 = statusMessage.substring(0, 16);

    if (
      lockerState == EXTENDED ||
      lockerState == EXTENDING ||
      lockerState == RETRACTING
    ) {

      line2 = "PIN: ";

      for (byte i = 0; i < pinIndex; i++) {
        line2 += "*";
      }
    }
  }

  else {

    switch (lockerState) {

      case RETRACTED:

        line1 = "Locker: Locked";
        line2 = "Waiting parcel";

        break;

      case EXTENDING:

        line1 = "Extending...";
        line2 = "Please wait";

        break;

      case EXTENDED:

        line1 = "Locker: Extended";

        line2 = "PIN: ";

        for (byte i = 0; i < pinIndex; i++) {
          line2 += "*";
        }

        break;

      case RETRACTING:

        line1 = "Retracting...";
        line2 = "Please wait";

        break;
    }
  }

  // Pad both lines to 16 characters.
  while (line1.length() < 16) {
    line1 += " ";
  }

  while (line2.length() < 16) {
    line2 += " ";
  }

  // Update only changed lines.
  if (line1 != previousLine1) {

    lcd.setCursor(0, 0);

    lcd.print(line1);

    previousLine1 = line1;
  }

  if (line2 != previousLine2) {

    lcd.setCursor(0, 1);

    lcd.print(line2);

    previousLine2 = line2;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  // Serial Monitor
  Serial.begin(9600);

  delay(200);

  Serial.println();

  Serial.println(
    F("========================================")
  );

  Serial.println(
    F("PARCEL LOCKER SYSTEM")
  );

  Serial.println(
    F("PIN: 2580")
  );

  Serial.println(
    F("Parcel detection: 20 seconds")
  );

  Serial.println(
    F("Solenoid unlock: 5 seconds")
  );

  Serial.println(
    F("Retraction delay: 30 seconds")
  );

  Serial.println(
    F("========================================")
  );

  // Actuator driver pins
  pinMode(ACT_RPWM, OUTPUT);
  pinMode(ACT_LPWM, OUTPUT);

  pinMode(ACT_REN, OUTPUT);
  pinMode(ACT_LEN, OUTPUT);

  // Solenoid driver pins
  pinMode(SOL_RPWM, OUTPUT);
  pinMode(SOL_LPWM, OUTPUT);

  pinMode(SOL_REN, OUTPUT);
  pinMode(SOL_LEN, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Stop actuators initially.
  stopActuators();

  enableActuatorDriver();

  // Initialize solenoid in locked condition.
  digitalWrite(SOL_REN, HIGH);
  digitalWrite(SOL_LEN, HIGH);

  lockSolenoid();

  solenoidUnlocked = false;

  // Initialize LCD
  lcd.init();

  lcd.backlight();

  lcd.setCursor(0, 0);

  lcd.print("Locker System");

  lcd.setCursor(0, 1);

  lcd.print("Initializing...");

  delay(1500);

  lcd.clear();

  // Reset timers
  extensionStartTime = 0;
  retractionStartTime = 0;

  totalExtensionTime = 0;
  retractDuration = 0;

  solenoidUnlockStartTime = 0;

  retractionPending = false;
  retractionRequestTime = 0;

  parcelDetectionActive = false;
  parcelDetectionStartTime = 0;

  emptyCount = 0;

  showMessage("System ready");

  Serial.println(
    F("[SYSTEM] Ready. Waiting for parcel detection.")
  );
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // 1. Read keypad continuously.
  handleKeypad();

  // 2. Automatically lock solenoid after 5 seconds.
  handleSolenoidAutoLock();

  // 3. Monitor parcel detection.
  handleParcelDetection();

  // 4. Check extension safety timeout.
  handleExtensionTimeout();

  // 5. Handle 30-second retraction delay
  //    and active actuator retraction.
  handleRetraction();

  // 6. Update LCD.
  updateLCD();
}
