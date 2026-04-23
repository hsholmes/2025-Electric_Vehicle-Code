#include <Wire.h>
#include <FspTimer.h>
#include <ArduinoBLE.h>

// Pin Definitions
#define LEFT_STEP_PIN 9
#define LEFT_DIR_PIN 12
#define LEFT_EN_PIN 8
#define RIGHT_STEP_PIN 10
#define RIGHT_DIR_PIN 6
#define RIGHT_EN_PIN 7
#define BUTTON_PIN 2

// ICM-20948 registers
#define ICM_ADDR 0x69
#define ICM_REG_BANK 0x7F
#define ICM_PWR_MGMT_1 0x06
#define ICM_GYRO_CFG 0x01
#define ICM_GYRO_ZOUT_H 0x37
#define ICM_WHO_AM_I 0x00

// Fixed Physical Parameters
const float WHEEL_DIAMETER_MM = 76.2;
const float STEPS_PER_REV = 200.0;
const int MICROSTEP = 8;
const float TRACK_WIDTH_MM = 270.0;
const float ACCEL_TIME_SEC = 2.0;
const float MM_PER_REV = PI * WHEEL_DIAMETER_MM;

// Runtime-tunable parameters (all are able to update via BLE for tuning)
volatile float rt_KP_HEADING = 12.0;
volatile float rt_BIAS = 10.0;
volatile float rt_DIST_CORRECTION_MM = -150.0;
volatile float rt_ARC_RADIUS_MM = 12226.56;
volatile float rt_ARC_ANGLE_DEG = 41.68;
volatile unsigned long rt_TARGET_TIME_MS = 12600;

// Arc-derived values (recalculated by recalcArc())
float LEFT_ARC_MM = 0;
float RIGHT_ARC_MM = 0;
long LEFT_TOTAL_STEPS = 0;
long RIGHT_TOTAL_STEPS = 0;
float LEFT_BASE_SPEED = 0;
float RIGHT_BASE_SPEED = 0;
float LEFT_ACCEL = 0;
float RIGHT_ACCEL = 0;


// Recalculate all arc-dependent values from current runtime parameters
// Called on startup and whenever any arc parameter changes via BLE
void recalcArc() {
  float angleRad = rt_ARC_ANGLE_DEG * PI / 180.0;

  // Right turn: left wheel is outer (longer), right wheel is inner (shorter)
  LEFT_ARC_MM = (rt_ARC_RADIUS_MM + TRACK_WIDTH_MM / 2.0) * angleRad;
  RIGHT_ARC_MM = (rt_ARC_RADIUS_MM - TRACK_WIDTH_MM / 2.0) * angleRad;

  // Distance correction added proportionally to both wheels
  long corrSteps = (long)(rt_DIST_CORRECTION_MM / MM_PER_REV * STEPS_PER_REV * MICROSTEP);
  LEFT_TOTAL_STEPS = (long)(LEFT_ARC_MM / MM_PER_REV * STEPS_PER_REV * MICROSTEP) + corrSteps;
  RIGHT_TOTAL_STEPS = (long)(RIGHT_ARC_MM / MM_PER_REV * STEPS_PER_REV * MICROSTEP) + corrSteps;

  float timeSec = rt_TARGET_TIME_MS / 1000.0;
  LEFT_BASE_SPEED = LEFT_TOTAL_STEPS / timeSec;
  RIGHT_BASE_SPEED = RIGHT_TOTAL_STEPS / timeSec;
  LEFT_ACCEL = LEFT_BASE_SPEED / ACCEL_TIME_SEC;
  RIGHT_ACCEL = RIGHT_BASE_SPEED / ACCEL_TIME_SEC;

  Serial.println("--- Arc recalculated ---");
  Serial.print("Radius:      ");
  Serial.print(rt_ARC_RADIUS_MM);
  Serial.println(" mm");
  Serial.print("Angle:       ");
  Serial.print(rt_ARC_ANGLE_DEG);
  Serial.println(" deg");
  Serial.print("Time:        ");
  Serial.print(rt_TARGET_TIME_MS);
  Serial.println(" ms");
  Serial.print("Left arc:    ");
  Serial.print(LEFT_ARC_MM, 1);
  Serial.println(" mm (outer)");
  Serial.print("Right arc:   ");
  Serial.print(RIGHT_ARC_MM, 1);
  Serial.println(" mm (inner)");
  Serial.print("Left steps:  ");
  Serial.println(LEFT_TOTAL_STEPS);
  Serial.print("Right steps: ");
  Serial.println(RIGHT_TOTAL_STEPS);
  Serial.print("Left speed:  ");
  Serial.print(LEFT_BASE_SPEED, 1);
  Serial.println(" st/s");
  Serial.print("Right speed: ");
  Serial.print(RIGHT_BASE_SPEED, 1);
  Serial.println(" st/s");
  Serial.println("------------------------");
}

// FspTimer stepper engine (50kHz ISR)
// Generates step pulses independently of loop() speed
#define TIMER_FREQ 50000

FspTimer stepTimer;

volatile long leftPosition = 0;
volatile long rightPosition = 0;
volatile bool leftRunning = false;
volatile bool rightRunning = false;
volatile float leftCurrentSpeed = 0;
volatile float rightCurrentSpeed = 0;
volatile float leftTargetSpeed = 0;
volatile float rightTargetSpeed = 0;
volatile float leftCounter = 0;
volatile float rightCounter = 0;
volatile long leftTotalSteps = 0;
volatile long rightTotalSteps = 0;

void stepISR(timer_callback_args_t* args) {
  // Each motor accelerates proportionally so both reach target at same time
  float leftAccelStep = (LEFT_BASE_SPEED / ACCEL_TIME_SEC) / TIMER_FREQ;
  float rightAccelStep = (RIGHT_BASE_SPEED / ACCEL_TIME_SEC) / TIMER_FREQ;

  // Ramp left speed toward target
  if (leftCurrentSpeed < leftTargetSpeed)
    leftCurrentSpeed = min(leftCurrentSpeed + leftAccelStep, leftTargetSpeed);
  else if (leftCurrentSpeed > leftTargetSpeed)
    leftCurrentSpeed = max(leftCurrentSpeed - leftAccelStep, leftTargetSpeed);

  // Ramp right speed toward target
  if (rightCurrentSpeed < rightTargetSpeed)
    rightCurrentSpeed = min(rightCurrentSpeed + rightAccelStep, rightTargetSpeed);
  else if (rightCurrentSpeed > rightTargetSpeed)
    rightCurrentSpeed = max(rightCurrentSpeed - rightAccelStep, rightTargetSpeed);

  // Left motor step
  if (leftRunning && leftPosition < leftTotalSteps && leftCurrentSpeed > 0) {
    leftCounter += leftCurrentSpeed / TIMER_FREQ;
    if (leftCounter >= 1.0) {
      leftCounter -= 1.0;
      digitalWrite(LEFT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, LOW);
      leftPosition++;
      if (leftPosition >= leftTotalSteps) {
        leftRunning = false;
        leftCurrentSpeed = 0;
      }
    }
  }

  // Right motor step
  if (rightRunning && rightPosition < rightTotalSteps && rightCurrentSpeed > 0) {
    rightCounter += rightCurrentSpeed / TIMER_FREQ;
    if (rightCounter >= 1.0) {
      rightCounter -= 1.0;
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(RIGHT_STEP_PIN, LOW);
      rightPosition++;
      if (rightPosition >= rightTotalSteps) {
        rightRunning = false;
        rightCurrentSpeed = 0;
      }
    }
  }
}

void startMotors() {
  leftPosition = rightPosition = 0;
  leftCounter = rightCounter = 0;
  leftCurrentSpeed = rightCurrentSpeed = 0;
  leftTotalSteps = LEFT_TOTAL_STEPS;
  rightTotalSteps = RIGHT_TOTAL_STEPS;
  leftTargetSpeed = LEFT_BASE_SPEED;
  rightTargetSpeed = RIGHT_BASE_SPEED;
  leftRunning = rightRunning = true;
}

void stopMotors() {
  leftRunning = rightRunning = false;
  leftCurrentSpeed = rightCurrentSpeed = 0;
}

// Heading and state
float headingAngle = 0.0;
float gyroZ_offset = 0.0;
unsigned long lastTime = 0;
unsigned long startTime = 0;
bool vehicleStarted = false;

// BLE Setup
// statusChar: Arduino → PC (stataus)
// cmdChar:    PC → Arduino (write, event-driven)
BLEService vehicleService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEStringCharacteristic statusChar(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLENotify,
  100);

BLEStringCharacteristic cmdChar(
  "19B10002-E8F2-537E-4F6C-D104768A1214",
  BLEWrite | BLEWriteWithoutResponse,
  50);

unsigned long lastBLESend = 0;

// Flag set by BLE callback, printed in loop to confirm callback fires
volatile bool bleDebugFlag = false;
String bleLastCmd = "";


// BLE command handler (event callback)
void onCmdReceived(BLEDevice central, BLECharacteristic characteristic) {

  // Read raw bytes directly - more reliable than .value() on R4
  int len = characteristic.valueLength();
  if (len <= 0) return;

  char buf[51] = { 0 };
  memcpy(buf, characteristic.value(), min(len, 50));
  String cmd = String(buf);
  cmd.trim();

  // Set debug flag so loop() can confirm callback fired
  bleDebugFlag = true;
  bleLastCmd = cmd;

  int colon = cmd.indexOf(':');
  if (colon <= 0) return;

  String key = cmd.substring(0, colon);
  float val = cmd.substring(colon + 1).toFloat();

  if (key == "KP") {
    rt_KP_HEADING = val;
    Serial.print("BLE → KP: ");
    Serial.println(val);

  } else if (key == "BIAS") {
    rt_BIAS = val;
    Serial.print("BLE → BIAS: ");
    Serial.println(val);

  } else if (key == "DIST") {
    rt_DIST_CORRECTION_MM = val;
    Serial.print("BLE → DIST: ");
    Serial.println(val);
    recalcArc();

  } else if (key == "RADIUS") {
    rt_ARC_RADIUS_MM = val;
    Serial.print("BLE → RADIUS: ");
    Serial.println(val);
    recalcArc();

  } else if (key == "ANGLE") {
    rt_ARC_ANGLE_DEG = val;
    Serial.print("BLE → ANGLE: ");
    Serial.println(val);
    recalcArc();

  } else if (key == "TIME") {
    rt_TARGET_TIME_MS = (unsigned long)val;
    Serial.print("BLE → TIME: ");
    Serial.print(val);
    Serial.println(" ms");
    recalcArc();

  } else if (key == "RESET") {
    stopMotors();
    digitalWrite(LEFT_EN_PIN, LOW);
    digitalWrite(RIGHT_EN_PIN, LOW);
    vehicleStarted = false;
    headingAngle = 0.0;
    lastTime = millis();
    recalcArc();
    Serial.println("BLE → RESET. Waiting for button...");
    if (BLE.connected()) statusChar.writeValue("STATUS:Ready - press button to run");

  } else {
    Serial.print("BLE → Unknown: ");
    Serial.println(cmd);
  }
}

// Direct I2C helpers
void icmSelectBank(byte bank) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_REG_BANK);
  Wire.write(bank << 4);
  Wire.endTransmission();
}

void icmWriteRegister(byte bank, byte reg, byte value) {
  icmSelectBank(bank);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Returns raw gyro Z value, or INT16_MIN (0x8000) as error sentinel
int16_t icmReadGyroZ() {
  icmSelectBank(0);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  // requestFrom returns number of bytes received - must be 2
  uint8_t received = Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)2);
  if (received < 2) {
    // I2C read failed - drain any partial data
    while (Wire.available()) Wire.read();
    return INT16_MIN;  // Sentinel value = read failed
  }

  byte high = Wire.read();
  byte low = Wire.read();
  return (int16_t)((high << 8) | low);
}

// Returns dps value, or NAN if I2C read failed
float icmGetGyroZ_dps() {
  int16_t raw = icmReadGyroZ();
  if (raw == INT16_MIN) return NAN;  // Signal read failure to caller
  return (raw - gyroZ_offset) / 131.0;
}

void initI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(100000);
}

bool icmInit() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("ICM-20948 attempt ");
    Serial.print(attempt);
    Serial.println("/3...");
    icmSelectBank(0);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(ICM_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
    byte whoAmI = Wire.available() ? Wire.read() : 0xFF;
    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    if (whoAmI == 0xEA) {
      icmWriteRegister(0, ICM_PWR_MGMT_1, 0x01);
      delay(100);
      icmWriteRegister(2, ICM_GYRO_CFG, 0x31);
      delay(50);  // DLPF 5.7Hz
      Serial.println("ICM-20948 OK!");
      return true;
    }
    initI2C();
    delay(500);
  }
  return false;
}

void icmCalibrateGyro() {
  Serial.println("Calibrating gyro... keep vehicle still!");
  for (int i = 0; i < 200; i++) {
    icmReadGyroZ();
    delay(5);
  }  // Warmup

  long sum = 0;
  int good = 0;
  int attempts = 0;

  // Keep reading until 500 good samples collected
  while (good < 500 && attempts < 800) {
    attempts++;
    int16_t raw = icmReadGyroZ();
    if (raw != INT16_MIN) {
      sum += raw;
      good++;
    }
    delay(5);
  }

  gyroZ_offset = sum / (float)good;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_offset, 2);
  Serial.print("Good samples:  ");
  Serial.print(good);
  Serial.print(" / ");
  Serial.println(attempts);
  if (good < 400) Serial.println("WARNING: Many I2C failures during calibration!");
  else Serial.println("Calibration OK.");
}

// BLE telemetry broadcast every 200ms
// H = heading error | L/R = motor speeds | C = correction
// D = arc progress scaled to 0-7000 for dashboard progress bar
void sendBLEStatus(float headingError, float leftSpd, float rightSpd, float correction) {
  if (!BLE.connected()) return;
  if (millis() - lastBLESend < 200) return;
  lastBLESend = millis();

  float arcPctScaled = (leftPosition / (float)LEFT_TOTAL_STEPS) * 7000.0;
  char buf[100];
  snprintf(buf, sizeof(buf),
           "H:%.1f L:%.0f R:%.0f C:%.1f D:%.0f",
           headingError, leftSpd, rightSpd, correction, arcPctScaled);
  statusChar.writeValue(buf);
}

// Stop function
// Sets vehicleStarted=false so loop returns to button-wait state
// BLE callback (onCmdReceived) remains active via BLE.poll()
void stopVehicle(const char* reason) {
  stopMotors();
  digitalWrite(LEFT_EN_PIN, HIGH);
  digitalWrite(RIGHT_EN_PIN, HIGH);

  float angleTraveled = (leftPosition / (float)LEFT_TOTAL_STEPS) * rt_ARC_ANGLE_DEG;

  Serial.print("Stopped! Reason: ");
  Serial.println(reason);
  Serial.print("Time elapsed:    ");
  Serial.print((millis() - startTime) / 1000.0, 2);
  Serial.println(" sec");
  Serial.print("Left steps:      ");
  Serial.println(leftPosition);
  Serial.print("Right steps:     ");
  Serial.println(rightPosition);
  Serial.print("Arc traveled:    ");
  Serial.print(angleTraveled, 1);
  Serial.println(" deg");
  Serial.print("Final gyro:      ");
  Serial.print(headingAngle, 2);
  Serial.println(" deg");
  Serial.println("Send RESET from dashboard to run again.");

  if (BLE.connected()) {
    char buf[100];
    snprintf(buf, sizeof(buf), "STOP:%s A:%.1f H:%.1f", reason, angleTraveled, headingAngle);
    statusChar.writeValue(buf);
  }

  // Return to button-wait state - loop() handles the rest
  vehicleStarted = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;
  Serial.println("=== System Starting (Arc Mode, Right Turn) ===");

  pinMode(LEFT_STEP_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_DIR_PIN, HIGH);  // Left forward
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(RIGHT_DIR_PIN, LOW);  // Right forward (inverted)
  pinMode(LEFT_EN_PIN, OUTPUT);
  digitalWrite(LEFT_EN_PIN, LOW);  // Disabled until start
  pinMode(RIGHT_EN_PIN, OUTPUT);
  digitalWrite(RIGHT_EN_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);
  Serial.println("[1] Pins OK");

  // Calculate initial arc values
  recalcArc();

  // FspTimer at 50kHz for hardware step generation
  uint8_t timerType = AGT_TIMER;
  int8_t timerCh = FspTimer::get_available_timer(timerType);
  if (timerCh < 0) {
    timerType = GPT_TIMER;
    timerCh = FspTimer::get_available_timer(timerType);
  }
  stepTimer.begin(TIMER_MODE_PERIODIC, timerType, timerCh, TIMER_FREQ, 0.0f, stepISR);
  stepTimer.setup_overflow_irq();
  stepTimer.open();
  stepTimer.start();
  Serial.println("[2] FspTimer OK");

  initI2C();
  Serial.println("[3] I2C OK");

  if (!icmInit()) {
    Serial.println("ICM-20948 FAILED!");
    while (true)
      ;
  }
  Serial.println("[4] ICM-20948 OK");

  Serial.println("Waiting 3000ms for DLPF to settle...");
  delay(3000);
  icmCalibrateGyro();
  Serial.println("[5] Calibration OK");

  if (!BLE.begin()) {
    Serial.println("BLE FAILED!");
    while (true)
      ;
  }
  BLE.setLocalName("SciolyEV");
  BLE.setAdvertisedService(vehicleService);
  vehicleService.addCharacteristic(statusChar);
  vehicleService.addCharacteristic(cmdChar);
  BLE.addService(vehicleService);

  // Register event callback - fires immediately when PC writes to cmdChar
  cmdChar.setEventHandler(BLEWritten, onCmdReceived);

  BLE.advertise();
  Serial.println("[6] BLE OK - advertising as 'SciolyEV'");
  Serial.println("---");
  Serial.println("Connect to 'SciolyEV' then press button to start.");
  Serial.println("Or send params from dashboard first, then press button.");
}


void loop() {

  // Gyro read FIRST to keep dt accurate
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (vehicleStarted) {
    float gz = icmGetGyroZ_dps();
    if (!isnan(gz)) {
      // Only integrate when read succeeded
      headingAngle += gz * dt;
    } else {
      // I2C failed this cycle - count for diagnostics
      static int failCount = 0;
      failCount++;
      static unsigned long lastFailPrint = 0;
      if (millis() - lastFailPrint > 1000) {
        lastFailPrint = millis();
        Serial.print("IMU read failures last sec: ");
        Serial.println(failCount);
        failCount = 0;
      }
    }
  }

  // Keep BLE alive (callback fires inside poll)
  BLE.poll();

  // Debug: confirm BLE callback fired
  if (bleDebugFlag) {
    bleDebugFlag = false;
    Serial.print(">>> Callback fired! Raw cmd: [");
    Serial.print(bleLastCmd);
    Serial.println("]");
    Serial.print(">>> Current params: KP=");
    Serial.print(rt_KP_HEADING);
    Serial.print(" BIAS=");
    Serial.print(rt_BIAS);
    Serial.print(" R=");
    Serial.print(rt_ARC_RADIUS_MM);
    Serial.print(" A=");
    Serial.print(rt_ARC_ANGLE_DEG);
    Serial.print(" T=");
    Serial.println(rt_TARGET_TIME_MS);
  }

  // Wait for button press (also handles post-stop and post-reset state)
  if (!vehicleStarted) {
    if (BLE.connected()) {
      static unsigned long lastIdle = 0;
      if (millis() - lastIdle > 1000) {
        lastIdle = millis();
        char buf[80];
        snprintf(buf, sizeof(buf), "STATUS:Ready KP:%.1f R:%.0f A:%.0f T:%lu",
                 rt_KP_HEADING, rt_ARC_RADIUS_MM, rt_ARC_ANGLE_DEG, rt_TARGET_TIME_MS);
        statusChar.writeValue(buf);
      }
    }

    if (digitalRead(BUTTON_PIN) == HIGH) {
      delay(50);
      if (digitalRead(BUTTON_PIN) == HIGH) {
        vehicleStarted = true;
        headingAngle = 0.0;
        startTime = millis();
        lastTime = millis();
        startMotors();

        Serial.println("Button pressed! Arc starting...");
        Serial.print("KP: ");
        Serial.println(rt_KP_HEADING);
        Serial.print("BIAS: ");
        Serial.println(rt_BIAS);
        Serial.print("RADIUS: ");
        Serial.println(rt_ARC_RADIUS_MM);
        Serial.print("ANGLE: ");
        Serial.println(rt_ARC_ANGLE_DEG);
        Serial.print("TIME: ");
        Serial.println(rt_TARGET_TIME_MS);

        if (BLE.connected()) statusChar.writeValue("STATUS:Running arc!");
        while (digitalRead(BUTTON_PIN) == HIGH)
          ;  // Wait for button release
      }
    }
    return;
  }

  // Arc IMU correction
  float arcProgress = leftPosition / (float)LEFT_TOTAL_STEPS;
  float expectedHeading = -(arcProgress * rt_ARC_ANGLE_DEG);
  float headingError = headingAngle - expectedHeading;
  float kpCorrection = rt_KP_HEADING * headingError;

  float leftSpd = LEFT_BASE_SPEED + kpCorrection + rt_BIAS;
  float rightSpd = RIGHT_BASE_SPEED - kpCorrection - rt_BIAS;

  leftTargetSpeed = constrain(leftSpd, LEFT_BASE_SPEED * 0.8, LEFT_BASE_SPEED * 1.2);
  rightTargetSpeed = constrain(rightSpd, RIGHT_BASE_SPEED * 0.8, RIGHT_BASE_SPEED * 1.2);

  // Serial debug every 200ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("Actual:");
    Serial.print(headingAngle, 1);
    Serial.print(" Exp:");
    Serial.print(expectedHeading, 1);
    Serial.print(" Err:");
    Serial.print(headingError, 1);
    Serial.print(" Corr:");
    Serial.print(kpCorrection, 1);
    Serial.print(" Arc:");
    Serial.print(arcProgress * 100.0, 0);
    Serial.println("%");
  }

  // BLE telemetry
  sendBLEStatus(headingError, leftSpd, rightSpd, kpCorrection);

  // Stop conditions
  if (!leftRunning && !rightRunning) stopVehicle("Arc complete");
  if (millis() - startTime >= rt_TARGET_TIME_MS) stopVehicle("Time limit reached");
}
