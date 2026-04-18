/*
 * =====================================================================================
 *  🤖 Robotic Arm — Master Firmware
 * =====================================================================================
 *  Version:    1.0.0 (Production Release)
 *  Maintainer: Ahmed Wassef
 *
 *  @brief      High-precision, non-blocking serial controller for 6-DOF robotic arms.
 *
 *  @details
 *  This firmware is the bridge between high-level controllers (ROS 2/Python) and
 *  physical servo hardware. It implements asynchronous trajectory interpolation
 *  to ensure smooth movements without freezing the serial listener.
 *
 *  @protocol   Standard S-Protocol: S:<Waist>,<Shoulder>,<Elbow>,<Roll>,<Pitch>,<Grip>\n
 *  @example    S:90,120,60,90,90,30\n
 *
 *  @features
 *    - strtok_r thread-safe packet parsing.
 *    - Milli-stepping interpolation engine (non-blocking).
 *    - Hardware-level mechanical safety clamping (constrain).
 * =====================================================================================
 */

#include <Servo.h>
#include <string.h>

// ─── Constants ───────────────────────────────────────────────────────────────
#define NUM_SERVOS           6
#define INTERP_INTERVAL_MS   20     // Pace for interpolation steps (ms)
#define SERIAL_BUF_SIZE      80     // RX buffer for ASCII packets
#define PROTOCOL_VERSION     "1.0.0"

// ─── Joint Hardware Structure ────────────────────────────────────────────────
struct Joint {
  const char* name;         // For debugging
  uint8_t     pin;          // PWM Pin
  uint8_t     minAngle;     // Safety Min
  uint8_t     maxAngle;     // Safety Max
  uint8_t     homeAngle;    // Boot Position
  uint8_t     currentAngle; // Physical State
  uint8_t     targetAngle;  // Goal State
  Servo       servo;
};

// ─── Joint Instances ─────────────────────────────────────────────────────────
// Index strictly maps to Protocol Order: Waist, Shoulder, Elbow, Roll, Pitch, Grip
Joint joints[NUM_SERVOS] = {
  { "Waist",        2,   0, 180,  90,  90,  90, Servo() },
  { "Shoulder",     3,  30, 150,  90,  90,  90, Servo() },
  { "Elbow",        4,  20, 160,  90,  90,  90, Servo() },
  { "Wrist Roll",   5,   0, 180,  90,  90,  90, Servo() },
  { "Wrist Pitch",  6,   0, 180,  90,  90,  90, Servo() },
  { "Gripper",      7,  20, 160,  30,  30,  30, Servo() },
};

// ─── Global State ────────────────────────────────────────────────────────────
char serialBuf[SERIAL_BUF_SIZE];
uint8_t bufIdx = 0;
unsigned long lastInterpMs = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  BOOT
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  // Initialize servos sequentially to limit peak current draw
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    joints[i].servo.attach(joints[i].pin);
    joints[i].servo.write(joints[i].homeAngle);
    joints[i].currentAngle = joints[i].homeAngle;
    joints[i].targetAngle  = joints[i].homeAngle;
    delay(80); 
  }

  Serial.println(F("SYSTEM_BOOT_COMPLETE"));
  Serial.println(F("ARM_READY_V1_0_0"));
}

// ═════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
  processSerial();
  interpolateMovement();
}

// ═════════════════════════════════════════════════════════════════════════════
//  SERIAL PARSER
// ═════════════════════════════════════════════════════════════════════════════

void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        serialBuf[bufIdx] = '\0';
        
        // Command Router
        if (bufIdx > 2 && serialBuf[0] == 'S' && serialBuf[1] == ':') {
          if (parseAngles(serialBuf + 2)) {
            Serial.print(F("ACK:"));
            Serial.println(serialBuf);
          } else {
            Serial.println(F("ERR:BAD_PAYLOAD"));
          }
        } 
        else if (strcmp(serialBuf, "PING") == 0) {
          Serial.print(F("PONG:"));
          Serial.println(F(PROTOCOL_VERSION));
        } 
        else if (strcmp(serialBuf, "HOME") == 0) {
          moveToHome();
          Serial.println(F("ACK:HOME_OK"));
        } 
        else {
          Serial.println(F("ERR:UNKNOWN_COMMAND"));
        }
        bufIdx = 0;
      }
    } 
    else if (bufIdx < (SERIAL_BUF_SIZE - 1)) {
      serialBuf[bufIdx++] = c;
    }
  }
}

bool parseAngles(char* angleStr) {
  char* saveptr;
  char* token = strtok_r(angleStr, ",", &saveptr);
  uint8_t tokenCount = 0;
  
  for (uint8_t i = 0; i < NUM_SERVOS && token != nullptr; i++) {
    int val = atoi(token);
    
    // Physical fix: Elbow servo is mounted upside down. Invert it at the bare-metal level.
    if (i == 2) {
      val = 180 - val;
    }
    
    // Physical Clamp: protect gears and motors from out-of-bounds targets
    joints[i].targetAngle = (uint8_t)constrain(val, joints[i].minAngle, joints[i].maxAngle);
    
    token = strtok_r(nullptr, ",", &saveptr);
    tokenCount++;
  }

  return (tokenCount == NUM_SERVOS && token == nullptr);
}

void moveToHome() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    joints[i].targetAngle = joints[i].homeAngle;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  MOTION ENGINE
// ═════════════════════════════════════════════════════════════════════════════

void interpolateMovement() {
  if (millis() - lastInterpMs < INTERP_INTERVAL_MS) return;
  lastInterpMs = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (joints[i].currentAngle < joints[i].targetAngle) {
      joints[i].currentAngle++;
      joints[i].servo.write(joints[i].currentAngle);
    } 
    else if (joints[i].currentAngle > joints[i].targetAngle) {
      joints[i].currentAngle--;
      joints[i].servo.write(joints[i].currentAngle);
    }
  }
}
