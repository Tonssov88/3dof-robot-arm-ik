#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define SERVO_FREQ 50  // Hz

// Calibrated pulse values for each servo - Based on my arm design!
#define BASE_PULSE_MIN     105
#define BASE_PULSE_MAX     505

#define SHOULDER_PULSE_MIN 110
#define SHOULDER_PULSE_MAX 505

#define ELBOW_PULSE_MIN    105
#define ELBOW_PULSE_MAX    505 

// ANGLE CONSTRAINTS to prevent self-collision
#define ELBOW_MIN_ANGLE    0.0    // Minimum safe elbow angle
#define ELBOW_MAX_ANGLE    150.0  // Maximum safe elbow angle (adjust this!)

// Movement settings
#define STEP_DELAY_MS 20          // Delay between each step (ms)

//arm geometry
const float L1 = 105.0;  // shoulder to elbow
const float L2 = 105.0;  // elbow to wrist

//starting coords
float coords[3] = {0.0f, 100.0f, 100.0f};
float currentCoords[3] = {0.0f, 100.0f, 100.0f};  // Current position
float targetCoords[3] = {0.0f, 100.0f, 100.0f};   // Target position

// Manual control mode
bool manualMode = false;
float manualAngles[3] = {90.0f, 90.0f, 90.0f};  // Base, Shoulder, Elbow

// Track changes to avoid constant serial output
float lastBaseDeg = -999;
float lastShoulderDeg = -999;
float lastElbowDeg = -999;

// Movement interpolation state
bool isMoving = false;
int currentStep = 0;

int interpolationSteps = 50;

struct armAngles {
  float baseAngle;
  float shoulderAngle;
  float elbowAngle;
};

float radToDeg(float rad) {
  return rad * 180.0 / PI;
}

uint16_t angleToPulse(float angleDeg, int servoChannel) {
  // Apply angle constraints BEFORE pulse conversion
  if (servoChannel == ELBOW_SERVO) {
    angleDeg = constrain(angleDeg, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  } else {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  }
  
  uint16_t pulseMin, pulseMax;
  
  // Use servo-specific calibration values
  switch(servoChannel) {
    case BASE_SERVO:
      pulseMin = BASE_PULSE_MIN;
      pulseMax = BASE_PULSE_MAX;
      break;
    case SHOULDER_SERVO:
      pulseMin = SHOULDER_PULSE_MIN;
      pulseMax = SHOULDER_PULSE_MAX;
      break;
    case ELBOW_SERVO:
      pulseMin = ELBOW_PULSE_MIN;
      pulseMax = ELBOW_PULSE_MAX;
      break;
    default:
      pulseMin = 105;
      pulseMax = 505;
  }
  
  return (uint16_t)((angleDeg / 180.0f) * (pulseMax - pulseMin) + pulseMin);
}

armAngles computeArmAngles(float coords[3]) {
  float x = coords[0];
  float y = coords[1];
  float z = coords[2];
  struct armAngles finalAngles;

  //Base angle Calc
  finalAngles.baseAngle = atan2(x, z);
  
  float posXZ = sqrt(x * x + z * z);
  float distToTarget = sqrt(posXZ * posXZ + y * y);
  
  // Check if position can be reached 
  if (distToTarget > (L1 + L2) || distToTarget < abs(L1 - L2)) {
    Serial.println("WARNING: Target unreachable!");
    Serial.print("Distance: "); Serial.print(distToTarget);
    Serial.print(" mm, Max reach: "); Serial.println(L1 + L2);
  }
  
  //Elbow Angle (interior angle of the joint)
  float cosElbow = (L1 * L1 + L2 * L2 - distToTarget * distToTarget) / (2 * L1 * L2);
  cosElbow = constrain(cosElbow, -1.0f, 1.0f);
  
  float elbowInteriorAngle = acos(cosElbow);
  finalAngles.elbowAngle = PI - elbowInteriorAngle;

  //Shoulder Angle
  float angleToTarget = atan2(y, posXZ);
  
  float cosShoulderTriangle = (L1 * L1 + distToTarget * distToTarget - L2 * L2) / (2 * L1 * distToTarget);
  cosShoulderTriangle = constrain(cosShoulderTriangle, -1.0f, 1.0f);
  float shoulderTriangleAngle = acos(cosShoulderTriangle);

  finalAngles.shoulderAngle = angleToTarget + shoulderTriangleAngle;

  return finalAngles;
}

void lerp3D(float start[3], float end[3], float t, float result[3]) {
  // Linear interpolation between two 3D points
  result[0] = start[0] + (end[0] - start[0]) * t;
  result[1] = start[1] + (end[1] - start[1]) * t;
  result[2] = start[2] + (end[2] - start[2]) * t;
}

void startMovement(float target[3]) {
  // Set new target and start interpolation
  targetCoords[0] = target[0];
  targetCoords[1] = target[1];
  targetCoords[2] = target[2];
  
  isMoving = true;
  currentStep = 0;
  
  Serial.print("Starting movement to (");
  Serial.print(target[0]); Serial.print(", ");
  Serial.print(target[1]); Serial.print(", ");
  Serial.print(target[2]); Serial.println(")");
}

bool updateMovement() {
  // Returns true if still moving, false if complete
  if (!isMoving) return false;
  
  currentStep++;
  
  if (currentStep >= interpolationSteps) {
    // Movement complete
    currentCoords[0] = targetCoords[0];
    currentCoords[1] = targetCoords[1];
    currentCoords[2] = targetCoords[2];
    isMoving = false;
    Serial.println("Movement complete");
    return false;
  }
  
  // Calculate interpolation factor (0.0 to 1.0)
  float t = (float)currentStep / (float)interpolationSteps;
  
  // Interpolate between current and target positions
  lerp3D(currentCoords, targetCoords, t, coords);
  
  return true;
}

void printAngles(float baseDeg, float shoulderDeg, float elbowDeg) {
  Serial.print(manualMode ? "[MANUAL] " : "[IK] ");
  Serial.print("Angles: B="); Serial.print(baseDeg, 1);
  Serial.print("° S="); Serial.print(shoulderDeg, 1);
  Serial.print("° E="); Serial.println(elbowDeg, 1);
}

bool readCoordsFromSerial(float coords[3]){
  static String line;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        int idx1 = line.indexOf(' ');
        int idx2 = line.indexOf(' ', idx1 + 1);

        if (idx1 > 0 && idx2 > idx1) {
          coords[0] = line.substring(0, idx1).toFloat();
          coords[1] = line.substring(idx1 + 1, idx2).toFloat();
          coords[2] = line.substring(idx2 + 1).toFloat();

          line = "";
          return true;
        } else {
          Serial.println("Invalid format. Use: x y z");
          line = "";
        }
      }
    }
    else {
      line += c;
    }
  }

  return false;
}

void parseCommand(String input) {
  input.trim();
  input.toLowerCase();
  
  if (input.startsWith("b ")) {
    float angle = input.substring(2).toFloat();
    manualAngles[0] = constrain(angle, 0.0f, 180.0f);
    manualMode = true;
    Serial.print("BASE set to ");
    Serial.print(manualAngles[0], 1);
    Serial.println("° (Manual mode)");
  }
  else if (input.startsWith("s ")) {
    float angle = input.substring(2).toFloat();
    manualAngles[1] = constrain(angle, 0.0f, 180.0f);
    manualMode = true;
    Serial.print("SHOULDER set to ");
    Serial.print(manualAngles[1], 1);
    Serial.println("° (Manual mode)");
  }
  else if (input.startsWith("e ")) {
    float angle = input.substring(2).toFloat();
    manualAngles[2] = constrain(angle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
    manualMode = true;
    Serial.print("ELBOW set to ");
    Serial.print(manualAngles[2], 1);
    Serial.print("° (constrained to ");
    Serial.print(ELBOW_MIN_ANGLE, 0);
    Serial.print("-");
    Serial.print(ELBOW_MAX_ANGLE, 0);
    Serial.println("°)");
  }
  else if (input.startsWith("a ")) {
    // Parse all three angles
    int idx1 = input.indexOf(' ');
    int idx2 = input.indexOf(' ', idx1 + 1);
    int idx3 = input.indexOf(' ', idx2 + 1);
    
    if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
      manualAngles[0] = constrain(input.substring(idx1 + 1, idx2).toFloat(), 0.0f, 180.0f);
      manualAngles[1] = constrain(input.substring(idx2 + 1, idx3).toFloat(), 0.0f, 180.0f);
      manualAngles[2] = constrain(input.substring(idx3 + 1).toFloat(), ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
      
      manualMode = true;
      Serial.print("All servos set: BASE=");
      Serial.print(manualAngles[0], 1);
      Serial.print("° SHOULDER=");
      Serial.print(manualAngles[1], 1);
      Serial.print("° ELBOW=");
      Serial.print(manualAngles[2], 1);
      Serial.println("° (Manual mode)");
    } else {
      Serial.println("Error: Format is 'a <base> <shoulder> <elbow>'");
    }
  }
  else if (input == "ik") {
    manualMode = false;
    Serial.println("Switched to IK mode - enter coordinates (x y z)");
  }
  else if (input == "manual") {
    manualMode = true;
    Serial.println("Switched to manual mode - use b/s/e commands");
  }
  else if (input == "home") {
    manualAngles[0] = 90.0f;
    manualAngles[1] = 90.0f;
    manualAngles[2] = 90.0f;
    manualMode = true;
    Serial.println("All servos set to 90° (home position)");
  }
  else if (input == "help") {
    printHelp();
  }
  else if (input.startsWith("steps ")) {
    int steps = input.substring(6).toInt();
    if (steps >= 1 && steps <= 100) {
        interpolationSteps = steps;
        Serial.print("Interpolation steps set to: ");
        Serial.println(steps);
    } else {
        Serial.println("Error: Steps must be 1-100");
    }
}
  else {
    // Try parsing as coordinates if in IK mode
    if (!manualMode) {
      return; // Let readCoordsFromSerial handle it
    } else {
      Serial.println("Unknown command. Type 'help' for commands.");
    }
  }
}

void printHelp() {
  Serial.println("\n=== COMMANDS ===");
  Serial.println("IK Mode (coordinates):");
  Serial.println("  x y z          - Move to coordinate (e.g., '0 0 210')");
  Serial.println("  ik             - Switch to IK mode");
  Serial.println();
  Serial.println("Manual Mode (angles):");
  Serial.println("  b <angle>      - Set BASE angle (e.g., 'b 90')");
  Serial.println("  s <angle>      - Set SHOULDER angle (e.g., 's 90')");
  Serial.println("  e <angle>      - Set ELBOW angle (e.g., 'e 90')");
  Serial.println("  a <b> <s> <e>  - Set ALL servos (e.g., 'a 90 90 90')");
  Serial.println("  manual         - Switch to manual mode");
  Serial.println();
  Serial.println("General:");
  Serial.println("  home           - Move all servos to 90°");
  //Serial.println("  steps (int)    - Change interpolation steps when using IK");
  Serial.println("  help           - Show this help");
  Serial.print("\nElbow constrained to: ");
  Serial.print(ELBOW_MIN_ANGLE, 0);
  Serial.print("-");
  Serial.print(ELBOW_MAX_ANGLE, 0);
  Serial.println("°");
  Serial.print("Interpolation: ");
  Serial.print(interpolationSteps);
  Serial.print(" steps, ");
  Serial.print(STEP_DELAY_MS);
  Serial.println("ms per step");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Robot Arm IK Controller ===");
  Serial.println("Type 'help' for commands");
  Serial.print("Elbow angle constrained to: ");
  Serial.print(ELBOW_MIN_ANGLE, 0);
  Serial.print("-");
  Serial.print(ELBOW_MAX_ANGLE, 0);
  Serial.println("°");
  Serial.println("Current mode: IK");
  Serial.println();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize current position
  currentCoords[0] = coords[0];
  currentCoords[1] = coords[1];
  currentCoords[2] = coords[2];
}

void loop() {
  static String inputBuffer = "";
  static unsigned long lastStepTime = 0;
  
  // Read serial input
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        // Check if it's a command or coordinate
        String input = inputBuffer;
        input.trim();
        
        // Check if it starts with a command letter or is numeric (coordinates)
        if (input.startsWith("b ") || input.startsWith("s ") || input.startsWith("e ") || 
            input.startsWith("a ") ||input.startsWith("steps ")|| input == "ik" || input == "manual" || 
            input == "home" || input == "help") {
          parseCommand(input);
        } else {
          // Try to parse as coordinates
          int idx1 = input.indexOf(' ');
          int idx2 = input.indexOf(' ', idx1 + 1);
          
          if (idx1 > 0 && idx2 > idx1) {
            float newTarget[3];
            newTarget[0] = input.substring(0, idx1).toFloat();
            newTarget[1] = input.substring(idx1 + 1, idx2).toFloat();
            newTarget[2] = input.substring(idx2 + 1).toFloat();
            
            manualMode = false;  // Switch to IK mode
            startMovement(newTarget);
          } else {
            Serial.println("Invalid format. Type 'help' for commands.");
          }
        }
        
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // Update interpolated movement in IK mode
  if (!manualMode && isMoving) {
    unsigned long currentTime = millis();
    if (currentTime - lastStepTime >= STEP_DELAY_MS) {
      updateMovement();
      lastStepTime = currentTime;
    }
  }

  float baseDeg, shoulderDeg, elbowDeg;

  if (manualMode) {
    // Use manual angles directly
    baseDeg = manualAngles[0];
    shoulderDeg = manualAngles[1];
    elbowDeg = manualAngles[2];
  } else {
    // Compute angles from IK (uses interpolated coords)
    armAngles angles = computeArmAngles(coords);
    baseDeg = radToDeg(angles.baseAngle);
    shoulderDeg = radToDeg(angles.shoulderAngle);
    elbowDeg = radToDeg(angles.elbowAngle);
  }

  // Only update servos if angles changed
  if (abs(baseDeg - lastBaseDeg) > 0.1 || 
      abs(shoulderDeg - lastShoulderDeg) > 0.1 || 
      abs(elbowDeg - lastElbowDeg) > 0.1) {
    
    // Convert to PCA9685 pulse widths using calibrated values
    uint16_t basePulse = angleToPulse(baseDeg, BASE_SERVO);
    uint16_t shoulderPulse = angleToPulse(shoulderDeg, SHOULDER_SERVO);
    uint16_t elbowPulse = angleToPulse(elbowDeg, ELBOW_SERVO);

    // Send commands to servos
    pwm.setPWM(BASE_SERVO, 0, basePulse);
    pwm.setPWM(SHOULDER_SERVO, 0, shoulderPulse);
    pwm.setPWM(ELBOW_SERVO, 0, elbowPulse);
    
    // Update tracking variables
    lastBaseDeg = baseDeg;
    lastShoulderDeg = shoulderDeg;
    lastElbowDeg = elbowDeg;
  }

  delay(10);
}
