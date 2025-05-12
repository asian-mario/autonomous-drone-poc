#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <PID_v1_bc.h>

// MPU6050 object for DMP
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// DMP orientation output
Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw/pitch/roll container

// ESC objects for motors
Servo esc1, esc2, esc3, esc4;

// Pin definitions
const int esc1Pin = 3;  // Front Left
const int esc2Pin = 5;  // Back Left
const int esc3Pin = 9;  // Front Right
const int esc4Pin = 6;  // Back Right

// Throttle parameters
const int minThrottle = 1100;
const int maxThrottle = 2100;
int currentThrottle = 1500;  // target throttle

// PID tuning parameters (increased for better responsiveness)
double rollSetpoint = 0.0, rollInput, rollOutput;
double pitchSetpoint = 0.0, pitchInput, pitchOutput;

// Create PID controllers with increased gains:
// (For roll: Kp = 1.2, Ki = 0.07, Kd = 0.5)
// (For pitch: Kp = 1.0, Ki = 0.06, Kd = 0.45)
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 0.5, 0.02, 0.35, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 0.5, 0.02, 0.35, DIRECT);



// Motor throttle adjustments
int throttleFL, throttleFR, throttleBL, throttleBR;

// Hover and Kill Switch flags
bool hoverMode = false;
bool killSwitch = false;
bool killMessageSent = false;

const int killThreshold = 60;
const int killDelay = 600;
unsigned long killStartTime = 0;


// Throttle ramp parameters
unsigned long hoverStartTime = 0;  // Records when hover mode started (for ramping)
const unsigned long rampDuration = 1800; // Ramp duration in milliseconds

// Roll trim variable: if the drone consistently leans right (i.e. right motors are weak),
// this value is subtracted from the left motor commands and added to the right motor commands.
// You can update this value via ESP32 input.
int rollTrim = 0;  // Initial value; can be modified via serial commands
const float deadband = 0.5;  // degrees

// Calibration routine: averages multiple accelerometer readings for a stable baseline.
// DEPRECATRED MANUAL CALIBRATION
/*void calibrateSensors() {
  const int samples = 100;
  long axSum = 0, aySum = 0, azSum = 0;
  int16_t ax, ay, az;

  // 1) Gather raw accel data
  for (int i = 0; i < samples; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    axSum += ax;
    aySum += ay;
    azSum += az;
    delay(5);
  }

  // 2) Compute average roll/pitch angles (in degrees)
  float avgAx = axSum / float(samples);
  float avgAy = aySum / float(samples);
  float avgAz = azSum / float(samples);
  avgAccelRoll  = atan2f(avgAy, avgAz) * RAD_TO_DEG;
  avgAccelPitch = atan2f(-avgAx, sqrtf(avgAy*avgAy + avgAz*avgAz)) * RAD_TO_DEG;

  // ─── NEW LINES TO SEED THE FILTER ───
  rollInput  = avgAccelRoll;
  pitchInput = avgAccelPitch;
  lastTime   = millis();
  // ─────────────────────────────────────

  Serial.print("Calibrated: roll="); Serial.print(avgAccelRoll);
  Serial.print("  pitch="); Serial.println(avgAccelPitch);
}*/



void setup() {
  Serial.begin(57600);
  Serial1.begin(57600);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("⚠️ MPU6050 connection failed"));
    while (1);
  }
  // load and configure the DMP firmware
  devStatus = mpu.dmpInitialize();
  // these offsets should be tuned to your board (0 is just a placeholder)
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("✅ DMP ready"));
  } else {
    Serial.print(F("⚠️ DMP init failed ("));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }


  esc1.attach(esc1Pin, 1000, 2000);
  esc2.attach(esc2Pin, 1000, 2000);
  esc3.attach(esc3Pin, 1000, 2000);
  esc4.attach(esc4Pin, 1000, 2000);
  
  setAllMotors(minThrottle);

  rollPID.SetOutputLimits(-200, 200);
  pitchPID.SetOutputLimits(-200, 200);
  delay(2000);

  rollPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);

  // Initial setpoints will be overwritten by calibrateSensors() upon HOVER_ON.
  pitchSetpoint = pitchInput;
  rollSetpoint = rollInput;

  Serial.println("READY");
  Serial1.println("READY");
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    Serial.print("Received from ESP32: ");
    Serial.println(command);
    parseCommand(command);
  }

  if (killSwitch) {
    setAllMotors(minThrottle);
    hoverMode = false;
    if (!killMessageSent) {
      Serial.println("⚠️ Motors stopped due to Kill Switch.");
      Serial1.println("Kill switch activated!");
      killMessageSent = true;
    }
    return;
  } else {
    killMessageSent = false;
  }

  if (hoverMode) {
    
    //DEPRECATED OLD PID ROUTINE
    /*mpu.update();
    
    static unsigned long lastTime = millis();
    unsigned long now = millis();
    double dt = (now - lastTime) / 1000.0;  // dt in seconds
    lastTime = now;

    // Compute accelerometer angles.
    float accelRoll  = atan2(-mpu.getAccX(), sqrt(sq(mpu.getAccY()) + sq(mpu.getAccZ()))) * 180 / PI;  
    float accelPitch = atan2(mpu.getAccY(), mpu.getAccZ()) * 180 / PI;  

    // ✅ Corrected Complementary Filter (Fixes GyroX inversion for pitch)
    pitchInput = alphaPitch * (pitchInput - mpu.getGyroX() * dt) + (1 - alphaPitch) * accelPitch;
    rollInput  = alphaRoll * (rollInput + mpu.getGyroY() * dt) + (1 - alphaRoll) * accelRoll;*/

    if (!dmpReady) return;             // wait until DMP is up
      fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) return;
    // read FIFO in packet-sized chunks
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    // extract quaternion & gravity vector
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // convert to degrees
    rollInput  = ypr[2] * RAD_TO_DEG;
    pitchInput = ypr[1] * RAD_TO_DEG;
    // (yaw is ypr[0] if you ever need it)


    // Apply a deadband: if the error is very small, force the sensor reading to equal the setpoint.
    const float deadband = 0.5;  // in degrees; adjust as needed
    if (fabs(rollInput - rollSetpoint) < deadband) {
      rollInput = rollSetpoint;
    }
    if (fabs(pitchInput - pitchSetpoint) < deadband) {
      pitchInput = pitchSetpoint;
    }

    // Check for dangerous angles.
    if (abs(rollInput) > killThreshold || abs(pitchInput) > killThreshold) {
      if (killStartTime == 0) {
        killStartTime = millis();
      }
      if (millis() - killStartTime > killDelay) {
        Serial.println("⚠️ Dangerous angle detected! Activating Kill Switch.");
        Serial1.println("ERROR Roll: " + String(rollInput));
        Serial1.println("ERROR Pitch: " + String(pitchInput));
        killSwitch = true;
        setAllMotors(minThrottle);
        return;
      }
    } else {
      killStartTime = 0;
    }
    

    rollPID.SetMode(AUTOMATIC);
    pitchPID.SetMode(AUTOMATIC);

    rollPID.Compute();
    pitchPID.Compute();

    // Limit extreme pitch corrections.
    if (abs(pitchOutput) > 40) {
      pitchOutput = (pitchOutput > 0) ? 40 : -40;
    }

    // Compute dynamic bias based on pitch angle (for front-heavy compensation).
    int dynamicBias = map(constrain((int)abs(pitchInput), 0, 30), 0, 30, 10, 25);

    // Throttle ramp: gradually increase throttle from minThrottle to currentThrottle.
    unsigned long elapsedTime = millis() - hoverStartTime;
    int rampThrottle = currentThrottle;
    if (elapsedTime < rampDuration) {
      rampThrottle = minThrottle + (int)((currentThrottle - minThrottle) * (elapsedTime / (float)rampDuration));
    }

    if (abs(rollInput) > 15) {  // Large deviation → boost correction
        rollTrim += constrain(rollInput * 0.2, -10, 10);   
    } else if (abs(rollInput) > 5) {  // Small correction → gradual adjustment
        rollTrim += constrain(rollInput * 0.1, -7, 7);
    } else {  
        rollTrim *= 0.98;  // Decay rollTrim slightly over time when near balanced
    }

    rollTrim = constrain(rollTrim, -160, 160);  // Prevent excessive correction
        

    // Apply an exponential decay to roll and pitch corrections when close to balanced
    if (abs(rollInput - rollSetpoint) < 1.0) { rollOutput *= 0.85; }  
    if (abs(pitchInput - pitchSetpoint) < 1.0) { pitchOutput *= 0.85; }

    // Motor mixing using the rampThrottle, PID outputs, and dynamic bias.
    // The rollTrim value (which is now modifiable) is applied to help correct bias.
    throttleFL = rampThrottle + constrain(-rollOutput + pitchOutput * 0.5 - rollTrim, -150, 150); 
    throttleBL = rampThrottle + constrain(-rollOutput - pitchOutput * 0.5 - rollTrim, -150, 150);
    throttleFR = rampThrottle + constrain( rollOutput + pitchOutput * 0.5 + rollTrim, -150, 150);
    throttleBR = rampThrottle + constrain( rollOutput - pitchOutput * 0.5 + rollTrim, -150, 150);

    /*Serial.println("ROLL: ");
    Serial.println(rollInput);
    Serial.println("PITCH: ");
    Serial.println(pitchInput);

    Serial.println("FL: ");
    Serial.println(throttleFL);
    Serial.println("BL: ");
    Serial.println(throttleBL);

    Serial.println("FR: ");
    Serial.println(throttleFR);
    Serial.println("BR: ");
    Serial.println(throttleBR);*/
    esc1.writeMicroseconds(throttleFL);
    esc2.writeMicroseconds(throttleBL);
    esc3.writeMicroseconds(throttleFR);
    esc4.writeMicroseconds(throttleBR);
  } else {
    setAllMotors(minThrottle);

  }

  delay(10);
}

void setAllMotors(int throttle) {
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
}

void parseCommand(String command) {
  command.trim();

  Serial.print("Processing Command: '");
  Serial.print(command);
  Serial.println("'");

  // Command for adjusting roll trim, e.g., "ROLLTRIM,30"
  if (command.startsWith("ROLLTRIM")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      int newTrim = command.substring(commaIndex + 1).toInt();
      rollTrim = newTrim;
      Serial.print("Updated roll trim: ");
      Serial.println(rollTrim);
      Serial1.print("Updated roll trim: ");
      Serial1.println(rollTrim);
    } else {
      Serial.println("⚠️ Error: ROLLTRIM command malformed!");
      Serial1.println("Error: ROLLTRIM command malformed!");
    }
  }
  else if (command == "HOVER_ON") {
    //i know this is blocking but i don't really want to chop my fingers off you know.
    delay(500)
    hoverMode = true;
    // hoverStartTime will be reset in calibrateSensors().
    Serial.println("✅ Hover mode activated.");
    Serial1.println("Hover mode activated.");
  } else if (command == "HOVER_OFF") {
    hoverMode = false;
    Serial.println("⛔ Hover mode deactivated.");
    Serial1.println("Hover mode deactivated.");
  } else if (command == "KILL") {
    killSwitch = true;
    Serial.println("⚠️ Kill switch activated! Stopping motors.");
    Serial1.println("Kill switch activated!");
  } else if (command.startsWith("PID")) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    int thirdComma = command.indexOf(',', secondComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
      Serial.println("⚠️ Error: Malformed PID command!");
      Serial1.println("Error: Malformed PID command!");
      return;
    }

    double newKp = command.substring(firstComma + 1, secondComma).toFloat();
    double newKi = command.substring(secondComma + 1, thirdComma).toFloat();
    double newKd = command.substring(thirdComma + 1).toFloat();

    if (newKp > 0 && newKi >= 0 && newKd > 0) {
      rollPID.SetTunings(newKp, newKi, newKd);
      pitchPID.SetTunings(newKp, newKi, newKd);
      Serial.println("✅ Updated PID values.");
      Serial1.println("PID updated.");
    } else {
      Serial.println("⚠️ Error: Invalid PID values!");
      Serial1.println("Error: Invalid PID values!");
    }
  } else {
    Serial.println("⚠️ Unknown command received.");
    Serial1.println("Unknown command received.");
  }
}
