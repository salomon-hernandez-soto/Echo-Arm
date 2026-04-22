// Author: Salomon Hernandez-Soto

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"ii

// Sets up mpu0 (on forearm) and mpu1 (on hand)
MPU6050 mpu0(0x68);
MPU6050 mpu1(0x69);

// Quaternion is 4-d complex vector that stores orientation data
// Gravity vector points towards gravity
// In use, gets overwritten each time both MPU's are read
Quaternion q;
VectorFloat gravity;

// ypr[0]=yaw (left-right), ypr[1]=pitch (side tilt), ypr[2]=roll (up-down)
// Note that these are in radians but are converted to degrees when used
float yprForearm[3];
float yprHand[3];

// MPU and DMP states
bool dmpReady0 = false;
bool dmpReady1 = false;
uint8_t devStatus0; 
uint8_t devStatus1;
uint8_t fifoBuffer0[64];
uint8_t fifoBuffer1[64];

// Servo digital output pins
const int baseServoPin = 2, shoulderServoPin = 3, elbowServoPin = 4, gripperServoPin = 5;

// Potentiometer analog pin and button digital input pins
const int gripperPotPin = A3, pauseButtonPin = 6, calibrateButtonPin = 7;

// LED status pins
const int ledR = 9, ledG = 10, ledB = 11;

// Servo period and pulse length, in microseconds (note 1 ms = 1000 microseconds)
// Also, declaring wait required to match the period
const unsigned long servoPeriod = 20000;
const unsigned long quarterServoPeriod = servoPeriod / 4;

unsigned long pulseLengthBase = 0, pulseLengthShoulder = 0, pulseLengthElbow = 0, pulseLengthGripper = 0;
unsigned long remainingWaitBase = 0, remainingWaitShoulder = 0, remainingWaitElbow = 0, remainingWaitGripper = 0;

// Maximum and minimum allowable pulse lengths (microseconds)
// Base can move 270 degrees (+- 135 from neutral position)
const int minPulseBase = 500, maxPulseBase = 2500;
const float baseGain = -(maxPulseBase - minPulseBase) / 180.0;
const int baseOffset = (maxPulseBase + minPulseBase) / 2;

// Shoulder locked to 180 degrees (+180 from neutral position)
const int minPulseShoulder = 885, maxPulseShoulder = 2215;
const float shoulderGain = (maxPulseShoulder - minPulseShoulder) / 180.0;
const int shoulderOffset = minPulseShoulder;

// Elbow locked to 180 degrees (+- 90 from neutral position)
const int minPulseElbow = 820, maxPulseElbow = 2180;
const float elbowGain = -(maxPulseElbow - minPulseElbow) / 180.0;
const int elbowOffset = (maxPulseElbow + minPulseElbow) / 2;

// Gripper locked to 90 degrees (+90 from minimum pulse)
// Gain and offset differ since gripper is controlled by potentiometer
const int minPulseGripper = 500, maxPulseGripper = 1400;
const float gripperGain = (maxPulseGripper - minPulseGripper) / 1023.0;
const int gripperOffset = minPulseGripper;

// Stores whether the arm has been paused and calibrated
bool pauseAndCalibrateOnce = false;

// Yaw zero angle
float yawZero = 0;
 
// Last potentiometer read time and time between potentiometer reads, in milliseconds
const unsigned long potReadPeriod = 100;
unsigned long potLastReadTime = 0;

// Last calibration time and minimum time between calibrates, in milliseconds
const unsigned long calibrateWaitingPeriod = 5000;
unsigned long lastCalibrateTime = 0;

// Last angles print time and time between angle prints, in milliseconds
const unsigned long printPeriod = 250;
unsigned long lastPrintTime = 0;

// Gain on the roll angles measured by the IMU's since they are inaccurate by default
float forearmRollGain = 1;
float handRollGain = 1;

// Adjust if yaw feels too insensitive, but may amplify drift
float forearmYawGain = 1.1;

/*
-------------------------------
START OF SETUP
-------------------------------
*/

bool setupMPU(MPU6050 &mpu, uint8_t &devStatus, const char *name) {
  // Print which MPU is being initialized
  Serial.print(F("Initializing "));
  Serial.println(name);

  mpu.initialize();

  // Test connection to MPU
  if (!mpu.testConnection()) {
    Serial.print(name);
    Serial.println(F(" connection failed"));
    return false;

  }

  // Print which MPU is initializing its digital motion processor
  Serial.print(F("Initializing DMP for "));
  Serial.println(name);

  devStatus = mpu.dmpInitialize();

  // If DMP fails to initialize, return false
  if (devStatus != 0) {
    Serial.print(name);
    Serial.print(F(" DMP init failed: "));
    Serial.println(devStatus);
    return false;

  }

  // If DMP initializes, return true
  mpu.setDMPEnabled(true);

  Serial.print(name);
  Serial.println(F(" DMP ready"));
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Set servo pins to digital output
  pinMode(baseServoPin, OUTPUT), pinMode(shoulderServoPin, OUTPUT), pinMode(elbowServoPin, OUTPUT), pinMode(gripperServoPin, OUTPUT);

  // Set LED pins to digital output
  pinMode(ledR, OUTPUT), pinMode(ledG, OUTPUT), pinMode(ledB, OUTPUT);

  // Set button pins to digital input
  pinMode(pauseButtonPin, INPUT_PULLUP), pinMode(calibrateButtonPin, INPUT_PULLUP);

  // Set up both MPU6050's and see if their digital motion processors start up correctly
  dmpReady0 = setupMPU(mpu0, devStatus0, "mpu0");
  dmpReady1 = setupMPU(mpu1, devStatus1, "mpu1");

  if (!dmpReady0 || !dmpReady1) {
    // Stop program
    while(true) {
      Serial.println("Failed to initialize, restart program and check wiring");
      delay(5000);
    }
  }
}

/*
-------------------------------
START OF LOOP
-------------------------------
*/

void loop() {
  // If pause button is pressed it will be in the low state
  bool paused = !digitalRead(pauseButtonPin);

  // Operation order tree
  if (paused) {
    // Change LED's to pause state
    updateLEDs('p');

    while (paused) {
      // Enter paused state on loop until unpaused
      paused = !digitalRead(pauseButtonPin);
      runPauseState();

    }
  } else {
    // Only runs like normal if the mpu's have been calibrated at least once
    if (!pauseAndCalibrateOnce) {
      // Change LED's to incomplete setup state
      updateLEDs('s');
      return;

    } else {
      // Change LED's to running state
      updateLEDs('r');

      while (!paused) {
        // Enter normal running state on loop until paused
        paused = !digitalRead(pauseButtonPin);
        runNormalState();

      }
    }
  }
}

void runPauseState() {
  // If calibrate button is pressed it will be in the low state
  bool calibrate = !digitalRead(calibrateButtonPin);

  // Calibration only possible when paused
  if (calibrate) {

    // Only calibrate if enough time has passed since the last calibrate
    if ((millis() - lastCalibrateTime) > calibrateWaitingPeriod || lastCalibrateTime == 0) {
    lastCalibrateTime = millis();

    // Change LED's to calibration state
    updateLEDs('c');
    runCalibration();

    } else {
      // Do nothing

    }
  } else {
    // Can still run gripper when paused
    runGripper();

  }
}

void runCalibration() {
  // Update whether user has calibrated once
  if (!pauseAndCalibrateOnce) {
    pauseAndCalibrateOnce = true;
  }

  // Calibrate the MPU6050's
  mpu0.CalibrateAccel(6);
  mpu0.CalibrateGyro(6);
  mpu0.PrintActiveOffsets();

  mpu1.CalibrateAccel(6);
  mpu1.CalibrateGyro(6);
  mpu1.PrintActiveOffsets();

  // Zero out the yaw for the forearm as it controls the base
  readMPUData(mpu0, fifoBuffer0, 0);
  yawZero = yprForearm[0] * (180.0 / PI);

  // Reset roll gains to 1
  forearmRollGain = 1.0;
  handRollGain = 1.0; 

  Serial.println("Click calibrate button when forearm and hand is vertical.");

  bool calibrateRoll = false;

  // When calibrate button is hit again, gain is measured such that a vertical forearm/hand maps to 90 degrees of roll
  while (!calibrateRoll) {
    calibrateRoll = !digitalRead(calibrateButtonPin);

    if (calibrateRoll) {
      // Read both MPU's
      readMPUData(mpu0, fifoBuffer0, 0);
      readMPUData(mpu1, fifoBuffer1, 1);

      // Read both roll angles (degrees)
      float forearmRoll = yprForearm[2] * (180.0 / PI);
      float handRoll = yprHand[2] * (180.0 / PI);

      // Find gain such that in the future when the forearm/hand is vertical, the IMU measured angle times the gain gives 90 degrees
      forearmRollGain = 90.0 / forearmRoll;
      handRollGain = 90.0 / handRoll;

    }
  }

  // Have a buffer so that calibration can't be entered again on accident
  Serial.println("Calibration complete, ending in 1 second...");
  delay(1000);

  // Return LED to paused state
  updateLEDs('p');
}

void runNormalState() {
  // Read data from both MPU's
  readMPUData(mpu0, fifoBuffer0, 0);
  readMPUData(mpu1, fifoBuffer1, 1);

  // Adjust yaw by gain if needed
  yprForearm[0] = yprForearm[0] * forearmYawGain;

  // Adjust roll angles by corresponding gain
  yprForearm[2] = yprForearm[2] * forearmRollGain;
  yprHand[2] = yprHand[2] * handRollGain;

  // Map forearm IMU yaw + roll and hand IMU roll to servo pulse lengths
  processIMUtoServoPulse(&pulseLengthBase, &remainingWaitBase, minPulseBase, maxPulseBase, baseOffset, baseGain, yprForearm[0], 0, true);
  processIMUtoServoPulse(&pulseLengthShoulder, &remainingWaitShoulder, minPulseShoulder, maxPulseShoulder, shoulderOffset, shoulderGain, yprForearm[2], 0, false);
  processIMUtoServoPulse(&pulseLengthElbow, &remainingWaitElbow, minPulseElbow, maxPulseElbow, elbowOffset, elbowGain, yprHand[2], yprForearm[2], false);
  processPotToServoPulse('r');

  // Write pulses to all servos
  moveEchoArm();
}

void moveEchoArm() {
  // Write pulses to each servo
  writeServoPulse(baseServoPin, pulseLengthBase, remainingWaitBase);
  writeServoPulse(shoulderServoPin, pulseLengthShoulder, remainingWaitShoulder);
  writeServoPulse(elbowServoPin, pulseLengthElbow, remainingWaitElbow);
  writeServoPulse(gripperServoPin, pulseLengthGripper, remainingWaitGripper);

  // Print forearm IMU yaw + roll and hand IMU roll at set intervals
  if ((millis() - lastPrintTime) > printPeriod || lastPrintTime == 0) {
    lastPrintTime = millis();
    printAngles();
  }
}

void printAngles() {
  Serial.print("Forearm Yaw: ");
  Serial.print(yprForearm[0] * (180.0 / PI));
  Serial.print(" | ");

  Serial.print("Forearm Roll: ");
  Serial.print(yprForearm[2] * (180.0 / PI));
  Serial.print(" | ");

  Serial.print("Hand Roll: ");
  Serial.print(yprHand[2] * (180.0 / PI));
  Serial.print(" | \n");
  Serial.println();
}

void runGripper() {
  // Just read and write pulses to the gripper
  processPotToServoPulse('p');
  writeServoPulse(gripperServoPin, pulseLengthGripper, remainingWaitGripper);
}

void writeServoPulse(int servoPin, long pulseLength, long remainingWait) {
  // Write HIGH for the pulse length, and LOW for the remainder of the pulse period
  // Assuming neither pulseLength or remainingWait is over 16000
  digitalWrite(servoPin, 1);
  delayMicroseconds(pulseLength);
  digitalWrite(servoPin, 0);
  delayMicroseconds(remainingWait);
}

void processIMUtoServoPulse(long *pulseLength, long *remainingWait, int minPulse, int maxPulse, int offset, float gain, float IMUangle, float servoBodyAngle, bool yaw) {
  // Convert read angle and servo body angle to degrees
  IMUangle = IMUangle * (180.0 / PI);
  servoBodyAngle = servoBodyAngle * (180.0 / PI);

  // Convert angle to be relative to the absolute frame, not the servo's frame
  float servoAngle = IMUangle - servoBodyAngle;
  
  if (yaw) {
    servoAngle = servoAngle - yawZero;
  }

  // Map angle to pulse length, all servos have 1500 as center point pulse
  *pulseLength = long((gain * servoAngle) + offset);

  // Lock between max and min boundaries
  if (*pulseLength < minPulse) {
    *pulseLength = minPulse;

  } else if (*pulseLength > maxPulse) {
    *pulseLength = maxPulse;

  }

  *remainingWait = quarterServoPeriod - *pulseLength;
}

void processPotToServoPulse(char state) {
  if (((millis() - potLastReadTime) > potReadPeriod) || potLastReadTime == 0) {
    potLastReadTime = millis();

    // Read potentiometer count
    int potVal = analogRead(gripperPotPin);

    // Map potentiometer count to gripper pulse length, result is self-clamping due to potentiometer count limits
    pulseLengthGripper = (gripperGain * potVal) + gripperOffset;

    if (state == 'p') {
      // Gripper servo is the only one being driven, wait should take up whole period
      remainingWaitGripper = servoPeriod - pulseLengthGripper;

    } else if (state == 'r') {
      // Gripper servo is one of four servos being driven, wait should take up quarter period
      remainingWaitGripper = quarterServoPeriod - pulseLengthGripper;

    }
  }
}

void updateLEDs(char state) {
  if (state == 'p') {
    // Status light turns RED when paused
    analogWrite(ledR, 255), analogWrite(ledG, 0), analogWrite(ledB, 0);

  } else if (state == 'c') {
    // Status light turns PURPLE when calibrating
    analogWrite(ledR, 255), analogWrite(ledG, 0), analogWrite(ledB, 255);

  } else if (state == 'r') {
    // Status light turns GREEN when running
    analogWrite(ledR, 0), analogWrite(ledG, 255), analogWrite(ledB, 0);

  } else if (state == 's') {
    // Status light turns YELLOW if unpaused without calibrating at least once
    analogWrite(ledR, 255), analogWrite(ledG, 100), analogWrite(ledB, 0);
  }
}

void readMPUData(MPU6050 &mpu, uint8_t *fifoBuffer, int mpuInUse) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    if (mpuInUse == 0) {
      mpu.dmpGetYawPitchRoll(yprForearm, &q, &gravity);

    } else {
      mpu.dmpGetYawPitchRoll(yprHand, &q, &gravity);

    }
  }
}