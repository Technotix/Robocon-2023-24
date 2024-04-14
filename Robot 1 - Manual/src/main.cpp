#include <Arduino.h>

// Add Comment to Disable Alternate XBOX Receiver Mode
#define XBOX_ALT_MODE

#include <Cytron_SmartDriveDuo.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <math.h>
#include <ODriveArduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#ifdef XBOX_ALT_MODE
#include <XBOXUSB.h>
#else
#include <XBOXRECV.h>
#endif


// Define XBOX Controller Buttons and Speed Parameters
#define conid 0
#define deadzone 0
#define minSpeed 10
#define maxSpeed 50

// Define Motor Pins
#define mdds_1_2 50
#define mdds_3_4 51

// Define Servo Pins
#define srvPin1_3 12
#define srvPin2 13
#define srvPin4_6 10
#define srvPin5 11

// Define Pneumatic Pins
#define leftPneumatic 53
#define rightPneumatic 52

// Define Stepper Pins
#define pul 30
#define dir 28


// Define All Objects
HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

Servo servo1_3, servo2, servo4_6, servo5;

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif


int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0, requested_state = 0, motornum = 0;
bool pneumLeft = false, pneumRight = false, odrv0 = false, odrv1 = false;


bool xboxConnCheck() {
#ifdef XBOX_ALT_MODE
  if (Xbox.Xbox360Connected) {
    return true;
  }
  return false;
#else
  if (Xbox.XboxReceiverConnected) {
    if (Xbox.Xbox360Connected[conid]) {
      return true;
    }
  }
  return false;
#endif
}

void updateMotors(int XSpeed, int YSpeed, int TSpeed) {

  front_wheel = constrain(XSpeed + TSpeed, -100, 100);
  right_wheel = constrain(-YSpeed + TSpeed, -100, 100);
  back_wheel = constrain(-XSpeed + TSpeed, -100, 100);
  left_wheel = constrain(YSpeed + TSpeed, -100, 100);

  motor1motor2.control(front_wheel, right_wheel);
  motor3motor4.control(back_wheel, left_wheel);
}

void setServo(Servo servo, bool angle) {
  servo.write(angle? 0 : 90);
}

void stepperUpdate(int cycles, int direction, int speed) {
  digitalWrite(dir, direction);
  for (int i = 0; i < cycles * 6400; i++) { // 6400 steps for 1 rotation
    digitalWrite(pul, HIGH);
    delayMicroseconds(speed);
    digitalWrite(pul, LOW);
    delayMicroseconds(speed);
  }
}



void setup() {
  Wire.begin();
  odrive_serial.begin(115200);
  Serial.begin(115200);

  #if!defined(MIPSEL)
  while (!Serial)
  ;
  #endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
    ;
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

  // Attach Servos
  servo1_3.attach(srvPin1_3, 1000, 2500);
  servo2.attach(srvPin2, 1000, 2500);
  servo4_6.attach(srvPin4_6, 1000, 2500);
  servo5.attach(srvPin5, 1000, 2500);

  // Set Pneumatic Pins
  pinMode(leftPneumatic, OUTPUT);
  pinMode(rightPneumatic, OUTPUT);

  // Set Stepper Pins
  pinMode(pul, OUTPUT);
  pinMode(dir, OUTPUT);

  // Set Initial Values
  updateMotors(0, 0, 0);
  setServo(servo1_3, 0);
  setServo(servo2, 0);
  setServo(servo4_6, 0);
  setServo(servo5, 0);
  digitalWrite(leftPneumatic, LOW);
  digitalWrite(rightPneumatic, LOW);
}

void loop() {
  Usb.Task();
  if (xboxConnCheck()) {
    #ifdef XBOX_ALT_MODE
    int leftHatY = Xbox.getAnalogHat(LeftHatY);
    int leftHatX = Xbox.getAnalogHat(LeftHatX);
    int rightHatX = Xbox.getAnalogHat(RightHatX);
    #else
    int leftHatY = Xbox.getAnalogHat(LeftHatY, conid);
    int leftHatX = Xbox.getAnalogHat(LeftHatX, conid);
    int rightHatX = Xbox.getAnalogHat(RightHatX, conid);
    #endif

    int XSpeed = 0, YSpeed = 0, TSpeed = 0;

    // Motor Control
    if (leftHatX > deadzone) {
      XSpeed = map(leftHatX, deadzone, 32767, minSpeed, maxSpeed);
    } else if (leftHatX < -deadzone) {
      XSpeed = map(leftHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
    }
    if (leftHatY > deadzone) {
      YSpeed = map(leftHatY, deadzone, 32767, minSpeed, maxSpeed);
    } else if (leftHatY < -deadzone) {
      YSpeed = map(leftHatY, -32768, -deadzone, -maxSpeed, -minSpeed);
    }
    if (rightHatX > deadzone) {
      TSpeed = map(rightHatX, deadzone, 32767, minSpeed, maxSpeed);
    } else if (rightHatX < -deadzone) {
      TSpeed = map(rightHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
    }

    updateMotors(XSpeed, YSpeed, TSpeed * 0.8);

    // Servo Control
    if (Xbox.getButtonClick(LB)) {
      setServo(servo1_3, 1);
    }
    if (Xbox.getButtonClick(LT)) {
      setServo(servo2, 1);
    }
    if (Xbox.getButtonClick(RB)) {
      setServo(servo4_6, 1);
    }
    if (Xbox.getButtonClick(RT)) {
      setServo(servo5, 1);
    }
    if (Xbox.getButtonClick(B)) {
      setServo(servo1_3, 0);
      setServo(servo2, 0);
    }
    if (Xbox.getButtonClick(X)) {
      setServo(servo4_6, 0);
      setServo(servo5, 0);
    }

    // Pneumatic Control
    if (Xbox.getButtonClick(LEFT)) {
      digitalWrite(leftPneumatic, pneumLeft ? LOW : HIGH);
      pneumLeft = !pneumLeft;
    }
    if (Xbox.getButtonClick(RIGHT)) {
      digitalWrite(rightPneumatic, pneumRight ? LOW : HIGH);
      pneumRight = !pneumRight;
    }

    // Calibrate ODrives
    if (Xbox.getButtonClick(A)) {
      for (int i = 0; i <= 1; i++) {
        requested_state = AXIS_STATE_MOTOR_CALIBRATION;
        if (!odrive.run_state(i, requested_state, true)) return;

        requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        if (!odrive.run_state(i, requested_state, true, 25.0f)) return;

        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        if (!odrive.run_state(i, requested_state, false)) return;
      }
    }

    // Set ODrive Velocities
    if (Xbox.getButtonClick(Y)) {
      odrive.SetVelocity(0, odrv0 ? 0 : 20);
      odrv0 = !odrv0;

      odrive.SetVelocity(1, odrv1 ? 0 : 20);
      odrv1 = !odrv1;
    }

    // Motor Debugging
    Serial.print("Motor 1: ");
    Serial.print(front_wheel);
    Serial.print("  Motor 2: ");
    Serial.print(right_wheel);
    Serial.print("  Motor 3: ");
    Serial.print(back_wheel);
    Serial.print("  Motor 4: ");
    Serial.print(left_wheel);

    // Servo Debugging
    Serial.print("  Servo 1-3: ");
    Serial.print(servo1_3.read());
    Serial.print("  Servo 2: ");
    Serial.print(servo2.read());
    Serial.print("  Servo 4-6: ");
    Serial.print(servo4_6.read());
    Serial.print("  Servo 5: ");
    Serial.print(servo5.read());

    // Pneumatic Debugging
    Serial.print("  Pneumatic Left: ");
    Serial.print(pneumLeft);
    Serial.print("  Pneumatic Right: ");
    Serial.print(pneumRight);

    // ODrive Debugging
    Serial.print("  ODrive 0: ");
    Serial.print(odrv0);
    Serial.print("  ODrive 1: ");
    Serial.println(odrv1);
  } else {
    updateMotors(0, 0, 0);
  }
}
  