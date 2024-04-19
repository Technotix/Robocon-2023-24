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
#define maxSpeed 75

// Define Motor Pins
#define mdds_1_2 38
#define mdds_3_4 39

// Define Servo Pins
#define srvPin1_3 7
#define srvPin2 6
#define srvPin4 4
#define srvPin5 3
#define srvPin6 5

#define leftPneumatic 48
#define rightPneumatic 49


// Define Ball Pickup Pins
#define pwm 8
#define dir 9


// Define All Objects
HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

Servo servo1_3, servo2, servo4, servo5, servo6;

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif


int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0, requested_state = 0, motornum = 0, pickupState = 0, srvState1_3 = 0, srvState2 = 0, srvState4_5 = 0, srvState6 = 0;
bool pneumLeft = false, pneumRight = false, odrv = false;


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

  front_wheel = constrain(YSpeed + TSpeed, -100, 100);
  right_wheel = constrain(-XSpeed + TSpeed, -100, 100);
  back_wheel = constrain(-YSpeed + TSpeed, -100, 100);
  left_wheel = constrain(XSpeed + TSpeed, -100, 100);

  motor1motor2.control(front_wheel, right_wheel);
  motor3motor4.control(back_wheel, left_wheel);
}

void setServo(Servo servo, bool angle) {
  servo.write(angle? 180 : 45);
}

void setPickup(int direction, int speed) {
  if (direction == 1) {
    digitalWrite(dir, LOW);
  } else {
    digitalWrite(dir, HIGH);
  }
  analogWrite(pwm, speed);
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
  servo4.attach(srvPin4, 1000, 2500);
  servo5.attach(srvPin5, 1000, 2500);
  servo6.attach(srvPin6, 1000, 2500);

  // Set Pneumatic Pins
  pinMode(leftPneumatic, OUTPUT);
  pinMode(rightPneumatic, OUTPUT);

  // Set Stepper Pins
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  // Set Initial Values
  updateMotors(0, 0, 0);
  setServo(servo1_3, 0);
  setServo(servo2, 1);
  setServo(servo4, 1);
  setServo(servo5, 0);
  setServo(servo6, 0);
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
    if (Xbox.getButtonClick(LT)) {
      if (srvState1_3 == 0) {
        setServo(servo1_3, 1);
        srvState1_3 = 1;
      } else {
        setServo(servo1_3, 0);
        srvState1_3 = 0;
      }
    }
    if (Xbox.getButtonClick(LB)) {
      if (srvState2 == 0) {
        setServo(servo2, 0);
        srvState2 = 1;
      } else {
        setServo(servo2, 1);
        srvState2 = 0;
      }
    }
    if (Xbox.getButtonClick(RT)) {
      if (srvState4_5 == 0) {
        setServo(servo4, 0);
        setServo(servo5, 1);
        srvState4_5 = 1;
      } else {
        setServo(servo4, 1);
        setServo(servo5, 0);
        srvState4_5 = 0;
      }
    }
    if (Xbox.getButtonClick(RB)) {
      if (srvState6 == 0) {
        setServo(servo6, 1);
        srvState6 = 1;
      } else {
        setServo(servo6, 0);
        srvState6 = 0;
      }
    }

    // Pneumatic Control
    if (Xbox.getButtonClick(BACK)) {
      digitalWrite(leftPneumatic, pneumLeft ? LOW : HIGH);
      pneumLeft = !pneumLeft;
    }
    if (Xbox.getButtonClick(START)) {
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
      odrive.SetVelocity(0, odrv ? 0 : 35);
      odrive.SetVelocity(1, odrv ? 0 : 30);
      odrv = !odrv;
    }

    // Ball Pickup Control
    if (Xbox.getButtonClick(UP)) {
      if (pickupState == 0) {
        if (odrv != 1) {
          odrive.SetVelocity(0, 35);
          odrive.SetVelocity(1, 30);
          odrv = 1;
        }
        setPickup(1, 250);
        pickupState = 1;
      } else {
        if (odrv != 0) {
          odrive.SetVelocity(0, 0);
          odrive.SetVelocity(1, 0);
          odrv = 0;
        }
        setPickup(0, 255);
        delay(500); // TODO - Asynchronous
        setPickup(0, 0);
        pickupState = 0;
      }
    }



    // Motor Debugging
    Serial.print("Motors - 1: ");
    Serial.print(front_wheel);
    Serial.print("  2: ");
    Serial.print(right_wheel);
    Serial.print("  3: ");
    Serial.print(back_wheel);
    Serial.print("  4: ");
    Serial.print(left_wheel);

    // Servo Debugging
    Serial.print("  Servos - 1_3: ");
    Serial.print(servo1_3.read());
    Serial.print("  2: ");
    Serial.print(servo2.read());
    Serial.print("  4: ");
    Serial.print(servo4.read());
    Serial.print("  5: ");
    Serial.print(servo5.read());
    Serial.print("  6: ");
    Serial.print(servo6.read());

    // Pneumatic Debugging
    Serial.print("  Pneumatics - Left: ");
    Serial.print(pneumLeft);
    Serial.print("  Right: ");
    Serial.print(pneumRight);

    // ODrive Debugging
    Serial.print("  ODrives: ");
    Serial.print(odrv);

    // Ball Pickup Debugging
    Serial.print("  Pickup State: ");
    Serial.println(pickupState);
  } else {
    updateMotors(0, 0, 0);
  }
}
  