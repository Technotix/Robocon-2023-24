#include <Arduino.h>

#define XBOX_ALT_MODE

#include <Cytron_SmartDriveDuo.h>
#include "CytronMotorDriver.h"
#include <math.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#ifdef XBOX_ALT_MODE
#include <XBOXUSB.h>
#else
#include <XBOXRECV.h>
#endif

#define deadzone 0
#define conid 0

#define minSpeed 10
#define maxSpeed 50
#define rMaxSpeed 200

#define mdds_1_2 47
#define mdds_3_4 49

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

CytronMD roller_in(PWM_DIR, 2, 37);  
CytronMD roller_1(PWM_DIR, 3, 33);   
CytronMD roller_3(PWM_DIR, 4, 45);   
CytronMD roller_2(PWM_DIR, 5, 35);   


USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

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

int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0, prev_id = 0, prev_sd = 0;

void updateMotors(int XSpeed, int YSpeed, int TSpeed) {

  front_wheel = constrain(XSpeed - TSpeed, -100, 100);
  right_wheel = constrain(-YSpeed - TSpeed, -100, 100);
  back_wheel = constrain(-XSpeed - TSpeed, -100, 100);
  left_wheel = constrain(YSpeed - TSpeed, -100, 100);

  motor1motor2.control(front_wheel, right_wheel);
  motor3motor4.control(back_wheel, -left_wheel);
}

void updateIntake(int dir) {
  if (dir == 1 & prev_id != 1) {
    roller_in.setSpeed(rMaxSpeed);
  } else if (dir == -1 & prev_id != -1) {
    roller_in.setSpeed(-rMaxSpeed);
  } else {
    roller_in.setSpeed(0);
  }
}

void updateShooting(int dir) {
  if (dir == 1 & prev_sd != 1) {
    roller_1.setSpeed(rMaxSpeed);
    roller_2.setSpeed(rMaxSpeed);
    roller_3.setSpeed(-rMaxSpeed);
  } else if (dir == -1 & prev_sd != -1) {
    roller_1.setSpeed(rMaxSpeed);
    roller_2.setSpeed(-rMaxSpeed);
    roller_3.setSpeed(rMaxSpeed);
  } else {
    roller_1.setSpeed(0);
    roller_2.setSpeed(0);
    roller_3.setSpeed(0);
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
#if !defined(_MIPSEL_)
  while (!Serial)
    ;
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
  updateIntake(0);
  updateShooting(0);
  updateMotors(0, 0, 0);
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

    int XSpeed = 0, YSpeed = 0, TSpeed = 0, intDir = 0, shootDir = 0;


    if (leftHatX > deadzone) {
      XSpeed = map(leftHatX, deadzone, 32767, minSpeed, maxSpeed);
    } else if (leftHatX < deadzone) {
      XSpeed = map(leftHatX, -32768, deadzone, -maxSpeed, minSpeed);
    }
    if (leftHatY > deadzone) {
      YSpeed = map(leftHatY, deadzone, 32767, minSpeed, maxSpeed);
    } else if (leftHatY < deadzone) {
      YSpeed = map(leftHatY, -32768, deadzone, -maxSpeed, minSpeed);
    }
    if (rightHatX > deadzone) {
      TSpeed = map(rightHatX, deadzone, 32767, minSpeed, maxSpeed);
    } else if (rightHatX < deadzone) {
      TSpeed = map(rightHatX, -32768, deadzone, -maxSpeed, minSpeed);
    }

    if (Xbox.getButtonPress(LB)) {
      intDir = 1;
    } else if (Xbox.getButtonPress(LT)) {
      intDir = -1;
    }
    
    if (Xbox.getButtonPress(RB)) {
      shootDir = 1;
    } else if (Xbox.getButtonPress(RT)) {
      shootDir = -1;
    }

    updateIntake(intDir);
    updateShooting(shootDir);
    updateMotors(XSpeed, YSpeed, TSpeed * 0.8);

    // Print
  } else {
    updateIntake(0);
    updateShooting(0);
    updateMotors(0, 0, 0);
  }
}
