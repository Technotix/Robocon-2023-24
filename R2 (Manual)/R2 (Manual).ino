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

#define mdds_1_2 39
#define mdds_3_4 41

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

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

int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0;
bool intake, shooting, rejecting, invert_intake;

void updateMotors(int XSpeed, int YSpeed, int TSpeed) {

  front_wheel = constrain(XSpeed - TSpeed, -100, 100);
  right_wheel = constrain(-YSpeed - TSpeed, -100, 100);
  back_wheel = constrain(-XSpeed - TSpeed, -100, 100);
  left_wheel = constrain(YSpeed - TSpeed, -100, 100);

  motor1motor2.control(front_wheel, back_wheel);
  motor3motor4.control(right_wheel, left_wheel);
}

CytronMD roller_3(PWM_DIR, 4, 47);   // roller_in on mdd20a
CytronMD roller_in(PWM_DIR, 5, 45);  // roller_3 on mdd20a
CytronMD roller_2(PWM_DIR, 3, 49);   // roller_2 on md20a
CytronMD roller_1(PWM_DIR, 6, 43);   // roller_1 on md20a

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

    int XSpeed = 0, YSpeed = 0, TSpeed = 0;


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

    updateMotors(XSpeed, YSpeed, TSpeed * 0.8);
    if (Xbox.getButtonClick(LB)) {
      intake = !intake;
      invert_intake = false;
    }
    if (Xbox.getButtonClick(LT)) {
      invert_intake = !invert_intake;
      intake = false;
    }
    if (Xbox.getButtonClick(RB)) {
      shooting = !shooting;
      rejecting = false;
    }
    if (Xbox.getButtonClick(RT)) {
      shooting = false;
      rejecting = !rejecting;
    }

    if (intake) {
      roller_in.setSpeed(255);
    } else if (invert_intake) {
      roller_in.setSpeed(-255);
    } else {
      roller_in.setSpeed(0);
    }

    if (shooting) {
      roller_1.setSpeed(-255);
      roller_2.setSpeed(-255);
      roller_3.setSpeed(255);
    } else if (rejecting) {
      roller_1.setSpeed(255);
      roller_2.setSpeed(-255);
      roller_3.setSpeed(255);
    } else {
      roller_1.setSpeed(0);
      roller_2.setSpeed(0);
      roller_3.setSpeed(0);
    }
    Serial.print(intake);
    Serial.print(shooting);
    Serial.println(rejecting);
  } else {
    roller_in.setSpeed(0);
    roller_1.setSpeed(0);
    roller_2.setSpeed(0);
    roller_3.setSpeed(0);
    updateMotors(0, 0, 0);
  }
}
