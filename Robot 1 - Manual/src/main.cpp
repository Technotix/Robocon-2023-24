#include <Arduino.h>

// Add Comment to Disable Alternate XBOX Receiver Mode
#define XBOX_ALT_MODE

#include <Cytron_SmartDriveDuo.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <math.h>
#include "CytronMotorDriver.h"
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

// Define Motor Pins
#define mdds_1_2 23
#define mdds_3_4 25

CytronMD linearLeft(PWM_DIR, 5, 49);
CytronMD linearRight(PWM_DIR, 4, 53);

// Define Ball Pickup Pins
#define pwm 2
#define dir 27

// Define All Objects
HardwareSerial &odrive_serial = Serial3;
ODriveArduino odrive(odrive_serial);

HardwareSerial &servo_serial = Serial2;

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0, requested_state = 0, motornum = 0, pickupState = 0, srvState1_2 = 1, srvState3_4 = 1, bp_state = 0, srv_1_2 = 0, srv_3_4 = 0;
bool odrv = false, mode1 = true, mode2 = false;

bool xboxConnCheck()
{
#ifdef XBOX_ALT_MODE
  if (Xbox.Xbox360Connected)
  {
    return true;
  }
  return false;
#else
  if (Xbox.XboxReceiverConnected)
  {
    if (Xbox.Xbox360Connected[conid])
    {
      return true;
    }
  }
  return false;
#endif
}

void updateMotors(int XSpeed, int YSpeed, int TSpeed)
{
  if (mode1)
  {
    front_wheel = constrain(XSpeed - TSpeed, -100, 100);
    right_wheel = constrain(-YSpeed + TSpeed, -100, 100);
    back_wheel = constrain(XSpeed + TSpeed, -100, 100);
    left_wheel = constrain(-YSpeed - TSpeed, -100, 100);
  }
  else
  {
    front_wheel = constrain(XSpeed - TSpeed, -100, 100);
    right_wheel = constrain(-YSpeed + TSpeed, -100, 100);
    back_wheel = constrain(XSpeed + TSpeed, -100, 100);
    left_wheel = constrain(-YSpeed - TSpeed, -100, 100);
  }

  motor1motor2.control(front_wheel, back_wheel);
  motor3motor4.control(right_wheel, left_wheel);
}

void setPickup(int direction, int speed)
{
  if (direction == 1)
  {
    digitalWrite(dir, LOW);
  }
  else
  {
    digitalWrite(dir, HIGH);
  }
  analogWrite(pwm, speed);
}

void setup()
{
  Wire.begin();
  odrive_serial.begin(115200);
  servo_serial.begin(9600);
  Serial.begin(115200);

#if !defined(MIPSEL)
  while (!Serial)
    ;
#endif
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));

  // Set Stepper Pins
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  // Set Initial Values
  updateMotors(0, 0, 0);
  servo_serial.write("#1P800#2P800#3P800#4P800T100\r\n");
}

void loop()
{
  Usb.Task();
  if (xboxConnCheck())
  {
    if (!Xbox.getButtonPress(LT))
    {
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

      if (!Xbox.getButtonPress(RT))
      {
        int minSpeed = 5;
        int maxSpeed = 50;

        if (leftHatX > deadzone)
        {
          XSpeed = map(leftHatX, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (leftHatX < -deadzone)
        {
          XSpeed = map(leftHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
        if (leftHatY > deadzone)
        {
          YSpeed = map(leftHatY, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (leftHatY < -deadzone)
        {
          YSpeed = map(leftHatY, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
        if (rightHatX > deadzone)
        {
          TSpeed = map(rightHatX, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (rightHatX < -deadzone)
        {
          TSpeed = map(rightHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
      }
      else
      {
        int minSpeed = 5;
        int maxSpeed = 15;

        if (leftHatX > deadzone)
        {
          XSpeed = map(leftHatX, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (leftHatX < -deadzone)
        {
          XSpeed = map(leftHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
        if (leftHatY > deadzone)
        {
          YSpeed = map(leftHatY, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (leftHatY < -deadzone)
        {
          YSpeed = map(leftHatY, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
        if (rightHatX > deadzone)
        {
          TSpeed = map(rightHatX, deadzone, 32767, minSpeed, maxSpeed);
        }
        else if (rightHatX < -deadzone)
        {
          TSpeed = map(rightHatX, -32768, -deadzone, -maxSpeed, -minSpeed);
        }
      }
      updateMotors(-XSpeed, -YSpeed, TSpeed);

      if (Xbox.getButtonClick(START))
      {
        mode1 = false;
        mode2 = true;
      }
      if (Xbox.getButtonClick(BACK))
      {
        mode1 = true;
        mode2 = false;
      }

      if (mode1) // Seedling Mode
      {
        // Servo Control
        if (Xbox.getButtonClick(LB))
        {
          if (srv_1_2 == 0)
          {
            servo_serial.write("#1P500#2P500T100\r\n");
            srv_1_2 = 1;
          }
          else
          {
            servo_serial.write("#1P2500#2P2500T100\r\n");
            srv_1_2 = 0;
          }
        }
        if (Xbox.getButtonClick(RB))
        {
          if (srv_3_4 == 0)
          {
            servo_serial.write("#3P500#4P500T100\r\n");
            srv_3_4 = 1;
          }
          else
          {
            servo_serial.write("#3P500#4P2500T100\r\n");
            srv_3_4 = 0;
          }
        }

        // Linear Actuator
        if (Xbox.getButtonPress(Y))
        {
          linearLeft.setSpeed(-255);
        }
        else if (Xbox.getButtonPress(A))
        {
          linearLeft.setSpeed(255);
        }
        else
        {
          linearLeft.setSpeed(0);
        }

        if (Xbox.getButtonPress(UP))
        {
          linearRight.setSpeed(-255);
        }
        else if (Xbox.getButtonPress(DOWN))
        {
          linearRight.setSpeed(255);
        }
        else
        {
          linearRight.setSpeed(0);
        }
      }
      else // Ball Throwing Mode
      {
        // Ball Pickup Control
        if (Xbox.getButtonClick(UP))
        {
          if (bp_state)
          {
            setPickup(1, 140);
            delay(150);
            setPickup(1, 0);
            bp_state = 0;
          }
          else
          {
            setPickup(0, 140);
            bp_state = 1;
          }
        }
        if (Xbox.getButtonClick(DOWN))
        {
          setPickup(1, 140);
          delay(150);
          setPickup(1, 0);
          bp_state = 0;
        }

        // Calibrate ODrives
        if (Xbox.getButtonClick(A))
        {
          updateMotors(0, 0, 0);
          linearLeft.setSpeed(0);
          linearRight.setSpeed(0);
          setPickup(0, 0);
          bp_state = 0;
          for (int i = 0; i <= 1; i++)
          {
            requested_state = AXIS_STATE_MOTOR_CALIBRATION;
            if (!odrive.run_state(i, requested_state, true))
              return;

            requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            if (!odrive.run_state(i, requested_state, true, 25.0f))
              return;

            requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
            if (!odrive.run_state(i, requested_state, false))
              return;
          }
        }

        // Set ODrive Velocities
        if (Xbox.getButtonClick(Y))
        {
          odrive.SetVelocity(0, odrv ? 0 : 30);
          odrive.SetVelocity(1, odrv ? 0 : 25);
          odrv = !odrv;
        }

        if (Xbox.getButtonClick(B))
        {
          odrive.SetVelocity(0, odrv ? 0 : 35);
          odrive.SetVelocity(1, odrv ? 0 : 30);
          odrv = !odrv;
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
      Serial.print("  Servos - 1_2: ");
      Serial.print(srv_1_2);
      Serial.print("  Servos - 3_4: ");
      Serial.print(srv_3_4);

      // ODrive Debugging
      Serial.print("  ODrives: ");
      Serial.print(odrv);

      // Ball Pickup Debugging
      Serial.print("  Pickup State: ");
      Serial.println(pickupState);
    }
    else
    {
      updateMotors(0, 0, 0);
      linearLeft.setSpeed(0);
      linearRight.setSpeed(0);
      setPickup(0, 0);
      bp_state = 0;
      odrive.SetVelocity(0, 0);
      odrive.SetVelocity(1, 0);
      odrv = false;
      Serial.println("Kill Switch Activated!");
    }
  }
  else
  {
    updateMotors(0, 0, 0);
    linearLeft.setSpeed(0);
    linearRight.setSpeed(0);
    setPickup(0, 0);
    bp_state = 0;
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odrv = false;
    Serial.println("Controller Disconnected!");
  }
}
