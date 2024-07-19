// Add Comment to Disable Alternate XBOX Receiver Mode
#define XBOX_ALT_MODE

#include <Arduino.h>
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

// Define Ball Pickup Pins
#define pwm 2
#define dir 27

// Define laser pins
#define l1 30
#define l2 3

// Define Linear Actuator Pins
#define ll_pwm 5
#define lr_pwm 4
#define ll_dir 49
#define lr_dir 53


// Define All Objects
HardwareSerial &odrive_serial = Serial3;
ODriveArduino odrive(odrive_serial);

HardwareSerial &servo_serial = Serial2;

Cytron_SmartDriveDuo motor1motor2(SERIAL_SIMPLIFIED, mdds_1_2, 115200);
Cytron_SmartDriveDuo motor3motor4(SERIAL_SIMPLIFIED, mdds_3_4, 115200);

CytronMD linearLeft(PWM_DIR, ll_pwm, ll_dir);
CytronMD linearRight(PWM_DIR, lr_pwm, lr_dir);

USB Usb;
#ifdef XBOX_ALT_MODE
XBOXUSB Xbox(&Usb);
#else
XBOXRECV Xbox(&Usb);
#endif

// Define Variables
int front_wheel = 0, right_wheel = 0, back_wheel = 0, left_wheel = 0, requested_state = 0, motornum = 0, pickupState = 0, srvState1_2 = 1, srvState3_4 = 1, bp_state = 0, srv_1_2 = 0, srv_3_4 = 0;
bool odrv = false, mode1 = true, mode2 = false, lu = false, ld = false, ru = false, rd = false;
double long ts = 0;

// Function Definitions
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
    front_wheel = constrain(-XSpeed - TSpeed, -100, 100);
    right_wheel = constrain(YSpeed + TSpeed, -100, 100);
    back_wheel = constrain(-XSpeed + TSpeed, -100, 100);
    left_wheel = constrain(YSpeed - TSpeed, -100, 100);
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


// Setup Function
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

  // Initialize Motors and laser
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);

  digitalWrite(l1, HIGH);
  digitalWrite(l2, HIGH);

  // Set Servo Positions and stop motors
  updateMotors(0, 0, 0);
  servo_serial.write("#1P1625#2P1550#3P1850#4P1875T100\r\n");
}


// Main Loop
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
        if (Xbox.getButtonClick(RB))
        {
          if (srv_1_2 == 0)
          {
            servo_serial.write("#1P2000#2P1800T100\r\n");
            srv_1_2 = 1;
          }
          else
          {
            servo_serial.write("#1P1185#2P1225T100\r\n");
            srv_1_2 = 0;
          }
        }
        if (Xbox.getButtonClick(LB))
        {
          if (srv_3_4 == 0)
          {
            servo_serial.write("#3P1400#4P1500T100\r\n");
            srv_3_4 = 1;
          }
          else
          {
            servo_serial.write("#3P2350#4P2400T100\r\n");
            srv_3_4 = 0;
          }
        }

        // Linear Actuator
        if (Xbox.getButtonPress(UP))
        {
          linearLeft.setSpeed(-255);
        }
        else if (Xbox.getButtonPress(DOWN))
        {
          linearLeft.setSpeed(255);
        }
        else
        {
          linearLeft.setSpeed(0);
        }

        if (Xbox.getButtonPress(Y))
        {
          linearRight.setSpeed(-255);
        }
        else if (Xbox.getButtonPress(A))
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