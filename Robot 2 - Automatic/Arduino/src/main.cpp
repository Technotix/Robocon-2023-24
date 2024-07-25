#include <SoftwareSerial.h>
#include <Cytron_SmartDriveDuo.h>
#include "CytronMotorDriver.h"
#include <LSA08.h>

#define mdds_2_4 29 // motor 2 and motor 4
#define mdds_1_3 31 // motor 1 and motor 3
#define BAUDRATE 115200

#define minSpeed 10
#define maxSpeed 60
#define rMaxSpeed 150

#define fspeed 80   // Forward speed
#define rspeed -20  // Rotate speed
#define setpoint 35 // Desired line sensor position
#define del 5       // Delay

Cytron_SmartDriveDuo motor2motor4(SERIAL_SIMPLIFIED, mdds_2_4, 115200);
Cytron_SmartDriveDuo motor1motor3(SERIAL_SIMPLIFIED, mdds_1_3, 115200);

#define LSASerial1 Serial2
#define LSASerial2 Serial3

CytronMD roller_in(PWM_DIR, 3, 35);
CytronMD roller_1(PWM_DIR, 2, 33);
CytronMD roller_2(PWM_DIR, 4, 37);
CytronMD roller_3(PWM_DIR, 5, 39);

LSA08 L1;
LSA08 L2;

int front_right = 0, back_right = 0, back_left = 0, front_left = 0;
bool lineMode = false;
int XSpeed = 0, YSpeed = 0, TSpeed = 0, intDir = 0, shootDir = 0;
int input1 = 0, prev_input1 = 0, input2 = 0, prev_input2 = 0, output = 0, r_counter = 0;
bool mode2 = false, mode3 = false;

unsigned long previousMillis = 0;

int calculateTSpeed(int inp);
bool isValid(int inp, int prev_input);
void printDebug();
void turn(bool dir);

void updateMotors(int XSpeed, int YSpeed, int TSpeed)
{
  front_right = constrain(-YSpeed + TSpeed, -100, 100);
  back_right = constrain(+XSpeed - TSpeed, -100, 100);
  back_left = constrain(-YSpeed - TSpeed, -100, 100);
  front_left = constrain(XSpeed + TSpeed, -100, 100);

  motor2motor4.control(back_right, front_left);
  motor1motor3.control(back_left, front_right);
}

void updateIntake(int dir)
{
  if (dir == 1)
  {
    roller_1.setSpeed(rMaxSpeed);
    roller_in.setSpeed(rMaxSpeed);
  }
  else if (dir == -1)
  {
    roller_1.setSpeed(-rMaxSpeed);
    roller_in.setSpeed(-rMaxSpeed);
  }
  else
  {
    roller_in.setSpeed(0);
    roller_1.setSpeed(0);
  }
}

void updateShooting(int dir)
{
  if (dir == 1)
  {
    roller_1.setSpeed(-rMaxSpeed);
    roller_2.setSpeed(rMaxSpeed);
    roller_3.setSpeed(-rMaxSpeed);
  }
  else if (dir == -1)
  {
    roller_1.setSpeed(-rMaxSpeed);
    roller_2.setSpeed(-rMaxSpeed);
    roller_3.setSpeed(rMaxSpeed);
  }
  else
  {
    roller_1.setSpeed(0);
    roller_2.setSpeed(0);
    roller_3.setSpeed(0);
  }
}

void setup()
{
  Serial.begin(9600);
  updateMotors(0, 0, 0);
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  LSASerial1.begin(38400);
  L1.AttachSerial(&LSASerial1);
  LSASerial2.begin(38400);
  L2.AttachSerial(&LSASerial2);
  updateMotors(0, 0, 0);
  pinMode(13, OUTPUT);
  Serial.println("Line Following Started");
}

void loop()
{
  String signal = "";
  if (lineMode)
  {
    digitalWrite(13, HIGH);
    input1 = L1.ReadLSA();
    input2 = L2.ReadLSA();
    if (mode3)
    {
      if (millis() - previousMillis <= 1000 * del)
      { // Check if delay has passed
        if (isValid(input1, prev_input1))
        {
          if (input1 != 255)
          {
            output = calculateTSpeed(input1);
            updateMotors(0, fspeed, output);
          }
          else
          {
            updateMotors(0, fspeed, 0);
          }
        }
      }
      else
      {
        updateMotors(0, fspeed, 0); // Move forward while waiting
      }
    }
    else if (mode2)
    {
      if (isValid(input2, prev_input2))
      {
        if (input2 != 255)
        {
          output = calculateTSpeed(input2);
          updateMotors(fspeed - 10, 0, output);
        }
        else
        {
          if (input1 < 15 || input1 > 55)
          {
            updateMotors(fspeed / 3, 0, 0);
          }
          else
          {
            mode3 = true;
            updateMotors(0, 0, 0);
            delay(50);
            input1 = L1.ReadLSA();
            updateMotors(0, fspeed / 3, calculateTSpeed(input1) / 3);
            delay(100);
            input1 = L1.ReadLSA();
            updateMotors(0, fspeed / 3, calculateTSpeed(input1) / 3);
            delay(100);
            input1 = L1.ReadLSA();
            updateMotors(0, fspeed / 3, calculateTSpeed(input1) / 3);
            delay(100);
            input1 = L1.ReadLSA();
            updateMotors(0, fspeed / 2, calculateTSpeed(input1) / 2);
            delay(100);
            input1 = L1.ReadLSA();
            updateMotors(0, fspeed / 2, calculateTSpeed(input1) / 2);
            delay(100);
            Serial.write("E");
            updateMotors(0, 0, 0);
          }
        }
      }
    }
    else if (isValid(input1, prev_input1))
    {
      if (input1 != 255)
      {
        output = calculateTSpeed(input1);
        updateMotors(0, fspeed, output);
      }
      else
      {
        if (input2 < 15 || input2 > 55)
        {
          updateMotors(0, fspeed / 3, 0);
        }
        else
        {
          mode2 = true;
          updateMotors(0, 0, 0);
          delay(50);
          input2 = L2.ReadLSA();
          updateMotors(fspeed / 3, 0, calculateTSpeed(input2) / 3);
          delay(100);
          input2 = L2.ReadLSA();
          updateMotors(fspeed / 3, 0, calculateTSpeed(input2) / 3);
          delay(100);
          input2 = L2.ReadLSA();
          updateMotors(fspeed / 3, 0, calculateTSpeed(input2) / 3);
          delay(100);
          input2 = L2.ReadLSA();
          updateMotors(fspeed / 2, 0, calculateTSpeed(input2) / 2);
          delay(100);
          input2 = L2.ReadLSA();
          updateMotors(fspeed / 2, 0, calculateTSpeed(input2) / 2);
          delay(100);
        }
      }
    }
    printDebug();
    prev_input1 = input1;
    prev_input2 = input2;
  }
  else
  {
    while (Serial.available() > 0)
    {
      char data = Serial.read();
      if (data == 'Z')
      {
        break;
      }
      signal += data;
    }
    signal.trim();

    if (signal == "R")
    {
      updateMotors(0, 0, -15);
      // Serial.print(signal);
      digitalWrite(13, HIGH);
      updateIntake(0);
      updateShooting(0);
    }
    else if (signal == "L")
    {
      updateMotors(0, 0, 20);
      digitalWrite(13, LOW);
      updateIntake(0);
      updateShooting(0);
    }

    else if (signal == "r")
    {
      updateMotors(0, 0, 15);
      updateIntake(0);
      updateShooting(0);
    }

    else if (signal == "F")
    {
      updateMotors(0, maxSpeed, 0);
      updateIntake(0);
      updateShooting(0);
    }
    else if (signal == "f")
    {
      updateMotors(0, maxSpeed / 4, 0);
      updateIntake(0);
      updateShooting(0);
    }
    else if (signal == "I")
    {
      updateIntake(-1);
      updateShooting(0);
    }

    else if (signal == "s")
    {
      updateMotors(0, 0, 0);
      intDir = 1;
      updateIntake(intDir);
      updateShooting(0);
    }
    else if (signal == "B")
    {
      updateMotors(0, -maxSpeed / 2, 0);
    }
    else if (signal == "T")
    {
      updateShooting(-1);
    }
    else if (signal == "O")
    {
      updateIntake(0);
    }
    else if (signal == "S")
    {
      updateMotors(0, 0, 0);
      updateIntake(0);

      updateShooting(0);
    }
    else if (signal == "Q")
    {
      lineMode = true;
    }
  }
}

int calculateTSpeed(int inp)
{
  return map(inp - setpoint, -setpoint, setpoint, rspeed, -rspeed);
}

bool isValid(int inp, int prev_input)
{
  return (inp < 70 || inp == 255) && (prev_input == inp);
}

void printDebug()
{
  Serial.print("Actual Pos 1: ");
  Serial.print(input1);
  Serial.print("  Actual Pos 2: ");
  Serial.print(input2);
  Serial.print("  Expected Pos: ");
  Serial.print(setpoint);
  Serial.print("  TSpeed: ");
  Serial.print(output);
  Serial.print(" Mode 2: ");
  Serial.print(mode2);
  Serial.print(" Mode 3: ");
  Serial.print(mode3);
  Serial.print("  RCount: ");
  Serial.print(r_counter);
  Serial.print("  Front_Left: ");
  Serial.print(front_left);
  Serial.print("  Front_Right: ");
  Serial.print(front_right);
  Serial.print("  Back_Left: ");
  Serial.print(back_left);
  Serial.print("  Back_Right:  ");
  Serial.println(back_right);
}

void turn(bool dir)
{
  if (dir)
  {
    output = rspeed;
  }
  else
  {
    output = -rspeed;
  }
  updateMotors(0, 0, output);
}
