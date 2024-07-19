// Define Color (Blue = 0, Red = 1)
#define team 0 // Not yet implemented

// Include Libraries
#include <Arduino.h>
#include <Cytron_SmartDriveDuo.h>
#include <LSA08.h>
#include <SoftwareSerial.h>

// Define Constants
#define fspeed 80   // Forward speed
#define rspeed 20   // Rotate speed
#define setpoint 35 // Desired line sensor position
#define del 5       // Delay

#define LSASerial1 Serial1
#define LSASerial2 Serial2
#define LSASerial3 Serial3

#define mdds_1_3 50
#define mdds_2_4 48

// Global Variables
Cytron_SmartDriveDuo motor1motor3(SERIAL_SIMPLIFIED, mdds_1_3, 115200); // Motors 1 & 3
Cytron_SmartDriveDuo motor2motor4(SERIAL_SIMPLIFIED, mdds_2_4, 115200); // Motors 2 & 4

LSA08 L1;
LSA08 L2;
LSA08 L3;

int front_left = 0, front_right = 0, back_left = 0, back_right = 0;
int input1 = 0, prev_input1 = 0, input2 = 0, prev_input2 = 0, input3 = 0, prev_input3 = 0, output = 0, r_counter = 0;
bool mode2 = false, mode3 = false;

unsigned long previousMillis = 0;

// Function Prototypes
void updateMotors(int XSpeed, int YSpeed, int TSpeed);
int calculateTSpeed(int inp);
bool isValid(int inp, int prev_input);
void printDebug();
void turn(bool dir);

void setup()
{
  Serial.begin(115200);
  LSASerial1.begin(38400);
  L1.AttachSerial(&LSASerial1);
  LSASerial2.begin(38400);
  L2.AttachSerial(&LSASerial2);
  LSASerial3.begin(38400);
  L3.AttachSerial(&LSASerial3);
  updateMotors(0, 0, 0);
  Serial.println("Line Following Started");
}

void loop()
{
  input1 = L1.ReadLSA();
  input2 = L2.ReadLSA();
  input3 = L3.ReadLSA();

  if (mode3)
  {
    if (isValid(input1, prev_input1) and (isValid(input3, prev_input3)))
    {
      if (input3 < 15 || input3 > 55)
      {
        updateMotors(fspeed / 2, 0, calculateTSpeed(input3));
      }
      else
      {
        while (input1 > 15 && input1 < 55)
        {
          input1 = L1.ReadLSA();
          updateMotors(0, fspeed / 2, calculateTSpeed(input1));
        }
      }
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
  prev_input3 = input3;
}

// Function Definitions
void updateMotors(int XSpeed, int YSpeed, int TSpeed)
{
  front_left = constrain(XSpeed + TSpeed, -100, 100);
  front_right = constrain(-YSpeed + TSpeed, -100, 100);
  back_left = constrain(-YSpeed - TSpeed, -100, 100);
  back_right = constrain(XSpeed - TSpeed, -100, 100);

  motor1motor3.control(back_left, front_right);
  motor2motor4.control(back_right, front_left);
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
  Serial.print("  Actual Pos 3: ");
  Serial.print(input3);
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