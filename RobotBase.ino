#include <BMSerial.h>
#include <RoboClaw.h>

#define LED_PIN 13

// host communication //
#define MAX_COMMAND 10
#define BUF_SIZE 32
byte buf[BUF_SIZE];
byte cmd = 0;
byte bytes = 0;
byte crc = 0;
boolean doCRC = false;

// power management //
#define AUX_PIN 7
#define MOTORS_PIN 8
#define UPS_PIN 9
#define BATTERY_PIN 10

// common vars //
unsigned long lastCommandTime;
unsigned long commandTime;
unsigned long cmdTimeout = 2500;
boolean IsConnected = false;

// RoboClaw //
#define RC_ADDR 0x80
#define RC_RX_PIN 12
#define RC_TX_PIN 11
RoboClaw roboclaw(RC_RX_PIN, RC_TX_PIN);
boolean driveEnabled = false;

// propulsion vars
#define MOTOR_VELOCITY_0 64
#define MOTOR_VELOCITY_MAX 63
#define MOTOR_TARGET_MAX 127
byte motor1 = MOTOR_VELOCITY_0;
byte motor1_target = MOTOR_VELOCITY_0;
byte motor2 = MOTOR_VELOCITY_0;
byte motor2_target = MOTOR_VELOCITY_0;
byte motor_velocity = 30;
byte motor_velocity_increment = 5;
unsigned int motor_counter_start = 3000;
unsigned int motor_counter = 0;

//=======================================

void setup() 
{
  // unused pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  
  // power management pins //
  pinMode(AUX_PIN, OUTPUT);
  pinMode(MOTORS_PIN, OUTPUT);
  pinMode(UPS_PIN, OUTPUT);
  pinMode(BATTERY_PIN, OUTPUT);
  digitalWrite(AUX_PIN, LOW);
  digitalWrite(MOTORS_PIN, LOW);
  digitalWrite(UPS_PIN, LOW);
  digitalWrite(BATTERY_PIN, LOW);
  pulsePin(AUX_PIN, 20);
  
  // host communication //
  Serial.begin(38400);
  
  // init motor driver //
  driveUART(true);
  driveUART(false);
}

//=======================================

void loop()
{
  // serial input
  ReadSerial();
  if (IsConnected) CheckCommandTimeout();
  
  // motors
  if (motor_counter > 0) motor_counter--;
  if (motor_counter == 0) updateMotors();
}

//=======================================

// read serial data into buffer. execute command
void ReadSerial()
{
  while (Serial.available())
  {
    buf[bytes] = Serial.read();
    if (buf[bytes] == 10 || buf[bytes] == 13 || bytes >= MAX_COMMAND)
    {
      if (bytes > 0)
      {
        if (!doCRC) Execute();
        else { if (crc8_ok()) Execute(); }
      }
      return;
    }
    bytes++;
  }
}

boolean crc8_ok()
{
  crc = 0;
  for (byte i = 0; i < bytes - 1; i++) crc8(buf[i]);
  if (crc == buf[bytes - 1]) return true;
  // if error
  bytes = 0; // empty input buffer
  driveStop(); // disable motors
  Serial.println("CRC"); // report CRC error
  return false;
}

void crc8(byte x)
{
  crc ^= x;
  x = 0;
  if (crc & 0x01) x ^= 0x5e;
  if (crc & 0x02) x ^= 0xbc;
  if (crc & 0x04) x ^= 0x61;
  if (crc & 0x08) x ^= 0xc2;
  if (crc & 0x10) x ^= 0x9d;
  if (crc & 0x20) x ^= 0x23;
  if (crc & 0x40) x ^= 0x46;
  if (crc & 0x80) x ^= 0x8c;
  crc = x;
}

//=======================================

void CheckCommandTimeout()
{
  if (cmdTimeout == 0) return;
  commandTime = millis();
  if (commandTime >= lastCommandTime) commandTime -= lastCommandTime; else lastCommandTime = 0;
  if (commandTime > cmdTimeout) OnDisconnected();
}

void OnDisconnected()
{
  driveStop();
  IsConnected = false;
}

void OnValidCommand()
{
  IsConnected = true;
  lastCommandTime = millis();
}

//=======================================

// report error
void err()
{ Serial.println("ERR"); }

// report negative acknowledge
void nak()
{ Serial.println("NAK"); }

// execute command in buffer //
// the first character is a command identifier
// the second is a subcommand
void Execute()
{
  bytes = 0; // empty input buffer (only one command at a time)
  cmd = buf[0];
  if (cmd == 0 || cmd == 32) return;
  
  if (cmd == 'D' || cmd == 'd') drive();
  else
  if (cmd == 'V' || cmd == 'v') ver();
  else
  if (cmd == 'P' || cmd == 'p') power();
  else
  if (cmd == 'S' || cmd == 's') set();
  else
  if (cmd == 'B' || cmd == 'b') battery();
  else
  if (cmd == 'R' || cmd == 'r') _read();
  else
  if (cmd == 'W' || cmd == 'w') _write();
  else
  if (cmd == 'Z' || cmd == 'z') { OnDisconnected(); return; }
  else
  {
    nak(); return;
  }
  
  OnValidCommand();
}

//=======================================












//=======================================

// 'drive' command. both motors in one command
// ex: DF - drive forward at 'motor_velocity'
// ex: DD127127 - drive differential. full forward
// ex: DD000000 - drive differential. full backward
void drive()
{
  cmd = buf[1];
  
  if (cmd == 'D' || cmd == 'd') // drive differential
  {
    motor1_target = bctoi(2, 3);
    motor2_target = bctoi(5, 3);
    if (motor1_target > MOTOR_TARGET_MAX) motor1_target = MOTOR_TARGET_MAX;
    if (motor2_target > MOTOR_TARGET_MAX) motor2_target = MOTOR_TARGET_MAX;
  }
  else
  if (cmd == 'Z' || cmd == 'z') driveStop(); // stop immediately
  else
  if (cmd == 'F' || cmd == 'f') { motor1_target = MOTOR_VELOCITY_0 + motor_velocity; motor2_target = motor1_target; }
  else
  if (cmd == 'B' || cmd == 'b') { motor1_target = MOTOR_VELOCITY_0 - motor_velocity; motor2_target = motor1_target; }
  else
  if (cmd == 'L' || cmd == 'l') { motor1_target = MOTOR_VELOCITY_0 - motor_velocity; motor2_target = MOTOR_VELOCITY_0 + motor_velocity; }
  else
  if (cmd == 'R' || cmd == 'r') { motor1_target = MOTOR_VELOCITY_0 + motor_velocity; motor2_target = MOTOR_VELOCITY_0 - motor_velocity; }
  else
  if (cmd == 'S' || cmd == 's') { motor1_target = MOTOR_VELOCITY_0; motor2_target = MOTOR_VELOCITY_0; }
  else
  if (cmd == 'E' || cmd == 'e') driveEnable(true);
  else
  if (cmd == 'O' || cmd == 'o') driveEnable(false);
  else
  if (cmd == 'V' || cmd == 'v')
  {
    motor_velocity = bctoi(2, 2);
    if (motor_velocity > MOTOR_VELOCITY_MAX) motor_velocity = MOTOR_VELOCITY_MAX;
  }
  else
  if (cmd == 'I' || cmd == 'i') motor_velocity_increment = bctoi(2, 2);
  else
  if (cmd == 'C' || cmd == 'c') motor_counter_start = (unsigned int)bctoi(2, 1) * 1000;
  else
  {
    nak();
  }
}

// update motor velocities
void updateMotors()
{
  if (!driveEnabled) return;
  
  // M1
  if (motor1 != motor1_target)
  {
    motor_counter = motor_counter_start;
    if (motor1 < motor1_target)
    {
      if (motor1_target - motor1 > motor_velocity_increment) motor1 += motor_velocity_increment;
      else motor1 = motor1_target;
    }
    else
    if (motor1 > motor1_target)
    {
      if (motor1 - motor1_target > motor_velocity_increment) motor1 -= motor_velocity_increment;
      else motor1 = motor1_target;
    }
    roboclaw.ForwardBackwardM1(RC_ADDR, motor1);
  }
  // M2
  if (motor2 != motor2_target)
  {
    motor_counter = motor_counter_start;
    if (motor2 < motor2_target)
    {
      if (motor2_target - motor2 > motor_velocity_increment) motor2 += motor_velocity_increment;
      else motor2 = motor2_target;
    }
    else
    if (motor2 > motor2_target)
    {
      if (motor2 - motor2_target > motor_velocity_increment) motor2 -= motor_velocity_increment;
      else motor2 = motor2_target;
    }
    roboclaw.ForwardBackwardM2(RC_ADDR, motor2);
  }
}

// stop both motors immediately
void driveStop()
{
  if (!driveEnabled) return;
  motor1_target = MOTOR_VELOCITY_0;
  motor1 = MOTOR_VELOCITY_0;
  motor2_target = MOTOR_VELOCITY_0;
  motor2 = MOTOR_VELOCITY_0;
  roboclaw.ForwardBackwardM1(RC_ADDR, motor1);
  roboclaw.ForwardBackwardM2(RC_ADDR, motor2);
}

// enable/disable motor driver communication and power
void driveEnable(boolean action)
{
  if (action)
  {
    pulsePin(MOTORS_PIN, 20);
    driveUART(true);
  }
  else
  {
    driveUART(false);
    pulsePin(MOTORS_PIN, 20);
  }
}

// enable/disable motor driver communication
void driveUART(boolean action)
{
  if (action) roboclaw.begin(38400);
  else roboclaw.end();
  driveEnabled = action;
}

//=======================================












//=======================================

// 'version' command
// ex: V
void ver()
{
  Serial.println("V4");
}

// 'power' command
void power()
{
  cmd = buf[1];
  if (cmd == 'A' || cmd == 'a') { pulsePin(AUX_PIN, 20); delay(3000); pulsePin(AUX_PIN, 20);  }
  else
  if (cmd == 'M' || cmd == 'm') pulsePin(MOTORS_PIN, 20);
  else
  if (cmd == 'U' || cmd == 'u') pulsePin(UPS_PIN, 5500);
  else
    nak();
}

// 'set parameter' command
// ex: ST25 - set timeout = 2500ms
// ex: SC1 - enable CRC
void set()
{
  cmd = buf[1];
  // command timeout
  if (cmd == 'T' || cmd == 't') cmdTimeout = bctoi(2, 2) * 100;
  else
  if (cmd == 'C' || cmd == 'c') doCRC = buf[2] == '1';
  else
    nak();
}

// 'battery voltage' command
// ex: B
// result in mV
void battery()
{
  digitalWrite(BATTERY_PIN, HIGH);
  analogRead(0);
  unsigned long voltage = (unsigned long)analogRead(0) * 625 >> 7;
  digitalWrite(BATTERY_PIN, LOW);
  Serial.write('B');
  Serial.println(voltage);
}

// 'digitalWrite' command
// ex: W031 - set pin 3 HIGH, W100 - set pin 10 LOW
void _write()
{
  byte pin = bctoi(1, 2);
  if (pin < 2 || pin > 10) { err(); return; }
  pinMode(pin, OUTPUT);
  if (buf[3] == '0') digitalWrite(pin, LOW); else digitalWrite(pin, HIGH);
}

//=======================================

// 'read' command
void _read()
{
  cmd = buf[1];
  if (cmd == 'D' || cmd == 'd') dread();
  else
  if (cmd == 'A' || cmd == 'a') aread();
  else
    nak();
}

// 'digitalRead' command
// ex: RD5 or RD05 or RD10
void dread()
{
  byte pin = bctoi(2, 2);
  if (pin < 2 || pin > 10) { err(); return; }
  pinMode(pin, INPUT_PULLUP);
  unsigned int a = digitalRead(pin);
  Serial.print("RD");
  Serial.print((unsigned int)pin);
  Serial.print("=");
  Serial.println(a);
}

// 'analogRead' command
// ex: RA0
// result in mV
void aread()
{
  byte pin = bctoi(2, 1);
  if (pin < 0 || pin > 7) { err(); return; }
  unsigned long a = (unsigned long)analogRead(pin) * 625 >> 7;
  Serial.print("RA");
  Serial.print((unsigned int)pin);
  Serial.print("=");
  Serial.println(a);
}

//=======================================
//
// helper functions
//
//=======================================

void pulsePin(int pin, int ms)
{
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

// converts string to number. reads 'count' chars starting at 'index' position in buffer
byte bctoi(byte index, byte count)
{
  byte i = 0;
  byte fin = index + count;
  if (fin > BUF_SIZE) fin = BUF_SIZE;
  while (buf[index] >= '0' && buf[index] <= '9' && index < fin)
  {
    i *= 10;
    i += buf[index] - '0';
    index++;
  }
  return i;
}
