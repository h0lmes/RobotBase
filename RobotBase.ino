#include <BMSerial.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <PololuMaestro.h>

#define LED_PIN 13
#define SSC_RX_PIN 2
#define SSC_TX_PIN 3

// host communication //
#define MAX_COMMAND 16
#define BUF_SIZE 18
byte buf[BUF_SIZE];
byte cmd = 0;
byte bytes = 0;
byte crc = 0;

// power management //
#define AUX_PIN 7
#define MOTORS_PIN 8
#define UPS_PIN 9
#define BATTERY_PIN 10

// common vars //
unsigned long lastCommandTime;
unsigned long commandTimeout = 2500;
boolean IsConnected = false;

// RoboClaw //
#define RC_ADDR 0x80
#define RC_RX_PIN 12
#define RC_TX_PIN 11
RoboClaw roboclaw(RC_RX_PIN, RC_TX_PIN);
boolean driveEnabled = false;
unsigned long lastDriveCommandTime;
unsigned long driveCommandTimeout = 60000;

// servos
BMSerial ssc(SSC_RX_PIN, SSC_TX_PIN);
MiniMaestro maestro(ssc);

// propulsion vars
#define MOTOR_COUNTER_START 3000
#define MOTOR_TARGET_MAX 127
#define MOTOR_VELOCITY_0 64
#define MOTOR_VELOCITY_MAX 63
#define MOTOR_TARGET_MAX 127
byte motor1 = MOTOR_VELOCITY_0;
byte motor1_target = MOTOR_VELOCITY_0;
byte motor2 = MOTOR_VELOCITY_0;
byte motor2_target = MOTOR_VELOCITY_0;
byte motor_increment = 5;
unsigned int motor_counter = 0;

//=======================================

void setup() 
{
  // unused pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
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

  // servo controller
  ssc.begin(38400);
  
  // host communication //
  Serial.begin(38400);
  
  // motor driver //
  driveUART(true);
  driveUART(false);
}

//=======================================

void loop()
{
  // serial input
  ReadSerial();
  if (IsConnected) CheckCommandTimeout();
  if (driveEnabled) CheckDriveCommandTimeout();
  
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
      if (bytes > 0) Execute();
      return;
    }
    bytes++;
  }
}

boolean crc8ok()
{
  crc = 0;
  for (byte i = 0; i < bytes - 1; i++) crc8(buf[i]);
  if (crc == buf[bytes - 1]) return true;
  // if error
  bytes = 0; // empty input buffer
  driveStop(); // disable motors
  Serial.println("crc"); // report CRC error
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
  if (commandTimeout == 0) return;
  unsigned long commandTime = millis();
  if (commandTime >= lastCommandTime) commandTime -= lastCommandTime; else lastCommandTime = 0;
  if (commandTime > commandTimeout) OnDisconnected();
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
{ Serial.println("err"); }

// report negative acknowledge
void nak()
{ Serial.println("nak"); }

// execute command in buffer //
// the first character is a command identifier
// the second is a subcommand
void Execute()
{
  bytes = 0; // empty input buffer (only one command at a time)
  cmd = buf[0];
  if (cmd == 0 || cmd == 32) return;
  
  if (cmd == 'd') drive();
  else
  if (cmd == 'p') power();
  else
  if (cmd == 's') servo_maestro();
  else
  if (cmd == 'b') battery();
  else
  if (cmd == 'v') Serial.println("v7");
  else
  if (cmd == 'r') _read();
  else
  if (cmd == 'w') _write();
  else
  if (cmd == 'z') { OnDisconnected(); return; }
  else
  {
    nak(); return;
  }
  
  OnValidCommand();
}

//=======================================












//=======================================

// 'drive' command. both motors in one command
// ex: DD127127 - full forward
// ex: DD064064 - stop
// ex: DD000000 - full backward
void drive()
{
  cmd = buf[1];
  
  if (cmd == 'd') // drive differential
  {
    motor1_target = bctoi(2, 3);
    motor2_target = bctoi(5, 3);
    if (motor1_target > MOTOR_TARGET_MAX) motor1_target = MOTOR_TARGET_MAX;
    if (motor2_target > MOTOR_TARGET_MAX) motor2_target = MOTOR_TARGET_MAX;
  }
  else
  if (cmd == 'r') // drive reset
  {
    driveUART(false);
    pm();
    delay(100);
    pm();
    driveUART(true);
  }
  else
  if (cmd == 'e') driveUART(true);
  else
  if (cmd == 'o') driveUART(false);
  else
  if (cmd == 'i') motor_increment = bctoi(2, 2);
  else
  {
    nak();
  }
}

// update motor velocities
void updateMotors()
{
  if (motor1 == motor1_target && motor2 == motor2_target) return;
  // enable motor driver if it is disabled
  if (!driveEnabled) { driveEnable(true); delay(20); }
  
  // M1
  if (motor1 != motor1_target)
  {
    motor_counter = MOTOR_COUNTER_START;
    if (motor1 < motor1_target)
    {
      if (motor1_target - motor1 > motor_increment) motor1 += motor_increment;
      else motor1 = motor1_target;
    }
    else
    if (motor1 > motor1_target)
    {
      if (motor1 - motor1_target > motor_increment) motor1 -= motor_increment;
      else motor1 = motor1_target;
    }
    roboclaw.ForwardBackwardM1(RC_ADDR, motor1);
    lastDriveCommandTime = millis();
  }
  // M2
  if (motor2 != motor2_target)
  {
    motor_counter = MOTOR_COUNTER_START;
    if (motor2 < motor2_target)
    {
      if (motor2_target - motor2 > motor_increment) motor2 += motor_increment;
      else motor2 = motor2_target;
    }
    else
    if (motor2 > motor2_target)
    {
      if (motor2 - motor2_target > motor_increment) motor2 -= motor_increment;
      else motor2 = motor2_target;
    }
    roboclaw.ForwardBackwardM2(RC_ADDR, motor2);
    lastDriveCommandTime = millis();
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
  lastDriveCommandTime = millis();
}

// enable/disable motor driver communication and power
void driveEnable(boolean action)
{
  if (action)
  {
    pm();
    driveUART(true);
    lastDriveCommandTime = millis();
  }
  else
  {
    driveUART(false);
    pm();
  }
}

// enable/disable motor driver communication
void driveUART(boolean action)
{
  if (action) roboclaw.begin(38400);
  else roboclaw.end();
  driveEnabled = action;
}

void CheckDriveCommandTimeout()
{
  if (driveCommandTimeout == 0) return;
  unsigned long driveCommandTime = millis();
  if (driveCommandTime >= lastDriveCommandTime) driveCommandTime -= lastDriveCommandTime; else lastDriveCommandTime = 0;
  if (driveCommandTime > driveCommandTimeout) driveEnable(false);
}

//=======================================












//=======================================

// ex: sp002000 - set servo 00 to the min position
// ex: sp159999 - set servo 15 to the max position
void servo_maestro()
{
  uint8_t channel;
  uint16_t value;
  cmd = buf[1];
  channel = bctoi(2, 2);
  value = bctoi16(4, 4);
  
  if (cmd == 'p') // servo position
  {
    if (channel < 18) maestro.setTarget(channel, value); else err();
  }
  if (cmd == 's') // servo speed
  {
    if (channel < 18) maestro.setSpeed(channel, value); else err();
  }
  if (cmd == 'a') // servo accel
  {
    if (channel < 18) maestro.setAcceleration(channel, value); else err();
  }
  else
  {
    nak();
  }
}

// ex: sp000500 - set servo 00 to the min position
// ex: sp152500 - set servo 15 to the max position
void servo_ssc32()
{
  uint8_t channel;
  uint16_t pos, time;
  cmd = buf[1];
  
  if (cmd == 'p') // servo pulse
  {
    channel = bctoi(2, 2);
    pos = bctoi16(4, 4);
    time = bctoi16(8, 4);
    if (channel > 31) err();
    else
    {
      ssc.print("#");
      ssc.print(channel);
      ssc.print("P");
      ssc.print(pos);
      if (time > 0) { ssc.print("T"); ssc.print(time); }
      ssc.println("");
    }
  }
  else
  {
    nak();
  }
}

//=======================================












//=======================================

// 'power' command
void power()
{
  cmd = buf[1];
  if (cmd == 'a') pa();
  else
  if (cmd == 'm') pm();
  else
  if (cmd == 'u') pu();
  else
    nak();
}

void pa()
{
  pulsePin(AUX_PIN, 20);
  delay(3000);
  pulsePin(AUX_PIN, 20);
}

void pm()
{
  pulsePin(MOTORS_PIN, 20);
}

void pu()
{
  pulsePin(UPS_PIN, 5500);
}

// 'battery voltage' command
// ex: B
// result in mV
void battery()
{
  digitalWrite(BATTERY_PIN, HIGH);
  delay(20);
  unsigned long voltage = (unsigned long)analogRead(0) * 625 >> 7;
  digitalWrite(BATTERY_PIN, LOW);
  Serial.write('b');
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
  if (cmd == 'd') dread();
  else
  if (cmd == 'a') aread();
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
  Serial.print("rd");
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
  Serial.print("ra");
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

// converts string to number. reads 'count' chars starting at 'index' position in buffer
uint16_t bctoi16(byte index, byte count)
{
  uint16_t i = 0;
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
