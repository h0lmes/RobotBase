#include <BMSerial.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <PololuMaestro.h>
#include <Adafruit_INA219.h>
#include <TM1650.h>

#define LED_PIN 13
#define RC_RX_PIN 12
#define RC_TX_PIN 11
#define SSC_RX_PIN 10
#define SSC_TX_PIN 9
// power management //
#define SERVO_PIN 5
#define POWER_PIN 6
#define MOTOR_ENABLE_PIN 7
#define MOTOR_DISABLE_PIN 8

// host communication //
#define MAX_COMMAND 16
#define BUF_SIZE 18
byte buf[BUF_SIZE];
byte cmd = 0;
byte bytes = 0;
byte crc = 0;

Adafruit_INA219 ina219;
TM1650 tm;

// common vars //
unsigned long lastCommandTime;
unsigned long commandTimeout = 2500;
boolean IsConnected = false;
unsigned long lastTmTime = 0;
int tmMode = 0;
#define DISPLAY_ALTER_INTERVAL 1500

// RoboClaw //
#define RC_ADDR 0x80
RoboClaw roboclaw(RC_RX_PIN, RC_TX_PIN);
boolean driveEnabled = false;
unsigned long lastDriveCommandTime;
unsigned long driveCommandTimeout = 20000;
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

// servo controller
BMSerial ssc(SSC_RX_PIN, SSC_TX_PIN);
MiniMaestro maestro(ssc);

//=======================================

void setup() 
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // power management //
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(MOTOR_DISABLE_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(POWER_PIN, LOW);
  digitalWrite(MOTOR_DISABLE_PIN, LOW);
  motor_power(false);
  
  // TM1650 display setup
  Wire.begin();
  tm.init();
  tm.displayOff();
  tm.displayString("INIT");
  tm.setBrightness(1);
  tm.displayOn();
  
  // host communication //
  Serial.begin(38400);
  
  // motor driver setup //
  driveUART(true);
  driveUART(false);
  
  // INA219 setup //
  ina219.begin();
  
  // ssc setup //
  sscUART(false);
}

//=======================================

void loop()
{
  // serial input
  ReadSerial();
  if (IsConnected) CheckCommandTimeout();
  else UpdateVA();
  if (driveEnabled) CheckDriveCommandTimeout();
  
  // motors
  if (motor_counter > 0) motor_counter--;
  if (motor_counter == 0) updateMotors();
}

void UpdateVA()
{
  unsigned long tmTime = millis();
  if (tmTime < lastTmTime) lastTmTime = 0;
  if (tmTime - lastTmTime > DISPLAY_ALTER_INTERVAL)
  {
    if (tmMode == 0)
    {
      float shuntvoltage = ina219.getShuntVoltage_mV();
      float busvoltage = ina219.getBusVoltage_V();
      int voltage = (int)(busvoltage * 100 + shuntvoltage / 10);
      displayInt(voltage, 2);
      tmMode = 1;
    }
    else
    if (tmMode == 1)
    {
      int mA = abs(int(ina219.getCurrent_mA()));
      displayInt(mA, 1);
      tmMode = 0;
    }
    
    lastTmTime = tmTime;
  }
}

void displayInt(int value, int dot)
{
    char c[4];
    c[0] = '0' + int(value / 1000);
    c[1] = '0' + int(value % 1000 / 100);
    c[2] = '0' + int(value % 100 / 10);
    c[3] = '0' + int(value % 10);
    if (dot == 1) c[0] = c[0] | 0b10000000;
    else
    if (dot == 2) c[1] = c[1] | 0b10000000;
    else
    if (dot == 3) c[2] = c[2] | 0b10000000;
    tm.displayString(c);
}

void ina()
{
  uint16_t reg;
  cmd = buf[1]; 
  if (cmd == '0') ina219.wireReadRegister(0, &reg);
  else if (cmd == '1') ina219.wireReadRegister(1, &reg);
  else if (cmd == '2') ina219.wireReadRegister(2, &reg);
  else if (cmd == '3') ina219.wireReadRegister(3, &reg);
  else if (cmd == '4') ina219.wireReadRegister(4, &reg);
  else if (cmd == '5') ina219.wireReadRegister(5, &reg);
  Serial.print('i');
  Serial.println(reg);
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
  driveEnable(false);
  IsConnected = false;
}

void OnValidCommand()
{
  IsConnected = true;
  lastCommandTime = millis();
  tm.displayString("CONN");
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
  if (cmd == 's') servo();
  else
  if (cmd == 'b') battery();
  else
  if (cmd == 'i') ina();
  else
  if (cmd == 'v') Serial.println("v9");
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

// 'power off' command
void power()
{
  pulsePin(POWER_PIN, 20);
}

//=======================================

// 'battery voltage' command
// result in mV and mA
void battery()
{
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  int current_mA = int(ina219.getCurrent_mA());
  unsigned long voltage = (unsigned long)(busvoltage * 1000 + shuntvoltage);
  Serial.write('b');
  Serial.println(voltage);
  Serial.write('a');
  Serial.println(current_mA);
}

//=======================================












//=======================================

// 'drive' command
// ex: dm127127 - full forward
// ex: dm064064 - stop
// ex: dm000000 - full backward
void drive()
{
  cmd = buf[1]; 
  if (cmd == 'm') // drive motors
  {
    motor1_target = bctoi(2, 3);
    motor2_target = bctoi(5, 3);
    if (motor1_target > MOTOR_TARGET_MAX) motor1_target = MOTOR_TARGET_MAX;
    if (motor2_target > MOTOR_TARGET_MAX) motor2_target = MOTOR_TARGET_MAX;
  }
  else
  if (cmd == 'r') driveReset();
  else nak();
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

void driveReset()
{
  driveEnable(false);
  delay(100);
  driveEnable(true);
}

// enable/disable motor driver communication and power
void driveEnable(boolean action)
{
  if (action)
  {
      motor_power(true);
      driveUART(true);
      lastDriveCommandTime = millis();
  }
  else
  {
      driveUART(false);
      motor_power(false);
  }
}

// enable/disable motor driver communication
void driveUART(boolean action)
{
  if (action) roboclaw.begin(38400);
  else roboclaw.end();
  driveEnabled = action;
}

void motor_power(boolean action)
{
  if (action)
  {
    pulsePin(MOTOR_DISABLE_PIN, 20);
    delay(20);
    pulsePin(MOTOR_ENABLE_PIN, 20);
  }
  else pulsePin(MOTOR_DISABLE_PIN, 20);
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
void servo()
{
  uint8_t channel;
  uint16_t value;
  cmd = buf[1];
  channel = bctoi(2, 2);
  value = bctoi16(4, 4);
  
  if (channel >= 18) // channels 18 and above = all servos
  {
    if (cmd == 's') // servo speed
    {
      for (channel = 0; channel < 18; channel++) maestro.setSpeed(channel, value);
    }
    else
    if (cmd == 'a') // servo accel
    {
      for (channel = 0; channel < 18; channel++) maestro.setAcceleration(channel, value);
    }
    else
      nak();
    return;
  }
  
  if (cmd == 'p') maestro.setTarget(channel, value); // servo position
  else
  if (cmd == 's') maestro.setSpeed(channel, value); // servo speed
  else
  if (cmd == 'a') maestro.setAcceleration(channel, value); // servo accel
  else
  if (cmd == 'e') sscEnable(true); // enable servo controller
  else
  if (cmd == 'd') sscEnable(false); // disable servo controller
  else
    nak();
}

// enable/disable servo controller communication and power
void sscEnable(boolean action)
{
  if (action)
  {
    digitalWrite(SERVO_PIN, HIGH);
    sscUART(true);
  }
  else
  {
    sscUART(false);
    digitalWrite(SERVO_PIN, LOW);
  }
}

// enable/disable servo controller communication
void sscUART(boolean action)
{
  if (action) ssc.begin(38400);
  else ssc.end();
}

//=======================================












//=======================================

// 'digitalWrite' command
// W031 - set pin 3 HIGH
// W100 - set pin 10 LOW
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
// RD5 or RD05 - read pin 5
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
// RA0 - read analog pin 0
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
//
//
//
// helper functions
//
//
//
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
