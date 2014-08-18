#include <BMSerial.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define LED_PIN 13

// serial input //
#define MAX_COMMAND 10
#define BUF_SIZE 32
byte buf[BUF_SIZE];
byte cmd = 0;
byte bytes = 0;
byte crc = 0;

// RoboClaw //
#define RC_ADDR 0x80
#define RC_RX_PIN 12
#define RC_TX_PIN 11
RoboClaw roboclaw(RC_RX_PIN, RC_TX_PIN);

//
#define AUX_POWER_PIN 7
#define MOTOR_POWER_PIN 8
#define UPS_POWER_PIN 9

// common vars //
unsigned long lastCommandTime;
unsigned long commandTime;
unsigned long cmdTimeout = 2500;
boolean ledCmdOn = false;
boolean IsConnected = false;

// drive vars
#define MOTOR_VELOCITY_0 64
#define MOTOR_VELOCITY_MAX 63
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
  // setup pins to lower power consumption
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  
  pinMode(MOTOR_POWER_PIN, OUTPUT);
  digitalWrite(MOTOR_POWER_PIN, LOW);
  pinMode(UPS_POWER_PIN, OUTPUT);
  digitalWrite(UPS_POWER_PIN, LOW);
  pinMode(AUX_POWER_PIN, OUTPUT);
  digitalWrite(AUX_POWER_PIN, HIGH);
  delay(20);
  digitalWrite(AUX_POWER_PIN, LOW);
  
  // start ports
  roboclaw.begin(38400);
  Serial.begin(38400);
}

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
      if (bytes > 0) Execute();
			//if (bytes > 0) if (crc8_ok()) Execute();
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
  if (ledCmdOn) switchCommandLed();
}

void OnValidCommand()
{
  IsConnected = true;
  lastCommandTime = millis();
  switchCommandLed();
}

void switchCommandLed()
{
  ledCmdOn = !ledCmdOn;
  //if (ledCmdOn) digitalWrite(LED_PIN, HIGH); else digitalWrite(LED_PIN, LOW);
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
  if (cmd == 'S' || cmd == 's') settings();
  else
  if (cmd == 'B' || cmd == 'b') battery();
  else
  if (cmd == 'R' || cmd == 'r')
  {
    cmd = buf[1];
    if (cmd == 'D' || cmd == 'd') dread();
    else
    if (cmd == 'A' || cmd == 'a') aread();
    else
    {
      nak(); return;
    }
  
    OnValidCommand();
    return;
  }
  else
  if (cmd == 'W' || cmd == 'w') dwrite();
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
// ex: DD127127 - drive differential. full forward at velocity 63
// ex: DD000000 - drive differential. full backward at vel 64
void drive()
{
  cmd = buf[1];
  
  if (cmd == 'D' || cmd == 'd') // drive differential
  {
    motor1_target = bctoi(2, 3);
    motor2_target = bctoi(5, 3);
    if (motor1_target > MOTOR_VELOCITY_0 + MOTOR_VELOCITY_MAX) motor1_target = MOTOR_VELOCITY_0 + MOTOR_VELOCITY_MAX;
    if (motor2_target > MOTOR_VELOCITY_0 + MOTOR_VELOCITY_MAX) motor2_target = MOTOR_VELOCITY_0 + MOTOR_VELOCITY_MAX;
  }
  else
  if (cmd == 'Z' || cmd == 'z') // stop immediately
	{
		motor1_target = MOTOR_VELOCITY_0;
		motor1 = MOTOR_VELOCITY_0;
		motor2_target = MOTOR_VELOCITY_0;
		motor2 = MOTOR_VELOCITY_0;
		roboclaw.ForwardBackwardM1(RC_ADDR, motor1);
		roboclaw.ForwardBackwardM2(RC_ADDR, motor2);
	}
  else
  if (cmd == 'F' || cmd == 'f') { motor1_target = MOTOR_VELOCITY_0 + motor_velocity; motor2_target = motor1_target; }
  else
  if (cmd == 'B' || cmd == 'b') { motor1_target = MOTOR_VELOCITY_0 - motor_velocity; motor2_target = motor1_target; }
  else
  if (cmd == 'L' || cmd == 'l') { motor1_target = MOTOR_VELOCITY_0 - motor_velocity; motor2_target = MOTOR_VELOCITY_0 + motor_velocity; }
  else
  if (cmd == 'R' || cmd == 'r') { motor1_target = MOTOR_VELOCITY_0 + motor_velocity; motor2_target = MOTOR_VELOCITY_0 - motor_velocity; }
  else
  if (cmd == 'S' || cmd == 's') { driveStop(); }
  else
  if (cmd == 'V' || cmd == 'v')
  {
    motor_velocity = bctoi(2, 2);
    if (motor_velocity > MOTOR_VELOCITY_MAX) motor_velocity = MOTOR_VELOCITY_MAX;
  }
  else
  if (cmd == 'I' || cmd == 'i') { motor_velocity_increment = bctoi(2, 2); }
  else
  if (cmd == 'C' || cmd == 'c') { motor_counter_start = bctoi(2, 1) * 1000; }
  else
  {
    nak();
  }
}

// stop both motors
void driveStop()
{
  motor1_target = MOTOR_VELOCITY_0;
  motor2_target = MOTOR_VELOCITY_0;
}

// update motor velocities
void updateMotors()
{
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

// 'version' command
// ex: V
void ver()
{
  Serial.println("V2");
}

// power control command
// ex: PU - full shutdown (UPS shutdown)
void power()
{
  cmd = buf[1];
  if (cmd == 'A' || cmd == 'a') { digitalWrite(AUX_POWER_PIN, HIGH); delay(50); digitalWrite(AUX_POWER_PIN, LOW); delay(2000); digitalWrite(AUX_POWER_PIN, HIGH); delay(50); digitalWrite(AUX_POWER_PIN, LOW); }
  else
  if (cmd == 'M' || cmd == 'm') { digitalWrite(MOTOR_POWER_PIN, HIGH); delay(50); digitalWrite(MOTOR_POWER_PIN, LOW); }
  else
  if (cmd == 'U' || cmd == 'u') { digitalWrite(UPS_POWER_PIN, HIGH); delay(5500); digitalWrite(UPS_POWER_PIN, LOW); }
}

// 'settings' command
// ex: ST2500 - set timeout to 2500ms. 0 to disable
void settings()
{
  cmd = buf[1];
  // command timeout
  if (cmd == 'T' || cmd == 't') { cmdTimeout = bctoi(2, 5); Serial.println(cmdTimeout); }
}

// 'battery voltage' command. result in mV
// ex: B
void battery()
{
  //unsigned int voltage = (unsigned int)roboclaw.ReadMainBatteryVoltage(RC_ADDR, 0);
  unsigned int voltage = (unsigned int)analogRead(0) * 312 >> 6;
  Serial.write('B');
  Serial.println(voltage);
}

// 'digitalWrite' command
// ex: "W031" or "W3 1" (write 1 to pin 3)
void dwrite()
{
  byte pin = bctoi(1, 2);
  if (pin < 2 || pin > 10) { err(); return; }
  pinMode(pin, OUTPUT);
  if (buf[3] == '0') digitalWrite(pin, 0);
	else digitalWrite(pin, 1);
}

// 'digitalRead' command
// ex: RD5 or RD10
void dread()
{
  byte pin = bctoi(2, 2);
  if (pin < 2 || pin > 10) { err(); return; }
  pinMode(pin, INPUT_PULLUP);
  unsigned int a;
  a = digitalRead(pin);
  Serial.print("RD");
  Serial.print((unsigned int) pin);
  Serial.print("=");
  Serial.println(a);
}

// 'analogRead' command
// ex: RA0
void aread()
{
  byte pin = bctoi(2, 1);
  if (pin < 0 || pin > 7) { err(); return; }
  unsigned long a;
  a = (unsigned long) analogRead(pin);
  a = a * 5000 / 1024;
  Serial.print("RA");
  Serial.print((unsigned int) pin);
  Serial.print("=");
  Serial.println(a);
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
