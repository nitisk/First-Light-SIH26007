/* First Light: Arduino Uno R3 / ATmega328P, Arduino AVR core >= 1.8.6.
 * USB serial at 115200. No third-party Arduino libraries.
 * Forward-only differential drive, TB6612FNG, HC-SR04, two encoders,
 * three analog reflectance sensors and MPU6050 through an I2C level shifter.
 * Power-up is stopped. Every motor command expires after 300 ms.
 */
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>

const uint8_t ENC_L=2, ENC_R=3, STBY=4, PWM_L=5, PWM_R=6;
const uint8_t IN_L1=7, IN_L2=8, IN_R1=9, IN_R2=10, TRIG=11, ECHO=12;
const uint8_t ESTOP=A3, FOG=13;
const uint8_t MPU=0x68;
const uint8_t MAX_PWM=110;
volatile uint32_t ticksL=0, ticksR=0;
uint32_t lastCommand=0, lastSample=0, sampleSeq=0;
uint8_t wantedL=0, wantedR=0;
bool commandValid=false, emergencyLatched=false, imuOK=false;
long rangeMM=-1;
char input[64];
uint8_t used=0;
bool overflowed=false;

void leftPulse() { ++ticksL; }
void rightPulse() { ++ticksR; }

void stopMotors() {
  analogWrite(PWM_L,0); analogWrite(PWM_R,0);
  digitalWrite(STBY,LOW);
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}

int16_t readWord() { uint16_t hi=Wire.read(); return (int16_t)((hi<<8)|Wire.read()); }

bool readIMU(int &ax, int &ay, int &az, int &gz) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false)!=0) return false;
  if (Wire.requestFrom(MPU,(uint8_t)14)!=(uint8_t)14) return false;
  int16_t x=readWord(), y=readWord(), z=readWord();
  readWord(); readWord(); readWord(); // temperature, gx, gy
  int16_t yaw=readWord();
  ax=(long)x*1000/16384; ay=(long)y*1000/16384; az=(long)z*1000/16384;
  gz=(long)yaw*100/131;
  return true;
}

bool unsignedNumber(const char *s, uint32_t &out) {
  if (!s || !*s) return false;
  out=0;
  for (; *s; ++s) {
    if (*s<'0' || *s>'9') return false;
    uint8_t digit=*s-'0';
    if (out>429496729UL || (out==429496729UL && digit>5)) return false;
    out=out*10+digit;
  }
  return true;
}

void processCommand() {
  // strsep-like manual split preserves empty fields and rejects trailing data.
  char *parts[4]; uint8_t n=0; parts[n++]=input;
  for (char *p=input; *p; ++p) {
    if (*p==',') {
      if (n>=4) { commandValid=false; stopMotors(); return; }
      *p=0; parts[n++]=p+1;
    }
  }
  uint32_t seq=0, left=0, right=0;
  bool valid=n==4 && strcmp(parts[0],"D")==0 && unsignedNumber(parts[1],seq)
             && unsignedNumber(parts[2],left) && unsignedNumber(parts[3],right)
             && left<=MAX_PWM && right<=MAX_PWM;
  if (!valid) { commandValid=false; stopMotors(); return; }
  wantedL=left; wantedR=right; lastCommand=millis(); commandValid=true;
}

void receiveCommands() {
  // Bound work per loop so serial flooding cannot starve the watchdog.
  for (uint8_t count=0; count<64 && Serial.available(); ++count) {
    char ch=Serial.read();
    if (ch=='\n') {
      if (!overflowed) { input[used]=0; processCommand(); }
      else { commandValid=false; stopMotors(); }
      used=0; overflowed=false;
    } else if (ch!='\r') {
      if (used<sizeof(input)-1 && !overflowed) input[used++]=ch;
      else { overflowed=true; commandValid=false; stopMotors(); }
    }
  }
}

void setup() {
  wdt_disable();
  pinMode(STBY,OUTPUT); pinMode(PWM_L,OUTPUT); pinMode(PWM_R,OUTPUT); stopMotors();
  pinMode(IN_L1,OUTPUT); pinMode(IN_L2,OUTPUT); pinMode(IN_R1,OUTPUT); pinMode(IN_R2,OUTPUT);
  digitalWrite(IN_L1,HIGH); digitalWrite(IN_L2,LOW);
  digitalWrite(IN_R1,HIGH); digitalWrite(IN_R2,LOW);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);
  pinMode(ENC_L,INPUT_PULLUP); pinMode(ENC_R,INPUT_PULLUP);
  // Normally-closed emergency contact to ground: open wire/button is a stop.
  pinMode(ESTOP,INPUT_PULLUP);
  // D13 uses an EXTERNAL 10k pull-up; switch closes to GND for fog demonstration.
  pinMode(FOG,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L),leftPulse,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R),rightPulse,RISING);
  Serial.begin(115200);
  Wire.begin(); Wire.setWireTimeout(25000,true);
  imuOK=writeRegister(0x6B,0) && writeRegister(0x1B,0) && writeRegister(0x1C,0);
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  if (digitalRead(ESTOP)==HIGH) emergencyLatched=true;
  receiveCommands();
  uint32_t now=millis();
  if (now-lastSample>=100) {
    lastSample=now;
    digitalWrite(TRIG,LOW); delayMicroseconds(2);
    digitalWrite(TRIG,HIGH); delayMicroseconds(10); digitalWrite(TRIG,LOW);
    unsigned long echo=pulseIn(ECHO,HIGH,24000UL);
    rangeMM=echo ? (long)(echo*343UL/2000UL) : -1;
    if (rangeMM<20 || rangeMM>4000) rangeMM=-1;
    int ax=0,ay=0,az=0,gz=0;
    imuOK=readIMU(ax,ay,az,gz);
    if (digitalRead(ESTOP)==HIGH) emergencyLatched=true;
    // Apply local guards before potentially blocking serial output.
    if (emergencyLatched || !imuOK || rangeMM<120) stopMotors();
    noInterrupts(); uint32_t left=ticksL,right=ticksR; interrupts();
    Serial.print(F("S,")); Serial.print(sampleSeq++); Serial.print(','); Serial.print(now);
    Serial.print(','); Serial.print(rangeMM); Serial.print(','); Serial.print(left); Serial.print(','); Serial.print(right);
    for (uint8_t pin=A0; pin<=A2; ++pin) { Serial.print(','); Serial.print(analogRead(pin)); }
    Serial.print(','); Serial.print(emergencyLatched ? 1 : 0);
    Serial.print(','); Serial.print(digitalRead(FOG)==LOW ? 1 : 0);
    Serial.print(','); Serial.print(imuOK ? 1 : 0);
    Serial.print(','); Serial.print(ax); Serial.print(','); Serial.print(ay);
    Serial.print(','); Serial.print(az); Serial.print(','); Serial.println(gz);
  }
  if (!commandValid || millis()-lastCommand>300 || emergencyLatched || !imuOK || rangeMM<120) {
    stopMotors();
  } else {
    digitalWrite(STBY,HIGH); analogWrite(PWM_L,wantedL); analogWrite(PWM_R,wantedR);
  }
}
