// CRC-framed serial: C,left_ticks_per_second,right_ticks_per_second*CRC16\n
// Telemetry: T,ms,left_ticks,right_ticks,imu_ok,fault,ax,ay,az,gx,gy,gz*CRC16\n
#include <Arduino.h>
#include <Wire.h>
#include "control.h"
#include "hardware_config.h"

volatile uint32_t ticksL=0, ticksR=0;
volatile uint8_t prevL=0,prevR=0;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
DRAM_ATTR const int8_t quad[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
void IRAM_ATTR encL() {
  portENTER_CRITICAL_ISR(&mux);
  uint8_t state=(digitalRead(ENC_L_A)<<1)|digitalRead(ENC_L_B);
  ticksL += ENCODER_L_SIGN*quad[(prevL<<2)|state]; prevL=state;
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR encR() {
  portENTER_CRITICAL_ISR(&mux);
  uint8_t state=(digitalRead(ENC_R_A)<<1)|digitalRead(ENC_R_B);
  ticksR += ENCODER_R_SIGN*quad[(prevR<<2)|state]; prevR=state;
  portEXIT_CRITICAL_ISR(&mux);
}
float targetL=0,targetR=0;
uint32_t lastCmd=0,lastStep=0,lastL=0,lastR=0,stallSince=0;
bool fault=false, imuReady=false;
SpeedLoop speedL,speedR;
char input[128]; size_t inputSize=0; bool overflow=false;

void stop() {
  targetL=targetR=0; speedL.integral=speedR.integral=0;
  ledcWrite(0,0); ledcWrite(1,0);
}
void motor(int channel,int dir,float duty,int sign) {
  duty*=sign;
  ledcWrite(channel,0);
  digitalWrite(dir,duty>=0?HIGH:LOW);
  ledcWrite(channel,(uint32_t)lroundf(fabsf(duty)));
}
bool regWrite(uint8_t reg,uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission()==0;
}
bool readImu(float* v) {
  if(!imuReady) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(uint8_t(MPU_ADDR),size_t(14),true)!=14) return false;
  int16_t raw[7];
  for(int i=0;i<7;++i) { uint8_t high=Wire.read(); uint8_t low=Wire.read(); raw[i]=(int16_t)((uint16_t(high)<<8)|low); }
  for(int i=0;i<3;++i) v[i]=raw[i]*(9.80665f/16384.0f);
  for(int i=0;i<3;++i) v[i+3]=raw[i+4]*(0.01745329252f/131.0f);
  return true;
}
void setup() {
  pinMode(DIR_L,OUTPUT); pinMode(DIR_R,OUTPUT);
  ledcSetup(0,20000,8); ledcAttachPin(PWM_L,0);
  ledcSetup(1,20000,8); ledcAttachPin(PWM_R,1); stop();
  Serial.begin(115200);
  for(int pin: {ENC_L_A,ENC_L_B,ENC_R_A,ENC_R_B}) pinMode(pin,INPUT);
  prevL=(digitalRead(ENC_L_A)<<1)|digitalRead(ENC_L_B);
  prevR=(digitalRead(ENC_R_A)<<1)|digitalRead(ENC_R_B);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A),encL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B),encL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A),encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B),encR,CHANGE);
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(10);
  imuReady=regWrite(0x6B,0x01); delay(100);
  imuReady=regWrite(0x1B,0x00)&&regWrite(0x1C,0x00)&&regWrite(0x1A,0x03)&&imuReady;
  lastCmd=lastStep=millis();
}
void loop() {
  // Bound serial work so a flood cannot starve the motor watchdog.
  for(int n=0;n<256 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\n') {
      input[inputSize]='\0'; float left=0,right=0;
      if(!overflow && parseCommand(input,left,right)) {
        if(fabsf(left)<1 && fabsf(right)<1) { fault=false; stallSince=0; }
        if(!fault) { targetL=left; targetR=right; lastCmd=millis(); }
      } else { stop(); }
      inputSize=0; overflow=false;
    } else if(c!='\r') {
      if(inputSize<sizeof(input)-1 && !overflow) input[inputSize++]=c;
      else { overflow=true; stop(); }
    }
  }
  uint32_t now=millis();
  if(now-lastCmd>250 || fault) stop();
  if(now-lastStep>=20) {
    float dt=(now-lastStep)/1000.0f; lastStep=now;
    portENTER_CRITICAL(&mux); uint32_t l=ticksL,r=ticksR; portEXIT_CRITICAL(&mux);
    float vl=(int32_t)(l-lastL)/dt,vr=(int32_t)(r-lastR)/dt; lastL=l;lastR=r;
    float ul=speedL.step(targetL,vl,dt),ur=speedR.step(targetR,vr,dt);
    bool stalled=(fabsf(targetL)>100 && fabsf(vl)<2 && fabsf(ul)>80) ||
                 (fabsf(targetR)>100 && fabsf(vr)<2 && fabsf(ur)>80);
    if(stalled) { if(!stallSince) stallSince=now; if(now-stallSince>750) { fault=true; stop(); ul=ur=0; } }
    else stallSince=0;
    motor(0,DIR_L,ul,MOTOR_L_SIGN); motor(1,DIR_R,ur,MOTOR_R_SIGN);
    float imu[6]={0}; bool valid=readImu(imu);
    char output[220];
    int count=snprintf(output,sizeof(output),"T,%lu,%ld,%ld,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f",
      (unsigned long)now,(long)(int32_t)l,(long)(int32_t)r,valid?1:0,fault?1:0,
      imu[0],imu[1],imu[2],imu[3],imu[4],imu[5]);
    if(count>0 && count<int(sizeof(output))) Serial.printf("%s*%04X\n",output,crc16(output,count));
  }
}
