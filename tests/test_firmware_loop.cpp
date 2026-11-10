#include "../firmware/rover_esp32/rover_esp32.ino"
#include <assert.h>
void command(const char* payload){char b[180];snprintf(b,sizeof(b),"%s*%04X\n",payload,crc16(payload,strlen(payload)));Serial.input+=b;}
void step(uint32_t ms){fake_ms+=ms;loop();}
int main(){
  setup();assert(fake_pwm[0]==0 && fake_pwm[1]==0);
  command("C,500,500");step(20);assert(fake_pwm[0]>0 && fake_pwm[1]>0);
  step(251);assert(fake_pwm[0]==0 && fake_pwm[1]==0);
  command("C,500,500");step(20);Serial.input="C,9000,9000*0000\n";step(20);assert(fake_pwm[0]==0);
  command("C,500,500");step(20);Serial.input=std::string(300,'x');step(20);assert(fake_pwm[0]==0);
  Serial.input="\n";step(20);command("C,0,0");step(20);
  // Actual ISR decoder: one full quadrature cycle yields four counts.
  fake_pins[ENC_L_B]=1;encL();fake_pins[ENC_L_A]=1;encL();fake_pins[ENC_L_B]=0;encL();fake_pins[ENC_L_A]=0;encL();assert(ticksL==4);
  for(int i=0;i<80;++i){command("C,3000,3000");step(20);}
  assert(fault);assert(fake_pwm[0]==0 && fake_pwm[1]==0);
  command("C,0,0");step(20);assert(!fault);
  // Invalid IMU is marked unavailable, rather than fabricated zero measurements.
  assert(Serial.output.find(",0,0,0.00000,")!=std::string::npos);
}
