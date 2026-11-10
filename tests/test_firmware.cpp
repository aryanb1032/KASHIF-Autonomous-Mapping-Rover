#include "../firmware/rover_esp32/control.h"
#include <assert.h>
#include <string>
static bool parse(std::string payload) {
  char suffix[8]; snprintf(suffix,sizeof(suffix),"*%04X",crc16(payload.c_str(),payload.size()));
  payload+=suffix;float l=0,r=0;return parseCommand(&payload[0],l,r);
}
int main(){
  assert(crc16("123456789",9)==0x29B1);
  assert(parse("C,100,-200"));assert(parse("C,0,0"));
  assert(!parse("C,nan,0"));assert(!parse("C,inf,1"));
  assert(!parse("C,0,0junk"));assert(!parse("C,20001,0"));assert(!parse("C,,1"));
  char bad[]="C,1,2*0000";float l,r;assert(!parseCommand(bad,l,r));
  SpeedLoop loop;
  for(int i=0;i<10000;++i) assert(fabsf(loop.step(20000,-20000,.02f))<=140);
  assert(loop.step(0,1000,.02f)==0);assert(loop.integral==0);
}
