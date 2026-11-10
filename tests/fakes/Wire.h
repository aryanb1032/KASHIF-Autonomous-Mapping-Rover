#pragma once
#include <stddef.h>
#include <stdint.h>
struct FakeWire {
  void begin(int,int){} void setClock(int){} void setTimeOut(int){}
  void beginTransmission(int){} void write(uint8_t){}
  int endTransmission(bool=true){return 4;}
  int requestFrom(uint8_t,size_t,bool){return 0;}
  int read(){return 0;}
};
inline FakeWire Wire;
