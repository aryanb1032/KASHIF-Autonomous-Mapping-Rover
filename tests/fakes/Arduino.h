#pragma once
#include <stdint.h>
#include <stddef.h>
#include <initializer_list>
#include <cmath>
#include <string>
#include <cstdarg>
#include <cstdio>
#define IRAM_ATTR
#define DRAM_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define CHANGE 3
#define portMUX_INITIALIZER_UNLOCKED 0
using portMUX_TYPE=int;
inline void portENTER_CRITICAL_ISR(int*){}
inline void portEXIT_CRITICAL_ISR(int*){}
inline void portENTER_CRITICAL(int*){}
inline void portEXIT_CRITICAL(int*){}
inline uint32_t fake_ms=0;
inline int fake_pwm[2]={0,0};
inline int fake_pins[40]={};
inline uint32_t millis(){return fake_ms;}
inline void delay(uint32_t d){fake_ms+=d;}
inline void pinMode(int,int){}
inline void digitalWrite(int p,int v){fake_pins[p]=v;}
inline int digitalRead(int p){return fake_pins[p];}
inline void ledcSetup(int,int,int){}
inline void ledcAttachPin(int,int){}
inline void ledcWrite(int c,uint32_t v){fake_pwm[c]=v;}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
struct FakeSerial {
  std::string input,output;
  void begin(int){}
  int available(){return input.size();}
  char read(){char c=input.front();input.erase(0,1);return c;}
  void printf(const char* fmt,...){char b[512];va_list a;va_start(a,fmt);vsnprintf(b,sizeof(b),fmt,a);va_end(a);output+=b;}
};
inline FakeSerial Serial;
