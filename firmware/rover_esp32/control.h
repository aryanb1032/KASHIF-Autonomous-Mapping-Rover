#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

inline uint16_t crc16(const char* text, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i=0; i<n; ++i) {
    crc ^= uint16_t(uint8_t(text[i])) << 8;
    for (int j=0; j<8; ++j) crc = (crc & 0x8000) ? (crc<<1)^0x1021 : crc<<1;
  }
  return crc;
}
inline bool parseCommand(char* line, float& left, float& right) {
  char* star = strrchr(line, '*');
  if (!star || strlen(star+1)!=4) return false;
  for (char* p=star+1; *p; ++p) if (!((*p>='0'&&*p<='9')||(*p>='A'&&*p<='F')||(*p>='a'&&*p<='f'))) return false;
  if (strtoul(star+1,nullptr,16)!=crc16(line,star-line)) return false;
  *star='\0'; int used=0;
  if (sscanf(line,"C,%f,%f%n",&left,&right,&used)!=2 || used!=int(strlen(line))) return false;
  return isfinite(left)&&isfinite(right)&&fabsf(left)<=20000&&fabsf(right)<=20000;
}
struct SpeedLoop {
  float integral=0;
  float step(float target, float measured, float dt) {
    if (fabsf(target)<1) { integral=0; return 0; }
    // PWM per tick/s; these gains require tuning for the physical motor and supply.
    float err=target-measured;
    float proposed=fmaxf(-70,fminf(70,integral+0.06f*err*dt));
    float output=0.0304f*target+0.03f*err+proposed;
    if (fabsf(output)<140 || err*output<0) integral=proposed;
    return fmaxf(-140,fminf(140,0.0304f*target+0.03f*err+integral));
  }
};
