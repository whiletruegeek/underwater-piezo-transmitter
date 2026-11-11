#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

namespace cal {
constexpr unsigned Count=37, Passes=5, Samples=1024;
constexpr uint32_t Magic=0x31435A50, Schema=1;
inline uint32_t crc32(const void* data,size_t n) {
  uint32_t c=0xffffffff; const auto* p=(const uint8_t*)data;
  while(n--) {c^=*p++; for(int i=0;i<8;++i)c=(c>>1)^((c&1)?0xedb88320:0);}
  return ~c;
}
enum Op:uint32_t { Hello=1, Start=2, Tone=3, Prepare=4, Commit=5, Finish=6, Ack=128 };
struct Packet {
  uint32_t magic=Magic, version=Schema, role=0, session=0, seq=0, op=0;
  uint32_t a=0,b=0,c=0,d=0,crc=0;
  void seal(){crc=crc32(this,offsetof(Packet,crc));}
  bool valid()const{return magic==Magic&&version==Schema&&crc==crc32(this,offsetof(Packet,crc));}
};
static_assert(sizeof(Packet)==44,"wire size");
// Sliding parser recovers from dropped bytes, line noise and partial frames.
struct Parser {
  uint8_t bytes[sizeof(Packet)]={}; size_t n=0; uint32_t errors=0;
  bool push(uint8_t b,Packet& out) {
    bytes[n++]=b; if(n<sizeof(Packet))return false;
    memcpy(&out,bytes,sizeof(out));
    if(out.valid()){n=0;return true;}
    memmove(bytes,bytes+1,--n); ++errors; return false;
  }
};
struct Profile {
  uint32_t magic=Magic, version=Schema, id=0, f0=0,f1=0,amp0=0,amp1=0,crc=0;
  void seal(){crc=crc32(this,offsetof(Profile,crc));}
  bool valid()const{return magic==Magic&&version==Schema&&id&&f0>=1000&&f0<=9500&&f1>=1500&&f1<=10000&&f1>=f0+500&&amp0&&amp1&&crc==crc32(this,offsetof(Profile,crc));}
};
inline unsigned frequency(unsigned i){return 1000+250*i;}
inline float amplitude(const uint16_t* x,unsigned n,float hz,float rate) {
  double sum=0; for(unsigned i=0;i<n;++i)sum+=x[i]; const float mean=sum/n;
  const float coeff=2*cosf(6.28318530718f*hz/rate);
  float q1=0,q2=0,weights=0;
  for(unsigned i=0;i<n;++i){
    const float w=.5f-.5f*cosf(6.28318530718f*i/(n-1)); weights+=w;
    const float q=(x[i]-mean)*w+coeff*q1-q2; q2=q1;q1=q;
  }
  const float power=q1*q1+q2*q2-coeff*q1*q2;
  return 2*sqrtf(power>0?power:0)/weights;
}
struct Summary {
  float minAmp=1e9f,maxAmp=0,noise=1; unsigned trials=0;
  void add(float lo,float hi,float bg){if(lo<minAmp)minAmp=lo;if(hi>maxAmp)maxAmp=hi;if(bg>noise)noise=bg;++trials;}
  bool good()const{return trials==Passes&&minAmp>=8&&minAmp>=4*noise&&maxAmp<=3*minAmp;}
  float score()const{return minAmp/noise;}
};
inline bool choose(const Summary* s,unsigned& a,unsigned& b) {
  float best=0; bool found=false;
  for(unsigned i=0;i<Count;++i)for(unsigned j=i+2;j<Count;++j){
    if(!s[i].good()||!s[j].good())continue;
    const float score=s[i].score()<s[j].score()?s[i].score():s[j].score();
    if(score>best){best=score;a=i;b=j;found=true;}
  }
  return found;
}
}
