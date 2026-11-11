#pragma once
// ESP-NOW control protocol for PZ016 TX / PZ017 RX. Replaces the PZ012-PZ015 UART cable.
// Radio carries control and reports only; acoustic payloads and schedules are never sent
// to the RX before its listening window has closed.
#include "timing_core.h"

namespace wl {
using cal::crc32;
constexpr uint32_t Magic=0x31575A50, TextMagic=0x54575A50, Schema=1;
constexpr uint8_t Channel=6;
constexpr uint16_t WakePayload=0xCA11;  // acoustic request: RX turns its radio on
enum Op:uint32_t {
  Hello=1, Start=2, Tone=3, Frame=4, Reveal=5,
  PrepareFreq=6, CommitFreq=7, PrepareTiming=8, CommitTiming=9, Finish=10,
  Begin=11, Abort=12, Release=13, Ack=128
};
enum Kind:uint32_t { KindFrequency=1, KindTiming=2, KindFull=3 };
enum State:uint32_t { Listen=0, CalIdle=1, CalRunning=2 };
struct Packet {
  uint32_t magic=Magic, version=Schema, role=0, session=0, seq=0, op=0;
  uint32_t a=0,b=0,c=0,d=0,crc=0;
  void seal(){crc=crc32(this,offsetof(Packet,crc));}
  bool valid()const{return magic==Magic&&version==Schema&&crc==crc32(this,offsetof(Packet,crc));}
};
static_assert(sizeof(Packet)==44,"packet size");
// One RX log line mirrored to the TX web page.
struct Text {
  uint32_t magic=TextMagic, version=Schema, role=0, seq=0, len=0;
  char text[220]={}; uint32_t crc=0;
  void set(const char* s){len=0;while(s[len]&&len<sizeof(text)-1){text[len]=s[len];++len;}text[len]=0;}
  void seal(){crc=crc32(this,offsetof(Text,crc));}
  bool valid()const{return magic==TextMagic&&version==Schema&&len<sizeof(text)&&text[len]==0&&crc==crc32(this,offsetof(Text,crc));}
};
static_assert(sizeof(Text)<=250,"ESP-NOW payload limit");

// Begin options, validated on RX. Defaults reproduce the wired PZ012/PZ015 runs plus 6 ms.
struct Options {
  uint32_t kind=KindFull, frames=25, mask=tim::AllDurations, silent=20;
  bool valid()const{return kind>=KindFrequency&&kind<=KindFull&&frames>=5&&frames<=50&&
                           mask&&!(mask&~tim::AllDurations)&&silent<=50;}
};
inline uint16_t testPayload(uint32_t random){uint16_t p=uint16_t(random);return p==WakePayload?uint16_t(p^1):p;}
}
