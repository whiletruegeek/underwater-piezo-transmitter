#pragma once
// Symbol-timing core for the Wi-Fi control releases (PZ016 TX, PZ017 RX) and native tests.
// Derived from timing-common (PZ014/PZ015); adds 6 ms symbols and a duration mask.
// Uses the saved frequency candidate (calibration_core.h Profile) unchanged.
#include "calibration_core.h"

namespace tim {
using cal::crc32;
constexpr unsigned HeaderSymbols=12, PayloadSymbols=16, FrameSymbols=28;
// 4 training symbols 0101 + 8 sync symbols; balanced, peak aperiodic sidelobe 2.
constexpr uint16_t Header=0x5B8; // 010110111000
constexpr unsigned DurationCount=5;
constexpr unsigned Durations[DurationCount]={24,16,12,8,6};
constexpr uint32_t AllDurations=(1u<<DurationCount)-1;
constexpr unsigned BlockSamples=32, WindowBlocks=4, Ring=1024, MaxEvents=8;
// Initial screening thresholds, logged with every run. Not tuned on data.
constexpr float EnergyFactor=10.f; // mean symbol window energy / mean quiet window energy
constexpr float SymbolRatio=2.f;   // header: correct-tone energy / wrong-tone energy
constexpr float ScoreMin=.6f;      // header: sum(correct-wrong)/sum(correct+wrong)
constexpr float MarginMin=.2f;     // payload: weakest |e1-e0|/(e1+e0) for timing acceptance
constexpr float HighPassHz=500.f;

inline int frameBit(uint16_t payload,unsigned i){
  return i<HeaderSymbols?(Header>>(HeaderSymbols-1-i))&1:(payload>>(PayloadSymbols-1-(i-HeaderSymbols)))&1;
}
// Frequency that TX must produce at elapsedUs after frame start; 0 = off.
inline uint32_t frameHz(uint16_t payload,unsigned symbolMs,uint32_t f0,uint32_t f1,uint64_t elapsedUs){
  const uint64_t i=elapsedUs/(symbolMs*1000ull);
  if(i>=FrameSymbols)return 0;
  return frameBit(payload,(unsigned)i)?f1:f0;
}
inline unsigned durationIndex(unsigned ms){for(unsigned i=0;i<DurationCount;++i)if(Durations[i]==ms)return i;return DurationCount;}
inline unsigned popcount(uint32_t v){unsigned n=0;while(v){n+=v&1;v>>=1;}return n;}

// UART control frames. Different magic from PZ012/PZ013 so mixed firmware never interoperates.
constexpr uint32_t Magic=0x31545A50, Schema=1;
enum Op:uint32_t { Hello=1, Start=2, Tone=3, Frame=4, Reveal=5, Prepare=6, Commit=7, Finish=8, Ack=128 };
struct Packet {
  uint32_t magic=Magic, version=Schema, role=0, session=0, seq=0, op=0;
  uint32_t a=0,b=0,c=0,d=0,crc=0;
  void seal(){crc=crc32(this,offsetof(Packet,crc));}
  bool valid()const{return magic==Magic&&version==Schema&&crc==crc32(this,offsetof(Packet,crc));}
};
static_assert(sizeof(Packet)==44,"wire size");
struct Parser {
  uint8_t bytes[sizeof(Packet)]={}; size_t n=0; uint32_t errors=0;
  bool push(uint8_t b,Packet& out) {
    bytes[n++]=b; if(n<sizeof(Packet))return false;
    memcpy(&out,bytes,sizeof(out));
    if(out.valid()){n=0;return true;}
    memmove(bytes,bytes+1,--n); ++errors; return false;
  }
};

// Saved symbol-timing candidate, linked to the frequency profile it was measured with.
struct Timing {
  uint32_t magic=0x32545A50, version=1, id=0, profile=0, f0=0, f1=0, symbolMs=0, crc=0;
  void seal(){crc=crc32(this,offsetof(Timing,crc));}
  bool valid()const{return magic==0x32545A50&&version==1&&id&&profile&&f0>=1000&&f1>=f0+500&&f1<=10000&&
                           durationIndex(symbolMs)<DurationCount&&crc==crc32(this,offsetof(Timing,crc));}
};

// High-pass (removes DC and mains hum), then complex tone sums per 32-sample block.
// The phasor runs continuously, so block sums add coherently into longer windows.
struct Block {float re[2],im[2];};
class Channelizer {
 public:
  void configure(float rate,uint32_t f0,uint32_t f1){
    const uint32_t f[2]={f0,f1};
    for(int k=0;k<2;++k){const float w=6.28318530718f*f[k]/rate;c_[k]=cosf(w);s_[k]=sinf(w);}
    a_=expf(-6.28318530718f*HighPassHz/rate); reset();
  }
  void reset(){for(int k=0;k<2;++k){pr_[k]=1;pi_[k]=0;re_[k]=0;im_[k]=0;}n_=0;primed_=false;y_=0;}
  bool push(uint16_t sample,Block& out){
    const float x=sample;
    if(!primed_){xPrev_=x;y_=0;primed_=true;}
    y_=a_*(y_+x-xPrev_);xPrev_=x;
    for(int k=0;k<2;++k){
      re_[k]+=y_*pr_[k]; im_[k]-=y_*pi_[k];
      const float r=pr_[k]*c_[k]-pi_[k]*s_[k]; pi_[k]=pr_[k]*s_[k]+pi_[k]*c_[k]; pr_[k]=r;
    }
    if(++n_<BlockSamples)return false;
    for(int k=0;k<2;++k){
      out.re[k]=re_[k];out.im[k]=im_[k];re_[k]=im_[k]=0;
      const float m=sqrtf(pr_[k]*pr_[k]+pi_[k]*pi_[k]);pr_[k]/=m;pi_[k]/=m;
    }
    n_=0;return true;
  }
 private:
  float c_[2]={1,1},s_[2]={0,0},pr_[2]={1,1},pi_[2]={0,0},re_[2]={0,0},im_[2]={0,0};
  float a_=0,y_=0,xPrev_=0; unsigned n_=0; bool primed_=false;
};

struct Detection {
  unsigned symbolMs=0; uint32_t start=0; uint16_t payload=0;
  float score=0,minMargin=0,minEnergy=0,lagBlocks=0; unsigned weak=0;
};

// Fixed-duration frame detector over 4-block (~4 ms) coherent window energies.
// Each symbol uses only windows fully inside it, after skipping its first quarter
// (switching transient/ringing). Symbol positions use fractional block lengths.
// A frame must start after at least one quiet symbol period (leading guard), so a
// detector cannot lock onto the middle of a frame sent at another speed.
class Detector {
 public:
  void configure(unsigned symbolMs,float rate){
    ms_=symbolMs; L_=symbolMs*rate/(1000.f*BlockSamples);
    unsigned g=(unsigned)lroundf(L_/4); if(g<1)g=1;
    for(unsigned i=0;i<FrameSymbols;++i){
      const unsigned off=(unsigned)lroundf(i*L_);
      a_[i]=off+g; b_[i]=off+(unsigned)floorf(L_)-WindowBlocks;
    }
    span_=b_[FrameSymbols-1]; lead_=(unsigned)ceilf(L_)+2; reset();
  }
  void reset(){active_=false;holdSet_=false;bestScore_=0;bestC_=0;hold_=0;}
  unsigned span()const{return span_;}
  unsigned lead()const{return lead_;}
  unsigned symbolMs()const{return ms_;}
  float symbolBlocks()const{return L_;}
  // c: candidate frame start, as a window index; all windows up to c+span exist.
  bool step(const float* w0,const float* w1,uint32_t c,float noise,Detection& out){
    if(holdSet_&&int32_t(c-hold_)<0)return false;
    float score;
    if(header(w0,w1,c,noise,score)&&(!active_||score>bestScore_)){active_=true;bestScore_=score;bestC_=c;}
    if(!active_||c-bestC_<=(uint32_t)ceilf(L_/2))return false;
    active_=false;holdSet_=true;hold_=bestC_+(uint32_t)lroundf((FrameSymbols-1)*L_);
    decode(w0,w1,bestC_,noise,out);out.score=bestScore_;out.start=bestC_;out.symbolMs=ms_;
    out.lagBlocks=float(c-bestC_)+span_+WindowBlocks-FrameSymbols*L_;
    return true;
  }
 private:
  unsigned ms_=0,span_=0,lead_=0,a_[FrameSymbols]={},b_[FrameSymbols]={}; float L_=0;
  bool active_=false,holdSet_=false; float bestScore_=0; uint32_t bestC_=0,hold_=0;
  void energy(const float* w0,const float* w1,uint32_t c,unsigned i,float& e0,float& e1,unsigned& m)const{
    e0=e1=0;for(unsigned s=a_[i];s<=b_[i];++s){e0+=w0[(c+s)%Ring];e1+=w1[(c+s)%Ring];}m=b_[i]-a_[i]+1;
  }
  bool header(const float* w0,const float* w1,uint32_t c,float noise,float& score)const{
    float quiet=0;unsigned q=0;
    for(uint32_t s=c-lead_;s+WindowBlocks+2<=c;++s){quiet+=w0[s%Ring]+w1[s%Ring];++q;}
    if(quiet/q>=EnergyFactor*noise)return false;
    float num=0,den=0;
    for(unsigned i=0;i<HeaderSymbols;++i){
      float e0,e1;unsigned m;energy(w0,w1,c,i,e0,e1,m);
      const float right=frameBit(0,i)?e1:e0, wrong=frameBit(0,i)?e0:e1;
      if((e0+e1)/m<EnergyFactor*noise||right<SymbolRatio*wrong)return false;
      num+=right-wrong;den+=right+wrong;
    }
    score=num/den;return score>=ScoreMin;
  }
  void decode(const float* w0,const float* w1,uint32_t c,float noise,Detection& out)const{
    out.payload=0;out.minMargin=1;out.minEnergy=1e30f;out.weak=0;
    for(unsigned i=0;i<FrameSymbols;++i){
      float e0,e1;unsigned m;energy(w0,w1,c,i,e0,e1,m);
      const float ratio=(e0+e1)/m/noise; if(ratio<out.minEnergy)out.minEnergy=ratio;
      if(ratio<EnergyFactor)++out.weak;
      if(i<HeaderSymbols)continue;
      out.payload=uint16_t(out.payload<<1|(e1>e0?1:0));
      const float margin=fabsf(e1-e0)/(e0+e1+1e-20f); if(margin<out.minMargin)out.minMargin=margin;
    }
  }
};

// Stream processor: all four symbol durations are searched continuously and blindly.
class Engine {
 public:
  void configure(float rate,uint32_t f0,uint32_t f1){
    rate_=rate;ch_.configure(rate,f0,f1);
    for(unsigned i=0;i<DurationCount;++i)det_[i].configure(Durations[i],rate);
    noise_=0;measuring_=false;restart();
  }
  // Call after lost or discontinuous ADC data; never join audio across a gap.
  void restart(){ch_.reset();blocks_=0;for(auto& d:det_)d.reset();}
  void beginNoise(){noiseSum_=0;noiseN_=0;measuring_=true;}
  float endNoise(){measuring_=false;return noiseN_?float(noiseSum_/noiseN_):0;}
  void setNoise(float n){noise_=n;for(auto& d:det_)d.reset();}
  float noise()const{return noise_;}
  float rate()const{return rate_;}
  void push(uint16_t sample){
    Block b;if(!ch_.push(sample,b))return;
    hist_[blocks_%WindowBlocks]=b;++blocks_;if(blocks_<WindowBlocks)return;
    const uint32_t s=blocks_-WindowBlocks;
    float e[2];
    for(int k=0;k<2;++k){float re=0,im=0;for(unsigned j=0;j<WindowBlocks;++j){re+=hist_[j].re[k];im+=hist_[j].im[k];}e[k]=re*re+im*im;}
    w0_[s%Ring]=e[0];w1_[s%Ring]=e[1];
    if(measuring_){noiseSum_+=e[0]+e[1];++noiseN_;}
    if(noise_<=0)return;
    for(auto& d:det_){
      if(s<d.span()+d.lead())continue;
      Detection out;
      if(!d.step(w0_,w1_,s-d.span(),noise_,out))continue;
      if(count_<MaxEvents)events_[(head_+count_++)%MaxEvents]=out;else ++lost_;
    }
  }
  bool pop(Detection& out){if(!count_)return false;out=events_[head_];head_=(head_+1)%MaxEvents;--count_;return true;}
  uint32_t lostEvents()const{return lost_;}
  float blockMs()const{return BlockSamples*1000.f/rate_;}
 private:
  Channelizer ch_; Detector det_[DurationCount]; Block hist_[WindowBlocks];
  float w0_[Ring]={},w1_[Ring]={},noise_=0,rate_=31744; uint32_t blocks_=0;
  bool measuring_=false; double noiseSum_=0; uint32_t noiseN_=0;
  Detection events_[MaxEvents]; unsigned head_=0,count_=0; uint32_t lost_=0;
};

// Timing selection among the durations in mask: the shortest one that passed,
// provided every longer selected duration also passed. DurationCount if none.
struct DurationStats {
  unsigned frames=0,detected=0,exact=0,bitErrors=0,missed=0,extra=0,weak=0;
  float minScore=1,minMargin=1,minEnergy=1e30f,maxTimingMs=0;
  bool passed()const{return frames&&detected==frames&&exact==frames&&!extra&&!weak&&minMargin>=MarginMin;}
};
inline unsigned choose(const DurationStats* s,uint32_t mask=AllDurations){
  unsigned chosen=DurationCount;
  for(unsigned i=0;i<DurationCount;++i){if(!(mask>>i&1))continue;if(!s[i].passed())break;chosen=i;}
  return chosen;
}
}
