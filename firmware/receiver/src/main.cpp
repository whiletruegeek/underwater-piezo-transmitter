#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <driver/adc.h>
#include <stdarg.h>
#include "firmware_identity.h"
#include "wireless.h"

// PZ017 receiver. Default LISTEN state: radio OFF, acoustic frames decoded with the
// saved frequency profile + timing; a valid frame flashes GPIO2 three times.
// CAL_IDLE / CAL_RUNNING: radio ON (ESP-NOW, channel 6) only for calibration control.
// Enter CAL_IDLE by holding BOOT >=3 s, USB 'c', or the acoustic wake frame (0xCA11).
// While a calibration job runs, the RX transmits only inside control exchanges, never
// during an acoustic capture or listening window.
constexpr uint32_t Role=2;
Preferences freqPrefs,timePrefs;
cal::Profile freq,freqPending;
tim::Timing timing,timingPending;
wl::Packet reply;bool gotReply=false;
uint32_t session=0,sequence=0,lastPeer=0,framesIn=0,framesOut=0,textDropped=0,peerFreq=0,peerTiming=0;
uint8_t txMac[6]={};bool txKnown=false,radio=false,measuring=false,cancelled=false;
// Radio silence covers each capture/listening window only. Leaving a window sends a
// keep-alive at once, so back-to-back windows never exceed the TX's 5 s link timeout.
void sendHello();
struct Quiet {Quiet(){measuring=true;}~Quiet(){measuring=false;sendHello();}};
int lastRpc=0;
wl::State state=wl::Listen;const char* phase="BOOT";const char* reason="NONE";
uint32_t idleSince=0,lastHello=0,lastIdentity=0,buttonTime=0,progressDone=0,progressTotal=0;
bool buttonHeld=false,beginRequested=false;int lastResult=-1;
wl::Options options;uint32_t lastBeginSeq=0,lastBeginSession=0,lastBeginResult=1;
bool listenMeasuring=false;uint32_t listenT0=0;

// ---------- ADC and blind acoustic decoding ----------
tim::Engine engine;
uint32_t overruns=0,badChannel=0,clipped=0;uint64_t sampleCount=0,adcStartedUs=0;
uint16_t capture[cal::Samples];unsigned captured=0;bool collecting=false;
// USB 'n': 3 s quiet-noise check in the current state (radio on or off) for A/B comparisons.
bool noiseCheck=false;double ncSum=0,ncSq=0;uint32_t ncN=0,ncClip=0;
float rateEstimate(){const double s=(esp_timer_get_time()-adcStartedUs)/1e6;return s>1?float(sampleCount/s):0;}
struct Seen {tim::Detection d;uint32_t endMs;};
Seen heard[8];unsigned heardCount=0;bool listening=false;uint32_t stray=0,strayWeak=0;
// LISTEN statistics, reported when the radio next turns on.
uint32_t listenFrames=0,listenWeak=0,listenOther=0,listenStart=0;uint16_t lastPayloads[5]={};bool listenReady=false;
uint32_t blinkStart=0;bool blinking=false;

// ---------- radio ----------
struct Msg {uint8_t mac[6];uint8_t len;uint8_t data[250];};
QueueHandle_t inbox;
void onRecv(const uint8_t* mac,const uint8_t* data,int len){
  if(len<=0||len>250)return;Msg m;memcpy(m.mac,mac,6);m.len=uint8_t(len);memcpy(m.data,data,len);xQueueSend(inbox,&m,0);
}
constexpr unsigned TextSlots=48;char textQ[TextSlots][220];unsigned tqHead=0,tqCount=0;uint32_t textSeq=0;
void queueText(const char* s){
  if(!radio)return;
  if(tqCount==TextSlots){++textDropped;tqHead=(tqHead+1)%TextSlots;--tqCount;}
  char* slot=textQ[(tqHead+tqCount)%TextSlots];strncpy(slot,s,219);slot[219]=0;++tqCount;
}
void logLine(bool mirror,const char* fmt,va_list args){
  char b[600];vsnprintf(b,sizeof(b),fmt,args);
  char line[640];snprintf(line,sizeof(line),"%s;LOGCRC=%08lX",b,(unsigned long)cal::crc32(b,strlen(b)));
  Serial.println(line);if(mirror)queueText(line);
}
void logf(const char* fmt,...){va_list a;va_start(a,fmt);logLine(false,fmt,a);va_end(a);}  // USB only
void logr(const char* fmt,...){va_list a;va_start(a,fmt);logLine(true,fmt,a);va_end(a);}   // USB + web page
bool quiet(){return measuring;}
void addPeer(const uint8_t* mac){
  if(esp_now_is_peer_exist(mac))return;
  esp_now_peer_info_t p={};memcpy(p.peer_addr,mac,6);p.channel=wl::Channel;p.ifidx=WIFI_IF_STA;p.encrypt=false;esp_now_add_peer(&p);
}
bool radioOn(){
  if(radio)return true;
  WiFi.mode(WIFI_STA);WiFi.disconnect();WiFi.setSleep(false);
  esp_wifi_set_promiscuous(true);esp_wifi_set_channel(wl::Channel,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);
  if(esp_now_init()!=ESP_OK){WiFi.mode(WIFI_OFF);logf("EVENT=RADIO_FAILED");return false;}
  esp_now_register_recv_cb(onRecv);
  const uint8_t broadcast[6]={255,255,255,255,255,255};addPeer(broadcast);
  radio=true;txKnown=false;tqHead=tqCount=0;logr("EVENT=RADIO_ON");return true;
}
void radioOff(){
  if(!radio)return;
  esp_now_deinit();WiFi.disconnect(true);WiFi.mode(WIFI_OFF);radio=false;txKnown=false;tqHead=tqCount=0;
  logf("EVENT=RADIO_OFF");
}
void sendRaw(const void* d,size_t n){
  if(!radio)return;const uint8_t broadcast[6]={255,255,255,255,255,255};
  esp_now_send(txKnown?txMac:broadcast,(const uint8_t*)d,n);
}
void send(wl::Packet p){p.role=Role;p.seal();sendRaw(&p,sizeof(p));++framesOut;}
void flushText(){
  if(!radio||!txKnown||quiet())return;
  for(unsigned n=0;tqCount&&n<6;++n){
    wl::Text t;t.role=Role;t.seq=++textSeq;t.set(textQ[tqHead]);t.seal();
    if(esp_now_send(txMac,(const uint8_t*)&t,sizeof(t))!=ESP_OK)break;
    tqHead=(tqHead+1)%TextSlots;--tqCount;
  }
}

// ---------- persistent records ----------
template<class T> bool readRec(Preferences& p,const char* key,T& r){r=T{};return p.getBytesLength(key)==sizeof(r)&&p.getBytes(key,&r,sizeof(r))==sizeof(r)&&r.valid();}
template<class T> bool saveRec(Preferences& p,const char* key,const T& r){
  if(!r.valid()||p.putBytes(key,&r,sizeof(r))!=sizeof(r))return false;T c;return readRec(p,key,c)&&memcmp(&r,&c,sizeof(r))==0;
}
template<class T> bool commitRec(Preferences& p,T& active,const T& pending){
  if(!pending.valid())return false;
  if(active.valid()&&!saveRec(p,"previous",active))return false;
  if(!saveRec(p,"active",pending))return false;active=pending;return true;
}
bool timingUsable(){return freq.valid()&&timing.valid()&&timing.profile==freq.id&&timing.f0==freq.f0&&timing.f1==freq.f1;}

const char* stateName(){return state==wl::Listen?"LISTEN":state==wl::CalIdle?"CAL_IDLE":"CAL_RUNNING";}
void identity(){
  Serial.printf("FIRMWARE=%s;BOARD_ID=%012llX;ROLE=RX;MODE=WIFI_CONTROL;STATE=%s;RADIO=%s;UPTIME_MS=%lu;PHASE=%s;PEER_MS=%lu;WIRE_RX=%lu;WIRE_TX=%lu;"
    "PROFILE=%08lX;F0=%lu;F1=%lu;TIMING=%08lX;SYMBOL_MS=%lu;TIMING_USABLE=%d;PEER_PROFILE=%08lX;PEER_TIMING=%08lX;HEARD=%lu;"
    "SAMPLE_HZ=%.1f;OV=%lu;CLIP=%lu;TEXT_DROPPED=%lu;COMMANDS_ENABLED=0\n",
    FIRMWARE_CODE,(unsigned long long)ESP.getEfuseMac(),stateName(),radio?"ON":"OFF",(unsigned long)millis(),phase,
    (unsigned long)(lastPeer?millis()-lastPeer:0xffffffff),(unsigned long)framesIn,(unsigned long)framesOut,
    (unsigned long)(freq.valid()?freq.id:0),(unsigned long)freq.f0,(unsigned long)freq.f1,(unsigned long)(timing.valid()?timing.id:0),
    (unsigned long)timing.symbolMs,timingUsable(),(unsigned long)peerFreq,(unsigned long)peerTiming,(unsigned long)listenFrames,
    rateEstimate(),(unsigned long)overruns,(unsigned long)clipped,(unsigned long)textDropped);
}

void enterListen(){
  radioOff();state=wl::Listen;listenReady=false;listenMeasuring=false;listenStart=millis();engine.setNoise(0);
  phase=timingUsable()?"LISTEN_BASELINE":"LISTEN_NO_PROFILE";
  logf("EVENT=LISTEN;PROFILE=%08lX;TIMING=%08lX;SYMBOL_MS=%lu;USABLE=%d",(unsigned long)freq.id,(unsigned long)timing.id,(unsigned long)timing.symbolMs,timingUsable());
}
void enterCalIdle(const char* via){
  if(state!=wl::Listen)return;
  state=wl::CalIdle;idleSince=millis();phase="CAL_IDLE";lastResult=-1;beginRequested=false;engine.setNoise(0);
  if(!radioOn()){enterListen();return;}
  logr("EVENT=CAL_MODE;VIA=%s;LISTEN_FRAMES=%lu;LISTEN_WEAK=%lu;LISTEN_OTHER=%lu;LAST=%04X,%04X,%04X,%04X,%04X",via,
       (unsigned long)listenFrames,(unsigned long)listenWeak,(unsigned long)listenOther,
       lastPayloads[0],lastPayloads[1],lastPayloads[2],lastPayloads[3],lastPayloads[4]);
}

void adcPump(){
  uint8_t b[1024];uint32_t n=0;
  const esp_err_t e=adc_digi_read_bytes(b,sizeof(b),&n,0);
  if(e==ESP_ERR_INVALID_STATE||(e!=ESP_OK&&e!=ESP_ERR_TIMEOUT)){++overruns;engine.restart();collecting=false;return;}
  for(uint32_t i=0;i+1<n;i+=2){const uint16_t w=b[i]|(uint16_t(b[i+1])<<8);++sampleCount;
    if((w>>12)!=6){++badChannel;engine.restart();continue;}
    const uint16_t v=w&4095;if(v<16||v>4079)++clipped;
    if(noiseCheck){ncSum+=v;ncSq+=double(v)*v;++ncN;if(v<16||v>4079)++ncClip;}
    if(collecting){capture[captured++]=v;if(captured==cal::Samples)collecting=false;}
    engine.push(v);
  }
  tim::Detection d;
  while(engine.pop(d)){
    const uint32_t endMs=millis()-uint32_t(lroundf(d.lagBlocks*engine.blockMs()));
    logf("DETECT;SESSION=%08lX;STATE=%s;SYMBOL_MS=%u;END_MS=%lu;PAYLOAD=%04X;SCORE=%.3f;MARGIN=%.3f;ENERGY=%.1f;WEAK=%u;LISTENING=%d",
         (unsigned long)session,stateName(),d.symbolMs,(unsigned long)endMs,d.payload,d.score,d.minMargin,d.minEnergy,d.weak,listening);
    if(state==wl::Listen){
      if(d.symbolMs!=timing.symbolMs){++listenOther;continue;}
      if(d.weak){++listenWeak;continue;}
      if(d.payload==wl::WakePayload){logf("EVENT=WAKE_FRAME");enterCalIdle("ACOUSTIC_WAKE");continue;}
      ++listenFrames;for(int k=4;k>0;--k)lastPayloads[k]=lastPayloads[k-1];lastPayloads[0]=d.payload;
      blinkStart=millis();blinking=true;
      logf("RX_FRAME;PAYLOAD=%04X;COUNT=%lu;SCORE=%.3f;MARGIN=%.3f",d.payload,(unsigned long)listenFrames,d.score,d.minMargin);
    } else if(listening){if(heardCount<8)heard[heardCount++]={d,endMs};}
    else if(d.weak)++strayWeak;else ++stray;
  }
}
void startAdc(){
  adc_digi_init_config_t init={};init.max_store_buf_size=32768;init.conv_num_each_intr=256;init.adc1_chan_mask=1<<6;
  adc_digi_pattern_config_t pattern={};pattern.atten=ADC_ATTEN_DB_11;pattern.channel=6;pattern.unit=0;pattern.bit_width=12;
  adc_digi_configuration_t conf={};conf.conv_limit_en=true;conf.conv_limit_num=250;conf.pattern_num=1;conf.adc_pattern=&pattern;
  conf.sample_freq_hz=32000;conf.conv_mode=ADC_CONV_SINGLE_UNIT_1;conf.format=ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  if(adc_digi_initialize(&init)!=ESP_OK||adc_digi_controller_configure(&conf)!=ESP_OK||adc_digi_start()!=ESP_OK){
    Serial.println("FATAL=ADC_INIT");while(true)delay(1000);
  }
  adcStartedUs=esp_timer_get_time();
}

void handle(const Msg& m){
  if(m.len!=sizeof(wl::Packet))return;
  wl::Packet p;memcpy(&p,m.data,sizeof(p));if(!p.valid()||p.role!=1)return;
  lastPeer=millis();++framesIn;
  if(!txKnown){memcpy(txMac,m.mac,6);addPeer(txMac);txKnown=true;logr("EVENT=TX_LINKED;MAC=%02X:%02X:%02X:%02X:%02X:%02X",m.mac[0],m.mac[1],m.mac[2],m.mac[3],m.mac[4],m.mac[5]);}
  if(p.op==wl::Hello){peerFreq=p.b;peerTiming=p.c;return;}
  if(p.op&wl::Ack){if(p.session==session&&p.seq==sequence){reply=p;gotReply=true;}return;}
  wl::Packet r=p;r.op|=wl::Ack;r.a=1;
  if(p.op==wl::Begin){
    if(p.seq==lastBeginSeq&&p.session==lastBeginSession)r.a=lastBeginResult;
    else{
      wl::Options o;o.kind=p.a;o.frames=p.b;o.mask=p.c;o.silent=p.d;
      if(state==wl::CalIdle&&!beginRequested&&o.valid()){options=o;beginRequested=true;r.a=0;}
      lastBeginSeq=p.seq;lastBeginSession=p.session;lastBeginResult=r.a;
    }
  } else if(p.op==wl::Abort){cancelled=true;r.a=0;}
  else if(p.op==wl::Release){if(state==wl::CalIdle){r.a=0;send(r);enterListen();return;}}
  send(r);
}

void sendHello(){
  if(!radio)return;
  wl::Packet h;h.op=wl::Hello;h.a=state;h.b=freq.valid()?freq.id:0;h.c=timing.valid()?timing.id:0;h.d=(progressDone<<16)|(progressTotal&0xffff);
  send(h);lastHello=millis();
}
void noiseReport();
void service(){
  adcPump();
  Msg m;while(xQueueReceive(inbox,&m,0)==pdTRUE)handle(m);
  const uint32_t now=millis();
  while(Serial.available()){
    const char c=Serial.read();
    if(c=='?')identity();
    if(c=='c'||c=='C')enterCalIdle("USB");
    if(c=='l'||c=='L'){if(state==wl::CalIdle)enterListen();}
    if(c=='x'||c=='X')cancelled=true;
    if((c=='n'||c=='N')&&state!=wl::CalRunning&&!noiseCheck&&!listenMeasuring)noiseReport();
    if(c>='1'&&c<='3'&&state==wl::CalIdle&&!beginRequested){options=wl::Options{};options.kind=uint32_t(c-'0');beginRequested=true;}
  }
  if(digitalRead(0)==LOW){if(!buttonHeld){buttonHeld=true;buttonTime=now;}}
  else if(buttonHeld){buttonHeld=false;if(uint32_t(now-buttonTime)>=3000){
    if(state==wl::Listen)enterCalIdle("BOOT");else if(state==wl::CalIdle)enterListen();else cancelled=true;}}
  if(!quiet()&&uint32_t(now-lastHello)>=500)sendHello();
  flushText();
  if(uint32_t(now-lastIdentity)>=5000&&!quiet()){identity();lastIdentity=now;}
  if(state==wl::CalIdle&&!beginRequested&&uint32_t(now-idleSince)>180000){logr("EVENT=CAL_IDLE_TIMEOUT");delay(50);flushText();enterListen();}
  if(state==wl::Listen&&!listenReady&&timingUsable()&&uint32_t(now-listenStart)>=300&&rateEstimate()>0){
    if(!listenMeasuring){engine.configure(rateEstimate(),freq.f0,freq.f1);engine.beginNoise();listenMeasuring=true;listenT0=now;}
    else if(uint32_t(now-listenT0)>=3000){engine.setNoise(engine.endNoise());listenMeasuring=false;listenReady=true;phase="LISTENING";
      logf("EVENT=LISTEN_READY;NOISE=%.3f;SYMBOL_MS=%lu",engine.noise(),(unsigned long)timing.symbolMs);}
  }
  bool led=false;
  if(state==wl::Listen){if(blinking){const uint32_t e=now-blinkStart;if(e>=360)blinking=false;else led=e%120<60;}}
  else if(state==wl::CalRunning)led=(now/125)%2;
  else if(lastResult==1)led=(now%2000)<100||(now%2000>=200&&now%2000<300);
  else if(lastResult==0)led=(now%2000)<600;
  else led=(now%1000)<100;
  digitalWrite(2,led?HIGH:LOW);delay(1);
}

// Broadband mean/RMS of raw ADC samples plus the detector's mean 4 ms window energy at
// both saved tones (same statistic as the timing/listen baselines). Radio state unchanged.
void noiseReport(){
  if(!freq.valid()||rateEstimate()<=0){Serial.println("NOISE_CHECK;ERROR=NO_PROFILE");return;}
  const bool wasReady=listenReady;const float keep=engine.noise();
  engine.configure(rateEstimate(),freq.f0,freq.f1);engine.beginNoise();
  ncSum=ncSq=0;ncN=ncClip=0;const uint32_t ov0=overruns;noiseCheck=true;
  const uint32_t t=millis();while(uint32_t(millis()-t)<3000){adcPump();Msg m;while(xQueueReceive(inbox,&m,0)==pdTRUE)handle(m);flushText();
    if(radio&&uint32_t(millis()-lastHello)>=500)sendHello();delay(1);}
  noiseCheck=false;const float window=engine.endNoise();
  engine.setNoise(state==wl::Listen&&wasReady?keep:0);
  const double mean=ncN?ncSum/ncN:0,rms=ncN?sqrt(fmax(0,ncSq/ncN-mean*mean)):0;
  logf("NOISE_CHECK;STATE=%s;RADIO=%s;SAMPLES=%lu;MEAN=%.1f;RMS=%.2f;WINDOW_NOISE=%.1f;CLIP=%lu;OV=%lu",stateName(),radio?"ON":"OFF",
       (unsigned long)ncN,mean,rms,window,(unsigned long)ncClip,(unsigned long)(overruns-ov0));
}
int rpc(uint32_t op,uint32_t a=0,uint32_t b=0,uint32_t c=0,uint32_t d=0){
  wl::Packet p;p.op=op;p.session=session;p.seq=++sequence;p.a=a;p.b=b;p.c=c;p.d=d;
  for(unsigned retry=0;retry<5;++retry){gotReply=false;send(p);const uint32_t t=millis();
    while(uint32_t(millis()-t)<300){service();if(gotReply&&reply.op==(op|wl::Ack))return lastRpc=int(reply.a);
      if(cancelled&&op!=wl::Finish)return lastRpc=-1;}
  }
  return lastRpc=-1;
}
bool pauseMs(uint32_t n){const uint32_t t=millis();while(uint32_t(millis()-t)<n){service();if(cancelled)return false;}return true;}

// ---------- frequency calibration (PZ013 procedure, radio control) ----------
bool getCapture(){captured=0;collecting=true;const uint32_t t=millis();
  while(collecting&&uint32_t(millis()-t)<500&&!cancelled)service();
  collecting=false;return captured==cal::Samples&&!cancelled;}
struct Health {float mean,rms;unsigned clip;};
Health health(){double s=0,ss=0;unsigned clip=0;for(auto v:capture){s+=v;ss+=double(v)*v;if(v<16||v>4079)++clip;}
  const double m=s/cal::Samples;return {(float)m,(float)sqrt(fmax(0,ss/cal::Samples-m*m)),clip};}
bool healthy(){const auto h=health();const bool ok=h.clip==0&&h.mean>300&&h.mean<3700;
  if(!ok)logr("EVENT=BAD_ANALOG;MEAN=%.1f;RMS=%.3f;CLIP=%u",h.mean,h.rms,h.clip);return ok;}
cal::Summary summaries[cal::Count];uint32_t runOv,runBad;float rate=31744;
bool fail(const char* why){reason=why;return false;}
bool trial(unsigned idx,unsigned pass,unsigned ordinal){
  const unsigned hz=cal::frequency(idx);
  if(rpc(wl::Tone,0,0)!=0)return fail("LINK");
  {Quiet q;if(!pauseMs(90)||!getCapture())return fail("CANCELLED");}if(!healthy())return fail("BAD_ANALOG");
  const float bg=cal::amplitude(capture,cal::Samples,hz,rate);
  if(rpc(wl::Tone,hz,500)!=0)return fail("LINK");
  float lo=1e9,hi=0,mean=0,rms=0;unsigned clips=0;Quiet q;if(!pauseMs(30))return fail("CANCELLED");
  for(int k=0;k<3;++k){if(!getCapture())return fail("CANCELLED");const auto h=health();clips+=h.clip;mean+=h.mean;rms+=h.rms;
    if(!healthy())return fail("BAD_ANALOG");float a=cal::amplitude(capture,cal::Samples,hz,rate);if(a<lo)lo=a;if(a>hi)hi=a;}
  measuring=false;if(rpc(wl::Tone,0,0)!=0)return fail("LINK");
  summaries[idx].add(lo,hi,bg);
  logf("TRIAL;SESSION=%08lX;PASS=%u;ORDER=%u;HZ=%u;NOISE=%.3f;MIN=%.3f;MAX=%.3f;MEAN=%.1f;RMS=%.3f;CLIP=%u;RATE=%.2f;OV=%lu",(unsigned long)session,pass,ordinal,hz,bg,lo,hi,mean/3,rms/3,clips,rate,(unsigned long)overruns);
  if(overruns!=runOv||badChannel!=runBad)return fail("ADC_HEALTH");
  return true;
}
bool frequencyCalibration(){
  session=esp_random()|1;sequence=0;phase="FREQ_BASELINE";runOv=overruns;runBad=badChannel;
  for(auto& s:summaries)s=cal::Summary{};
  logr("EVENT=CAL_START;SESSION=%08lX;SCOPE=FREQUENCY_CANDIDATE_NOT_COMMAND_QUALIFICATION;CONTROL=ESPNOW",(unsigned long)session);
  if(rpc(wl::Start,wl::KindFrequency)!=0)return fail("NO_PEER_ACK");
  if(rpc(wl::Tone,0,0)!=0||!pauseMs(250))return fail("LINK");
  const uint64_t n0=sampleCount;const int64_t us0=esp_timer_get_time();
  for(unsigned j=0;j<20;++j){
    const uint32_t t=millis();{Quiet q;if(!getCapture())return fail("CANCELLED");}if(!healthy())return fail("BAD_ANALOG");
    for(unsigned i=0;i<cal::Count;++i){const float a=cal::amplitude(capture,cal::Samples,cal::frequency(i),rate);if(a>summaries[i].noise)summaries[i].noise=a;}
    const auto h=health();logf("BASELINE;SESSION=%08lX;INDEX=%u;MEAN=%.1f;RMS=%.3f;CLIP=%u",(unsigned long)session,j,h.mean,h.rms,h.clip);
    while(uint32_t(millis()-t)<1000)if(!pauseMs(1))return fail("CANCELLED");
  }
  rate=float(double(sampleCount-n0)*1000000/(esp_timer_get_time()-us0));
  if(rate<31000||rate>32500||overruns!=runOv||badChannel!=runBad)return fail("ADC_HEALTH");
  phase="FREQ_SWEEP";progressDone=0;progressTotal=cal::Count*cal::Passes;
  for(unsigned pass=0;pass<cal::Passes;++pass){
    unsigned order[cal::Count];for(unsigned i=0;i<cal::Count;++i)order[i]=i;
    for(unsigned i=cal::Count-1;i>0;--i){unsigned j=esp_random()%(i+1);unsigned tmp=order[j];order[j]=order[i];order[i]=tmp;}
    for(unsigned j=0;j<cal::Count;++j){if(!trial(order[j],pass,j))return false;++progressDone;}
  }
  unsigned a=0,b=0;const bool chosen=cal::choose(summaries,a,b);
  for(unsigned i=0;i<cal::Count;++i){const auto& s=summaries[i];logr("SUMMARY;SESSION=%08lX;HZ=%u;MIN=%.3f;MAX=%.3f;NOISE=%.3f;SNR=%.3f;N=%u;PASS=%u",(unsigned long)session,cal::frequency(i),s.minAmp,s.maxAmp,s.noise,s.score(),s.trials,s.good());}
  if(!chosen){logr("EVENT=REJECT;REASON=NO_STABLE_PAIR");return fail("NO_STABLE_PAIR");}
  phase="FREQ_BURSTS";float worst[2]={1e9,1e9};
  for(unsigned k=0;k<30;++k){const unsigned side=esp_random()%2,idx=side?b:a,hz=cal::frequency(idx);
    if(rpc(wl::Tone,0,0)!=0||!pauseMs(100)||rpc(wl::Tone,hz,100)!=0)return fail("LINK");
    {Quiet q;if(!pauseMs(8)||!getCapture())return fail("CANCELLED");}
    if(!healthy())return fail("BAD_ANALOG");
    const float amp=cal::amplitude(capture,cal::Samples,hz,rate);if(amp<worst[side])worst[side]=amp;
    logf("BURST;SESSION=%08lX;INDEX=%u;HZ=%u;AMP=%.3f",(unsigned long)session,k,hz,amp);
    if(amp<8||amp<4*summaries[idx].noise||overruns!=runOv||badChannel!=runBad)return fail("BURST_CHECK");
  }
  if(rpc(wl::Tone,0,0)!=0)return fail("LINK");
  logr("BURSTS;SESSION=%08lX;F0=%u;MIN0=%.3f;F1=%u;MIN1=%.3f",(unsigned long)session,cal::frequency(a),worst[0],cal::frequency(b),worst[1]);
  cal::Profile p;p.id=session;p.f0=cal::frequency(a);p.f1=cal::frequency(b);
  p.amp0=(uint32_t)(summaries[a].minAmp*1000);p.amp1=(uint32_t)(summaries[b].minAmp*1000);p.seal();
  phase="FREQ_SAVING";if(!saveRec(freqPrefs,"pending",p))return fail("NVS");freqPending=p;
  if(rpc(wl::PrepareFreq,p.f0,p.f1,p.amp0,p.amp1)!=0||rpc(wl::CommitFreq)!=0||!commitRec(freqPrefs,freq,freqPending))return fail("COMMIT");
  if(rpc(wl::Finish,1)!=0)return fail("FINISH");
  logr("EVENT=SAVED_CANDIDATE;PROFILE=%08lX;F0=%lu;F1=%lu;AMP0=%lu;AMP1=%lu;CRC=%08lX;COMMAND_QUALIFIED=0",(unsigned long)p.id,(unsigned long)p.f0,(unsigned long)p.f1,(unsigned long)p.amp0,(unsigned long)p.amp1,(unsigned long)p.crc);
  return true;
}

// ---------- symbol-timing search (PZ015 procedure, radio control) ----------
tim::DurationStats stats[tim::DurationCount];
bool timingSearch(const wl::Options& o){
  session=esp_random()|1;sequence=0;phase="TIMING_START";stray=strayWeak=0;unsigned silentFalse=0,silentWeak=0;
  for(auto& s:stats)s=tim::DurationStats{};
  const uint32_t ov0=overruns,bad0=badChannel,clip0=clipped;const float r=rateEstimate();
  logr("EVENT=TIMING_START;SESSION=%08lX;PROFILE=%08lX;F0=%lu;F1=%lu;FRAMES=%lu;MASK=%lu;SILENT=%lu;RATE=%.1f;CONTROL=ESPNOW",
       (unsigned long)session,(unsigned long)freq.id,(unsigned long)freq.f0,(unsigned long)freq.f1,(unsigned long)o.frames,(unsigned long)o.mask,(unsigned long)o.silent,r);
  logf("THRESHOLDS;ENERGY_FACTOR=%.1f;SYMBOL_RATIO=%.1f;SCORE_MIN=%.2f;MARGIN_MIN=%.2f",tim::EnergyFactor,tim::SymbolRatio,tim::ScoreMin,tim::MarginMin);
  if(!freq.valid())return fail("NO_FREQUENCY_PROFILE");
  if(r<31000||r>32500)return fail("ADC_RATE");
  const int started=rpc(wl::Start,wl::KindTiming,freq.id,freq.f0,freq.f1);
  if(started!=0)return fail(started==3?"PEER_PROFILE_MISMATCH":"NO_PEER_ACK");
  if(rpc(wl::Tone,0)!=0||!pauseMs(300))return fail("LINK");
  phase="TIMING_BASELINE";engine.configure(r,freq.f0,freq.f1);engine.beginNoise();
  {Quiet q;if(!pauseMs(3000))return fail("CANCELLED");}
  const float noise=engine.endNoise();
  logr("BASELINE;SESSION=%08lX;NOISE=%.3f;OV=%lu;CLIP=%lu",(unsigned long)session,noise,(unsigned long)(overruns-ov0),(unsigned long)(clipped-clip0));
  if(noise<=0||overruns!=ov0||badChannel!=bad0||clipped!=clip0)return fail("BASELINE_HEALTH");
  engine.setNoise(noise);phase="TIMING_TRIALS";
  static uint8_t order[tim::DurationCount*50+50];unsigned n=0;
  for(unsigned i=0;i<tim::DurationCount;++i)if(o.mask>>i&1)for(unsigned k=0;k<o.frames;++k)order[n++]=uint8_t(i);
  for(unsigned k=0;k<o.silent;++k)order[n++]=tim::DurationCount;
  for(unsigned i=n-1;i>0;--i){const unsigned j=esp_random()%(i+1);const uint8_t t=order[j];order[j]=order[i];order[i]=t;}
  progressDone=0;progressTotal=n;
  for(unsigned trial=0;trial<n;++trial){
    const unsigned kind=order[trial];const bool isFrame=kind<tim::DurationCount;const unsigned T=isFrame?tim::Durations[kind]:0;
    heardCount=0;listening=true;const uint32_t t0=millis();
    if(isFrame&&rpc(wl::Frame,T,trial+1)!=0){listening=false;return fail("FRAME_REQUEST");}
    {Quiet q;if(!pauseMs(isFrame?300+1000+tim::FrameSymbols*T+300:2000)){listening=false;return fail("CANCELLED");}}
    listening=false;
    uint16_t payload=0;uint32_t delayMs=0,durationUs=0;
    if(isFrame){
      int res=-1;for(unsigned k=0;k<10&&(res=rpc(wl::Reveal,trial+1))==2;++k)if(!pauseMs(100))break;
      if(res!=0)return fail("REVEAL");
      payload=uint16_t(reply.b);delayMs=reply.c;durationUs=reply.d;
    }
    const int32_t expectedEnd=int32_t(delayMs+durationUs/1000);
    int matched=-1;unsigned extra=0,weakHere=0;
    for(unsigned k=0;k<heardCount;++k){
      const auto& h=heard[k];const int32_t err=int32_t(h.endMs-t0)-expectedEnd;
      const bool here=isFrame&&h.d.symbolMs==T&&abs(err)<=100;
      if(h.d.weak){if(here)++weakHere;else if(!isFrame)++silentWeak;continue;}
      if(here&&matched<0)matched=int(k);else ++extra;
    }
    if(isFrame){
      auto& s=stats[kind];++s.frames;s.extra+=extra;
      if(matched>=0){const auto& h=heard[matched];++s.detected;const unsigned errs=tim::popcount(uint32_t(h.d.payload^payload));
        s.bitErrors+=errs;if(!errs)++s.exact;
        s.minScore=fminf(s.minScore,h.d.score);s.minMargin=fminf(s.minMargin,h.d.minMargin);s.minEnergy=fminf(s.minEnergy,h.d.minEnergy);
        s.maxTimingMs=fmaxf(s.maxTimingMs,fabsf(float(int32_t(h.endMs-t0)-expectedEnd)));
      } else {++s.missed;if(weakHere)++s.weak;}
    } else silentFalse+=extra;
    const auto* h=matched>=0?&heard[matched]:nullptr;
    logf("TRIAL;SESSION=%08lX;INDEX=%u;KIND=%s;SYMBOL_MS=%u;DELAY_MS=%lu;DURATION_US=%lu;SENT=%04X;HEARD=%u;MATCH=%d;DECODED=%04X;BIT_ERRORS=%u;SCORE=%.3f;MARGIN=%.3f;ENERGY=%.1f;TIMING_MS=%ld;EXTRA=%u;WEAK=%u;OV=%lu;CLIP=%lu",
         (unsigned long)session,trial,isFrame?"FRAME":"SILENT",T,(unsigned long)delayMs,(unsigned long)durationUs,payload,heardCount,matched>=0,
         h?h->d.payload:0,h?tim::popcount(uint32_t(h->d.payload^payload)):0,h?h->d.score:0,h?h->d.minMargin:0,h?h->d.minEnergy:0,
         h?long(int32_t(h->endMs-t0)-expectedEnd):0L,extra,weakHere,(unsigned long)(overruns-ov0),(unsigned long)(clipped-clip0));
    ++progressDone;
    if(overruns!=ov0||badChannel!=bad0||clipped!=clip0||engine.lostEvents())return fail("ADC_HEALTH");
  }
  for(unsigned i=0;i<tim::DurationCount;++i){if(!(o.mask>>i&1))continue;const auto& s=stats[i];
    logr("SUMMARY;SESSION=%08lX;SYMBOL_MS=%u;FRAMES=%u;DETECTED=%u;EXACT=%u;BIT_ERRORS=%u;MISSED=%u;EXTRA=%u;WEAK=%u;MIN_SCORE=%.3f;MIN_MARGIN=%.3f;MIN_ENERGY=%.1f;MAX_TIMING_MS=%.0f;PASS=%u",
         (unsigned long)session,tim::Durations[i],s.frames,s.detected,s.exact,s.bitErrors,s.missed,s.extra,s.weak,s.minScore,s.minMargin,s.minEnergy,s.maxTimingMs,s.passed());}
  logr("FALSE_DETECTIONS;SESSION=%08lX;SILENT_TRIALS=%lu;SILENT_FALSE=%u;SILENT_WEAK=%u;STRAY=%lu;STRAY_WEAK=%lu",
       (unsigned long)session,(unsigned long)o.silent,silentFalse,silentWeak,(unsigned long)stray,(unsigned long)strayWeak);
  if(silentFalse||stray)return fail("FALSE_DETECTIONS");
  const unsigned chosen=tim::choose(stats,o.mask);
  if(chosen>=tim::DurationCount)return fail("NO_PASSING_DURATION");
  const unsigned T=tim::Durations[chosen];
  tim::Timing q;q.id=session;q.profile=freq.id;q.f0=freq.f0;q.f1=freq.f1;q.symbolMs=T;q.seal();
  phase="TIMING_SAVING";if(!saveRec(timePrefs,"pending",q))return fail("NVS");timingPending=q;
  if(rpc(wl::PrepareTiming,T,freq.id)!=0||rpc(wl::CommitTiming)!=0||!commitRec(timePrefs,timing,timingPending))return fail("COMMIT");
  if(rpc(wl::Finish,1)!=0)return fail("FINISH");
  logr("EVENT=TIMING_SAVED;TIMING=%08lX;SYMBOL_MS=%u;PROFILE=%08lX;F0=%lu;F1=%lu;CRC=%08lX;COMMAND_QUALIFIED=0",
       (unsigned long)q.id,T,(unsigned long)q.profile,(unsigned long)q.f0,(unsigned long)q.f1,(unsigned long)q.crc);
  return true;
}

void runJob(){
  state=wl::CalRunning;beginRequested=false;cancelled=false;reason="NONE";lastResult=-1;
  const wl::Options o=options;
  logr("EVENT=JOB_START;KIND=%lu;FRAMES=%lu;MASK=%lu;SILENT=%lu",(unsigned long)o.kind,(unsigned long)o.frames,(unsigned long)o.mask,(unsigned long)o.silent);
  bool ok=true;const char* failedStage="NONE";
  if(o.kind&wl::KindFrequency){ok=frequencyCalibration();if(!ok){failedStage="FREQUENCY";rpc(wl::Finish,0);
    logr("EVENT=CAL_FAILED;SESSION=%08lX;PHASE=%s;REASON=%s;RPC=%d;OV=%lu;CLIP=%lu",(unsigned long)session,phase,reason,lastRpc,(unsigned long)overruns,(unsigned long)clipped);}}
  if(ok&&(o.kind&wl::KindTiming)){ok=timingSearch(o);listening=false;if(!ok){failedStage="TIMING";rpc(wl::Finish,0);
    logr("EVENT=TIMING_FAILED;SESSION=%08lX;PHASE=%s;REASON=%s;RPC=%d;OV=%lu;CLIP=%lu",(unsigned long)session,phase,reason,lastRpc,(unsigned long)overruns,(unsigned long)clipped);}}
  engine.setNoise(0);state=wl::CalIdle;idleSince=millis();lastResult=ok;phase=ok?"CAL_DONE":"CAL_FAILED";progressDone=progressTotal=0;
  logr("EVENT=SESSION_DONE;RESULT=%s;FAILED_STAGE=%s;REASON=%s;PROFILE=%08lX;TIMING=%08lX;SYMBOL_MS=%lu",ok?"PASS":"FAIL",failedStage,reason,
       (unsigned long)freq.id,(unsigned long)timing.id,(unsigned long)timing.symbolMs);
  identity();
}

void setup(){
  pinMode(2,OUTPUT);pinMode(0,INPUT_PULLUP);Serial.begin(115200);
  inbox=xQueueCreate(24,sizeof(Msg));
  if(!inbox||!freqPrefs.begin("pzcandidate",false)||!timePrefs.begin("pztiming",false)){Serial.println("FATAL=INIT");while(true)delay(1000);}
  readRec(freqPrefs,"active",freq);readRec(timePrefs,"active",timing);
  WiFi.mode(WIFI_OFF);startAdc();
  delay(200);Serial.println("SPOKEN_CODE=" FIRMWARE_SPOKEN_CODE);Serial.println("DESCRIPTION=" FIRMWARE_DESCRIPTION);
  enterListen();identity();
}
void loop(){
  service();
  if(state==wl::CalIdle&&beginRequested)runJob();
}
