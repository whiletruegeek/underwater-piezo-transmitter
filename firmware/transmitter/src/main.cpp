#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_periph.h>
#include <stdarg.h>
#include "firmware_identity.h"
#include "wireless.h"
#include "web_ui.h"

// Wi-Fi control transmitter. Access point + phone web page; ESP-NOW control link to the
// RX, which coordinates measurements exactly as in PZ013/PZ015.
// DRIVE_BRIDGE=0 (PZ016): D25 -> 1 kohm -> piezo red, piezo black -> GND (0..3.3 V).
// DRIVE_BRIDGE=1: DRV8833 IN1=D25, IN2=D26 driven as exact complements (LEDC channel 0
// routed inverted to D26), OUT1 -> 1 kohm -> piezo red, OUT2 -> piezo black (+-VM).
// Silence = both inputs low (coast), so no DC is ever left across the piezo.
#ifndef DRIVE_BRIDGE
#define DRIVE_BRIDGE 0
#endif
constexpr int In1=25,In2=26;
const char* const DriveName=DRIVE_BRIDGE?"DRV8833_BRIDGE_D25_D26":"DIRECT_D25";
constexpr uint32_t Role=1;
const char* const Ssid="PiezoModem-TX";
const char* const Password="piezo1234";
Preferences freqPrefs,timePrefs;
cal::Profile freq,freqPending;
tim::Timing timing,timingPending;
WebServer server(80);

// ---------- log ring shown on the web page ----------
constexpr unsigned LogSlots=90;char logRing[LogSlots][220];uint32_t logNext=0;
void addLog(const char* s){char* slot=logRing[logNext%LogSlots];strncpy(slot,s,219);slot[219]=0;++logNext;}
void logf(const char* fmt,...){
  char b[400];va_list a;va_start(a,fmt);vsnprintf(b,sizeof(b),fmt,a);va_end(a);
  char line[440];snprintf(line,sizeof(line),"%s;LOGCRC=%08lX",b,(unsigned long)cal::crc32(b,strlen(b)));
  Serial.println(line);char tagged[440];snprintf(tagged,sizeof(tagged),"TX %s",b);addLog(tagged);
}

// ---------- radio ----------
struct Msg {uint8_t mac[6];uint8_t len;uint8_t data[250];};
QueueHandle_t inbox;
void onRecv(const uint8_t* mac,const uint8_t* data,int len){
  if(len<=0||len>250)return;Msg m;memcpy(m.mac,mac,6);m.len=uint8_t(len);memcpy(m.data,data,len);xQueueSend(inbox,&m,0);
}
uint8_t rxMac[6]={};bool rxKnown=false;uint32_t rxLast=0,rxState=0,rxFreq=0,rxTiming=0,rxProgress=0,framesIn=0,framesOut=0,textIn=0,textBad=0;
void addPeer(const uint8_t* mac){
  if(esp_now_is_peer_exist(mac))return;
  esp_now_peer_info_t p={};memcpy(p.peer_addr,mac,6);p.channel=wl::Channel;p.ifidx=WIFI_IF_AP;p.encrypt=false;esp_now_add_peer(&p);
}
void send(wl::Packet p){
  p.role=Role;p.seal();const uint8_t broadcast[6]={255,255,255,255,255,255};
  esp_now_send(rxKnown?rxMac:broadcast,(const uint8_t*)&p,sizeof(p));++framesOut;
}

// ---------- tone output ----------
uint32_t toneHz=0,toneUntil=0;
bool bridgeActive=false;
void setHz(uint32_t hz){
#if DRIVE_BRIDGE
  if(hz){ledcWriteTone(0,hz);if(!bridgeActive){esp_rom_gpio_connect_out_signal(In2,LEDC_HS_SIG_OUT0_IDX,true,false);bridgeActive=true;}}
  else{ledcWrite(0,0);if(bridgeActive){esp_rom_gpio_connect_out_signal(In2,SIG_GPIO_OUT_IDX,false,false);bridgeActive=false;}digitalWrite(In2,LOW);}
#else
  if(hz)ledcWriteTone(0,hz);else ledcWrite(0,0);
#endif
  toneHz=hz;
}
// Reads both input pads during a 10 ms 2 kHz tone: every sample must be complementary,
// IN1 must toggle, and both must be low before and after (bridge idle = coast).
const char* driveTest="NOT_APPLICABLE";uint32_t driveSamples=0,driveMismatch=0,driveToggles=0,driveIdle=0;
void bridgeSelfTest(){
#if DRIVE_BRIDGE
  auto levels=[](){const uint32_t r=REG_READ(GPIO_IN_REG);return uint32_t(((r>>In1)&1)|(((r>>In2)&1)<<1));};
  const bool idle0=levels()==0;setHz(2000);uint32_t last=levels()&1;
  const int64_t t=esp_timer_get_time();
  while(esp_timer_get_time()-t<10000){const uint32_t v=levels();++driveSamples;if(v!=1&&v!=2)++driveMismatch;if((v&1)!=last){++driveToggles;last=v&1;}}
  setHz(0);delayMicroseconds(50);const bool idle1=levels()==0;
  driveIdle=(idle0?1:0)|(idle1?2:0);
  driveTest=(idle0&&idle1&&driveMismatch<=2&&driveToggles>=30)?"PASS":"FAIL";
#endif
}
void tone(uint32_t hz,uint32_t duration){setHz(hz);toneUntil=millis()+duration;}
// Blocking, busy-waited symbol edges; at most 28*24 = 672 ms.
uint32_t transmitFrame(uint16_t payload,unsigned symbolMs){
  const int64_t t0=esp_timer_get_time();uint32_t last=0;
  for(unsigned i=0;i<tim::FrameSymbols;++i){
    const int64_t at=t0+int64_t(i)*symbolMs*1000;while(esp_timer_get_time()<at){}
    const uint32_t hz=tim::frameBit(payload,i)?freq.f1:freq.f0;if(hz!=last){setHz(hz);last=hz;}
  }
  const int64_t end=t0+int64_t(tim::FrameSymbols)*symbolMs*1000;while(esp_timer_get_time()<end){}
  setHz(0);return uint32_t(esp_timer_get_time()-t0);
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

// ---------- calibration session (RX requests) ----------
uint32_t session=0;bool running=false;wl::Packet cachedReply;
struct Scheduled {bool due=false,done=false;uint32_t trial=0,symbolMs=0,delayMs=0,at=0,durationUs=0;uint16_t payload=0;} frame;
uint32_t calFrames=0;
void handleRequest(const wl::Packet& p){
  if(p.seq&&cachedReply.seq==p.seq&&cachedReply.session==p.session){send(cachedReply);return;}
  wl::Packet r=p;r.op|=wl::Ack;r.a=1;
  if(p.op==wl::Start&&p.session){
    const bool freqOk=p.a!=wl::KindTiming||(freq.valid()&&freq.id==p.b&&freq.f0==p.c&&freq.f1==p.d);
    if(freqOk){session=p.session;running=true;frame=Scheduled{};setHz(0);r.a=0;logf("EVENT=SESSION;KIND=%lu;SESSION=%08lX",(unsigned long)p.a,(unsigned long)session);}
    else r.a=3;
  } else if(p.session==session&&running){
    if(p.op==wl::Tone&&(p.a==0||(p.a>=1000&&p.a<=10000))&&p.b<=2000){tone(p.a,p.b);r.a=0;}
    if(p.op==wl::Frame&&!frame.due&&tim::durationIndex(p.a)<tim::DurationCount&&freq.valid()){
      // Payload and delay are chosen here and revealed only after the RX window.
      frame=Scheduled{};frame.due=true;frame.trial=p.b;frame.symbolMs=p.a;
      frame.delayMs=300+esp_random()%1001;frame.payload=wl::testPayload(esp_random());frame.at=millis()+frame.delayMs;r.a=0;
    }
    if(p.op==wl::Reveal){if(frame.trial==p.a&&frame.done){r.a=0;r.b=frame.payload;r.c=frame.delayMs;r.d=frame.durationUs;}else r.a=2;}
    if(p.op==wl::PrepareFreq){
      cal::Profile q;q.id=session;q.f0=p.a;q.f1=p.b;q.amp0=p.c;q.amp1=p.d;q.seal();
      setHz(0);if(saveRec(freqPrefs,"pending",q)){freqPending=q;r.a=0;}
    }
    if(p.op==wl::CommitFreq&&freqPending.id==session&&commitRec(freqPrefs,freq,freqPending)){r.a=0;logf("EVENT=FREQ_SAVED;PROFILE=%08lX;F0=%lu;F1=%lu",(unsigned long)freq.id,(unsigned long)freq.f0,(unsigned long)freq.f1);}
    if(p.op==wl::PrepareTiming&&p.b==freq.id){
      tim::Timing q;q.id=session;q.profile=freq.id;q.f0=freq.f0;q.f1=freq.f1;q.symbolMs=p.a;q.seal();
      setHz(0);if(saveRec(timePrefs,"pending",q)){timingPending=q;r.a=0;}
    }
    if(p.op==wl::CommitTiming&&timingPending.id==session&&commitRec(timePrefs,timing,timingPending)){r.a=0;logf("EVENT=TIMING_SAVED;TIMING=%08lX;SYMBOL_MS=%lu",(unsigned long)timing.id,(unsigned long)timing.symbolMs);}
    if(p.op==wl::Finish){setHz(0);frame.due=false;running=false;r.a=0;logf("EVENT=SESSION_END;RESULT=%lu;FRAMES=%lu",(unsigned long)p.a,(unsigned long)calFrames);}
  }
  cachedReply=r;send(r);
}

// Commands from the web page to the RX; acknowledged, retried, bounded wait.
wl::Packet cmdReply;bool cmdGot=false;uint32_t cmdSeq=0,cmdSession=0;
void pumpRadio();
int command(uint32_t op,uint32_t a=0,uint32_t b=0,uint32_t c=0,uint32_t d=0){
  if(!rxKnown||millis()-rxLast>3000)return -2;
  wl::Packet p;p.op=op;p.session=cmdSession;p.seq=++cmdSeq;p.a=a;p.b=b;p.c=c;p.d=d;
  for(unsigned retry=0;retry<5;++retry){cmdGot=false;send(p);const uint32_t t=millis();
    while(millis()-t<250){pumpRadio();if(cmdGot&&cmdReply.seq==p.seq&&cmdReply.op==(op|wl::Ack))return int(cmdReply.a);delay(1);}
  }
  return -1;
}
void pumpRadio(){
  Msg m;
  while(xQueueReceive(inbox,&m,0)==pdTRUE){
    if(m.len==sizeof(wl::Text)){
      wl::Text t;memcpy(&t,m.data,sizeof(t));
      if(t.valid()&&t.role==2){++textIn;char line[240];snprintf(line,sizeof(line),"RX %s",t.text);addLog(line);}else ++textBad;
      continue;
    }
    if(m.len!=sizeof(wl::Packet))continue;
    wl::Packet p;memcpy(&p,m.data,sizeof(p));if(!p.valid()||p.role!=2)continue;
    ++framesIn;rxLast=millis();
    if(!rxKnown||memcmp(rxMac,m.mac,6)){memcpy(rxMac,m.mac,6);addPeer(rxMac);rxKnown=true;
      logf("EVENT=RX_LINKED;MAC=%02X:%02X:%02X:%02X:%02X:%02X",m.mac[0],m.mac[1],m.mac[2],m.mac[3],m.mac[4],m.mac[5]);}
    if(p.op==wl::Hello){rxState=p.a;rxFreq=p.b;rxTiming=p.c;rxProgress=p.d;continue;}
    if(p.op&wl::Ack){if(p.session==cmdSession){cmdReply=p;cmdGot=true;}continue;}
    handleRequest(p);
  }
}

// ---------- air test frames from the web page ----------
uint32_t testRemaining=0,testGap=1000,testNext=0,testSent=0;int32_t testFixed=-1;
void sendTestFrame(uint16_t payload){
  const uint32_t us=transmitFrame(payload,timing.symbolMs);++testSent;
  logf("TXTEST;PAYLOAD=%04X;SYMBOL_MS=%lu;DURATION_US=%lu;SENT=%lu",payload,(unsigned long)timing.symbolMs,(unsigned long)us,(unsigned long)testSent);
}

// ---------- web ----------
String jsonEscape(const char* s){String o;for(;*s;++s){if(*s=='"'||*s=='\\')o+='\\';if(uint8_t(*s)>=32)o+=*s;}return o;}
const char* rxStateName(){return rxState==wl::Listen?"LISTEN":rxState==wl::CalIdle?"CAL_IDLE":rxState==wl::CalRunning?"CAL_RUNNING":"?";}
void reply(bool ok,const String& msg){server.send(200,"application/json",String("{\"ok\":")+(ok?"true":"false")+",\"msg\":\""+jsonEscape(msg.c_str())+"\"}");}
String hex8(uint32_t v){char b[9];snprintf(b,sizeof(b),"%08lX",(unsigned long)v);return b;}
void handleStatus(){
  const uint32_t age=rxKnown?millis()-rxLast:0xffffffff;
  String s="{";
  char board[13];snprintf(board,sizeof(board),"%012llX",(unsigned long long)ESP.getEfuseMac());
  s+="\"tx\":{\"code\":\"" FIRMWARE_CODE "\",\"spoken\":\"" FIRMWARE_SPOKEN_CODE "\",\"board\":\""+String(board)+"\"";
  s+=",\"profile\":\""+hex8(freq.valid()?freq.id:0)+"\",\"f0\":"+freq.f0+",\"f1\":"+freq.f1;
  s+=",\"timing\":\""+hex8(timing.valid()?timing.id:0)+"\",\"symbolMs\":"+timing.symbolMs+",\"usable\":"+(timingUsable()?"true":"false");
  s+=",\"drive\":\""+String(DriveName)+"\",\"driveTest\":\""+String(driveTest)+"\"";
  s+=",\"running\":"+String(running?"true":"false")+",\"testRemaining\":"+testRemaining+",\"testSent\":"+testSent+"}";
  s+=",\"rx\":{\"linked\":"+String(rxKnown&&age<3000?"true":"false")+",\"ageMs\":"+age+",\"state\":\""+rxStateName()+"\"";
  s+=",\"profile\":\""+hex8(rxFreq)+"\",\"timing\":\""+hex8(rxTiming)+"\",\"done\":"+(rxProgress>>16)+",\"total\":"+(rxProgress&0xffff)+"}";
  s+=",\"logNext\":"+String(logNext)+"}";
  server.send(200,"application/json",s);
}
void handleLog(){
  uint32_t since=server.hasArg("since")?strtoul(server.arg("since").c_str(),nullptr,10):0;
  if(logNext>LogSlots&&since<logNext-LogSlots)since=logNext-LogSlots;
  String s="{\"next\":"+String(logNext)+",\"lines\":[";
  for(uint32_t i=since;i<logNext;++i){if(i>since)s+=",";s+="\""+jsonEscape(logRing[i%LogSlots])+"\"";}
  s+="]}";server.send(200,"application/json",s);
}
void handleBegin(){
  wl::Options o;
  o.kind=server.hasArg("kind")?uint32_t(server.arg("kind").toInt()):uint32_t(wl::KindFull);
  o.frames=server.hasArg("frames")?server.arg("frames").toInt():25;
  o.mask=server.hasArg("mask")?server.arg("mask").toInt():tim::AllDurations;
  o.silent=server.hasArg("silent")?server.arg("silent").toInt():20;
  if(!o.valid())return reply(false,"invalid options");
  if(running||testRemaining)return reply(false,"transmitter busy");
  cmdSession=esp_random()|1;const int r=command(wl::Begin,o.kind,o.frames,o.mask,o.silent);
  logf("EVENT=BEGIN_REQUEST;KIND=%lu;FRAMES=%lu;MASK=%lu;SILENT=%lu;RESULT=%d",(unsigned long)o.kind,(unsigned long)o.frames,(unsigned long)o.mask,(unsigned long)o.silent,r);
  if(r==0)return reply(true,"calibration started on receiver");
  reply(false,r==-2?"receiver radio is off or not linked: wake it first":r==1?"receiver not in calibration mode or busy":"no answer from receiver");
}
void handleSimple(uint32_t op,const char* name){
  cmdSession=esp_random()|1;const int r=command(op);
  logf("EVENT=%s_REQUEST;RESULT=%d",name,r);
  reply(r==0,r==0?String(name)+" accepted":r==-2?"receiver not linked":"receiver refused or no answer");
}
void handleWake(){
  if(!timingUsable())return reply(false,"no saved timing: hold BOOT on the receiver for 3 s instead");
  if(running||testRemaining)return reply(false,"transmitter busy");
  const uint32_t us=transmitFrame(wl::WakePayload,timing.symbolMs);
  logf("TXWAKE;PAYLOAD=%04X;SYMBOL_MS=%lu;DURATION_US=%lu",wl::WakePayload,(unsigned long)timing.symbolMs,(unsigned long)us);
  reply(true,"acoustic wake frame sent");
}
void handleTest(){
  if(!timingUsable())return reply(false,"no saved timing on transmitter");
  if(running)return reply(false,"calibration running");
  const long count=server.hasArg("count")?server.arg("count").toInt():1;
  const long gap=server.hasArg("gap")?server.arg("gap").toInt():1000;
  if(count<1||count>500||gap<300||gap>10000)return reply(false,"count 1-500, gap 300-10000 ms");
  testFixed=-1;
  if(server.hasArg("payload")&&server.arg("payload").length()){
    char* end=nullptr;const long v=strtol(server.arg("payload").c_str(),&end,16);
    if(*end||v<0||v>0xffff||v==wl::WakePayload)return reply(false,"payload: 4 hex digits, not CA11");
    testFixed=v;
  }
  testRemaining=count;testGap=gap;testNext=millis();
  logf("EVENT=TEST_START;COUNT=%ld;GAP_MS=%ld;PAYLOAD=%s",count,gap,testFixed<0?"RANDOM":server.arg("payload").c_str());
  reply(true,"sending test frames");
}

void identity(){
  Serial.printf("FIRMWARE=%s;BOARD_ID=%012llX;ROLE=TX;MODE=WIFI_CONTROL;DRIVE=%s;DRIVE_TEST=%s;UPTIME_MS=%lu;RUNNING=%d;RX_LINKED=%d;PEER_MS=%lu;WIRE_RX=%lu;WIRE_TX=%lu;TEXT_IN=%lu;TEXT_BAD=%lu;"
    "PROFILE=%08lX;F0=%lu;F1=%lu;TIMING=%08lX;SYMBOL_MS=%lu;TIMING_USABLE=%d;RX_STATE=%s;AP=%s;STATIONS=%d;COMMANDS_ENABLED=0\n",
    FIRMWARE_CODE,(unsigned long long)ESP.getEfuseMac(),DriveName,driveTest,(unsigned long)millis(),running,rxKnown,(unsigned long)(rxKnown?millis()-rxLast:0xffffffff),
    (unsigned long)framesIn,(unsigned long)framesOut,(unsigned long)textIn,(unsigned long)textBad,
    (unsigned long)(freq.valid()?freq.id:0),(unsigned long)freq.f0,(unsigned long)freq.f1,(unsigned long)(timing.valid()?timing.id:0),
    (unsigned long)timing.symbolMs,timingUsable(),rxStateName(),Ssid,WiFi.softAPgetStationNum());
}

void setup(){
  pinMode(2,OUTPUT);pinMode(In1,OUTPUT);digitalWrite(In1,LOW);Serial.begin(115200);
#if DRIVE_BRIDGE
  pinMode(In2,OUTPUT);digitalWrite(In2,LOW);
#endif
  ledcSetup(0,2500,8);ledcAttachPin(In1,0);setHz(0);
#if DRIVE_BRIDGE
  // Input buffers only. gpio_set_direction() would re-route the pad to plain GPIO and
  // detach D25 from LEDC (the PZ021 bug).
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[In1]);PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[In2]);
#endif
  bridgeSelfTest();
  inbox=xQueueCreate(24,sizeof(Msg));
  if(!inbox||!freqPrefs.begin("pzcandidate",false)||!timePrefs.begin("pztiming",false)){Serial.println("FATAL=INIT");while(true)delay(1000);}
  readRec(freqPrefs,"active",freq);readRec(timePrefs,"active",timing);
  WiFi.mode(WIFI_AP);WiFi.softAP(Ssid,Password,wl::Channel);WiFi.setSleep(false);
  if(esp_now_init()!=ESP_OK){Serial.println("FATAL=ESPNOW");while(true)delay(1000);}
  esp_now_register_recv_cb(onRecv);
  const uint8_t broadcast[6]={255,255,255,255,255,255};addPeer(broadcast);
  server.on("/",[](){server.send_P(200,"text/html",WebPage);});
  server.on("/status",handleStatus);server.on("/log",handleLog);server.on("/begin",handleBegin);
  server.on("/abort",[](){testRemaining=0;handleSimple(wl::Abort,"ABORT");});
  server.on("/release",[](){handleSimple(wl::Release,"RELEASE");});
  server.on("/wake",handleWake);server.on("/test",handleTest);
  server.on("/stoptest",[](){testRemaining=0;logf("EVENT=TEST_STOPPED;SENT=%lu",(unsigned long)testSent);reply(true,"test stopped");});
  server.begin();
  delay(200);Serial.println("SPOKEN_CODE=" FIRMWARE_SPOKEN_CODE);Serial.println("DESCRIPTION=" FIRMWARE_DESCRIPTION);
  logf("EVENT=DRIVE_TEST;DRIVE=%s;RESULT=%s;SAMPLES=%lu;MISMATCH=%lu;TOGGLES=%lu;IDLE_LOW_BEFORE=%lu;IDLE_LOW_AFTER=%lu",DriveName,driveTest,
       (unsigned long)driveSamples,(unsigned long)driveMismatch,(unsigned long)driveToggles,(unsigned long)(driveIdle&1),(unsigned long)(driveIdle>>1));
  logf("EVENT=BOOT;AP=%s;IP=%s;CHANNEL=%u;PROFILE=%08lX;TIMING=%08lX",Ssid,WiFi.softAPIP().toString().c_str(),wl::Channel,(unsigned long)freq.id,(unsigned long)timing.id);
  identity();
}
uint32_t lastHello=0,lastIdentity=0;
void loop(){
  pumpRadio();
  server.handleClient();
  const uint32_t now=millis();
  while(Serial.available()){const char c=Serial.read();if(c=='?')identity();}
  if(toneHz&&int32_t(now-toneUntil)>=0)setHz(0);
  if(running&&frame.due&&int32_t(now-frame.at)>=0){
    frame.durationUs=transmitFrame(frame.payload,frame.symbolMs);frame.done=true;frame.due=false;++calFrames;
    logf("TXFRAME;SESSION=%08lX;TRIAL=%lu;SYMBOL_MS=%lu;DURATION_US=%lu",(unsigned long)session,(unsigned long)frame.trial,(unsigned long)frame.symbolMs,(unsigned long)frame.durationUs);
  }
  if(running&&millis()-rxLast>5000){setHz(0);frame.due=false;running=false;logf("EVENT=LINK_LOST");}
  if(!running&&testRemaining&&int32_t(now-testNext)>=0){
    sendTestFrame(testFixed<0?wl::testPayload(esp_random()):uint16_t(testFixed));
    --testRemaining;testNext=millis()+testGap;if(!testRemaining)logf("EVENT=TEST_DONE;SENT=%lu",(unsigned long)testSent);
  }
  // No unsolicited radio traffic from TX while the RX is measuring.
  if(!running&&now-lastHello>=500){wl::Packet h;h.op=wl::Hello;h.b=freq.valid()?freq.id:0;h.c=timing.valid()?timing.id:0;send(h);lastHello=now;}
  if(now-lastIdentity>=5000){identity();lastIdentity=now;}
  digitalWrite(2,running?((now/125)%2):rxKnown&&now-rxLast<3000?HIGH:((now%1000)<100));
  delay(1);
}
