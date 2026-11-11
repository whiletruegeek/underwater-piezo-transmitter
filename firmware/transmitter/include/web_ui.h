#pragma once
#include <pgmspace.h>
// Single-page phone UI served by the PZ016 transmitter at http://192.168.4.1/
static const char WebPage[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Piezo Modem</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1c2230;--mute:#667085;--line:#dde1e7;--ok:#1a7f37;--bad:#c0362c;--acc:#2458d6;--warn:#b26b00}
@media (prefers-color-scheme:dark){:root{--bg:#0f1218;--card:#181c25;--ink:#e8ebf1;--mute:#98a2b3;--line:#2a303c;--ok:#4cc26a;--bad:#ff6b5e;--acc:#6c9bff;--warn:#f0b24a}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:760px;margin:0 auto;padding:12px 16px 40px}
h1{font-size:20px;margin:6px 0 2px}h2{font-size:15px;margin:0 0 10px;text-transform:uppercase;letter-spacing:.04em;color:var(--mute)}
.sub{color:var(--mute);font-size:13px;margin-bottom:12px}
section{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin:12px 0}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}@media(max-width:520px){.grid{grid-template-columns:1fr}}
.kv{display:flex;justify-content:space-between;gap:8px;padding:3px 0;border-bottom:1px dashed var(--line);font-size:14px}.kv:last-child{border:0}
.kv b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
.pill{display:inline-block;padding:2px 8px;border-radius:99px;font-size:12px;font-weight:600;border:1px solid currentColor}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}.mute{color:var(--mute)}
label{display:block;font-size:13px;color:var(--mute);margin:8px 0 4px}
select,input[type=number],input[type=text]{width:100%;padding:9px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--ink);font-size:15px}
.row{display:flex;gap:8px;flex-wrap:wrap}.row>*{flex:1 1 120px}
.checks{display:flex;gap:6px;flex-wrap:wrap}.checks label{display:flex;align-items:center;gap:4px;margin:0;padding:6px 10px;border:1px solid var(--line);border-radius:8px;color:var(--ink)}
button{padding:11px 14px;border-radius:9px;border:1px solid var(--line);background:var(--bg);color:var(--ink);font-size:15px;font-weight:600;cursor:pointer}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff}button.danger{color:var(--bad)}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.btns button{flex:1 1 140px}
.bar{height:8px;background:var(--line);border-radius:99px;overflow:hidden;margin-top:6px}.bar i{display:block;height:100%;background:var(--acc);width:0}
#msg{min-height:20px;font-size:14px;margin-top:10px}
table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}th,td{padding:4px 6px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}
tr.pass td{color:var(--ok);font-weight:600}.scroll{max-height:320px;overflow:auto}
pre{margin:0;font-size:11px;line-height:1.35;white-space:pre-wrap;word-break:break-all;max-height:300px;overflow:auto}
.hint{font-size:12px;color:var(--mute);margin-top:8px}
</style></head><body><main>
<h1>Piezo modem</h1><div class="sub" id="code">connecting...</div>
<div class="grid">
<section><h2>Transmitter</h2><div id="tx"></div></section>
<section><h2>Receiver</h2><div id="rx"></div><div class="bar"><i id="prog"></i></div></section>
</div>
<section><h2>Calibration</h2>
<label>Run</label><select id="kind"><option value="3">Frequency sweep + symbol timing</option><option value="1">Frequency sweep only</option><option value="2">Symbol timing only</option></select>
<div class="row"><div><label>Frames per symbol length</label><select id="frames"><option>10</option><option selected>25</option><option>50</option></select></div>
<div><label>Silent windows</label><input id="silent" type="number" min="0" max="50" value="20"></div></div>
<label>Symbol lengths (ms)</label><div class="checks" id="durs"></div>
<div class="btns"><button class="primary" onclick="begin()">Start calibration</button><button class="danger" onclick="act('abort')">Abort</button></div>
<div class="btns"><button onclick="act('wake')">Wake receiver (sound)</button><button onclick="act('release')">Release receiver (radio off)</button></div>
<div class="hint">The receiver's radio is off while it listens. Wake it with the sound button, or hold its BOOT button for 3 s.</div>
<div id="msg"></div></section>
<section><h2>Air test</h2>
<div class="row"><div><label>Frames</label><input id="count" type="number" min="1" max="500" value="10"></div>
<div><label>Gap (ms)</label><input id="gap" type="number" min="300" max="10000" value="1000"></div>
<div><label>Payload (hex, blank = random)</label><input id="payload" type="text" maxlength="4" placeholder="random"></div></div>
<div class="btns"><button class="primary" onclick="test()">Send test frames</button><button onclick="act('stoptest')">Stop</button></div>
<div class="hint">The receiver blinks three times for each frame it decodes. It reports its count the next time it wakes.</div></section>
<section><h2>Frequency sweep</h2><div id="freqNote" class="mute">No sweep in the log yet.</div><div class="scroll"><table id="freq"></table></div></section>
<section><h2>Symbol timing</h2><div id="timNote" class="mute">No timing search in the log yet.</div><table id="tim"></table></section>
<section><h2>Events</h2><div id="events" class="mute">-</div></section>
<section><h2>Log</h2><pre id="log"></pre></section>
</main><script>
const D=[24,16,12,8,6];let since=0,lines=[],tx={};
document.getElementById('durs').innerHTML=D.map((d,i)=>`<label><input type="checkbox" value="${1<<i}" checked> ${d}</label>`).join('');
const $=id=>document.getElementById(id);
const kv=(k,v,c='')=>`<div class="kv"><span>${k}</span><b class="${c}">${v}</b></div>`;
const f=l=>Object.fromEntries(l.replace(/^(RX|TX) /,'').split(';').filter(x=>x.includes('=')).map(x=>[x.slice(0,x.indexOf('=')),x.slice(x.indexOf('=')+1)]));
function say(t,ok){$('msg').innerHTML=`<span class="${ok?'ok':'bad'}">${t}</span>`}
async function call(u){try{const r=await fetch(u);const j=await r.json();say(j.msg,j.ok)}catch(e){say('no response from transmitter',false)}}
function act(a){if(a==='abort'&&!confirm('Abort the running calibration?'))return;call('/'+a)}
function begin(){let m=0;document.querySelectorAll('#durs input:checked').forEach(c=>m|=+c.value);
 if(!m){say('choose at least one symbol length',false);return}
 call(`/begin?kind=${$('kind').value}&frames=${$('frames').value}&mask=${m}&silent=${$('silent').value}`)}
function test(){call(`/test?count=${$('count').value}&gap=${$('gap').value}&payload=${encodeURIComponent($('payload').value.trim())}`)}
async function status(){try{const s=await(await fetch('/status')).json();tx=s.tx;
 $('code').textContent=`${s.tx.code} · say "${s.tx.spoken}" · board ${s.tx.board}`;
 $('tx').innerHTML=kv('Drive',s.tx.drive+(s.tx.driveTest==='NOT_APPLICABLE'?'':' · self-test '+s.tx.driveTest),s.tx.driveTest==='FAIL'?'bad':'')+kv('Frequency profile',s.tx.profile)+kv('Tones',s.tx.f0?`${s.tx.f0} / ${s.tx.f1} Hz`:'none')+
  kv('Symbol timing',s.tx.symbolMs?`${s.tx.symbolMs} ms (${s.tx.timing})`:'none',s.tx.usable?'ok':'warn')+
  kv('Activity',s.tx.running?'calibrating':s.tx.testRemaining?`testing, ${s.tx.testRemaining} left`:'idle')+kv('Test frames sent',s.tx.testSent);
 const r=s.rx,link=r.linked;
 $('rx').innerHTML=kv('Radio',link?'<span class="pill ok">linked</span>':'<span class="pill mute">off / not linked</span>')+
  kv('State',link?r.state:'LISTEN (assumed)')+kv('Frequency profile',link?r.profile:'-',link&&r.profile!==s.tx.profile?'bad':'')+
  kv('Timing',link?r.timing:'-',link&&r.timing!==s.tx.timing?'bad':'')+kv('Progress',r.total?`${r.done} / ${r.total}`:'-');
 $('prog').style.width=r.total?(100*r.done/r.total)+'%':'0';
 if(s.logNext!==since)await log();}catch(e){$('code').textContent='transmitter not reachable'}}
async function log(){const j=await(await fetch('/log?since='+since)).json();since=j.next;lines=lines.concat(j.lines).slice(-400);render()}
function render(){
 $('log').textContent=lines.slice(-80).join('\n');$('log').scrollTop=1e9;
 let fr=[],tr=[],ev=[],saved=null,start=-1,tstart=-1;
 lines.forEach((l,i)=>{if(l.includes('EVENT=CAL_START'))start=i;if(l.includes('EVENT=TIMING_START'))tstart=i});
 lines.forEach((l,i)=>{const x=f(l);if(l.includes('SUMMARY;')&&x.HZ&&i>start)fr.push(x);if(l.includes('SUMMARY;')&&x.SYMBOL_MS&&i>tstart)tr.push(x);
  if(l.includes('EVENT=SAVED_CANDIDATE')&&i>start)saved=x;if(l.includes('EVENT='))ev.push(l)});
 if(fr.length){$('freqNote').innerHTML=saved?`Saved <b>${saved.F0} / ${saved.F1} Hz</b> as profile ${saved.PROFILE}`:'Latest sweep (no pair saved)';
  $('freq').innerHTML='<tr><th>Hz</th><th>min</th><th>max</th><th>noise</th><th>SNR</th><th>pass</th></tr>'+fr.map(x=>`<tr class="${x.PASS==='1'?'pass':''}"><td>${x.HZ}</td><td>${(+x.MIN).toFixed(1)}</td><td>${(+x.MAX).toFixed(1)}</td><td>${(+x.NOISE).toFixed(1)}</td><td>${(+x.SNR).toFixed(1)}</td><td>${x.PASS==='1'?'yes':''}</td></tr>`).join('')}
 if(tr.length){$('timNote').textContent='Latest timing search';
  $('tim').innerHTML='<tr><th>Symbol</th><th>exact</th><th>bit err</th><th>extra</th><th>min margin</th><th>pass</th></tr>'+tr.map(x=>`<tr class="${x.PASS==='1'?'pass':''}"><td>${x.SYMBOL_MS} ms</td><td>${x.EXACT}/${x.FRAMES}</td><td>${x.BIT_ERRORS}</td><td>${x.EXTRA}</td><td>${(+x.MIN_MARGIN).toFixed(3)}</td><td>${x.PASS==='1'?'yes':''}</td></tr>`).join('')}
 $('events').innerHTML=ev.slice(-12).reverse().map(l=>`<div class="kv"><span>${l.replace(/;LOGCRC=\w+/,'').replace(/;/g,' · ')}</span></div>`).join('')||'-'}
status();setInterval(status,1000);
</script></body></html>)HTML";
