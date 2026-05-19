/**
 * Heltec Wireless Stick V3 — Manette robot 2 roues + LED  (v3 WebSocket)
 *
 * Carte    : Heltec Wireless Stick V3  (MCU : ESP32-S3FN8)
 * LED      : GPIO 35
 *
 * Moteurs (L298N) :
 *   ENA → GPIO 2   IN1 → GPIO 4   IN2 → GPIO 3
 *   ENB → GPIO 7   IN3 → GPIO 5   IN4 → GPIO 6
 *
 * OLED SSD1306 64×32 :
 *   SDA → GPIO 17 | SCL → GPIO 18 | RST → GPIO 21
 *   VCC → Vext via GPIO 36 (LOW = ON)
 *
 * Modèle de commande :
 *   MG = clamp(Y + X, -255, 255)
 *   MD = clamp(Y - X, -255, 255)
 *
 * Courbe PWM moteur :
 *   |val| < 13 (5%)  → PWM = 0
 *   |val| ≥ 13       → PWM = 128 + (|val|-13)*127/242  [128..255]
 *   Le gain [0,1] est appliqué côté client avant envoi.
 *
 * Protocole WebSocket  ws://ip:81/
 *   Client → ESP32 :
 *     {"t":"m",  "mg":N,"md":N}   moteurs    (N ∈ [-255,255])
 *     {"t":"led","v":N}           LED        (N ∈ [0,255])
 *     {"t":"rst"}                 reset tout
 *     {"t":"state"}               demande l'état
 *   ESP32 → Client :
 *     {"mg":N,"md":N,"led":N,"lon":B}  état courant
 *
 * HTTP (port 80) :
 *   GET /   → page HTML (seule route conservée)
 *
 * Bibliothèques requises :
 *   - "ESP8266 and ESP32 OLED driver for SSD1306"  (ThingPulse)
 *   - "WebSockets" par Markus Sattler               (Links2004)
 *
 * Board IDE : Heltec WiFi LoRa 32(V3) / Wireless Shell(V3)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include "SSD1306Wire.h"

// ─── WiFi ─────────────────────────────────────────────────────────────────────
#include "credentials.h"
// credentials.h file must declare myWIFI_SSID and myWIFI_PASSWORD
const char* WIFI_SSID     = myWIFI_SSID;
const char* WIFI_PASSWORD = myWIFI_PASSWORD;

// ─── Broches LED ──────────────────────────────────────────────────────────────
#define LED_PIN   35
#define PWM_FREQ  5000
#define PWM_RES   8

// ─── Broches moteurs ──────────────────────────────────────────────────────────
#define ENA  2
#define IN1  4
#define IN2  3
#define IN3  5
#define IN4  6
#define ENB  7

// ─── OLED ─────────────────────────────────────────────────────────────────────
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define OLED_ADDR 0x3C
#define VEXT_PIN  36

SSD1306Wire      oled(OLED_ADDR, OLED_SDA, OLED_SCL, GEOMETRY_64_32, I2C_TWO);
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ─── État global ──────────────────────────────────────────────────────────────
int     mgVal  = 0, mdVal = 0;
uint8_t ledBrightness = 0;
bool    ledOn  = false;

// ─── Page HTML ────────────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Robot · Manette</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap');
:root{
  --bg:#080a0d;--ridge:#1e2230;--accent:#00e5ff;--green:#00ff88;
  --yellow:#ffd60a;--red:#ff2d55;--text:#b8c4d8;--muted:#344060;
  --sk-bg:#0c0f18;--sk-rim:#252a40;--knob:#1e2235;--knob-hi:#2a3050;
}
*{box-sizing:border-box;margin:0;padding:0;touch-action:none;}
body{
  background:var(--bg);font-family:'Orbitron',sans-serif;
  min-height:100vh;display:flex;flex-direction:column;
  align-items:center;justify-content:center;gap:10px;padding:14px;
}
.title{font-size:.52rem;letter-spacing:.4em;color:var(--muted);text-transform:uppercase;display:flex;align-items:center;gap:6px;}
.ws-dot{width:8px;height:8px;border-radius:50%;background:#333;flex-shrink:0;transition:background .3s;}
.ws-dot.ok {background:var(--green);box-shadow:0 0 6px var(--green);}
.ws-dot.err{background:var(--red);  box-shadow:0 0 6px var(--red);}

/* ── Manette ──────────────────────────────────────────── */
.pad{
  background:linear-gradient(155deg,#181c28 0%,#10131a 55%,#0c0f18 100%);
  border:1px solid var(--ridge);
  border-radius:38px 38px 55px 55px/28px 28px 48px 48px;
  box-shadow:0 2px 0 #242840 inset,0 -2px 0 #060810 inset,
             0 24px 64px rgba(0,0,0,.75),0 0 0 1px #06080e;
  width:380px;padding:18px 16px 26px;
  display:flex;flex-direction:column;gap:12px;position:relative;
}
.pad::before{
  content:'';position:absolute;top:0;left:50%;transform:translateX(-50%);
  width:110px;height:3px;border-radius:0 0 50% 50%;
  background:linear-gradient(90deg,transparent,#252a40,transparent);
}

/* Haut */
.top-row{display:flex;align-items:center;justify-content:space-between;padding:0 6px;}
.ip-tag{font-size:.44rem;letter-spacing:.1em;color:var(--muted);}
.led-cluster{display:flex;align-items:center;gap:7px;}
.led-dot{width:10px;height:10px;border-radius:50%;background:#111620;border:1px solid var(--sk-rim);transition:background .3s,box-shadow .3s;}
.led-dot.on{background:#d8f0ff;box-shadow:0 0 8px #00ccff,0 0 2px #fff;}
.led-lbl{font-size:.46rem;letter-spacing:.15em;color:var(--muted);}
.led-lbl.on{color:var(--accent);}

/* Gain */
.gain-row{display:flex;align-items:center;gap:10px;padding:0 8px;}
.gain-lbl{font-size:.44rem;letter-spacing:.18em;color:var(--muted);white-space:nowrap;}
.gain-lbl span{color:var(--accent);}
.gain-track{flex:1;position:relative;height:6px;border-radius:3px;background:var(--sk-bg);border:1px solid var(--sk-rim);}
.gain-fill{position:absolute;left:0;top:0;bottom:0;border-radius:3px;background:linear-gradient(90deg,#003344,var(--accent));pointer-events:none;}
.gain-thumb{
  position:absolute;top:50%;transform:translate(-50%,-50%);
  width:18px;height:18px;border-radius:50%;
  background:radial-gradient(ellipse at 35% 30%,#2a3050,#1e2235);
  border:2px solid var(--accent);box-shadow:0 2px 6px rgba(0,0,0,.8);
  cursor:grab;left:100%;
}
.gain-thumb:active{cursor:grabbing;}

/* Centre */
.center-row{display:flex;align-items:center;justify-content:space-between;gap:6px;padding:0 2px;}

/* Sliders moteurs verticaux */
.motor-slider-wrap{display:flex;flex-direction:column;align-items:center;gap:4px;width:42px;}
.motor-slider-lbl{font-size:.4rem;letter-spacing:.2em;color:var(--muted);text-transform:uppercase;}
.motor-slider-val{font-size:.66rem;letter-spacing:.04em;color:var(--accent);min-width:34px;text-align:center;}
.motor-dir-badge{font-size:.34rem;letter-spacing:.1em;text-transform:uppercase;padding:2px 4px;border-radius:3px;border:1px solid;}
.dir-avant  {color:var(--green); border-color:var(--green); background:rgba(0,255,136,.08);}
.dir-arriere{color:var(--yellow);border-color:var(--yellow);background:rgba(255,214,10,.08);}
.dir-stop   {color:var(--muted); border-color:var(--muted); background:transparent;}

.vslider-track{position:relative;width:24px;height:130px;background:var(--sk-bg);border:1px solid var(--sk-rim);border-radius:12px;overflow:visible;}
.vslider-track::before{content:'';position:absolute;left:2px;right:2px;top:50%;height:1px;background:var(--muted);opacity:.4;}
.vslider-fill{position:absolute;left:4px;right:4px;border-radius:4px;pointer-events:none;}
.vslider-thumb{
  position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
  width:20px;height:20px;border-radius:50%;
  background:radial-gradient(ellipse at 35% 30%,#2a3050,#1e2235);
  border:2px solid #353a55;box-shadow:0 3px 8px rgba(0,0,0,.8),0 1px 0 #454a68 inset;
  cursor:grab;touch-action:none;
}
.vslider-thumb:active{cursor:grabbing;}
.vslider-thumb::after{content:'';position:absolute;width:7px;height:7px;border-radius:50%;background:var(--accent);top:50%;left:50%;transform:translate(-50%,-50%);opacity:.5;}

/* Joystick */
.stick-wrap{display:flex;flex-direction:column;align-items:center;gap:6px;}
.stick-zone{position:relative;width:136px;height:136px;}
.stick-base{
  position:absolute;inset:0;border-radius:50%;
  background:radial-gradient(ellipse at 38% 33%,#1a1e2e,var(--sk-bg));
  border:3px solid var(--sk-rim);box-shadow:0 4px 14px rgba(0,0,0,.65) inset,0 1px 0 #262b42;
}
.stick-base::before,.stick-base::after{content:'';position:absolute;background:#151828;border-radius:2px;}
.stick-base::before{width:2px;height:58%;top:21%;left:calc(50% - 1px);}
.stick-base::after {width:58%;height:2px;left:21%;top:calc(50% - 1px);}
.knob{
  position:absolute;width:52px;height:52px;border-radius:50%;
  background:radial-gradient(ellipse at 34% 28%,var(--knob-hi),var(--knob));
  border:2px solid #353a55;box-shadow:0 5px 12px rgba(0,0,0,.85),0 1px 0 #454a68 inset;
  cursor:grab;transform:translate(-50%,-50%);left:50%;top:50%;user-select:none;
}
.knob:active{cursor:grabbing;}
.knob::after{content:'';position:absolute;width:9px;height:9px;border-radius:50%;background:var(--accent);top:8px;left:50%;transform:translateX(-50%);opacity:.45;}
.js-vals{display:flex;gap:10px;}
.js-val{font-size:.44rem;letter-spacing:.05em;color:var(--muted);}
.js-val span{color:var(--accent);}

/* Boutons */
.btn-col{display:flex;flex-direction:column;align-items:center;gap:10px;}
.btn-stop{
  font-family:'Orbitron',sans-serif;font-size:.58rem;font-weight:900;
  letter-spacing:.1em;width:64px;height:64px;border-radius:50%;
  border:3px solid #8b0000;background:radial-gradient(ellipse at 38% 28%,#c0392b,#7a0000);
  color:#ffaaaa;cursor:pointer;text-transform:uppercase;
  box-shadow:0 6px 0 #480000,0 8px 18px rgba(200,0,0,.45),0 1px 0 #ff5555 inset;
  position:relative;top:0;transition:top .07s,box-shadow .07s;
}
.btn-stop:active{top:4px;box-shadow:0 2px 0 #480000,0 3px 10px rgba(200,0,0,.3),0 1px 0 #ff5555 inset;}
.btn-led{
  font-family:'Orbitron',sans-serif;font-size:.44rem;letter-spacing:.1em;
  width:46px;height:23px;border-radius:5px;
  border:1px solid var(--muted);background:transparent;color:var(--text);
  cursor:pointer;text-transform:uppercase;transition:border-color .15s,color .15s;
}
.btn-led:hover{border-color:var(--accent);color:var(--accent);}

/* Axes */
.axes-row{display:flex;flex-direction:column;gap:5px;padding:0 6px;}
.axis-line{display:flex;align-items:center;gap:8px;}
.axis-lbl{font-size:.38rem;letter-spacing:.15em;color:var(--muted);width:12px;}
.axis-track{flex:1;height:4px;border-radius:2px;background:var(--sk-bg);border:1px solid var(--ridge);position:relative;}
.axis-track::before{content:'';position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:var(--muted);opacity:.4;}
.axis-cur{position:absolute;top:50%;transform:translate(-50%,-50%);width:9px;height:9px;border-radius:50%;background:var(--accent);box-shadow:0 0 5px var(--accent);left:50%;}
</style>
</head>
<body>

<div class="title">
  <span class="ws-dot" id="ws-dot"></span>Robot de Maxime
</div>

<div class="pad">

  <!-- Haut -->
  <div class="top-row">
    <div class="ip-tag" id="ip-tag">…</div>
    <div class="led-cluster">
      <div class="led-dot" id="led-dot"></div>
      <div class="led-lbl" id="led-lbl">LED</div>
    </div>
  </div>

  <!-- Gain -->
  <div class="gain-row">
    <div class="gain-lbl">GAIN &nbsp;<span id="gain-val">1.00</span></div>
    <div class="gain-track" id="gain-track">
      <div class="gain-fill"  id="gain-fill"  style="width:100%"></div>
      <div class="gain-thumb" id="gain-thumb" style="left:100%"></div>
    </div>
  </div>

  <!-- Centre -->
  <div class="center-row">

    <!-- Slider MG -->
    <div class="motor-slider-wrap">
      <div class="motor-slider-lbl">MG</div>
      <div class="motor-slider-val" id="mg-num">0</div>
      <div class="vslider-track" id="vs-mg">
        <div class="vslider-fill"  id="vs-mg-fill"></div>
        <div class="vslider-thumb" id="vs-mg-thumb"></div>
      </div>
      <div class="motor-dir-badge dir-stop" id="mg-dir">stop</div>
    </div>

    <!-- Joystick -->
    <div class="stick-wrap">
      <div class="stick-zone" id="stick-zone">
        <div class="stick-base"></div>
        <div class="knob" id="knob"></div>
      </div>
      <div class="js-vals">
        <div class="js-val">X&thinsp;<span id="xval">0</span></div>
        <div class="js-val">Y&thinsp;<span id="yval">0</span></div>
      </div>
    </div>

    <!-- Slider MD -->
    <div class="motor-slider-wrap">
      <div class="motor-slider-lbl">MD</div>
      <div class="motor-slider-val" id="md-num">0</div>
      <div class="vslider-track" id="vs-md">
        <div class="vslider-fill"  id="vs-md-fill"></div>
        <div class="vslider-thumb" id="vs-md-thumb"></div>
      </div>
      <div class="motor-dir-badge dir-stop" id="md-dir">stop</div>
    </div>

    <!-- Boutons -->
    <div class="btn-col">
      <button class="btn-stop" id="btn-stop">STOP</button>
      <button class="btn-led"  onclick="sendLedToggle()">LED</button>
    </div>

  </div>

  <!-- Axes X/Y -->
  <div class="axes-row">
    <div class="axis-line">
      <div class="axis-lbl">X</div>
      <div class="axis-track"><div class="axis-cur" id="ax-cur"></div></div>
    </div>
    <div class="axis-line">
      <div class="axis-lbl">Y</div>
      <div class="axis-track"><div class="axis-cur" id="ay-cur"></div></div>
    </div>
  </div>

</div>

<script>
const clamp     = (v,a,b) => Math.max(a, Math.min(b, v));
const toAxisPct = v => ((v + 255) / 510 * 100).toFixed(2);
const DEAD = 13;

document.getElementById('ip-tag').textContent = location.hostname;

// ══ WebSocket ═════════════════════════════════════════════
let ws = null, wsReady = false;
const wsDot = document.getElementById('ws-dot');

function wsConnect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = () => {
    wsReady = true; wsDot.className = 'ws-dot ok';
    wsSendRaw({t:'state'});
  };
  ws.onclose = ws.onerror = () => {
    wsReady = false; wsDot.className = 'ws-dot err';
    setTimeout(wsConnect, 2000);
  };
  ws.onmessage = evt => {
    try {
      const d = JSON.parse(evt.data);
      if (d.mg !== undefined) {
        curMG = d.mg; curMD = d.md;
        applySliderUI('mg', curMG);
        applySliderUI('md', curMD);
        const Y = Math.round((curMG+curMD)/2);
        const X = Math.round((curMG-curMD)/2);
        syncJsDisplay(X, Y);
      }
      if (d.lon !== undefined) applyLedUI(d.lon === true);
    } catch(e){}
  };
}
function wsSendRaw(obj) { if(wsReady) ws.send(JSON.stringify(obj)); }

// Throttle moteurs 30 ms
let wsPending=null, wsThrottle=null;
function wsMotors(mg, md) {
  const smg = Math.round(mg * gainVal);
  const smd = Math.round(md * gainVal);
  wsPending = {t:'m', mg:smg, md:smd};
  if (wsThrottle) return;
  wsThrottle = setTimeout(() => {
    wsThrottle=null;
    if(wsPending){ wsSendRaw(wsPending); wsPending=null; }
  }, 30);
}

// ══ Gain [0..1] ═══════════════════════════════════════════
let gainVal=1.0, gainDragging=false;
const gainTrack=document.getElementById('gain-track');
const gainThumb=document.getElementById('gain-thumb');
const gainFill =document.getElementById('gain-fill');
const gainValEl=document.getElementById('gain-val');
function setGain(v){
  gainVal=clamp(v,0,1);
  const p=(gainVal*100).toFixed(0)+'%';
  gainThumb.style.left=p; gainFill.style.width=p;
  gainValEl.textContent=gainVal.toFixed(2);
}
function gainFromX(cx){ const r=gainTrack.getBoundingClientRect(); return clamp((cx-r.left)/r.width,0,1); }
gainTrack.addEventListener('mousedown',  e=>{gainDragging=true;setGain(gainFromX(e.clientX));e.stopPropagation();});
gainTrack.addEventListener('touchstart', e=>{gainDragging=true;setGain(gainFromX(e.touches[0].clientX));e.stopPropagation();},{passive:false});
document.addEventListener('mousemove', e=>{if(gainDragging)setGain(gainFromX(e.clientX));});
document.addEventListener('touchmove', e=>{if(gainDragging){e.preventDefault();setGain(gainFromX(e.touches[0].clientX));}},{passive:false});
document.addEventListener('mouseup',  ()=>gainDragging=false);
document.addEventListener('touchend', ()=>gainDragging=false);

// ══ Sliders moteurs verticaux (lecture + bypass) ══════════
let curMG = 0, curMD = 0;

function vSliderPct(val){ return ((-val + 255) / 510 * 100); }

function applySliderUI(prefix, val) {
  const pct   = vSliderPct(val);
  const track = document.getElementById('vs-'+prefix);
  const thumb = document.getElementById('vs-'+prefix+'-thumb');
  const fill  = document.getElementById('vs-'+prefix+'-fill');
  const h     = track.clientHeight || 130;
  const thumbPx = pct/100*h, zeroPx = h/2;
  thumb.style.top       = thumbPx + 'px';
  thumb.style.transform = 'translate(-50%, -50%)';
  if (val >= 0) {
    fill.style.top    = thumbPx+'px';
    fill.style.height = Math.max(0, zeroPx-thumbPx)+'px';
    fill.style.background = 'linear-gradient(180deg,#004433,#00ff88)';
  } else {
    fill.style.top    = zeroPx+'px';
    fill.style.height = Math.max(0, thumbPx-zeroPx)+'px';
    fill.style.background = 'linear-gradient(180deg,#ffd60a,#443300)';
  }
  document.getElementById(prefix+'-num').textContent = val;
  const [lbl,cls] = val>DEAD ? ['avant','dir-avant'] : val<-DEAD ? ['arrière','dir-arriere'] : ['stop','dir-stop'];
  const d = document.getElementById(prefix+'-dir');
  d.textContent=lbl; d.className='motor-dir-badge '+cls;
}

// Sliders bypass : touch identifier pour multi-touch
let vsMgTouchId=null, vsMdTouchId=null;

function makeVSlider(prefix, onChangeCb) {
  const track = document.getElementById('vs-'+prefix);
  let touchId = null;
  function valFromY(cy){ const r=track.getBoundingClientRect(); return Math.round((1-clamp((cy-r.top)/r.height,0,1))*510-255); }

  track.addEventListener('mousedown', e=>{
    touchId='mouse'; onChangeCb(valFromY(e.clientY)); e.stopPropagation();
  });
  track.addEventListener('touchstart', e=>{
    if(touchId!==null) return;
    const t=e.changedTouches[0]; touchId=t.identifier;
    onChangeCb(valFromY(t.clientY)); e.preventDefault();
  },{passive:false});

  document.addEventListener('mousemove', e=>{if(touchId==='mouse') onChangeCb(valFromY(e.clientY));});
  document.addEventListener('touchmove', e=>{
    if(touchId===null||touchId==='mouse') return;
    for(const t of e.changedTouches){
      if(t.identifier===touchId){ onChangeCb(valFromY(t.clientY)); e.preventDefault(); break; }
    }
  },{passive:false});
  document.addEventListener('mouseup',  ()=>{ if(touchId==='mouse'){ touchId=null; onChangeCb(0); } });
  document.addEventListener('touchend', e=>{
    if(touchId===null||touchId==='mouse') return;
    for(const t of e.changedTouches){ if(t.identifier===touchId){ touchId=null; onChangeCb(0); break; } }
  });
}

makeVSlider('mg', val=>{
  curMG=clamp(val,-255,255); applySliderUI('mg',curMG);
  const Y=Math.round((curMG+curMD)/2), X=Math.round((curMG-curMD)/2);
  syncJsDisplay(X,Y); positionKnob(clamp(X/255,-1,1),clamp(-Y/255,-1,1));
  wsMotors(curMG,curMD);
});
makeVSlider('md', val=>{
  curMD=clamp(val,-255,255); applySliderUI('md',curMD);
  const Y=Math.round((curMG+curMD)/2), X=Math.round((curMG-curMD)/2);
  syncJsDisplay(X,Y); positionKnob(clamp(X/255,-1,1),clamp(-Y/255,-1,1));
  wsMotors(curMG,curMD);
});

// ══ Joystick ══════════════════════════════════════════════
function syncJsDisplay(x, y) {
  document.getElementById('xval').textContent = x;
  document.getElementById('yval').textContent = y;
  document.getElementById('ax-cur').style.left = toAxisPct(x)  + '%';
  document.getElementById('ay-cur').style.left = toAxisPct(-y) + '%';
}

function applyJoystick(x, y) {
  syncJsDisplay(x, y);
  curMG = clamp(y + x, -255, 255);
  curMD = clamp(y - x, -255, 255);
  applySliderUI('mg', curMG);
  applySliderUI('md', curMD);
  wsMotors(curMG, curMD);
}

const zone   = document.getElementById('stick-zone');
const knob   = document.getElementById('knob');
const RADIUS = 38;
let jsTouchId = null, zoneRect;

function positionKnob(nx, ny) {
  knob.style.left = (zone.offsetWidth/2  + nx*RADIUS) + 'px';
  knob.style.top  = (zone.offsetHeight/2 + ny*RADIUS) + 'px';
}

function onMoveJs(cx, cy) {
  const zx=zoneRect.left+zoneRect.width/2, zy=zoneRect.top+zoneRect.height/2;
  let dx=cx-zx, dy=cy-zy, d=Math.sqrt(dx*dx+dy*dy);
  if(d>RADIUS){dx=dx/d*RADIUS;dy=dy/d*RADIUS;}
  positionKnob(dx/RADIUS, dy/RADIUS);
  applyJoystick(Math.round(dx/RADIUS*255), Math.round(-dy/RADIUS*255));
}

zone.addEventListener('mousedown', e=>{
  jsTouchId='mouse'; zoneRect=zone.getBoundingClientRect(); onMoveJs(e.clientX,e.clientY);
});
zone.addEventListener('touchstart', e=>{
  if(jsTouchId!==null) return;
  const t=e.changedTouches[0]; jsTouchId=t.identifier;
  zoneRect=zone.getBoundingClientRect(); onMoveJs(t.clientX,t.clientY);
  e.preventDefault();
},{passive:false});

document.addEventListener('mousemove', e=>{if(jsTouchId==='mouse') onMoveJs(e.clientX,e.clientY);});
document.addEventListener('touchmove', e=>{
  if(jsTouchId===null||jsTouchId==='mouse') return;
  for(const t of e.changedTouches){
    if(t.identifier===jsTouchId){ onMoveJs(t.clientX,t.clientY); e.preventDefault(); break; }
  }
},{passive:false});
document.addEventListener('mouseup', ()=>{
  if(jsTouchId==='mouse'){ jsTouchId=null; positionKnob(0,0); applyJoystick(0,0); }
});
document.addEventListener('touchend', e=>{
  if(jsTouchId===null||jsTouchId==='mouse') return;
  for(const t of e.changedTouches){
    if(t.identifier===jsTouchId){ jsTouchId=null; positionKnob(0,0); applyJoystick(0,0); break; }
  }
});

// ══ LED ═══════════════════════════════════════════════════
let ledOn = false;
function applyLedUI(on){
  ledOn=on;
  document.getElementById('led-dot').className='led-dot'+(on?' on':'');
  document.getElementById('led-lbl').className='led-lbl'+(on?' on':'');
}
function sendLedToggle(){ wsSendRaw({t:'led', v: ledOn?0:180}); applyLedUI(!ledOn); }

// ══ STOP ══════════════════════════════════════════════════
document.getElementById('btn-stop').addEventListener('click', ()=>{
  curMG=0; curMD=0;
  positionKnob(0,0); applyJoystick(0,0);
  applySliderUI('mg',0); applySliderUI('md',0);
  wsSendRaw({t:'rst'});
});

// ══ Init ══════════════════════════════════════════════════
setGain(1.0);
requestAnimationFrame(()=>{
  positionKnob(0,0);
  applySliderUI('mg',0);
  applySliderUI('md',0);
});
wsConnect();
</script>
</body>
</html>
)rawliteral";

// ─── Courbe PWM ───────────────────────────────────────────────────────────────
int motorPWM(int val) {
  int a = abs(val);
  if (a < 13) return 0;
  return constrain(128 + (a - 13) * 127 / 242, 128, 255);
}

// ─── Moteurs ──────────────────────────────────────────────────────────────────
void motorLeft(int val) {
  bool fwd = (val >= 0);
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
  ledcWrite(ENA, (uint8_t)motorPWM(val));
}
void motorRight(int val) {
  bool fwd = (val >= 0);
  digitalWrite(IN3, fwd ? HIGH : LOW);
  digitalWrite(IN4, fwd ? LOW  : HIGH);
  ledcWrite(ENB, (uint8_t)motorPWM(val));
}
void stopMotors() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  ledcWrite(ENA,0); ledcWrite(ENB,0);
  mgVal=0; mdVal=0;
}

// ─── LED ──────────────────────────────────────────────────────────────────────
void applyLed() { ledcWrite(LED_PIN, ledOn ? ledBrightness : 0); }

// ─── OLED ─────────────────────────────────────────────────────────────────────
String dirStr(int v){ if(v>13) return "AV"; if(v<-13) return "AR"; return ".."; }
void updateOled() {
  oled.clear();
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0,  0, WiFi.localIP().toString());
  oled.drawString(0, 11, "G: "+String(mgVal)+" "+dirStr(mgVal));
  oled.drawString(0, 22, "D: "+String(mdVal)+" "+dirStr(mdVal));
  oled.display();
}

// ─── Broadcast état ───────────────────────────────────────────────────────────
void broadcastState() {
  String j = "{\"mg\":"  + String(mgVal)
           + ",\"md\":" + String(mdVal)
           + ",\"led\":" + String(ledBrightness)
           + ",\"lon\":" + (ledOn ? "true" : "false")
           + "}";
  wsServer.broadcastTXT(j);
}

// ─── WebSocket events ─────────────────────────────────────────────────────────

// Extrait un entier depuis une clé JSON : "key":valeur
int jsonInt(const String& s, const char* key) {
  int k = s.indexOf(key);
  if (k < 0) return 0;
  k = s.indexOf(':', k) + 1;
  while (k < (int)s.length() && s[k] == ' ') k++;
  int end = k;
  if (s[end] == '-') end++;
  while (end < (int)s.length() && isDigit(s[end])) end++;
  return s.substring(k, end).toInt();
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {

  if (type == WStype_CONNECTED) {
    broadcastState();
    return;
  }
  if (type == WStype_DISCONNECTED) {
    stopMotors();           // sécurité : arrêt si client déconnecté
    updateOled();
    return;
  }
  if (type != WStype_TEXT) return;

  String msg((char*)payload);
  msg = msg.substring(0, length);

  if (msg.indexOf("\"m\"") > 0 || msg.indexOf("\"mg\"") > 0) {
    // {"t":"m","mg":N,"md":N}
    mgVal = constrain(jsonInt(msg, "\"mg\""), -255, 255);
    mdVal = constrain(jsonInt(msg, "\"md\""), -255, 255);
    motorLeft(mgVal);
    motorRight(mdVal);
    updateOled();
    // Pas de broadcast sur chaque trame moteur (trop fréquent)

  } else if (msg.indexOf("\"led\"") > 0) {
    // {"t":"led","v":N}
    ledBrightness = (uint8_t)constrain(jsonInt(msg, "\"v\""), 0, 255);
    ledOn = (ledBrightness > 0);
    applyLed(); updateOled();
    broadcastState();

  } else if (msg.indexOf("\"rst\"") > 0) {
    // {"t":"rst"}
    stopMotors();
    ledOn = false; ledBrightness = 0;
    applyLed(); updateOled();
    broadcastState();

  } else if (msg.indexOf("\"state\"") > 0) {
    broadcastState();
  }
}

// ─── HTTP handler ─────────────────────────────────────────────────────────────
void handleRoot() { httpServer.send_P(200, "text/html", INDEX_HTML); }

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(100);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH); delay(50);

  Wire1.begin(OLED_SDA, OLED_SCL, 400000UL);
  oled.init();
  oled.flipScreenVertically();
  oled.setFont(ArialMT_Plain_10);
  oled.clear();
  oled.drawString(0, 0, "Connexion...");
  oled.display();

  ledcAttach(LED_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(LED_PIN, 0);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  stopMotors();

  Serial.print("Connexion WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP : " + WiFi.localIP().toString());

  updateOled();

  httpServer.on("/", handleRoot);
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  Serial.println("HTTP :80  WebSocket :81  démarrés");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  httpServer.handleClient();
  wsServer.loop();
}
