#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ==========================================
// 1. MAPA DE HARDWARE ZEPHYR PRIME (TOTAL)
// ==========================================
#define PIN_IZQ_FWD  0   
#define PIN_IZQ_BWD  1   
#define PIN_DER_FWD  6   
#define PIN_DER_BWD  7   
#define PIN_ALT_UP   4   
#define PIN_ALT_DOWN 3   

#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

const int MPU_ADDR = 0x68; 
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ==========================================
// 2. SISTEMA DE NAVEGACIÓN Y VARIABLES GLOBALES
// ==========================================
WebServer server(80);
String log_buffer = ""; 

// Comandos de Vuelo del Piloto (UI)
int target_altitud = 0; // PWM o CM dependiendo del modo
int target_giro = 0;    
int target_empuje = 0;  
int flight_mode = 1;    // 1:MAN, 2:YAW, 3:ALT, 4:FULL
int modo_anterior = 1;  

// --- VARIABLES EJE HORIZONTAL (YAW) ---
const float INVERSOR_GIROSCOPIO = 1.0; 
float gyroZ = 0;           
float gyroZ_offset = 0;    
float Kp_y = 1.0, Ki_y = 0.05, Kd_y = 0.5;
int limite_pwm_yaw = 150; 
int max_rampa_yaw = 4;        
float pid_i_y = 0, error_ant_y = 0;

// --- VARIABLES EJE VERTICAL (ALTITUD Z) ---
int distancia_laser_mm = 500;
int distancia_anterior_mm = 500;
float setpoint_altitud_mm = 500; 
float Kp_z = 0.06, Ki_z = 0.00, Kd_z = 1.80;  
int limite_pwm_z = 120; 
int max_rampa_z = 6;
int umbral_ruido_z = 10;   
int banda_muerta_z = 40;   
int max_salto_mm = 150;    
int rechazos_consecutivos = 0;
float pid_i_z = 0;

// Timers y Motores Físicos
unsigned long ultimoComando = 0;
unsigned long ultimoPID_Y = 0;
unsigned long ultimoPID_Z = 0;
unsigned long ultimoLog = 0;

int pwm_actual_izq = 0;
int pwm_actual_der = 0;
int pwm_actual_alt = 0;

int target_M_Izq = 0;
int target_M_Der = 0;
int target_M_Alt = 0;

// Calibración Inercial Dinámica
bool calibracion_pendiente = false;
unsigned long tiempo_inicio_calibracion = 0;
int segundos_restantes = 3;

// =============================================
// 3. INTERFAZ WEB MAESTRA
// =============================================
const char HTML_UI[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Zephyr Prime UI</title>
  <style>
    html, body { overscroll-behavior: none; font-family: 'Courier New', monospace; background: #0a0c10; color: #00d4ff; text-align: center; margin: 0; padding: 0; user-select: none; touch-action: none; overflow: hidden; width: 100vw; height: 100vh; }
    #startScreen { position: fixed; top: 0; left: 0; width: 100vw; height: 100vh; background: #0a0c10; z-index: 9999; display: flex; flex-direction: column; justify-content: center; align-items: center; }
    .btn-start { background: #141a24; border: 2px solid #00d4ff; border-radius: 10px; color: #00d4ff; font-size: 24px; font-weight: bold; padding: 20px 40px; }
    #flightInterface { display: none; width: 100%; height: 100%; flex-direction: column; position: relative; }
    
    .top-panel { display: flex; justify-content: space-between; align-items: center; padding: 5px 10px; border-bottom: 1px solid #141a24; }
    .mode-selector { display: flex; gap: 4px; }
    .mode-btn { background: #141a24; border: 1px solid #00d4ff; color: #00d4ff; padding: 6px 8px; font-size: 11px; font-weight: bold; border-radius: 5px; cursor: pointer; }
    .mode-btn.active { background: #00d4ff; color: #0a0c10; }
    
    #btn-prec { border-color: #ffcc00; color: #ffcc00; margin-left: 5px; }
    #btn-prec.active { background: #ffcc00; color: #0a0c10; }
    #btn-tune { border-color: #00ff88; color: #00ff88; margin-left: 5px; }
    #btn-tune.active { background: #00ff88; color: #0a0c10; }

    .status-panel { text-align: right; font-size: 14px; font-weight: bold; color: #888; }
    .status-ok { color: #00ff88; text-shadow: 0 0 5px #00ff88; }
    .status-ko { color: #ff2244; text-shadow: 0 0 5px #ff2244; }
    
    /* INDICADORES TELEMÉTRICOS SIEMPRE VISIBLES */
    .telemetry-row { display: flex; justify-content: space-around; font-size: 13px; font-weight: bold; color: #ffcc00; margin-top: 5px; }
    
    /* CONSOLA DE INGENIERÍA OCULTA POR DEFECTO */
    #tuneContainer { display: none; margin-top: 5px; }
    .tune-grid { display: flex; justify-content: center; gap: 10px; padding: 0 10px; }
    .tune-box { background: #141a24; border: 1px solid #00ff88; padding: 5px; border-radius: 5px; color: #00ff88; font-size: 10px; width: 50%; }
    .tune-title { font-weight: bold; border-bottom: 1px solid #00ff88; margin-bottom: 5px; padding-bottom: 2px; }
    .pid-input { background: #0a0c10; color: #00d4ff; border: 1px solid #00d4ff; text-align: center; font-weight: bold; margin: 1px; font-size: 10px; }
    .btn-apply { background: #00ff88; color: #0a0c10; font-weight: bold; border: none; padding: 6px 15px; border-radius: 3px; cursor: pointer; margin-top: 5px; font-size: 11px; }
    .btn-cal { background: #ffcc00; color: #0a0c10; border: none; padding: 6px 15px; border-radius: 3px; cursor: pointer; font-weight: bold; margin-left: 5px; font-size: 11px;}
    
    #terminal { background: #000; border-top: 1px solid #00d4ff; border-bottom: 1px solid #00d4ff; text-align: left; padding: 5px 10px; height: 12vh; overflow-y: auto; font-size: 10px; line-height: 1.3; margin-top: 5px; color: #00ff88; }

    .controls-container { display: flex; justify-content: space-around; align-items: center; height: 50vh; width: 100vw; margin-top: 10px;}
    .d-pad { display: flex; flex-direction: column; align-items: center; gap: 10px; }
    .d-pad-row { display: flex; gap: 40px; }
    .d-pad-title { color: #ffcc00; font-size: 11px; letter-spacing: 2px; margin-bottom: 5px; }
    
    .dir-btn { background: #141a24; border: 2px solid #00d4ff; border-radius: 15px; color: #00d4ff; font-size: 24px; font-weight: bold; width: 55px; height: 55px; display: flex; justify-content: center; align-items: center; box-shadow: 0 0 10px rgba(0,212,255,0.2); transition: transform 0.05s; cursor: pointer; user-select: none; }
    .dir-btn.active { background: #00d4ff; color: #141a24; transform: scale(0.95); }
    .left-pad .dir-btn { width: 75px; height: 75px; border-radius: 50%; font-size: 15px; }
    .left-pad .dir-btn.active { background: #ff2244; color: white; border-color: #ff2244; }
    .center-gap { width: 55px; height: 55px; }
    input[type=range][orient=vertical] { appearance: slider-vertical; width: 20px; height: 160px; padding: 0 5px; background: transparent; cursor: pointer; }
  </style>
</head>
<body>
  <div id="startScreen"><button class="btn-start" onclick="iniciarVuelo()">ZEPHYR PRIME: INICIAR</button></div>
  <div id="flightInterface">
    <div class="top-panel">
      <div class="mode-selector">
        <div class="mode-btn active" id="m1" onclick="setMode(1)">1: MAN</div>
        <div class="mode-btn" id="m2" onclick="setMode(2)">2: YAW</div>
        <div class="mode-btn" id="m3" onclick="setMode(3)">3: ALT</div>
        <div class="mode-btn" id="m4" onclick="setMode(4)">4: FULL</div>
        <div class="mode-btn" id="btn-prec" onclick="togglePrecision()">🎯 40%</div>
        <div class="mode-btn" id="btn-tune" onclick="toggleTune()">⚙️ TUNE</div>
      </div>
      <div class="status-panel"><span id="wifi-status" class="status-ko">KO</span></div>
    </div>
    
    <div id="tuneContainer">
      <div class="tune-grid">
        <div class="tune-box">
          <div class="tune-title">YAW (HORZ)</div>
          P:<input type="number" id="yP" class="pid-input" step="0.01" value="1.00" style="width: 35px;">
          I:<input type="number" id="yI" class="pid-input" step="0.01" value="0.05" style="width: 35px;">
          D:<input type="number" id="yD" class="pid-input" step="0.01" value="0.50" style="width: 35px;"><br>
          LIM:<input type="number" id="yL" class="pid-input" step="5" value="150" style="width: 35px;">
          RMP:<input type="number" id="yR" class="pid-input" step="1" value="4" style="width: 35px;">
        </div>
        <div class="tune-box">
          <div class="tune-title">ALT (VERT)</div>
          P:<input type="number" id="zP" class="pid-input" step="0.01" value="0.06" style="width: 35px;">
          I:<input type="number" id="zI" class="pid-input" step="0.01" value="0.00" style="width: 35px;">
          D:<input type="number" id="zD" class="pid-input" step="0.01" value="1.80" style="width: 35px;"><br>
          LIM:<input type="number" id="zL" class="pid-input" step="5" value="120" style="width: 30px;">
          RMP:<input type="number" id="zR" class="pid-input" step="1" value="6" style="width: 25px;"><br>
          GAT:<input type="number" id="zG" class="pid-input" step="1" value="10" style="width: 25px;">
          DDB:<input type="number" id="zB" class="pid-input" step="5" value="40" style="width: 25px;">
          OUT:<input type="number" id="zO" class="pid-input" step="10" value="150" style="width: 25px;">
        </div>
      </div>
      <button class="btn-apply" onclick="sendTune()">APLICAR TODO</button>
      <button class="btn-cal" onclick="recalibrar()">🔄 CAL. YAW</button>
      <div id="terminal">Iniciando recepción de telemetría total...</div>
    </div>

    <div class="telemetry-row">
      <div id="lblL">ALT Z: 0</div>
      <div id="lblR">YAW: 0 | FWD: 0</div>
    </div>

    <div class="controls-container">
      <div class="d-pad left-pad" id="panel-botones-alt">
        <div class="d-pad-title">ALTITUD</div>
        <div class="dir-btn" id="btn-up">SUBIR</div>
        <div class="dir-btn" id="btn-down">BAJAR</div>
      </div>

      <div class="d-pad left-pad" id="panel-slider-alt" style="display:none;">
        <div class="d-pad-title">OBJETIVO (cm)</div>
        <input type="range" orient="vertical" id="alt-slider" min="20" max="150" value="50">
        <div id="slider-val" style="font-size: 16px; color: #00ff88; font-weight: bold; margin-top: 5px;">50 cm</div>
      </div>

      <div class="d-pad right-pad">
        <div class="d-pad-title">DIRECCIÓN</div>
        <div class="dir-btn" id="btn-fw">▲</div>
        <div class="d-pad-row"><div class="dir-btn" id="btn-l">◄</div><div class="center-gap"></div><div class="dir-btn" id="btn-r">►</div></div>
        <div class="dir-btn" id="btn-bw">▼</div>
      </div>
    </div>
  </div>

  <script {nonce}>
    function iniciarVuelo() {
      let elem = document.documentElement;
      if (elem.requestFullscreen) elem.requestFullscreen(); 
      if (screen.orientation && screen.orientation.lock) screen.orientation.lock('landscape').catch(()=>{});
      document.getElementById('startScreen').style.display = 'none';
      document.getElementById('flightInterface').style.display = 'flex';
    }

    let controlA = 0, controlG = 0, controlE = 0, mode = 1;
    let keys = { a_up: false, a_down: false, e_fw: false, e_bw: false, g_l: false, g_r: false };
    
    const MAX_ALT = 200;  
    const MAX_FWD = 200;  
    const MAX_YAW = 150;  
    
    let isPrecisionMode = false;
    let lockBumpless = false;
    let isTuneOpen = false;

    const slider = document.getElementById('alt-slider');
    const sliderVal = document.getElementById('slider-val');
    slider.addEventListener('input', function() { sliderVal.innerText = this.value + " cm"; });
    
    function setMode(m) {
      mode = m;
      document.querySelectorAll('.mode-btn').forEach(btn => btn.classList.remove('active'));
      document.getElementById('m'+m).classList.add('active');
      if(isPrecisionMode) document.getElementById('btn-prec').classList.add('active');
      if(isTuneOpen) document.getElementById('btn-tune').classList.add('active');
      
      if(m === 3 || m === 4) {
        document.getElementById('panel-botones-alt').style.display = 'none';
        document.getElementById('panel-slider-alt').style.display = 'flex';
        lockBumpless = true; 
        controlA = -1; // Bandera para que el ESP32 fije la altitud real actual
      } else {
        document.getElementById('panel-botones-alt').style.display = 'flex';
        document.getElementById('panel-slider-alt').style.display = 'none';
        controlA = 0;
      }
    }

    function togglePrecision() {
      isPrecisionMode = !isPrecisionMode;
      document.getElementById('btn-prec').classList.toggle('active', isPrecisionMode);
    }

    function toggleTune() {
      isTuneOpen = !isTuneOpen;
      document.getElementById('btn-tune').classList.toggle('active', isTuneOpen);
      document.getElementById('tuneContainer').style.display = isTuneOpen ? 'block' : 'none';
    }

    function sendTune() {
      let q = `/tune?yp=${document.getElementById('yP').value}&yi=${document.getElementById('yI').value}&yd=${document.getElementById('yD').value}&yl=${document.getElementById('yL').value}&yr=${document.getElementById('yR').value}`;
      q += `&zp=${document.getElementById('zP').value}&zi=${document.getElementById('zI').value}&zd=${document.getElementById('zD').value}&zl=${document.getElementById('zL').value}&zr=${document.getElementById('zR').value}&zg=${document.getElementById('zG').value}&zb=${document.getElementById('zB').value}&zo=${document.getElementById('zO').value}`;
      fetch(q);
    }

    function recalibrar() { fetch(`/cal`); }
    
    function bindBtn(id, k) {
      const b = document.getElementById(id);
      const p = (e) => { e.preventDefault(); b.classList.add('active'); keys[k] = true; };
      const r = (e) => { e.preventDefault(); b.classList.remove('active'); keys[k] = false; };
      b.addEventListener('touchstart', p, {passive: false}); b.addEventListener('touchend', r);
      b.addEventListener('mousedown', p); b.addEventListener('mouseup', r); b.addEventListener('mouseleave', r);
    }
    bindBtn('btn-up', 'a_up'); bindBtn('btn-down', 'a_down');
    bindBtn('btn-fw', 'e_fw'); bindBtn('btn-bw', 'e_bw'); bindBtn('btn-l', 'g_l'); bindBtn('btn-r', 'g_r');
    
    // Lazo Físico Responsivo UI (40ms)
    setInterval(() => {
      let pFactor = isPrecisionMode ? 0.4 : 1.0;
      let cMaxAlt = MAX_ALT * pFactor;
      let cMaxFwd = MAX_FWD * pFactor;
      let cMaxYaw = MAX_YAW * pFactor;
      let cStep   = isPrecisionMode ? 10 : 25; 
      let zStep   = 10; // Rampa constante para Z manual

      // Lógica Z (Altitud)
      if (mode === 3 || mode === 4) {
        if (!lockBumpless) { controlA = slider.value; }
        document.getElementById('lblL').innerText = `OBJ Z: ${slider.value} cm`; 
      } else {
        if (keys.a_up) controlA = Math.min(cMaxAlt, controlA + zStep); 
        else if (keys.a_down) controlA = Math.max(-cMaxAlt, controlA - zStep); 
        else { if (controlA > 0) controlA = Math.max(0, controlA - zStep); if (controlA < 0) controlA = Math.min(0, controlA + zStep); }
        if (controlA > cMaxAlt) controlA = cMaxAlt;
        if (controlA < -cMaxAlt) controlA = -cMaxAlt;
        document.getElementById('lblL').innerText = `ALT PWM: ${Math.round(controlA)}`; 
      }

      // Lógica XY (Yaw y Fwd)
      if (keys.e_fw) controlE = Math.min(cMaxFwd, controlE + cStep); 
      else if (keys.e_bw) controlE = Math.max(-cMaxFwd, controlE - cStep); 
      else { if (controlE > 0) controlE -= cStep; if (controlE < 0) controlE += cStep; }
      if (controlE > cMaxFwd) controlE = cMaxFwd; if (controlE < -cMaxFwd) controlE = -cMaxFwd;
      
      if (keys.g_r) controlG = Math.min(cMaxYaw, controlG + cStep); 
      else if (keys.g_l) controlG = Math.max(-cMaxYaw, controlG - cStep); 
      else { if (controlG > 0) controlG -= cStep; if (controlG < 0) controlG += cStep; }
      if (controlG > cMaxYaw) controlG = cMaxYaw; if (controlG < -cMaxYaw) controlG = -cMaxYaw;
      
      document.getElementById('lblR').innerText = `YAW: ${Math.round(controlG)} | FWD: ${Math.round(controlE)}`;
    }, 40);

    let lastSend = 0;
    const wifiStatus = document.getElementById('wifi-status');

    // Lazo de Envío Telemétrico (100ms)
    setInterval(() => {
      let now = Date.now();
      if(now - lastSend > 100) {
        lastSend = now;
        fetch(`/a?a=${Math.round(controlA)}&g=${Math.round(controlG)}&e=${Math.round(controlE)}&m=${mode}`)
          .then(res => res.text())
          .then(realAltCm => {
            wifiStatus.innerText = "OK";
            wifiStatus.className = "status-ok";
            // BUMPLESS TRANSFER: Almacena la altitud real al cambiar a PID Automático
            if (lockBumpless && (mode === 3 || mode === 4)) {
              slider.value = realAltCm;
              document.getElementById('slider-val').innerText = realAltCm + " cm";
              controlA = realAltCm; 
              lockBumpless = false; 
            }
          })
          .catch(() => {
            wifiStatus.innerText = "KO";
            wifiStatus.className = "status-ko";
          });
      }
    }, 100);

    // Renderizado del Terminal (200ms)
    setInterval(() => {
      if(isTuneOpen) {
        fetch('/log')
          .then(res => res.text())
          .then(texto => {
            if(texto.length > 0) {
              let term = document.getElementById('terminal');
              term.innerHTML = texto;
              term.scrollTop = term.scrollHeight;
            }
          });
      }
    }, 200);
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 4. FUNCIONES FÍSICAS DE HARDWARE
// ==========================================
void ejecutarCalibracionGiro() {
  long sumaZ = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gz_raw = (Wire.read() << 8 | Wire.read()); 
    sumaZ += gz_raw; 
    delay(3);
  }
  gyroZ_offset = sumaZ / 500.0;
}

void iniciarMPU6050() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  delay(3000); 
  ejecutarCalibracionGiro();
}

void moverMotor(int pinPositivo, int pinNegativo, int potencia) {
  potencia = constrain(potencia, -255, 255);
  if (abs(potencia) < 25) { analogWrite(pinPositivo, 0); analogWrite(pinNegativo, 0); } 
  else if (potencia > 0) { analogWrite(pinPositivo, potencia); analogWrite(pinNegativo, 0); } 
  else { analogWrite(pinPositivo, 0); analogWrite(pinNegativo, abs(potencia)); }
}

int suavizarAceleracion(int pwm_actual, int pwm_objetivo, int rampa) {
  if (pwm_objetivo > pwm_actual + rampa) return pwm_actual + rampa;
  if (pwm_objetivo < pwm_actual - rampa) return pwm_actual - rampa;
  return pwm_objetivo;
}

// ==========================================
// 5. RUTINAS DE CONTROL MAESTRAS (MERGE)
// ==========================================
void ejecutarLeyesDeControl() {
  unsigned long tiempoActual = millis();

  // A) PRIORIDAD ABSOLUTA: Recalibración
  if (calibracion_pendiente) {
    moverMotor(PIN_IZQ_FWD, PIN_IZQ_BWD, 0);
    moverMotor(PIN_DER_FWD, PIN_DER_BWD, 0);
    moverMotor(PIN_ALT_UP, PIN_ALT_DOWN, 0);
    pwm_actual_izq = 0; pwm_actual_der = 0; pwm_actual_alt = 0;

    if (tiempoActual - tiempo_inicio_calibracion >= 1000) {
      tiempo_inicio_calibracion = tiempoActual;
      if (segundos_restantes > 0) {
        log_buffer += "Calibrando en " + String(segundos_restantes) + "s...<br>";
        segundos_restantes--;
      } else {
        ejecutarCalibracionGiro();
        log_buffer += "<span style='color:#00ff88'><b>[OK] MPU Cero: " + String(gyroZ_offset) + "</b></span><br>";
        calibracion_pendiente = false;
      }
    }
    return; // Sale del loop para bloquear PID
  }

  // Falla de seguridad de señal WiFI
  if (tiempoActual - ultimoComando > 500) {
    target_altitud = 0; target_giro = 0; target_empuje = 0; 
  }

  // Reset inercial de la integral al cambiar de modos
  if (flight_mode != modo_anterior) {
    pid_i_y = 0;      
    error_ant_y = 0;
    pid_i_z = 0;
    modo_anterior = flight_mode;
  }

  // B) LAZO HORIZONTAL YAW (Rápido a 50Hz / 20ms)
  if (tiempoActual - ultimoPID_Y >= 20) {
    float dt = (tiempoActual - ultimoPID_Y) / 1000.0;
    ultimoPID_Y = tiempoActual;
    
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    
    int16_t gz_raw = (Wire.read() << 8 | Wire.read());
    float gz_crudo = (gz_raw - gyroZ_offset) / 131.0;
    gyroZ = gz_crudo * INVERSOR_GIROSCOPIO; 

    if (flight_mode == 1 || flight_mode == 3) {
      // YAW MANUAL
      if (target_empuje == 0 && target_giro != 0) {
        target_M_Izq = target_giro;   
        target_M_Der = -target_giro;  
      } else {
        target_M_Izq = target_empuje + (target_giro * 0.5);
        target_M_Der = target_empuje - (target_giro * 0.5);
      }
    }
    else if (flight_mode == 2 || flight_mode == 4) {
      // YAW PID
      float setpoint_y = map(target_giro, -150, 150, -120, 120); 
      float error_y = setpoint_y - gyroZ; 
      
      float pid_p_y = Kp_y * error_y;
      pid_i_y = constrain(pid_i_y + (Ki_y * error_y * dt), -50, 50);
      float pid_d_y = Kd_y * ((error_y - error_ant_y) / dt);
      error_ant_y = error_y;
      
      float PID_Yaw = constrain(pid_p_y + pid_i_y + pid_d_y, -limite_pwm_yaw, limite_pwm_yaw);

      if (target_empuje == 0 && target_giro != 0) {
        target_M_Izq = PID_Yaw; 
        target_M_Der = -PID_Yaw;
      } else {
        target_M_Izq = target_empuje + PID_Yaw;
        target_M_Der = target_empuje - PID_Yaw;
      }
    }

    target_M_Izq = constrain(target_M_Izq, -200, 200);
    target_M_Der = constrain(target_M_Der, -200, 200);

    pwm_actual_izq = suavizarAceleracion(pwm_actual_izq, target_M_Izq, max_rampa_yaw);
    pwm_actual_der = suavizarAceleracion(pwm_actual_der, target_M_Der, max_rampa_yaw);

    moverMotor(PIN_IZQ_FWD, PIN_IZQ_BWD, pwm_actual_izq);
    moverMotor(PIN_DER_FWD, PIN_DER_BWD, pwm_actual_der);
  }

  // C) LAZO VERTICAL Z (Medio a 25Hz / 40ms)
  if (tiempoActual - ultimoPID_Z >= 40) {
    float dt = (tiempoActual - ultimoPID_Z) / 1000.0;
    ultimoPID_Z = tiempoActual;

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    
    // FILTRO DE VALORES ATÍPICOS (Z-OUTLIER)
    if (measure.RangeStatus != 4) { 
      int lectura_cruda = measure.RangeMilliMeter;
      int salto_mm = abs(lectura_cruda - distancia_anterior_mm);
      
      if (salto_mm <= max_salto_mm) {
        distancia_laser_mm = lectura_cruda;
        rechazos_consecutivos = 0;
      } else {
        rechazos_consecutivos++;
        if (rechazos_consecutivos > 10) {
          distancia_laser_mm = lectura_cruda;
          rechazos_consecutivos = 0;
        }
      }
    }

    if (flight_mode == 1 || flight_mode == 2) {
      // Z MANUAL (target_altitud trae el PWM directo progresivo)
      target_M_Alt = target_altitud;
    } 
    else if (flight_mode == 3 || flight_mode == 4) {
      // Z PID (target_altitud trae la meta en centímetros o -1 para Lock)
      if (target_altitud == -1) {
        setpoint_altitud_mm = distancia_laser_mm;
      } else {
        setpoint_altitud_mm = target_altitud * 10.0;
      }

      float error_z = setpoint_altitud_mm - distancia_laser_mm;
      
      float pid_p_z = 0;
      if (abs(error_z) > banda_muerta_z) {
        pid_p_z = Kp_z * error_z;
      }
      
      float delta_distancia = distancia_laser_mm - distancia_anterior_mm;
      float pid_d_z = 0;
      if (abs(delta_distancia) > umbral_ruido_z) {
        pid_d_z = -Kd_z * (delta_distancia / dt);
      }
      
      pid_i_z = constrain(pid_i_z + (Ki_z * error_z * dt), -40, 40);

      target_M_Alt = constrain(pid_p_z + pid_i_z + pid_d_z, -limite_pwm_z, limite_pwm_z);
    }

    distancia_anterior_mm = distancia_laser_mm;
    pwm_actual_alt = suavizarAceleracion(pwm_actual_alt, target_M_Alt, max_rampa_z);
    
    moverMotor(PIN_ALT_UP, PIN_ALT_DOWN, pwm_actual_alt);
  }

  // D) LAZO DE TELEMETRÍA (Lento a 5Hz / 200ms)
  if (tiempoActual - ultimoLog >= 200) {
    ultimoLog = tiempoActual;
    
    String alerta = (rechazos_consecutivos > 0) ? "[!]" : "";
    
    // Una sola línea compacta para la consola
    String linea = "[M" + String(flight_mode) + "] " +
                   "Z:" + String((int)(setpoint_altitud_mm/10)) + "/" + String(distancia_laser_mm/10) + "cm " + alerta + 
                   "| Y:" + String(target_giro) + "/" + String(gyroZ, 0) + "deg " +
                   "| Pz:" + String(pwm_actual_alt) + " Pxy:" + String(pwm_actual_izq) + "<br>";
    
    log_buffer += linea;
    
    if (log_buffer.length() > 2000) { 
      log_buffer = log_buffer.substring(log_buffer.length() - 1500); 
    }
  }
}

// ==========================================
// 6. INICIALIZACIÓN DE LA NAVE
// ==========================================
void setup() {
  Serial.begin(115200); 
  
  pinMode(PIN_IZQ_FWD, OUTPUT); pinMode(PIN_IZQ_BWD, OUTPUT);
  pinMode(PIN_DER_FWD, OUTPUT); pinMode(PIN_DER_BWD, OUTPUT);
  pinMode(PIN_ALT_UP, OUTPUT);  pinMode(PIN_ALT_DOWN, OUTPUT);

  moverMotor(PIN_IZQ_FWD, PIN_IZQ_BWD, 0);
  moverMotor(PIN_DER_FWD, PIN_DER_BWD, 0);
  moverMotor(PIN_ALT_UP, PIN_ALT_DOWN, 0);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // Forzar I2C Fast Mode para ambos sensores

  iniciarMPU6050(); 

  lox.begin();
  lox.setMeasurementTimingBudgetMicroSeconds(20000); 

  WiFi.mode(WIFI_AP);
  WiFi.softAP("Zephyr-Prime", "vuelo2026", 6, 0, 2);

  server.on("/", []() { server.send(200, "text/html", HTML_UI); });
  
  server.on("/a", []() {
    target_altitud = server.arg("a").toInt(); 
    target_giro = server.arg("g").toInt();
    target_empuje = server.arg("e").toInt();
    flight_mode = server.arg("m").toInt();
    ultimoComando = millis();
    
    // Siempre devolvemos la altitud real al cliente para la función Bumpless Transfer
    server.send(200, "text/plain", String(distancia_laser_mm / 10));
  });

  server.on("/tune", []() {
    Kp_y = server.arg("yp").toFloat(); Ki_y = server.arg("yi").toFloat(); Kd_y = server.arg("yd").toFloat();
    limite_pwm_yaw = server.arg("yl").toInt(); max_rampa_yaw = server.arg("yr").toInt();
    
    Kp_z = server.arg("zp").toFloat(); Ki_z = server.arg("zi").toFloat(); Kd_z = server.arg("zd").toFloat();
    limite_pwm_z = server.arg("zl").toInt(); max_rampa_z = server.arg("zr").toInt();
    umbral_ruido_z = server.arg("zg").toInt(); banda_muerta_z = server.arg("zb").toInt(); max_salto_mm = server.arg("zo").toInt();
    
    log_buffer += "<b>[SISTEMA] Dual-Tuning Aplicado con Exito.</b><br>";
    server.send(200, "text/plain", "OK");
  });

  server.on("/cal", []() {
    calibracion_pendiente = true;
    tiempo_inicio_calibracion = millis();
    segundos_restantes = 3;
    log_buffer += "<b>[SISTEMA] Iniciando recalibracion... ¡Suelte la nave!</b><br>";
    server.send(200, "text/plain", "OK");
  });

  server.on("/log", []() { server.send(200, "text/plain", log_buffer); });

  server.begin();
  Serial.println("[ZEPHYR PRIME] RC_FINAL: Merge Completo YAW + ALT Activo.");
}

void loop() {
  server.handleClient();
  ejecutarLeyesDeControl();
}