/*
 * SmartPet v2 — ESP8266 + Servo 9g + LCD 16x2 I2C
 *
 * Bibliotecas necessárias (Library Manager):
 *   - ESP8266WebServer  (inclusa no core ESP8266)
 *   - LiquidCrystal_I2C (Frank de Brabander)
 *   - Servo             (inclusa no core ESP8266)
 *   - WiFiManager       (tzapu) — instalar pelo Library Manager
 *
 * Pinos:
 *   LCD SDA → D2 (GPIO4)
 *   LCD SCL → D1 (GPIO5)
 *   Servo   → D4 (GPIO2)
 *
 * Melhorias v2:
 *   1. LCD com 2 ou 3 telas rotativas (3ª ativa so com timer + agendamento simultaneos)
 *   2. Alimentacao nao-bloqueante via state machine (WiFi nunca trava durante o servo)
 *   3. Reconexao automatica de WiFi a cada 10 s
 *   4. Timeout de 20 s no setup (nao trava para sempre sem rede)
 *   5. Modo offline: servo + LCD funcionam mesmo sem WiFi
 *   6. Horarios fixos salvos em EEPROM (persistem apos queda de energia/WiFi)
 */

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <time.h>

// Pino do botao de reset WiFi (GPIO0 = botao FLASH da maioria das placas)
#define RESET_BTN_PIN 0

// Fuso horario: Manaus / AM (UTC-4)
const long GMT_OFFSET_SEC      = -4 * 3600;
const int  DAYLIGHT_OFFSET_SEC = 0;

// Objetos
ESP8266WebServer  server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo             servoMotor;

// Pinos
#define SDA_PIN   D2
#define SCL_PIN   D1
#define SERVO_PIN D4

// Constantes
int           anguloFechado  = 0;
int           anguloAberto   = 180;
int           sweepStepDeg  = 1;    // graus por passo (1=suave, max=range total)
int           servoPos       = 0;    // posicao atual do servo
unsigned long portionMs      = 3000UL;
const unsigned long LCD_TROCA_MS   = 2500UL;
const unsigned long TRAVA_MANUAL_MS = 2500UL;

// State machine de alimentacao
enum FeedState { FEED_IDLE, FEED_OPENING, FEED_HOLDING, FEED_CLOSING };
FeedState     feedState       = FEED_IDLE;
unsigned long feedTimer       = 0;
bool          feedRequested   = false;
String        requestSource   = "";
unsigned long lastManualReqMs = 0;

bool isBusy() { return feedState != FEED_IDLE; }

// LCD
// lcdPage = -1 faz o primeiro (lcdPage+1)%N = 0 (tela 1 primeiro)
int           lcdPage      = -1;
unsigned long lastLcdChange = 0;

// Timer unico
bool   timerActive      = false;
time_t timerTargetEpoch = 0;

// Horarios fixos
const int MAX_HORARIOS = 4;
bool scheduleEnabled[MAX_HORARIOS] = {false, false, false, false};
int  scheduleHour   [MAX_HORARIOS] = {8, 12, 18, 22};
int  scheduleMinute [MAX_HORARIOS] = {0,  0,  0,  0};
int  lastExecDay    [MAX_HORARIOS] = {-1, -1, -1, -1};

// EEPROM
#define EEPROM_MAGIC  0xAB   // incrementado: removido sweepStepMs, mantido sweepStepDeg
#define EEPROM_SIZE   64

struct ScheduleData {
  uint8_t  magic;
  bool     enabled[MAX_HORARIOS];
  uint8_t  hour   [MAX_HORARIOS];
  uint8_t  minute [MAX_HORARIOS];
  uint16_t portionMs;
  uint8_t  anguloAberto;
  uint8_t  anguloFechado;
  uint8_t  sweepStepDeg;
};

// =====================================================================
// EEPROM — salvar / carregar horarios
// =====================================================================

void saveSchedules() {
  ScheduleData d;
  d.magic        = EEPROM_MAGIC;
  d.portionMs    = (uint16_t)constrain(portionMs, 500, 8000);
  d.anguloAberto  = (uint8_t)constrain(anguloAberto,  0, 180);
  d.anguloFechado = (uint8_t)constrain(anguloFechado, 0, 180);
  d.sweepStepDeg = (uint8_t)constrain(sweepStepDeg, 1, 20);
  for (int i = 0; i < MAX_HORARIOS; i++) {
    d.enabled[i] = scheduleEnabled[i];
    d.hour[i]    = (uint8_t)scheduleHour[i];
    d.minute[i]  = (uint8_t)scheduleMinute[i];
  }
  EEPROM.put(0, d);
  EEPROM.commit();
  Serial.println("[EEPROM] Dados salvos.");
}

void loadSchedules() {
  ScheduleData d;
  EEPROM.get(0, d);
  if (d.magic != EEPROM_MAGIC) {
    Serial.println("[EEPROM] Sem dados validos. Usando padroes.");
    return;
  }
  for (int i = 0; i < MAX_HORARIOS; i++) {
    scheduleEnabled[i] = d.enabled[i];
    scheduleHour[i]    = d.hour[i];
    scheduleMinute[i]  = d.minute[i];
  }
  if (d.portionMs >= 500 && d.portionMs <= 8000)
    portionMs = d.portionMs;
  if (d.anguloAberto <= 180)
    anguloAberto = d.anguloAberto;
  if (d.anguloFechado <= 180)
    anguloFechado = d.anguloFechado;
  if (d.sweepStepDeg >= 1 && d.sweepStepDeg <= 20)
    sweepStepDeg = d.sweepStepDeg;
  Serial.println("[EEPROM] Dados carregados.");
}

// =====================================================================
// UTILITARIOS
// =====================================================================

String twoDigits(int n) {
  return (n < 10) ? "0" + String(n) : String(n);
}

bool getLocalTimeSafe(struct tm* info) {
  time_t now = time(nullptr);
  if (now < 100000) return false;
  localtime_r(&now, info);
  return true;
}

String getCurrentTimeStr() {
  struct tm t;
  if (!getLocalTimeSafe(&t)) return "--:--:--";
  return twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min) + ":" + twoDigits(t.tm_sec);
}

// HH:MM do proximo horario fixo habilitado
String getNextScheduleStr() {
  struct tm nowTm;
  if (!getLocalTimeSafe(&nowTm)) return "--:--";

  int nowMin  = nowTm.tm_hour * 60 + nowTm.tm_min;
  int bestIdx = -1, bestMin = 99999;

  for (int i = 0; i < MAX_HORARIOS; i++) {
    if (!scheduleEnabled[i]) continue;
    if (lastExecDay[i] == nowTm.tm_yday) continue;  // ja executou hoje
    int s = scheduleHour[i] * 60 + scheduleMinute[i];
    if (s >= nowMin && s < bestMin) { bestMin = s; bestIdx = i; }
  }
  if (bestIdx >= 0)
    return twoDigits(scheduleHour[bestIdx]) + ":" + twoDigits(scheduleMinute[bestIdx]);

  // amanha: todos os habilitados sao candidatos (independente de lastExecDay)
  bestMin = 99999;
  for (int i = 0; i < MAX_HORARIOS; i++) {
    if (!scheduleEnabled[i]) continue;
    int s = scheduleHour[i] * 60 + scheduleMinute[i];
    if (s < bestMin) { bestMin = s; bestIdx = i; }
  }
  if (bestIdx >= 0)
    return twoDigits(scheduleHour[bestIdx]) + ":" + twoDigits(scheduleMinute[bestIdx]);

  return "--:--";
}

// HH:MM:SS do timer ativo
String getTimerTargetStr() {
  if (!timerActive) return "--:--:--";
  struct tm t;
  localtime_r(&timerTargetEpoch, &t);
  return twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min) + ":" + twoDigits(t.tm_sec);
}

// Retorna o mais proximo entre timer e agendamento (comparacao por epoca)
String getNextOpeningStr() {
  bool hasTimer    = timerActive;
  bool hasSchedule = false;
  for (int i = 0; i < MAX_HORARIOS; i++)
    if (scheduleEnabled[i]) { hasSchedule = true; break; }

  if (!hasTimer && !hasSchedule) return "Nenhuma";
  if ( hasTimer && !hasSchedule) return getTimerTargetStr();
  if (!hasTimer &&  hasSchedule) return getNextScheduleStr();

  // Ambos ativos: compara epocas
  time_t now = time(nullptr);
  if (now < 100000) return getTimerTargetStr();

  struct tm nowTm;
  localtime_r(&now, &nowTm);

  int nowMin = nowTm.tm_hour * 60 + nowTm.tm_min;
  int bH = -1, bM = -1, bMin = 99999;
  for (int i = 0; i < MAX_HORARIOS; i++) {
    if (!scheduleEnabled[i]) continue;
    if (lastExecDay[i] == nowTm.tm_yday) continue;  // ja executou hoje
    int s = scheduleHour[i] * 60 + scheduleMinute[i];
    if (s >= nowMin && s < bMin) { bMin = s; bH = scheduleHour[i]; bM = scheduleMinute[i]; }
  }
  if (bH < 0) {
    bMin = 99999;
    for (int i = 0; i < MAX_HORARIOS; i++) {
      if (!scheduleEnabled[i]) continue;
      int s = scheduleHour[i] * 60 + scheduleMinute[i];
      if (s < bMin) { bMin = s; bH = scheduleHour[i]; bM = scheduleMinute[i]; }
    }
  }

  time_t schedEpoch = 0;
  if (bH >= 0) {
    struct tm schedTm = nowTm;
    schedTm.tm_hour = bH; schedTm.tm_min = bM; schedTm.tm_sec = 0;
    schedEpoch = mktime(&schedTm);
    if (schedEpoch <= now) schedEpoch += 86400;
  }

  if (schedEpoch == 0 || timerTargetEpoch <= schedEpoch)
    return getTimerTargetStr();
  return getNextScheduleStr();
}

// =====================================================================
// LCD
// =====================================================================

void lcdShow(const String& l1, const String& l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1.substring(0, 16));
  lcd.setCursor(0, 1); lcd.print(l2.substring(0, 16));
}

/*
 * Telas rotativas (troca a cada LCD_TROCA_MS):
 *
 *   Tela 0 (sempre):  "SmartPet HH:MM"    /  "WiFi:SSID"
 *   Tela 1 (sempre):  "IP:xxx.xxx.xxx.x"  /  "Prox:HH:MM"
 *   Tela 2 (so com timer E agendamento):
 *                     "Timer:HH:MM"        /  "Agend:HH:MM"
 */
void showNormalLCD() {
  unsigned long now = millis();
  if (now - lastLcdChange < LCD_TROCA_MS) return;
  lastLcdChange = now;

  bool hasTimer    = timerActive;
  bool hasSchedule = false;
  for (int i = 0; i < MAX_HORARIOS; i++)
    if (scheduleEnabled[i]) { hasSchedule = true; break; }

  int totalPages = (hasTimer && hasSchedule) ? 4 : 3;
  lcdPage = (lcdPage + 1) % totalPages;

  struct tm t;
  bool timeOk = getLocalTimeSafe(&t);

  switch (lcdPage) {
    case 0: {
      String hhmm = timeOk
        ? twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min)
        : "--:--";
      lcdShow("SmartPet " + hhmm, "WiFi:" + WiFi.SSID());
      break;
    }
    case 1: {
      lcdShow("IP:", WiFi.localIP().toString());
      break;
    }
    case 2: {
      lcdShow("Prox:", getNextOpeningStr());
      break;
    }
    case 3: {
      lcdShow("Timer:" + getTimerTargetStr(), "Agend:" + getNextScheduleStr());
      break;
    }
  }
}

void showConnectingLCD() { lcdShow("SmartPet",    "Conectando..."); }
void showFeedingLCD()    { lcdShow("Alimentando", requestSource);   }
void showClosingLCD()    { lcdShow("Fechando",    "Aguarde...");     }

void showTimerSetLCD() {
  if (!timerActive) return;
  struct tm t;
  localtime_r(&timerTargetEpoch, &t);
  lcdShow("Timer ativo",
    twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min) + ":" + twoDigits(t.tm_sec));
}

// =====================================================================
// ALIMENTACAO — state machine nao-bloqueante
// =====================================================================

void requestFeed(String source) {
  if (isBusy() || feedRequested) return;
  feedRequested = true;
  requestSource = source;
}

void processFeed() {
  switch (feedState) {

    case FEED_IDLE:
      if (!feedRequested) return;
      feedRequested = false;
      feedState = FEED_OPENING;
      feedTimer = millis();
      servoMotor.attach(SERVO_PIN);   // liga o sinal apenas quando vai mover
      showFeedingLCD();
      Serial.println("[Feed] Abrindo: " + requestSource);
      break;

    case FEED_OPENING:
      if (millis() - feedTimer < 1UL) return;
      feedTimer = millis();
      if (servoPos != anguloAberto) {
        int step = (anguloAberto > servoPos) ? sweepStepDeg : -sweepStepDeg;
        servoPos += step;
        if ((step > 0 && servoPos > anguloAberto) || (step < 0 && servoPos < anguloAberto))
          servoPos = anguloAberto;
        servoMotor.write(servoPos);
      } else {
        feedState = FEED_HOLDING;
        feedTimer = millis();
      }
      break;

    case FEED_HOLDING:
      if (millis() - feedTimer < portionMs) return;
      feedState = FEED_CLOSING;
      feedTimer = millis();
      showClosingLCD();
      break;

    case FEED_CLOSING:
      if (millis() - feedTimer < 1UL) return;
      if (servoPos != anguloFechado) {
        feedTimer = millis();
        int step = (anguloFechado > servoPos) ? sweepStepDeg : -sweepStepDeg;
        servoPos += step;
        if ((step > 0 && servoPos > anguloFechado) || (step < 0 && servoPos < anguloFechado))
          servoPos = anguloFechado;
        servoMotor.write(servoPos);
      } else if (millis() - feedTimer >= 300UL) {
        servoMotor.detach();
        feedState     = FEED_IDLE;
        requestSource = "";
        lcdPage       = -1;
        lastLcdChange = millis() - LCD_TROCA_MS;
        Serial.println("[Feed] Concluido.");
      }
      break;
  }
}

// =====================================================================
// WEB — handlers
// =====================================================================

String htmlPage();

void handleApiStatus() {
  time_t now = time(nullptr);
  struct tm t;
  String ts = "--:--:--";
  if (now > 100000) {
    localtime_r(&now, &t);
    ts = twoDigits(t.tm_hour) + ":" + twoDigits(t.tm_min) + ":" + twoDigits(t.tm_sec);
  }
  String json = "{\"time\":\"" + ts + "\"";
  json += ",\"busy\":"        + String(isBusy()        ? "true" : "false");
  json += ",\"feedReq\":"     + String(feedRequested   ? "true" : "false");
  json += ",\"timerActive\":" + String(timerActive     ? "true" : "false");
  json += ",\"timerEpoch\":"  + String((long)timerTargetEpoch);
  json += ",\"serverEpoch\":" + String((long)now);
  json += ",\"nextSched\":\"" + getNextScheduleStr() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleManualFeed() {
  unsigned long nowMs = millis();
  if (isBusy() || feedRequested) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Sistema ocupado. Aguarde.\"}" );
    return;
  }
  if (nowMs - lastManualReqMs < TRAVA_MANUAL_MS) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Aguarde antes de repetir.\"}" );
    return;
  }
  lastManualReqMs = nowMs;
  requestFeed("Manual");
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Alimentacao agendada!\"}");
}

void handleSetTimer() {
  if (!server.hasArg("th") || !server.hasArg("tm") || !server.hasArg("ts")) {
    server.send(400, "text/plain", "Parametros ausentes.");
    return;
  }
  int h = server.arg("th").toInt();
  int m = server.arg("tm").toInt();
  int s = server.arg("ts").toInt();

  if (h < 0 || m < 0 || s < 0 || m > 59 || s > 59) {
    server.send(400, "text/plain", "Valores invalidos.");
    return;
  }
  unsigned long total = (unsigned long)h * 3600UL + (unsigned long)m * 60UL + s;
  if (total == 0) {
    timerActive = false; timerTargetEpoch = 0;
    server.send(200, "text/html",
      "<html><body><h2>Timer cancelado</h2><a href='/'>Voltar</a></body></html>");
    return;
  }
  time_t now = time(nullptr);
  if (now < 100000) {
    server.send(200, "text/html",
      "<html><body><h2>Hora indisponivel</h2>"
      "<p>NTP ainda sincronizando.</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  timerTargetEpoch = now + total;
  timerActive      = true;
  showTimerSetLCD();
  server.send(200, "text/html",
    "<html><body><h2>Timer configurado</h2>"
    "<p>Disparo unico registrado.</p><a href='/'>Voltar</a></body></html>");
}

void handleSetSchedules() {
  for (int i = 0; i < MAX_HORARIOS; i++) {
    scheduleEnabled[i] = server.hasArg("en" + String(i));
    if (server.hasArg("h" + String(i)) && server.hasArg("m" + String(i))) {
      int h = server.arg("h" + String(i)).toInt();
      int m = server.arg("m" + String(i)).toInt();
      if (h >= 0 && h <= 23 && m >= 0 && m <= 59) {
        if (h != scheduleHour[i] || m != scheduleMinute[i])
          lastExecDay[i] = -1;  // horario mudou: permite disparar hoje no novo horario
        scheduleHour[i]   = h;
        scheduleMinute[i] = m;
      }
    }
  }
  saveSchedules();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleResetWifi() {
  lcdShow("Trocando WiFi", "Reiniciando...");
  server.send(200, "text/plain", "ok");
  delay(400);
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

// Move servo direto para um angulo — usado apenas no modo teste da calibracao
void handleServoGoto() {
  if (!server.hasArg("deg")) {
    server.send(400, "text/plain", "Parametro ausente.");
    return;
  }
  int deg = server.arg("deg").toInt();
  if (deg < 0 || deg > 180) {
    server.send(400, "text/plain", "Valor fora do intervalo (0-180).");
    return;
  }
  if (isBusy() || feedRequested) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Sistema ocupado.\"}");
    return;
  }
  int startPos = servoPos;
  int step = (deg > servoPos) ? 1 : -1;
  servoMotor.attach(SERVO_PIN);
  while (servoPos != deg) {
    servoPos += step;
    servoMotor.write(servoPos);
    delay(2);
  }
  // Aguarda o servo fisicamente chegar antes de desligar o sinal
  int traveled = abs(deg - startPos);
  delay(max(200, traveled * 2));
  servoMotor.detach();
  server.send(200, "application/json", "{\"ok\":true,\"deg\":" + String(deg) + "}");
}

void handleSetServoConfig() {
  if (!server.hasArg("ao") || !server.hasArg("af") || !server.hasArg("sd")) {
    server.send(400, "text/plain", "Parametros ausentes.");
    return;
  }
  int ao = server.arg("ao").toInt();
  int af = server.arg("af").toInt();
  int sd = server.arg("sd").toInt();
  if (ao < 0 || ao > 180 || af < 0 || af > 180 || sd < 1 || sd > 20) {
    server.send(400, "text/plain", "Valores invalidos.");
    return;
  }
  anguloAberto  = ao;
  anguloFechado = af;
  sweepStepDeg  = sd;
  saveSchedules();
  // Retorna para posicao fechada usando o passo configurado
  servoMotor.attach(SERVO_PIN);
  while (servoPos != anguloFechado) {
    int delta = anguloFechado - servoPos;
    int move  = (abs(delta) >= sweepStepDeg) ? sweepStepDeg : abs(delta);
    servoPos += (delta > 0) ? move : -move;
    servoMotor.write(servoPos);
    delay(2);
  }
  // Aguarda o servo fisicamente chegar antes de desligar o sinal
  delay(300);
  servoMotor.detach();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSetPortion() {
  if (!server.hasArg("ms")) {
    server.send(400, "text/plain", "Parametro ausente.");
    return;
  }
  int ms = server.arg("ms").toInt();
  if (ms < 500 || ms > 8000) {
    server.send(400, "text/plain", "Valor fora do intervalo (500-8000ms).");
    return;
  }
  portionMs = (unsigned long)ms;
  saveSchedules();
  server.send(200, "application/json", "{\"ok\":true,\"ms\":" + String(portionMs) + "}");
}

String htmlPage() {
  String html = "<!DOCTYPE html><html lang='pt-BR'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
  html += "<title>SmartPet</title><style>";
  html += ":root{--p:#0b7a75;--pd:#085e5a;--d:#e53935;--w:#f57c00;--bg:#f0f4f8;--card:#fff;--bdr:#e0e0e0;--mu:#777;--r:14px;--sh:0 2px 12px rgba(0,0,0,.08);}";
  html += "*{box-sizing:border-box;margin:0;padding:0;}";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:var(--bg);color:#1a1a2e;padding:16px;}";
  html += ".wrap{max-width:560px;margin:0 auto;}";
  html += ".hdr{margin-bottom:16px;}h1{font-size:1.7rem;color:var(--p);font-weight:700;}";
  html += ".sub{color:var(--mu);font-size:.85rem;margin-top:2px;}";
  html += ".card{background:var(--card);border-radius:var(--r);box-shadow:var(--sh);padding:18px 16px;margin-bottom:14px;}";
  html += ".ct{font-size:1rem;font-weight:700;color:var(--p);padding-bottom:10px;margin-bottom:14px;border-bottom:2px solid var(--bdr);}";
  html += ".ig{display:grid;grid-template-columns:auto 1fr;gap:6px 14px;font-size:.9rem;align-items:center;}";
  html += ".ig b{color:var(--mu);font-weight:600;}";
  html += ".badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.8rem;font-weight:700;}";
  html += ".bok{background:#e8f5e9;color:#2e7d32;}.bsy{background:#fff3e0;color:var(--w);}.bq{background:#e3f2fd;color:#1565c0;}";
  html += "button{display:inline-flex;align-items:center;justify-content:center;border:none;border-radius:10px;font-size:.95rem;font-weight:600;padding:12px 18px;cursor:pointer;transition:opacity .15s;width:100%;}";
  html += "button:hover{opacity:.85;}button:disabled{background:#bdbdbd!important;cursor:not-allowed;opacity:1;}";
  html += ".bp{background:var(--p);color:#fff;}.bd{background:var(--d);color:#fff;}.bw{background:var(--w);color:#fff;}";
  html += ".br{display:flex;gap:10px;flex-wrap:wrap;margin-top:12px;}.br button{flex:1;min-width:120px;}";
  html += "input[type=number]{border:2px solid var(--bdr);border-radius:8px;padding:8px;font-size:.95rem;width:66px;text-align:center;}";
  html += "input[type=number]:focus{border-color:var(--p);outline:none;}";
  html += "input[type=range]{width:100%;accent-color:var(--p);margin:6px 0;}";
  html += ".pr{display:flex;align-items:center;gap:14px;margin:10px 0 2px;}.pv{font-size:1.5rem;font-weight:700;color:var(--p);min-width:52px;}.prf{flex:1;}";
  html += ".rl{display:flex;justify-content:space-between;font-size:.78rem;color:#aaa;margin-bottom:12px;}";
  html += ".cd{border-radius:12px;padding:16px;text-align:center;margin:14px 0;}";
  html += ".cd-on{background:#e8f5e9;}.cd-off{background:#f5f5f5;color:#aaa;}.cd-done{background:#fdecea;color:var(--d);}";
  html += ".cdn{font-size:2.2rem;font-weight:700;letter-spacing:3px;}.cdl{font-size:.75rem;color:var(--mu);margin-top:4px;}";
  html += ".ti{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:14px;}";
  html += ".ti label{display:flex;flex-direction:column;align-items:center;gap:5px;font-size:.82rem;color:var(--mu);font-weight:600;}";
  html += ".si{display:flex;align-items:center;gap:10px;padding:10px 0;border-bottom:1px solid var(--bdr);flex-wrap:wrap;}";
  html += ".si:last-of-type{border-bottom:none;}";
  html += "input[type=checkbox]{width:18px;height:18px;accent-color:var(--p);flex-shrink:0;}";
  html += ".sit{font-size:.88rem;color:var(--mu);}.sih{display:flex;align-items:center;gap:4px;}";
  html += ".stag{display:inline-block;padding:2px 8px;border-radius:12px;font-size:.72rem;font-weight:700;margin-left:4px;}";
  html += ".stag-ok{background:#e8f5e9;color:#2e7d32;}.stag-pd{background:#fff8e1;color:#f57c00;}";
  html += ".si-dis{opacity:.4;}";
  html += ".calib-row{display:flex;align-items:center;gap:12px;margin-bottom:14px;}";
  html += ".calib-lbl{font-size:.85rem;color:#777;font-weight:600;min-width:110px;}";
  html += ".calib-val{font-size:1.1rem;font-weight:700;color:var(--p);min-width:42px;text-align:right;}";
  html += ".calib-range{flex:1;}";
  html += ".msg{font-size:.88rem;font-weight:600;margin-top:10px;min-height:20px;}";
  html += ".wrow{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap;margin-top:14px;}";
  html += "@media(max-width:400px){.cdn{font-size:1.6rem;}.pv{font-size:1.2rem;}}";
  html += "</style>";

  // --- JS ---
  html += "<script>";
  html += "var T=" + String((long)timerTargetEpoch) + ";";
  html += "var N=" + String((long)time(nullptr)) + ";";
  html += "var R=Date.now()/1000;";
  html += "function sl(){return T>0?(T-N)-(Date.now()/1000-R):-1;}";
  html += "function p(n){return n<10?'0'+n:''+n;}";
  html += "function fc(s){if(s<=0)return'--:--:--';return p(Math.floor(s/3600))+':'+p(Math.floor(s%3600/60))+':'+p(Math.floor(s%60));}";
  html += "var ci=null;";
  html += "function startCd(){if(ci)clearInterval(ci);ci=setInterval(function(){";
  html += "var r=sl(),e=document.getElementById('cd'),b=document.getElementById('cdBox');";
  html += "if(r<=0){e.innerText='Disparado!';b.className='cd cd-done';clearInterval(ci);ci=null;}";
  html += "else e.innerText=fc(r);},500);}";
  html += "function saveTimer(f){var h=+f.th.value||0,m=+f.tm.value||0,s=+f.ts.value||0,tot=h*3600+m*60+s;";
  html += "var btn=f.querySelector('button[type=submit]');btn.disabled=true;btn.innerText='Aguarde...';";
  html += "fetch('/setTimer?th='+h+'&tm='+m+'&ts='+s).then(function(){";
  html += "if(tot===0){T=0;document.getElementById('cd').innerText='Inativo';document.getElementById('cdBox').className='cd cd-off';if(ci){clearInterval(ci);ci=null;}}";
  html += "else{T=N+(Date.now()/1000-R)+tot;document.getElementById('cdBox').className='cd cd-on';startCd();}";
  html += "btn.disabled=false;btn.innerText='Salvar timer';}).catch(function(){btn.disabled=false;btn.innerText='Salvar timer';});return false;}";
  html += "function cancelTimer(){fetch('/setTimer?th=0&tm=0&ts=0').then(function(){";
  html += "T=0;document.getElementById('cd').innerText='Inativo';document.getElementById('cdBox').className='cd cd-off';";
  html += "if(ci){clearInterval(ci);ci=null;}});}";
  html += "function saveSchedules(f){var q='',ns=f.querySelectorAll('input[type=number]'),cs=f.querySelectorAll('input[type=checkbox]');";
  html += "for(var i=0;i<ns.length;i++)q+='&'+ns[i].name+'='+encodeURIComponent(ns[i].value);";
  html += "for(var i=0;i<cs.length;i++)if(cs[i].checked)q+='&'+cs[i].name+'=on';";
  html += "var btn=f.querySelector('button[type=submit]');btn.disabled=true;btn.innerText='Salvando...';";
  html += "fetch('/setSchedules?'+q.substring(1)).then(function(){";
  html += "var mg=document.getElementById('schedMsg');mg.innerText='Salvo!';mg.style.color='#0b7a75';";
  html += "setTimeout(function(){mg.innerText='';},3000);btn.disabled=false;btn.innerText='Salvar horarios';})";
  html += ".catch(function(){document.getElementById('schedMsg').innerText='Erro.';btn.disabled=false;btn.innerText='Salvar horarios';});return false;}";
  html += "function manualFeed(btn){btn.disabled=true;btn.innerText='Aguarde...';";
  html += "fetch('/manual').then(function(r){return r.json();}).then(function(d){";
  html += "var mg=document.getElementById('manualMsg');mg.innerText=d.msg;mg.style.color=d.ok?'#0b7a75':'#e53935';";
  html += "setTimeout(function(){mg.innerText='';},3000);btn.disabled=false;btn.innerText='Alimentar agora';})";
  html += ".catch(function(){document.getElementById('manualMsg').innerText='Erro de comunicacao.';btn.disabled=false;btn.innerText='Alimentar agora';});}";
  html += "function updatePortion(v){document.getElementById('portionVal').innerText=(v/1000).toFixed(1)+'s';}";
  html += "function updateAngle(id,v){document.getElementById(id).innerText=v+'°';}";
  html += "function updateStepDeg(v){var d=parseInt(v);document.getElementById('stepDegVal').innerText=d+(d==1?' grau':' graus');}";  html += "function testServo(sliderId,btnId,lbl){";
  html += "var deg=document.getElementById(sliderId).value;";
  html += "var btn=document.getElementById(btnId);btn.disabled=true;btn.innerText='Movendo...';";
  html += "fetch('/servoGoto?deg='+deg).then(function(r){return r.json();}).then(function(d){";
  html += "btn.disabled=false;btn.innerText=lbl;}).catch(function(){btn.disabled=false;btn.innerText=lbl;});}";
  html += "function saveServo(){";
  html += "var ao=document.getElementById('slAo').value;";
  html += "var af=document.getElementById('slAf').value;";
  html += "var sd=document.getElementById('slSd').value;";
  html += "var btn=document.getElementById('servoBtn');btn.disabled=true;btn.innerText='Salvando...';";
  html += "fetch('/setServo?ao='+ao+'&af='+af+'&sd='+sd)";
  html += ".then(function(r){return r.json();}).then(function(d){";
  html += "var mg=document.getElementById('servoMsg');mg.innerText=d.ok?'Salvo!':'Erro.';";
  html += "mg.style.color=d.ok?'#0b7a75':'#e53935';";
  html += "setTimeout(function(){mg.innerText='';},3000);";
  html += "btn.disabled=false;btn.innerText='Salvar calibracao';})";
  html += ".catch(function(){btn.disabled=false;btn.innerText='Salvar calibracao';});}";
  html += "function savePortion(){var v=document.getElementById('portionSlider').value;";
  html += "var btn=document.getElementById('portionBtn');btn.disabled=true;btn.innerText='Salvando...';";
  html += "fetch('/setPortion?ms='+v).then(function(r){return r.json();}).then(function(d){";
  html += "var mg=document.getElementById('portionMsg');mg.innerText=d.ok?'Salvo!':'Erro.';mg.style.color=d.ok?'#0b7a75':'#e53935';";
  html += "setTimeout(function(){mg.innerText='';},3000);btn.disabled=false;btn.innerText='Salvar porcao';})";
  html += ".catch(function(){btn.disabled=false;btn.innerText='Salvar porcao';});}";
  html += "function resetWifi(){if(!confirm('Desconectar do WiFi atual?\\nO dispositivo vai reiniciar no modo AP (SmartPet-Setup / 192.168.4.1).'))return;";
  html += "fetch('/resetWifi').then(function(){document.getElementById('wifiMsg').innerText='Reiniciando... conecte no AP SmartPet-Setup e acesse 192.168.4.1';}).catch(function(){});}";
  html += "setInterval(function(){fetch('/api/status').then(function(r){return r.json();}).then(function(d){";
  html += "var cl=document.getElementById('liveClock');if(cl)cl.innerText=d.time;";
  html += "var st=document.getElementById('liveStatus');if(st){st.innerText=d.busy?'Alimentando':(d.feedReq?'Na fila':'Pronto');";
  html += "st.className='badge '+(d.busy?'bsy':(d.feedReq?'bq':'bok'));}";
  html += "var ns=document.getElementById('nextSchedSpan');if(ns)ns.innerText=d.nextSched||'--:--';";
  html += "if(!d.timerActive&&T>0&&ci===null){T=0;var ex=document.getElementById('cd');if(ex)ex.innerText='Disparado!';var bx=document.getElementById('cdBox');if(bx)bx.className='cd cd-done';}";
  html += "if(d.timerActive&&d.timerEpoch>0&&T===0){T=d.timerEpoch;N=d.serverEpoch;R=Date.now()/1000;";
  html += "if(sl()>0){var bx2=document.getElementById('cdBox');if(bx2)bx2.className='cd cd-on';startCd();}}";
  html += "}).catch(function(){});},2000);";
  html += "function syncRow(cb){cb.closest('.si').classList.toggle('si-dis',!cb.checked);}";
  html += "window.onload=function(){";
  html += "document.querySelectorAll('.si input[type=checkbox]').forEach(function(cb){syncRow(cb);});";
  html += "if(T>0&&sl()>0)startCd();};";
  html += "</script></head><body><div class='wrap'>";

  // Header
  html += "<div class='hdr'><h1>SmartPet</h1><p class='sub'>Alimentador automatico</p></div>";

  // Status card
  String stCls  = isBusy() ? "bsy" : (feedRequested ? "bq" : "bok");
  String stText = isBusy() ? "Alimentando" : (feedRequested ? "Na fila" : "Pronto");
  html += "<div class='card'><p class='ct'>Status</p>";
  html += "<div class='ig'>";
  html += "<b>Rede</b><span>" + WiFi.SSID() + "</span>";
  html += "<b>IP</b><span>" + WiFi.localIP().toString() + "</span>";
  html += "<b>Hora</b><span id='liveClock'>" + getCurrentTimeStr() + "</span>";
  html += "<b>Estado</b><span id='liveStatus' class='badge " + stCls + "'>" + stText + "</span>";
  html += "</div>";
  html += "<div class='wrow'><button class='bd' style='width:auto;padding:10px 16px;font-size:.85rem' onclick='resetWifi()'>Trocar WiFi</button></div>";
  html += "<p id='wifiMsg' class='msg'></p></div>";

  // Alimentar agora
  html += "<div class='card'><p class='ct'>Alimentar agora</p>";
  html += "<button class='bp' type='button' onclick='manualFeed(this)'>Alimentar agora</button>";
  html += "<p id='manualMsg' class='msg'></p></div>";

  // Tamanho da porcao
  String curPs = String(portionMs / 1000.0f, 1) + "s";
  html += "<div class='card'><p class='ct'>Tamanho da porcao</p>";
  html += "<p style='font-size:.85rem;color:#777;margin-bottom:8px'>Tempo de abertura do servo por alimentacao.</p>";
  html += "<div class='pr'><span class='pv' id='portionVal'>" + curPs + "</span>";
  html += "<div class='prf'><input type='range' id='portionSlider' min='500' max='8000' step='100' value='" + String(portionMs) + "' oninput='updatePortion(this.value)'></div></div>";
  html += "<div class='rl'><span>0.5s (pouco)</span><span>8.0s (muito)</span></div>";
  html += "<button id='portionBtn' class='bp' onclick='savePortion()'>Salvar porcao</button>";
  html += "<p id='portionMsg' class='msg'></p></div>";

  // Calibracao do servo
  html += "<div class='card'><p class='ct'>Calibracao do Servo</p>";
  html += "<p style='font-size:.85rem;color:#777;margin-bottom:14px'>Ajuste os angulos e a velocidade de movimento. Use os botoes de teste para verificar o ponto exato.</p>";

  html += "<div class='calib-row'>";
  html += "<span class='calib-lbl'>Angulo aberto</span>";
  html += "<div class='calib-range'><input type='range' id='slAo' min='0' max='180' step='1' value='" + String(anguloAberto) + "' oninput='updateAngle(\"aoVal\",this.value)'></div>";
  html += "<span class='calib-val' id='aoVal'>" + String(anguloAberto) + "&#176;</span>";
  html += "<button id='btnTestAo' class='bp' style='width:auto;padding:10px 14px;font-size:.82rem' onclick='testServo(\"slAo\",\"btnTestAo\",\"Testar abertura\")'>Testar abertura</button>";
  html += "</div>";

  html += "<div class='calib-row'>";
  html += "<span class='calib-lbl'>Angulo fechado</span>";
  html += "<div class='calib-range'><input type='range' id='slAf' min='0' max='180' step='1' value='" + String(anguloFechado) + "' oninput='updateAngle(\"afVal\",this.value)'></div>";
  html += "<span class='calib-val' id='afVal'>" + String(anguloFechado) + "&#176;</span>";
  html += "<button id='btnTestAf' class='bp' style='width:auto;padding:10px 14px;font-size:.82rem' onclick='testServo(\"slAf\",\"btnTestAf\",\"Testar fechamento\")'>Testar fechamento</button>";
  html += "</div>";

  html += "<div class='calib-row'>";
  html += "<span class='calib-lbl'>Passo</span>";
  html += "<div class='calib-range'>";
  html += "<input type='range' id='slSd' min='1' max='20' step='1' value='" + String(sweepStepDeg) + "' oninput='updateStepDeg(this.value)'>";
  html += "<div style='display:flex;justify-content:space-between;font-size:.75rem;color:#aaa;margin-top:2px'><span>Suave (1&#176;)</span><span>Rapido (20&#176;)</span></div>";
  html += "</div>";
  html += "<span class='calib-val' id='stepDegVal'>" + String(sweepStepDeg) + (sweepStepDeg == 1 ? " grau" : " graus") + "</span>";
  html += "</div>";

  html += "<button id='servoBtn' class='bp' onclick='saveServo()'>Salvar calibracao</button>";
  html += "<p id='servoMsg' class='msg'></p></div>";

  // Temporizador
  html += "<div class='card'><p class='ct'>Temporizador (disparo unico)</p>";  html += "<form onsubmit='return saveTimer(this)'>";
  html += "<div class='ti'>";
  html += "<label>Horas<input type='number' name='th' min='0' max='23' value='0'></label>";
  html += "<label>Min<input type='number' name='tm' min='0' max='59' value='0'></label>";
  html += "<label>Seg<input type='number' name='ts' min='0' max='59' value='0'></label>";
  html += "</div>";
  html += "<div class='br'><button type='submit' class='bp'>Salvar timer</button>";
  html += "<button type='button' class='bd' onclick='cancelTimer()'>Cancelar timer</button></div>";
  html += "</form>";
  {
    time_t nowEpoch = time(nullptr);
    long rem = timerActive ? (long)(timerTargetEpoch - nowEpoch) : -1L;
    String cdCls, cdTxt;
    if (rem > 0) {
      cdCls = "cd cd-on";
      cdTxt = twoDigits((int)(rem/3600))+":"+twoDigits((int)((rem%3600)/60))+":"+twoDigits((int)(rem%60));
    } else if (timerActive) {
      cdCls = "cd cd-done"; cdTxt = "Disparado!";
    } else {
      cdCls = "cd cd-off";  cdTxt = "Inativo";
    }
    html += "<div id='cdBox' class='" + cdCls + "'>";
    html += "<div class='cdn' id='cd'>" + cdTxt + "</div>";
    html += "<div class='cdl'>tempo ate o proximo disparo</div></div>";
  }
  html += "</div>";

  // Horarios Fixos
  html += "<div class='card'><p class='ct'>Horarios Fixos</p>";
  html += "<p style='font-size:.85rem;color:#777;margin-bottom:4px'>Dispara <b>todo dia</b> no horario definido.</p>";
  html += "<p style='font-size:.85rem;margin-bottom:12px'>Proximo: <b id='nextSchedSpan'>" + getNextScheduleStr() + "</b></p>";
  html += "<form onsubmit='return saveSchedules(this)'>";
  {
    struct tm nowTm;
    bool hasNow = getLocalTimeSafe(&nowTm);
    for (int i = 0; i < MAX_HORARIOS; i++) {
      bool firedToday = hasNow && (lastExecDay[i] == nowTm.tm_yday);
      String rowCls = scheduleEnabled[i] ? "si" : "si si-dis";
      html += "<div class='" + rowCls + "'>";
      html += "<input type='checkbox' name='en" + String(i) + "' onchange='syncRow(this)' " + (scheduleEnabled[i] ? "checked" : "") + ">";
      html += "<span class='sit'>" + String(i+1) + ". todo dia as</span>";
      html += "<div class='sih'>";
      html += "<input type='number' name='h" + String(i) + "' min='0' max='23' value='" + String(scheduleHour[i]) + "' title='hora (0-23)'>";
      html += "<b style='color:#aaa;font-size:1.1rem'>:</b>";
      html += "<input type='number' name='m" + String(i) + "' min='0' max='59' value='" + String(scheduleMinute[i]) + "' title='minuto (0-59)'>";
      html += "</div>";
      if (scheduleEnabled[i]) {
        if (firedToday)
          html += "<span class='stag stag-ok'>Hoje</span>";
        else
          html += "<span class='stag stag-pd'>Aguardando</span>";
      }
      html += "</div>";
    }
  }
  html += "<div style='margin-top:14px'><button type='submit' class='bp'>Salvar horarios</button></div>";
  html += "</form><p id='schedMsg' class='msg'></p></div>";

  html += "</div></body></html>";
  return html;
}

// =====================================================================
// AUTOMACAO
// =====================================================================

void checkTimerTrigger() {
  if (!timerActive || isBusy() || feedRequested) return;
  time_t now = time(nullptr);
  if (now > 100000 && now >= timerTargetEpoch) {
    timerActive = false;
    requestFeed("Timer");
  }
}

void checkSchedulesTrigger() {
  struct tm t;
  if (!getLocalTimeSafe(&t)) return;
  for (int i = 0; i < MAX_HORARIOS; i++) {
    if (!scheduleEnabled[i]) continue;
    if (t.tm_hour == scheduleHour[i] &&
        t.tm_min  == scheduleMinute[i] &&
        lastExecDay[i] != t.tm_yday) {
      lastExecDay[i] = t.tm_yday;  // marca sempre, mesmo se ocupado — evita disparo duplo
      if (!isBusy() && !feedRequested)
        requestFeed("Horario " + String(i + 1));
      break;
    }
  }
}

// Segurar FLASH por 3 s em qualquer momento reseta o WiFi
void checkResetButton() {
  static unsigned long pressedAt = 0;
  static bool          counting  = false;

  if (digitalRead(RESET_BTN_PIN) == LOW) {
    if (!counting) { counting = true; pressedAt = millis(); }
    if (millis() - pressedAt >= 3000UL) {
      lcdShow("WiFi reset!", "Reiniciando...");
      Serial.println("[WiFi] Reset pelo botao FLASH.");
      delay(600);
      WiFiManager wm;
      wm.resetSettings();
      ESP.restart();
    }
  } else {
    counting = false;
  }
}

// Reconexao WiFi (verifica a cada 10 s)
void checkWiFiReconnect() {
  static unsigned long lastCheck  = 0;
  static bool          wasOffline = false;
  if (millis() - lastCheck < 10000UL) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    wasOffline = true;
    Serial.println("[WiFi] Desconectado. Reconectando...");
    WiFi.reconnect();
  } else if (wasOffline) {
    // reconectou apos ter estado offline: garante NTP sincronizado
    wasOffline = false;
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
    Serial.println("[WiFi] Reconectado. NTP re-configurado. IP: " + WiFi.localIP().toString());
    lcdShow("WiFi OK", WiFi.localIP().toString());
  }
}

// =====================================================================
// SETUP
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  EEPROM.begin(EEPROM_SIZE);
  loadSchedules();
  servoPos = anguloFechado;

  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(anguloFechado);
  delay(400);               // tempo para chegar na posicao inicial
  servoMotor.detach();      // desliga o sinal — servo livre e sem vibrar

  // Botao de reset: segurar ao ligar apaga credenciais salvas
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  if (digitalRead(RESET_BTN_PIN) == LOW) {
    lcdShow("WiFi reset!", "Reiniciando...");
    Serial.println("[WiFi] Reset das credenciais solicitado.");
    WiFiManager wm;
    wm.resetSettings();
    delay(1500);
    ESP.restart();
  }

  showConnectingLCD();

  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 min para configurar; apos isso segue offline

  // Callback: avisa no LCD quando o portal AP esta ativo
  wm.setAPCallback([](WiFiManager* wm) {
    lcdShow("Config WiFi", "192.168.4.1");
    Serial.println("[WiFi] Portal ativo: AP 'SmartPet-Setup' — acesse 192.168.4.1");
  });

  bool connected = wm.autoConnect("SmartPet-Setup");
  // Se o usuario nao configurar em 3 min, connected = false (modo offline)

  if (connected) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
    Serial.println("[WiFi] Conectado. IP: " + WiFi.localIP().toString());
    lcdShow("SmartPet", WiFi.localIP().toString());
  } else {
    Serial.println("[WiFi] Sem config. Modo offline.");
    lcdShow("SmartPet", "Sem WiFi");
  }

  server.on("/",             handleRoot);
  server.on("/api/status",   handleApiStatus);
  server.on("/manual",       handleManualFeed);
  server.on("/setTimer",     handleSetTimer);
  server.on("/setSchedules", handleSetSchedules);
  server.on("/setPortion",   handleSetPortion);
  server.on("/resetWifi",    handleResetWifi);
  server.on("/servoGoto",    handleServoGoto);
  server.on("/setServo",     handleSetServoConfig);
  server.begin();

  delay(2000);
  lastLcdChange = millis();
}

// =====================================================================
// LOOP
// =====================================================================

void loop() {
  checkResetButton();
  server.handleClient();
  checkWiFiReconnect();
  checkTimerTrigger();
  checkSchedulesTrigger();
  processFeed();

  if (!isBusy() && !feedRequested) {
    showNormalLCD();
  } 
}