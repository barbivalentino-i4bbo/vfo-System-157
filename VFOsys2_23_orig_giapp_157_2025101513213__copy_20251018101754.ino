/* 
  VFOsys2_23_multifunc.ino
  VFO System Digital - Full Version (troncone 1/3)
  BUILD_NUMBER: 157_multifunc
  NOTA: parte 1 di 3 — includes, variabili globali e setup
*/

#define NAME "VFO System"
#define VERSION "by JF3HZB"
#define ID "Mods AI+I4BBO"
#define BUILD_NUMBER "157"
#define NUM_RIGHE_TESTO 6

#include <Wire.h>
#include <si5351.h>
#include "dial.hpp"
#include "driver/pcnt.h"
#include <EEPROM.h>

// display / grafica
LGFX lcd;
LGFX_Sprite sp, sprites[2];
bool flip;
int sprite_height;
DIAL dial;

// --------- PIN e costanti ----------
#define PULSE_INPUT_PIN 16
#define PULSE_CTRL_PIN 17
#define PIN_PAGE 4
#define STEP_PLUS_PIN 26
#define STEP_MINUS_PIN 27
#define PIN_BANDA_PLUS 12
#define PIN_BANDA_MINUS 13
#define PIN_STEP_RX 32
#define PIN_BFO_MODE 25
#define PIN_VFO_SWITCH 14  // NON TOCCARE
#define PIN_RXTX 33        // pin 33: ciclo VFO-A -> VFO-IQ -> GEN

#define init_frq 7100000
#define max_frq 160000000
#define min_frq 10000

// --------- Variabili globali ----------
int32_t Dial_frq, Dial_frq_A = 7100000, Dial_frq_B = 28200000;
int32_t afstp;
int16_t RE_Count = 0;
bool vfo_is_A = true;
bool in_gen_mode = false;
bool in_step_rx_mode = false;
bool pulsante_premuto = false;
bool stato_prec_gen = HIGH, stato_prec_bfo = HIGH, pin4_stato_prec = HIGH;
bool f_fchange = 1;
int cnt = 2, step = 1000, step_rx_index = 0, riga_attiva = 0;
unsigned long pin4_tempo = 0, tempo_messaggio_banda = 0;
String messaggio_banda = "";

// --- pagina TESTO variabili ---
int32_t if_offset = 9000000;   // IF
int32_t freq_clk1 = 54000000;  // quarzo virtuale (CLK1)
int32_t freq_clk2 = 45000000;  // quarzo virtuale programmabile (CLK2 / BFO in certi modi)

// --- potenza CLK e EEPROM ---
uint8_t clk0_pwr = 3;
uint8_t clk1_pwr = 3;
uint8_t clk2_pwr = 3;
#define ADDR_CLK0_PWR 120
#define ADDR_CLK1_PWR 121
#define ADDR_CLK2_PWR 122

// --- Pagine e step ---
enum Pagina { GRAFICA, TESTO };
Pagina pagina_corrente = GRAFICA;

const int step_rx_values[] = {25, 1000, 10000};
const int step_am_values[] = {250, 2500, 10000};
int step_am_index = 0;
const int NUM_STEP_RX = sizeof(step_rx_values) / sizeof(step_rx_values[0]);

// --- Bande ---
const int NUM_BANDE = 26;
const int32_t bande[NUM_BANDE] = {
  1835000, 3650000, 5360000, 7100000, 10120000, 14070000,
  18100000, 21200000, 24940000, 28400000,
  300000, 1000000, 27200000,
  2400000, 3300000, 3950000, 4900000, 6100000, 7200000,
  9600000, 11850000, 13700000, 15450000, 17700000,
  21650000, 25850000
};
const char* nomi_bande[NUM_BANDE] = {
  "160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m",
  "LW", "MW", "CB",
  "SW120m", "SW90m", "SW75m", "SW60m", "SW49m", "SW41m",
  "SW31m", "SW25m", "SW22m", "SW19m", "SW16m",
  "SW13m", "SW11m"
};
int banda_corrente = 0;

// --- BFO e offset ---
int modalita_bfo = 0;
const char* nomi_bfo[] = {"USB", "LSB", "CW", "AM"};
long offset_bfo[] = {1500, -1500, 800, 0};

// --- Si5351 wrapper ---
Si5351 si5351;

// Forward declarations (tronconi successivi)
void aggiorna_clk1();
void aggiorna_clk2();
void aggiorna_bfo();
void disegna_grafica();
void gestisci_pagina_testo();
void gestisci_step_rx();
void gestisci_step();
void gestisci_banda();
void gestisci_bfo();

// ---------- Helper Si5351 ----------
void si5351_init() {
  Wire.begin();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
}

void set_car_freq(uint32_t freq, uint8_t on, uint8_t clk) {
  si5351_clock clk_id;
  if (clk == 0) clk_id = SI5351_CLK0;
  else if (clk == 1) clk_id = SI5351_CLK1;
  else clk_id = SI5351_CLK2;

  if (on) si5351.set_freq((uint64_t)freq * 100ULL, clk_id);
  si5351.output_enable(clk_id, on);
}

// Helper per cambiare potenza CLK e memorizzazione EEPROM
void cambia_potenza_clk(uint8_t &clk_pwr, uint8_t addr, int16_t count, si5351_clock clk) {
  clk_pwr = (clk_pwr + count + 4) % 4; // evita negativi
  si5351.drive_strength(clk, static_cast<si5351_drive>(clk_pwr));
  EEPROM.write(addr, clk_pwr);
}

// ==================== setup() ====================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(128);

  in_gen_mode = true;
  modalita_bfo = 3; // AM default
  f_fchange = 1;

  // carico EEPROM
  int32_t val;
  EEPROM.get(0, val); if (val > -50000000 && val < 50000000) if_offset = val;
  EEPROM.get(4, val); if (val > 1000000 && val < 50000000) freq_clk1 = val;
  EEPROM.get(8, val); if (val > 1000000 && val < 50000000) freq_clk2 = val;

  uint8_t r0 = EEPROM.read(ADDR_CLK0_PWR);
  uint8_t r1 = EEPROM.read(ADDR_CLK1_PWR);
  uint8_t r2 = EEPROM.read(ADDR_CLK2_PWR);
  if (r0 <= 3) clk0_pwr = r0;
  if (r1 <= 3) clk1_pwr = r1;
  if (r2 <= 3) clk2_pwr = r2;

  // Rotary encoder PCNT
  pcnt_config_t pcnt_config_A;
  pcnt_config_A.pulse_gpio_num = PULSE_INPUT_PIN;
  pcnt_config_A.ctrl_gpio_num = PULSE_CTRL_PIN;
  pcnt_config_A.channel = PCNT_CHANNEL_0;
  pcnt_config_A.unit = PCNT_UNIT_0;
  pcnt_config_A.pos_mode = PCNT_COUNT_INC;
  pcnt_config_A.neg_mode = PCNT_COUNT_DEC;
  pcnt_config_A.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config_A.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config_A.counter_h_lim = 10000;
  pcnt_config_A.counter_l_lim = -10000;

  pcnt_unit_config(&pcnt_config_A);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // pinMode
  pinMode(PIN_PAGE, INPUT_PULLUP);
  pinMode(STEP_PLUS_PIN, INPUT_PULLUP);
  pinMode(STEP_MINUS_PIN, INPUT_PULLUP);
  pinMode(PIN_BANDA_PLUS, INPUT_PULLUP);
  pinMode(PIN_BANDA_MINUS, INPUT_PULLUP);
  pinMode(PIN_STEP_RX, INPUT_PULLUP);
  pinMode(PIN_BFO_MODE, INPUT_PULLUP);
  pinMode(PIN_VFO_SWITCH, INPUT_PULLUP); // NON TOCCARE
  pinMode(PIN_RXTX, INPUT_PULLUP);

  // display setup
  LCD_setup();
  lcd.setTextColor(TFT_CYAN);
  lcd.setFont(&fonts::Font0);
  lcd.setTextSize(lcd.height()/64.0f);
  lcd.setCursor((lcd.width()-lcd.textWidth(NAME))/2, 0.1f*lcd.height());
  lcd.printf(NAME);
  lcd.setCursor((lcd.width()-lcd.textWidth(VERSION))/2, 0.4f*lcd.height());
  lcd.printf(VERSION);
  lcd.setCursor((lcd.width()-lcd.textWidth(ID))/2, 0.7f*lcd.height());
  lcd.printf(ID);
  lcd.setCursor((lcd.width()-lcd.textWidth("Build "+String(BUILD_NUMBER)))/2, 0.85f*lcd.height());
  lcd.printf("Build %s", BUILD_NUMBER);
  delay(2000);

  // sprite double buffer
  sprite_height = lcd.height();
  sprites[0].createSprite(lcd.width(), sprite_height);
  sprites[1].createSprite(lcd.width(), sprite_height);
  flip = false;

  // Si5351 init
  si5351_init();
  si5351.drive_strength(SI5351_CLK0, static_cast<si5351_drive>(clk0_pwr));
  si5351.drive_strength(SI5351_CLK1, static_cast<si5351_drive>(clk1_pwr));
  si5351.drive_strength(SI5351_CLK2, static_cast<si5351_drive>(clk2_pwr));

  Dial_frq = bande[banda_corrente];
  aggiorna_clk2();
  aggiorna_clk1();
  aggiorna_bfo();
}

// fine troncone 1/3
// ==================== Troncone 2/3 ====================

// ------------------- Modalità VFO -------------------
enum VFO_Mode { VFO_A, VFO_IQ, GEN };
VFO_Mode vfo_mode = VFO_A;

void loop() {
  // Gestione step / banda / BFO
  gestisci_step_rx();
  gestisci_step();
  gestisci_banda();
  gestisci_bfo();

  // --- Gestione pin RXTX (GEN) ---
  bool stato_gen = digitalRead(PIN_RXTX);
  if (stato_gen == LOW && stato_prec_gen == HIGH) {
    in_gen_mode = !in_gen_mode;
    f_fchange = 1;
    Serial.println(in_gen_mode ? "GEN attivo" : "RX attivo");
  }
  stato_prec_gen = stato_gen;

  // --- Gestione pin 33: ciclo modalità VFO-A -> VFO-IQ -> GEN ---
  static bool stato_prec_pin33 = HIGH;
  bool stato_pin33 = digitalRead(PIN_RXTX);
  if (stato_pin33 == LOW && stato_prec_pin33 == HIGH) {
    switch (vfo_mode) {
      case VFO_A: vfo_mode = VFO_IQ; break;
      case VFO_IQ: vfo_mode = GEN; break;
      case GEN: vfo_mode = VFO_A; break;
    }
    f_fchange = 1;
    Serial.print("Modalità VFO: ");
    switch(vfo_mode) {
      case VFO_A: Serial.println("VFO-A"); break;
      case VFO_IQ: Serial.println("VFO-I/Q"); break;
      case GEN: Serial.println("GEN"); break;
    }
  }
  stato_prec_pin33 = stato_pin33;

  // --- Lettura rotary encoder ---
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  pcnt_counter_clear(PCNT_UNIT_0);

  if (pagina_corrente == TESTO && count != 0) {
    if (riga_attiva == 0) if_offset += count * step;
    else if (riga_attiva == 1) freq_clk1 += count * step;
    else if (riga_attiva == 2) freq_clk2 += count * step;
    else if (riga_attiva == 3) cambia_potenza_clk(clk0_pwr, ADDR_CLK0_PWR, count, SI5351_CLK0);
    else if (riga_attiva == 4) cambia_potenza_clk(clk1_pwr, ADDR_CLK1_PWR, count, SI5351_CLK1);
    else if (riga_attiva == 5) cambia_potenza_clk(clk2_pwr, ADDR_CLK2_PWR, count, SI5351_CLK2);

    if_offset = constrain(if_offset, -50000000, 150000000);
    freq_clk1  = constrain(freq_clk1, 1000000, 150000000);
    freq_clk2  = constrain(freq_clk2, 1000000, 150000000);
    f_fchange = 1;
  } 
  else if (count != 0) {
    afstp = count * step;
    int32_t tfrq = Dial_frq + afstp;
    tfrq = (tfrq / step) * step;
    Dial_frq = constrain(tfrq, min_frq, max_frq);
    if (vfo_is_A) Dial_frq_A = Dial_frq;
    else Dial_frq_B = Dial_frq;
    afstp = 0;
    f_fchange = 1;
  }

  // --- Aggiornamento CLK/BFO secondo modalità ---
  if (f_fchange == 1) {
    f_fchange = 0;
    aggiorna_bfo();
    aggiorna_clk2();
    aggiorna_clk1();
  }

  // --- Gestione pagina TESTO / GRAFICA ---
  bool pin4_val = digitalRead(PIN_PAGE);
  if (pin4_val == LOW && pin4_stato_prec == HIGH) pin4_tempo = millis();

  if (pin4_val == HIGH && pin4_stato_prec == LOW) {
    unsigned long durata = millis() - pin4_tempo;
    if (durata > 2000 && pagina_corrente == TESTO) {
      // salva in EEPROM
      EEPROM.put(0, if_offset);
      EEPROM.put(4, freq_clk1);
      EEPROM.put(8, freq_clk2);
      EEPROM.write(ADDR_CLK0_PWR, clk0_pwr);
      EEPROM.write(ADDR_CLK1_PWR, clk1_pwr);
      EEPROM.write(ADDR_CLK2_PWR, clk2_pwr);
      EEPROM.commit();
      Serial.println("Pagina TESTO salvata in EEPROM");
      pagina_corrente = GRAFICA;
    } else {
      riga_attiva = (riga_attiva + 1) % NUM_RIGHE_TESTO;
      if (pagina_corrente == GRAFICA) pagina_corrente = TESTO;
    }
  }
  pin4_stato_prec = pin4_val;

  // --- Disegna pagina ---
  if (pagina_corrente == GRAFICA) disegna_grafica();
  if (pagina_corrente == TESTO) gestisci_pagina_testo();
}
// ==================== Troncone 3/3 ====================

// ------------------- Funzioni aggiornamento oscillatori -------------------
void aggiorna_clk1() {
  if (vfo_mode == VFO_IQ) {
    // in I/Q CLK1 oscurato
    set_car_freq(0, 0, 1);
  } else {
    set_car_freq(freq_clk1, 1, 1); // CLK1 = quarzo virtuale normale
  }
}

void aggiorna_clk2() {
  uint32_t freq;
  if (vfo_mode == GEN) freq = Dial_frq;
  else if (vfo_mode == VFO_IQ) freq = Dial_frq + if_offset; // somma IF in I/Q
  else freq = Dial_frq + if_offset;
  freq = constrain(freq, 10000, 160000000);
  set_car_freq(freq, 1, 2); // CLK2 sempre quarzo virtuale programmabile
}

void aggiorna_bfo() {
  if (vfo_mode == VFO_IQ) set_car_freq(freq_clk2, 1, 0); // BFO disabilitato in I/Q
  else if (vfo_mode == GEN) set_car_freq(0, 0, 0);        // GEN: BFO = 0
  else {
    if (modalita_bfo == 3) set_car_freq(0, 0, 0); // AM disabilita BFO
    else set_car_freq(freq_clk2 + offset_bfo[modalita_bfo], 1, 0);
  }
}

// =================== Gestione pulsanti e step ===================
void gestisci_bfo() {
  bool stato_bfo = digitalRead(PIN_BFO_MODE);
  if (stato_bfo == LOW && stato_prec_bfo == HIGH) {
    modalita_bfo = (modalita_bfo + 1) % 4;
    Serial.println("Modalità BFO: " + String(nomi_bfo[modalita_bfo]));
    f_fchange = 1;
  }
  stato_prec_bfo = stato_bfo;
}

void gestisci_step_rx() {
  bool stato = digitalRead(PIN_STEP_RX);
  if (!pulsante_premuto && stato == LOW) {
    if (modalita_bfo == 3 || vfo_mode == GEN) { // AM o generatore
      step = step_am_values[step_am_index];
      step_am_index = (step_am_index + 1) % 3;
      Serial.println(vfo_mode == GEN ? "STEP GEN" : "STEP RX AM: " + String(step));
    } else {
      step_rx_index = (step_rx_index + 1) % NUM_STEP_RX;
      step = step_rx_values[step_rx_index];
      Serial.println("STEP RX: " + String(step));
    }
    in_step_rx_mode = true;
    pulsante_premuto = true;
  }
  if (stato == HIGH) pulsante_premuto = false;
}

void gestisci_step() {
  static bool lastPlus = HIGH, lastMinus = HIGH;
  bool nowPlus = digitalRead(STEP_PLUS_PIN), nowMinus = digitalRead(STEP_MINUS_PIN);

  if (in_step_rx_mode && (nowPlus == LOW || nowMinus == LOW)) in_step_rx_mode = false;

  if (!in_step_rx_mode) {
    if (lastPlus == HIGH && nowPlus == LOW) cnt = min(cnt + 1, 5);
    if (lastMinus == HIGH && nowMinus == LOW) cnt = max(cnt - 1, 0);
    switch (cnt) {
      case 0: step = 10; break;
      case 1: step = 100; break;
      case 2: step = 1000; break;
      case 3: step = 2500; break;
      case 4: step = 25000; break;
      case 5: step = 250000; break;
    }
  }

  lastPlus = nowPlus;
  lastMinus = nowMinus;
}

void gestisci_banda() {
  static bool lastPlus = HIGH, lastMinus = HIGH;
  bool nowPlus = digitalRead(PIN_BANDA_PLUS), nowMinus = digitalRead(PIN_BANDA_MINUS);

  if (lastPlus == HIGH && nowPlus == LOW) {
    banda_corrente = min(banda_corrente + 1, NUM_BANDE - 1);
    Dial_frq = bande[banda_corrente];
    f_fchange = 1;
    messaggio_banda = "Banda: " + String(nomi_bande[banda_corrente]);
    tempo_messaggio_banda = millis();
    Serial.println(messaggio_banda);
  }

  if (lastMinus == HIGH && nowMinus == LOW) {
    banda_corrente = max(banda_corrente - 1, 0);
    Dial_frq = bande[banda_corrente];
    f_fchange = 1;
    messaggio_banda = "Banda: " + String(nomi_bande[banda_corrente]);
    tempo_messaggio_banda = millis();
    Serial.println(messaggio_banda);
  }

  lastPlus = nowPlus;
  lastMinus = nowMinus;
}

// ------------------- Funzione pagina TESTO -------------------
void gestisci_pagina_testo() {
  sprites[flip].clear(TFT_BLACK);
  sprites[flip].setTextDatum(TL_DATUM);
  sprites[flip].setFont(&fonts::Font0);
  sprites[flip].setTextSize(1.0f);

  int dy = 14;                 
  int x = 10;                  
  int y = 15;                  

  for (int i = 0; i < NUM_RIGHE_TESTO; i++) {
    String label, valore, riga;
    char buf[16];
    switch(i) {
      case 0: 
        label = "IF +-    =";
        sprintf(buf, "%03ld.%03ld.%03ld", (long)abs(if_offset)/1000000, ((long)abs(if_offset)%1000000)/1000, (long)abs(if_offset)%1000);
        valore = (if_offset>=0?"+":"-")+String(buf);
        break;
      case 1:
        //label = "FREQ CLK1 =";
        //valore = (vfo_mode == VFO_IQ ? "---" : String(freq_clk1)); // oscurata in IQ
        //break;
      label = "FREQ CLK1 =";
    if (vfo_mode == VFO_IQ) valore = "---"; // oscurata in IQ
    else {
        sprintf(buf, "%03ld.%03ld.%03ld", freq_clk1/1000000, (freq_clk1%1000000)/1000, freq_clk1%1000);
        valore = String(buf);
    }
    break;
      case 2:
        label = "FREQ CLK2 =";
        sprintf(buf, "%03ld.%03ld.%03ld", (long)freq_clk2/1000000, ((long)freq_clk2%1000000)/1000, (long)freq_clk2%1000);
        valore = String(buf);
        break;
      case 3:
        label = "CLK0 POWER =";
        valore = String((clk0_pwr+1)*2) + "mA";
        break;
      case 4:
        label = "CLK1 POWER =";
        valore = String((clk1_pwr+1)*2) + "mA";
        break;
      case 5:
        label = "CLK2 POWER =";
        valore = String((clk2_pwr+1)*2) + "mA";
        break;
    }
    riga = (i==riga_attiva?"▶ ":"  ") + label + " " + valore;
    sprites[flip].setTextColor(i==riga_attiva?TFT_YELLOW:TFT_WHITE);
    sprites[flip].drawString(riga, x, y+i*dy);
  }

  sprites[flip].setTextColor(TFT_WHITE);
  sprites[flip].drawString("STEP         = " + String(step), x, y+6*dy);
  sprites[flip].setTextColor(TFT_CYAN);
  sprites[flip].drawString("Premi 3 sec: mem+usc", x, y+7*dy);
  sprites[flip].pushSprite(&lcd, 0, 0);
  flip = !flip;
}

// ------------------- Disegna grafica (invariata) -------------------
void disegna_grafica() {
  sprites[flip].clear(TFT_BLACK);

  // Disegna dial (resta invariato)
  dial.draw(Dial_frq,0);

  // Frequenza: font grande, centrato verticalmente
  sprites[flip].setFont(&fonts::Font0);
  float textSize = lcd.height() / 48.0f; // aumenta dimensione font
  sprites[flip].setTextSize(textSize);
  sprites[flip].setTextColor(TFT_WHITE);
  sprites[flip].setTextDatum(MC_DATUM);

  char str[16];
  sprintf(str, "%03d.%03d.%02d", Dial_frq/1000000, (Dial_frq%1000000)/1000, (Dial_frq%1000)/10);

  int y_freq = lcd.height() / 8; // inizio più vicino alla cima
  sprites[flip].drawString(str, lcd.width()/2, y_freq);

  // Label penultima riga (modalità)
  sprites[flip].setTextSize(1.2f);
  sprites[flip].setTextDatum(MC_DATUM);
  int h_bfo = sprites[flip].fontHeight();

  String label_penultima;
  switch(vfo_mode) {
    case VFO_A:    label_penultima = String(nomi_bfo[modalita_bfo]); break;
    case GEN:      label_penultima = "GEN"; break;
    case VFO_IQ:   label_penultima = "I/Q"; break;
  }
  sprites[flip].setTextColor(TFT_YELLOW);
  sprites[flip].drawString(label_penultima, lcd.width()/2, lcd.height() - 2*h_bfo - lcd.height()/50);

  // Label ultima riga (step / messaggio banda)
  String label_ultima = "";
  if (millis() - tempo_messaggio_banda < 2000 && messaggio_banda != "") label_ultima = messaggio_banda;
  else label_ultima += String(vfo_mode == GEN ? "GEN" : (vfo_mode == VFO_A ? "VFO-A" : "VFO-I/Q")) + "   " + "STEP: " + String(step) + " Hz";

  int h_step = sprites[flip].fontHeight();
  sprites[flip].setTextDatum(MC_DATUM);
  sprites[flip].setTextColor(TFT_CYAN);
  sprites[flip].drawString(label_ultima, lcd.width()/2, lcd.height() - h_step);

  sprites[flip].pushSprite(&lcd, 0, 0);
  flip = !flip;
}



// ==================== Fine Troncone 3/3 ====================
