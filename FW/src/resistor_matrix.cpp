#include "resistor_matrix.h"
#include "pins.h"

// ---- 回路パラメータ ----
// 電源電圧はPT2基板が3.3V駆動のため3.3に変更(判定しきい値自体はVCCに依存しない比の
// 式のため無影響。表示用の推定抵抗値計算にのみ影響する)。
static constexpr float VCC = 3.3f;
static constexpr float R_PD = 10000.0f; // SIG-GND間の共通プルダウン抵抗[Ω]
                                         // (要:基板BOM/実測値で確認・調整)
static constexpr int ADC_MAX = 1023;    // 10bit ADC
static constexpr int N_SAMPLES = 8;     // ADC平均回数(ノイズ低減)
static constexpr unsigned long SETTLE_US = 200; // マルチプレクサ切替後の安定待ち[us]

const float RM_R_CLASS[RM_N_CLASSES] = {2200.0f, 4700.0f, 10000.0f, 22000.0f, 47000.0f};
const char *const RM_R_CLASS_LABEL[RM_N_CLASSES] = {"2.2k", "4.7k", "10k", "22k", "47k"};

static float classThresholdADC[RM_N_CLASSES - 1];

float rmResistance[4][4];
int8_t rmClass[4][4];

static const uint8_t MUX_ADDR_PINS[4] = {PIN_RM_MUX_S0, PIN_RM_MUX_S1, PIN_RM_MUX_S2, PIN_RM_MUX_S3};

static void computeClassThresholds() {
  float expectedADC[RM_N_CLASSES];
  for (uint8_t c = 0; c < RM_N_CLASSES; c++) {
    float v = VCC * R_PD / (RM_R_CLASS[c] + R_PD);
    expectedADC[c] = (v / VCC) * ADC_MAX;
  }
  for (uint8_t t = 0; t < RM_N_CLASSES - 1; t++) {
    classThresholdADC[t] = (expectedADC[t] + expectedADC[t + 1]) / 2.0f;
  }
}

static float measureRawADC(uint8_t pin) {
  long sum = 0;
  for (int n = 0; n < N_SAMPLES; n++) {
    sum += analogRead(pin);
  }
  return (float)sum / N_SAMPLES;
}

static float adcToResistance(float adcAvg) {
  float v = adcAvg * VCC / ADC_MAX;
  const float V_EPS = 0.005f;
  if (v < V_EPS) return 1.0e9f; // 断線とみなす
  return R_PD * (VCC / v - 1.0f);
}

static int8_t classifyADC(float adcAvg) {
  for (uint8_t c = 0; c < RM_N_CLASSES - 1; c++) {
    if (adcAvg >= classThresholdADC[c]) {
      return c;
    }
  }
  if (adcAvg < classThresholdADC[RM_N_CLASSES - 2] * 0.3f) {
    return -1;
  }
  return RM_N_CLASSES - 1;
}

void resistorMatrixInit() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MUX_ADDR_PINS[i], OUTPUT);
  }
  computeClassThresholds();
}

void resistorMatrixScan() {
  // chとセル位置(row,col)の対応: ch = row*4+col (PCB上のYピン配線もこれに合わせる)
  for (uint8_t ch = 0; ch < 16; ch++) {
    digitalWrite(MUX_ADDR_PINS[0], (ch >> 0) & 1);
    digitalWrite(MUX_ADDR_PINS[1], (ch >> 1) & 1);
    digitalWrite(MUX_ADDR_PINS[2], (ch >> 2) & 1);
    digitalWrite(MUX_ADDR_PINS[3], (ch >> 3) & 1);
    delayMicroseconds(SETTLE_US);

    float adcAvg = measureRawADC(PIN_RM_MUX_SIG);
    uint8_t row = ch / 4;
    uint8_t col = ch % 4;
    rmResistance[row][col] = adcToResistance(adcAvg);
    rmClass[row][col] = classifyADC(adcAvg);
  }
}

void resistorMatrixPrint() {
  Serial.println(F("---- Resistor matrix class ----"));
  for (uint8_t row = 0; row < 4; row++) {
    Serial.print(F("Row"));
    Serial.print(row);
    Serial.print(F(": "));
    for (uint8_t col = 0; col < 4; col++) {
      int8_t c = rmClass[row][col];
      if (c < 0) {
        Serial.print(F("---\t"));
      } else {
        Serial.print(RM_R_CLASS_LABEL[c]);
        Serial.print(F("\t"));
      }
    }
    Serial.println();
  }
}

void resistorMatrixPrintResistance() {
  Serial.println(F("---- Resistor matrix value [ohm] ----"));
  for (uint8_t row = 0; row < 4; row++) {
    Serial.print(F("Row"));
    Serial.print(row);
    Serial.print(F(": "));
    for (uint8_t col = 0; col < 4; col++) {
      Serial.print(rmResistance[row][col], 0);
      Serial.print(F("\t"));
    }
    Serial.println();
  }
}
