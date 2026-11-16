// =============================================================
//  radar_firmware.ino  — HB100 Speed Radar  v5.0
//  ESP32 DevKit V1  |  GPIO34  |  LM324 amp  |  115200 baud
//
//  HOW IT WORKS
//  ─────────────────────────────────────────────────────────────
//  The HB100 transmits 10.525 GHz microwaves. When a moving
//  object reflects them back, the reflected frequency is
//  shifted by the Doppler effect:
//
//      f_doppler = 2 × v × f_carrier / c
//
//  Rearranging for speed:
//      v (m/s)  = f_doppler × 0.01424
//      v (km/h) = f_doppler × 0.05127
//
//  Example: person walking at 1.5 m/s → f_doppler ≈ 105 Hz
//           car at 60 km/h            → f_doppler ≈ 1171 Hz
//
//  This firmware collects ADC samples, runs a 512-point FFT,
//  finds the dominant Doppler frequency, and sends the result
//  over Serial for the Processing GUI.
//
//  LIBRARY REQUIRED:
//    "arduinoFFT" by Enrique Condes — install via Arduino
//    Library Manager (Tools → Manage Libraries → search
//    "arduinoFFT" → install version 2.x)
//
//  SERIAL OUTPUT (115200 baud), one line per FFT window:
//    ms,freq_hz,speed_mps,speed_kmh,peak_magnitude,motion
//
//  TUNING:
//    MAGNITUDE_THRESHOLD  — raise if too many false readings
//    MIN_FREQ_HZ          — ignore Doppler below this (noise)
//    SAMPLING_FREQ        — keep at 5000, do not change
// =============================================================

#include <arduinoFFT.h>

// ── Pin ──────────────────────────────────────────────────────
#define RADAR_PIN          34       // ADC1-CH6, input-only GPIO

// ── FFT parameters ───────────────────────────────────────────
#define SAMPLES            512      // FFT size (power of 2)
#define SAMPLING_FREQ      5000     // Hz — covers 0–2500 Hz Doppler
                                    //      = 0–35.6 m/s = 0–128 km/h

// ── Detection thresholds ─────────────────────────────────────
#define MAGNITUDE_THRESHOLD  1200   // Peak FFT magnitude to call it a signal
                                    // Raise to 2000+ if seeing false speeds
                                    // Lower to 800 if misses real targets
#define MIN_FREQ_HZ          8.0    // Ignore Doppler below this (electrical hum etc.)
                                    // ≈ 8 Hz × 0.01424 m/s = 0.11 m/s (way below walking)
#define MAX_FREQ_HZ          2400.0 // Ignore above this (aliasing artefacts)

// ── Speed formula constants ───────────────────────────────────
// v_mps  = f_d × c / (2 × f_c) = f_d × 299792458 / (2 × 10525000000)
#define MPS_PER_HZ   0.014238       // m/s per Hz of Doppler shift
#define KMPH_PER_HZ  0.051258       // km/h per Hz of Doppler shift

// ── Sample buffers ───────────────────────────────────────────
float vReal[SAMPLES];
float vImag[SAMPLES];

// arduinoFFT v2.x (template-based API)
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// ── Timing ───────────────────────────────────────────────────
const unsigned long SAMPLE_PERIOD_US = 1000000UL / SAMPLING_FREQ;  // 200 µs

// =============================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  analogReadResolution(12);                              // 0–4095
  analogSetPinAttenuation(RADAR_PIN, ADC_11db);         // input range 0–3.9 V

  Serial.println("RADAR_SPEED_READY,HB100+LM324+ESP32,v5.0");
  Serial.println("# format: ms,freq_hz,speed_mps,speed_kmh,magnitude,motion");
}

// =============================================================
void loop() {
  // ── 1. Collect samples with precise timing ────────────────
  unsigned long targetUs = micros();

  for (int i = 0; i < SAMPLES; i++) {
    // Spin-wait for the scheduled sample time
    while ((long)(micros() - targetUs) < 0) { /* spin */ }

    vReal[i] = (float)analogRead(RADAR_PIN);
    vImag[i] = 0.0f;
    targetUs += SAMPLE_PERIOD_US;
  }

  // ── 2. Remove DC offset (average subtraction) ────────────
  float dcOffset = 0.0f;
  for (int i = 0; i < SAMPLES; i++) dcOffset += vReal[i];
  dcOffset /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= dcOffset;

  // ── 3. Apply Hamming window ───────────────────────────────
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

  // ── 4. Compute FFT ────────────────────────────────────────
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // ── 5. Find dominant peak (skip DC + sub-MIN_FREQ bins) ──
  int minBin = max(1, (int)ceil(MIN_FREQ_HZ * SAMPLES / (float)SAMPLING_FREQ));
  int maxBin = min(SAMPLES / 2 - 1,
                   (int)floor(MAX_FREQ_HZ * SAMPLES / (float)SAMPLING_FREQ));

  float peakMag = 0.0f;
  int   peakBin = minBin;
  for (int i = minBin; i <= maxBin; i++) {
    if (vReal[i] > peakMag) {
      peakMag = vReal[i];
      peakBin = i;
    }
  }

  // ── 6. Convert to frequency and speed ────────────────────
  float peakFreq  = (float)peakBin * SAMPLING_FREQ / (float)SAMPLES;
  float speedMps  = peakFreq * MPS_PER_HZ;
  float speedKmph = peakFreq * KMPH_PER_HZ;
  bool  motion    = (peakMag >= MAGNITUDE_THRESHOLD);

  if (!motion) {
    peakFreq  = 0.0f;
    speedMps  = 0.0f;
    speedKmph = 0.0f;
  }

  // ── 7. Emit main CSV fields ──────────────────────────────
  Serial.print(millis());         Serial.print(',');
  Serial.print(peakFreq,  2);     Serial.print(',');
  Serial.print(speedMps,  3);     Serial.print(',');
  Serial.print(speedKmph, 2);     Serial.print(',');
  Serial.print(peakMag,   0);     Serial.print(',');
  Serial.print(motion ? 1 : 0);

  // ── 8. Append 51 spectrum bins (bins 1–51 = 9.8–498 Hz) ──
  //  Covers 0–500 Hz:  walking ≈ 70–150 Hz, running ≈ 300–500 Hz
  //  Each value is normalised to 0–1000 (1000 = strongest bin
  //  anywhere in the full spectrum, so relative magnitudes are real).
  //  Bin width = 5000 / 512 ≈ 9.77 Hz per bin.
  //
  //  Total extra chars: 51 × 4 ≈ 204 → well within 115200-baud budget.

  // Find the absolute peak across ALL positive-frequency bins
  // so the displayed bars are correctly scaled even when the
  // peak Doppler is outside the 0–500 Hz display range.
  float specPeak = 1.0f;
  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > specPeak) specPeak = vReal[i];
  }

  for (int i = 1; i <= 51; i++) {
    Serial.print(',');
    int val = (int)(vReal[i] / specPeak * 1000.0f);
    Serial.print(constrain(val, 0, 1000));
  }
  Serial.println();   // end of line

  // No extra delay — 512-sample collection already takes ~102 ms
}
