# HB100 Microwave Doppler Speed Radar & Telemetry System

[![ESP32](https://img.shields.io/badge/Microcontroller-ESP32%20DevKit%20V1-blue.svg)](https://www.espressif.com/)
[![Carrier](https://img.shields.io/badge/Microwave-10.525%20GHz%20X--Band-emerald.svg)](https://en.wikipedia.org/wiki/X_band)
[![Processing](https://img.shields.io/badge/GUI-Processing%204-cyan.svg)](https://processing.org/)
[![FFT](https://img.shields.io/badge/DSP-512--pt%20FFT%20%40%205%20kHz-orange.svg)](https://github.com/kosme/arduinoFFT)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

An end-to-end microwave Doppler radar platform that measures the radial velocity of moving targets in real time. It pairs an **HB100 X-band transceiver (10.525 GHz)** with an **LM324 op-amp** analog conditioning stage, runs an on-chip **512-point FFT digital signal processor** on the **ESP32**, and streams telemetry to an interactive **Processing 4** desktop dashboard featuring a progressive analog speedometer and a real-time 51-channel Doppler frequency spectrum analyzer.

---

## 🚀 Key Features

* **10.525 GHz Microwave Sensing:** All-weather, light-independent sensing that penetrates non-metallic materials (drywall, wood, plastic).
* **On-Chip Embedded DSP:** 5000 Hz microsecond-accurate ADC sampling, DC-offset cancellation, Hamming windowing, and 512-point Fast Fourier Transform (FFT) on the ESP32.
* **Discrete Speed Conversion:** Resolves Doppler shift down to ~9.77 Hz/bin (~0.5 km/h discrete velocity resolution).
* **Non-Linear Progressive Speedometer (0–100 km/h):** Uses a concave power-curve scale ($t = (v/100)^{0.55}$) dedicating **over 41% of the dial to 0–20 km/h**, giving high needle deflection for human gestures, slow walking, and indoor movement.
* **51-Channel Doppler Spectrum Analyzer:** Visual bar chart plotting energy distribution across 0 to 500 Hz with dynamic peak frequency tracking.
* **Zero-Lag Telemetry:** Live serial binding between ESP32 and desktop workstation at 115200 baud with no smoothing lag.
* **Interactive Presentation Included:** Standalone modern 16:9 HTML slide deck (`Radar_Project_Presentation.html`) and slide-by-slide guide (`PRESENTATION.md`).

---

## 📐 Theoretical Principles & Speed Derivation

When a 10.525 GHz microwave beam hits a moving target, the reflected frequency undergoes a shift ($f_d$) proportional to radial speed ($v$):

$$f_d = \frac{2 \cdot v \cdot f_0 \cdot \cos\theta}{c}$$

Where:
* $f_d$: Doppler intermediate frequency (Hz)
* $v$: Target velocity (m/s)
* $f_0$: Carrier transmit frequency ($10.525\text{ GHz} = 10.525 \times 10^9\text{ Hz}$)
* $c$: Speed of light ($2.9979 \times 10^8\text{ m/s}$)
* $\theta$: Angle between target motion and radar antenna axis
* **Factor 2:** Round-trip propagation path

For direct axial movement ($\cos\theta \approx 1$):

$$v\ (\text{m/s}) = f_d \times 0.01424$$

$$v\ (\text{km/h}) = f_d \times 0.05126$$

### Empirical Velocity vs. Frequency

| Motion Type | Typical Doppler Freq ($f_d$) | Equivalent Velocity |
| :--- | :--- | :--- |
| **Subtle Hand Wave** | ~35 Hz | ~1.8 km/h (0.5 m/s) |
| **Casual Walking** | ~70 – 105 Hz | ~3.6 – 5.4 km/h (1.0 – 1.5 m/s) |
| **Brisk Running** | ~280 – 350 Hz | ~14.4 – 18.0 km/h (4.0 – 5.0 m/s) |
| **Vehicle Motion** | ~1170 Hz | ~60.0 km/h (16.7 m/s) |

---

## 🛠️ System Architecture

```
┌─────────────────┐       ┌──────────────────────┐       ┌──────────────────┐
│   HB100 RADAR   │  IF   │ 1µF Coupling Cap     │  Amp  │  Voltage Divider │
│  10.525 GHz TX  ├──────►│ LM324 Op-Amp Stage 1 ├──────►│  10kΩ / 10kΩ (÷2)│
│  Mixer Receiver │ (µV)  │ LM324 Op-Amp Stage 2 │ (0-5V)│  Clamps to 0-2.5V│
└─────────────────┘       └──────────────────────┘       └────────┬─────────┘
                                                                  │
                                                        ADC1-CH6  ▼ (GPIO34)
┌─────────────────────────────────┐   UART @ 115200   ┌─────────────────────┐
│       PROCESSING 4 GUI          │◄──────────────────┤      ESP32 SoC      │
│ • Progressive Gauge (0-100 km/h)│    Live Stream    │ • 5 kHz ADC Sampler │
│ • 51-Ch Doppler Spectrum Chart  │   (ms,freq,kmh,   │ • Hamming Windowing │
│ • 8-Second History & Peak Memory│    magnitude,bins)│ • 512-Point FFT DSP │
└─────────────────────────────────┘                   └─────────────────────┘
```

---

## 🔌 Hardware Wiring & Schematic

```
HB100 Microwave Module:
  VCC  ────► +5V DC Supply
  GND  ────► Common GND
  IF   ────► [1 µF Electrolytic / Film Cap] ────► LM324 Pin 3 (IN1+)

LM324 Quad Operational Amplifier (DIP-14):
  Pin  3 (IN1+) ◄─── From 1 µF Coupling Capacitor
  Pin  4 (VCC+) ────► +5V DC Rail
  Pin 11 (GND)  ────► Common System Ground
  Pin  7 (OUT2) ────► Amplified Analog Output
                       │
                     [10 kΩ Resistor]
                       │
                       ├────► ESP32 GPIO34 (ADC Input)
                       │
                     [10 kΩ Resistor]
                       │
                      GND

ESP32 DevKit V1:
  GPIO34 (D34)  ◄─── Center of 10 kΩ / 10 kΩ Voltage Divider
  GND           ────► Common System Ground
  VIN           ────► 5V DC Supply
```

> ⚠️ **Electrical Safety Notice:** Before connecting the ESP32, measure the 3V3 rail with a multimeter to verify it reads $\le 3.3\text{V}$. The 2 × 10 kΩ divider scales the 5V op-amp output down to a maximum of 2.5V, protecting the ESP32 ADC.

---

## 📂 Repository File Structure

```
├── radar_firmware/
│   └── radar_firmware.ino         # ESP32 firmware (512-pt FFT, 5 kHz sampler, UART stream)
├── radar_gui_speed/
│   └── radar_gui_speed.pde        # Processing 4 GUI (progressive gauge, spectrum, history)
├── Radar_Project_Presentation.html # 16:9 interactive modern presentation slide deck
├── PRESENTATION.md                # Complete slide-by-slide presentation script & notes
├── Radar_Project.txt              # Hardware log, component audit, and pin history
└── README.md                      # Project documentation
```

---

## ⚡ Quick Start Guide

### 1. ESP32 Firmware Installation
1. Open **Arduino IDE 2.x**.
2. Go to **Tools → Manage Libraries...**, search for **`arduinoFFT`** (by Enrique Condes), and install **Version 2.x**.
3. Select board: **ESP32 Dev Module**.
4. Select port: **COM3** (or your platform's serial port).
5. Open `radar_firmware/radar_firmware.ino` and click **Upload (▶)**.
6. Open **Serial Monitor** at **115200 baud** to verify telemetry output:
   ```csv
   RADAR_SPEED_READY,HB100+LM324+ESP32,v5.0
   # format: ms,freq_hz,speed_mps,speed_kmh,magnitude,motion,bin1..bin51
   1240,0.00,0.000,0.00,310,0,12,18,9,4,...
   2350,97.66,1.391,5.01,8420,1,24,65,890,1000,...
   ```

### 2. Processing 4 GUI Execution
1. Install and launch [Processing 4](https://processing.org/download).
2. Open `radar_gui_speed/radar_gui_speed.pde`.
3. Verify `SERIAL_PORT = "COM3"` matches your board's serial port.
4. Click **Run (▶)**.

---

## 🖥️ Telemetry Dashboard Controls

| Key | Function |
| :---: | :--- |
| **`M`** | Toggle display units between **km/h** and **m/s** |
| **`C`** | Clear session maximum speed record |
| **`+`** | Raise magnitude threshold (reduces sensitivity / eliminates false triggers) |
| **`-`** | Lower magnitude threshold (increases sensitivity for subtle motion) |
| **`R`** | Reconnect serial port if disconnected |
| **`S`** | Save instantaneous timestamped screenshot (`speed_radar_####.png`) |

---

## 📡 Serial Telemetry Protocol

Each window frame is broadcast at 115200 baud as a comma-separated ASCII string:

```
ms,freq_hz,speed_mps,speed_kmh,magnitude,motion,b1,b2,b3,...,b51
```

* **`ms`:** System uptime timestamp (milliseconds).
* **`freq_hz`:** Dominant detected Doppler peak frequency (Hz).
* **`speed_mps`:** Calculated velocity in meters per second ($f_d \times 0.01424$).
* **`speed_kmh`:** Calculated velocity in kilometers per hour ($f_d \times 0.05126$).
* **`magnitude`:** FFT peak magnitude (indicates reflection strength).
* **`motion`:** Binary detection flag (`1` = valid motion, `0` = noise/quiescent).
* **`b1` – `b51`:** 51 normalized spectral energy bins (0–1000) spanning 0 to 500 Hz ($9.77\text{ Hz}$ per bin).

---

## 🎯 Best Practices for Measurement

The HB100 is a **continuous-wave (CW) Doppler sensor** measuring **radial velocity**:
* **Best Alignment:** Aim the antenna face directly toward or away from the moving object ($\theta \approx 0^\circ \implies \cos\theta \approx 1$).
* **Perpendicular Motion:** Objects moving strictly across the beam ($\theta \approx 90^\circ \implies \cos 90^\circ = 0$) generate little to no Doppler shift.
* **Low-Speed Detection:** When testing hand gestures, move your hand directly toward the sensor face at 10–50 cm distance.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
