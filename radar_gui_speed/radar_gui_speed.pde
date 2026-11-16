// =============================================================
//  radar_gui_speed.pde  —  Processing 4   (v5.1 — layout fix)
//  HB100 Doppler Speed Radar GUI
//
//  Serial input from ESP32 (COM3, 115200 baud):
//    ms,freq_hz,speed_mps,speed_kmh,magnitude,motion
//
//  FIXES in v5.1:
//    • Needle now shows the EXACT live value — no lerp lag
//    • Digital number and speed bar are in separate zones, no overlap
//
//  CONTROLS:
//    [M]  – toggle km/h / m/s display
//    [C]  – clear max-speed record
//    [R]  – reconnect serial port
//    [S]  – save screenshot
//    [+]  – raise magnitude threshold (reduce false readings)
//    [-]  – lower magnitude threshold (increase sensitivity)
// =============================================================

import processing.serial.*;

// ── Serial ─────────────────────────────────────────────────
final String SERIAL_PORT = "COM3";
final int    BAUD_RATE   = 115200;

// ── Speed range ────────────────────────────────────────────
final float SPEED_MAX_KMH = 100.0;
final float MPS_PER_HZ    = 0.014238;
final float KMPH_PER_HZ   = 0.051258;

// ── Live data (updated from serial) ────────────────────────
float   liveFreqHz    = 0;
float   liveSpeedMps  = 0;
float   liveSpeedKmh  = 0;   // ← needle and number both use THIS directly
float   liveMagnitude = 0;
boolean liveMotion    = false;
long    liveMs        = 0;
long    lastDataSys   = 0;

float   maxSpeedKmh   = 0;
float   magThreshold  = 1200;

// ── Display mode ───────────────────────────────────────────
boolean showKmh = true;

// ── Gauge geometry ─────────────────────────────────────────
//  LAYOUT (all Y values, window = 1050 × 760)
//  ┌────────────────────────────────────────────┐
//  │  Gauge            y: 0 → 390               │
//  │  Digital display  y: 405 → 475             │
//  │  Speed bar        y: 490 → 516   ← separate│
//  │  History graph    y: 530 → 710             │
//  │  Status bar       y: 735 → 760             │
//  └────────────────────────────────────────────┘
int   gaugeX  = 625;   // gauge centre X
int   gaugeY  = 192;   // gauge centre Y
int   gaugeR  = 168;   // gauge outer radius
//  gauge face:  top = gaugeY - gaugeR = 24
//  gauge face:  bot = gaugeY + gaugeR = 360

// ── Needle sweep angles ────────────────────────────────────
//   0 km/h  → 135° → lower-left  (cos=-0.71, sin=+0.71)  ✓
// 100 km/h  → 405° → lower-right (cos=+0.71, sin=+0.71)   ✓
final float ANG_ZERO = radians(135);
final float ANG_MAX  = radians(405);

// ── History ────────────────────────────────────────────────
int     HIST_LEN = 480;         // ~8 s at 60 fps
float[] histKmh  = new float[HIST_LEN];
int     histHead = 0;

// ── Doppler spectrum (51 bins, 0–500 Hz, normalised 0–1000) ─
//  Bin width = 5000 / 512 ≈ 9.77 Hz.  Bin i → freq = i × 9.77 Hz.
//  bin  7  ≈  68 Hz ≈ 1.0 m/s  (slow walk)
//  bin 15  ≈ 147 Hz ≈ 2.1 m/s  (brisk walk)
//  bin 36  ≈ 352 Hz ≈ 5.0 m/s  (running)
//  bin 51  ≈ 498 Hz ≈ 7.1 m/s  (sprinting)
int   SPEC_BINS    = 51;
int[] specBins     = new int[SPEC_BINS];
final float HZ_PER_BIN = 5000.0 / 512.0;   // ≈ 9.77 Hz

// ── Serial / misc ──────────────────────────────────────────
Serial  port;
boolean portOpen = false;
PFont   mono;
int     fc = 0;

// =============================================================
void setup() {
  size(1050, 820);    // taller window to fit spectrum panel
  frameRate(60);
  mono = createFont("Courier New", 13, true);
  textFont(mono);
  openPort();
}

// =============================================================
void draw() {
  background(0, 12, 0);

  // ── History: record the LIVE (exact) value every frame ──
  histKmh[histHead] = liveSpeedKmh;
  histHead = (histHead + 1) % HIST_LEN;

  // ── Max tracking ────────────────────────────────────────
  if (liveSpeedKmh > maxSpeedKmh) maxSpeedKmh = liveSpeedKmh;

  // ── Draw in order: background → gauge → number → bar → graph
  drawSeparators();
  drawHUD();
  drawSpeedometer();     // uses liveSpeedKmh directly — no lag
  drawDigitalDisplay();  // below gauge, clear of bar
  drawSpeedBar();        // clearly below the number
  drawHistoryGraph();
  drawDopplerSpectrum(); // FFT spectrum bar chart
  drawStatusBar();

  fc++;
}

// =============================================================
//  Needle angle — takes the exact km/h value, returns radians
//  Expanded progressive low-speed scale:
//  0-10 has the largest gap, 10-20 slightly less, etc.
//  Gives significantly more turn/movement for small speeds (0-20 km/h)
// =============================================================
float needleAngle(float kmh) {
  kmh = constrain(kmh, 0, SPEED_MAX_KMH);
  // Power curve (exponent 0.55) gives expanding low-end resolution:
  // 0-10 km/h takes ~28.2% of the dial sweep
  // 10-20 km/h takes ~13.1% of the dial sweep
  // Higher speeds take progressively smaller slots
  float norm = pow(kmh / SPEED_MAX_KMH, 0.55);
  return map(norm, 0, 1.0, ANG_ZERO, ANG_MAX);
}

// =============================================================
//  SEPARATORS
// =============================================================
void drawSeparators() {
  stroke(0, 70, 0);
  strokeWeight(1);
  line(205, 0,   205, 795);     // HUD | main vertical
  line(205, 528, 1050, 528);    // gauge area | history
  line(205, 638, 1050, 638);    // history | spectrum
  line(0,   795, 1050, 795);    // status bar top
}

// =============================================================
//  LEFT HUD
// =============================================================
void drawHUD() {
  textFont(mono);
  textAlign(LEFT, BASELINE);
  int px = 12, py = 28, lh = 22;

  fill(0, 220, 0); textSize(14);
  text("HB100  RADAR", px, py); py += 7;

  fill(0, 100, 0); textSize(11);
  text("────────────────────", px, py); py += lh;

  boolean fresh = (System.currentTimeMillis() - lastDataSys) < 3000;
  fill(0, 200, 0); textSize(12);
  text("PORT  : " + (portOpen ? SERIAL_PORT + " ✓" : "OFFLINE"), px, py); py += lh;
  text("STATUS: " + (portOpen ? (fresh ? "LIVE" : "IDLE") : "ERROR"), px, py); py += lh;
  text("UPTIME: " + nf(liveMs / 1000.0, 1, 1) + "s", px, py); py += lh;

  fill(0, 100, 0); textSize(11);
  text("────────────────────", px, py); py += lh;

  fill(0, 200, 0); textSize(12);
  text("FREQ  : " + nf(liveFreqHz,   3, 1) + " Hz",  px, py); py += lh;
  text("SPEED : " + nf(liveSpeedMps, 1, 2) + " m/s", px, py); py += lh;
  text("SPEED : " + nf(liveSpeedKmh, 3, 1) + " kmh", px, py); py += lh;
  text("MAGN  : " + nf(liveMagnitude, 5, 0),           px, py); py += lh;
  text("THOLD : " + nf(magThreshold, 4, 0),            px, py); py += lh;

  fill(0, 100, 0); textSize(11);
  text("────────────────────", px, py); py += lh;

  fill(0, 200, 0); textSize(12);
  text("MAX   : " + nf(maxSpeedKmh,       3, 1) + " kmh", px, py); py += lh;
  text("MAX   : " + nf(maxSpeedKmh / 3.6, 1, 2) + " m/s",  px, py); py += lh;

  fill(0, 100, 0); textSize(11);
  text("────────────────────", px, py); py += lh;

  // Motion indicator
  boolean blink = (fc / 15) % 2 == 0;
  if (liveMotion && blink) {
    noStroke(); fill(180, 0, 0);
    rect(px - 2, py - 17, 185, 25, 3);
    fill(255); textSize(14);
    text("⚡ DETECTED", px + 6, py);
  } else {
    fill(liveMotion ? color(255, 100, 100) : color(0, 180, 0));
    textSize(14);
    text(liveMotion ? "⚡ DETECTED" : "  SCANNING...", px, py);
  }
  py += 32;

  fill(0, 100, 0); textSize(10);
  text("[M] km/h ↔ m/s",         px, py); py += 15;
  text("[C] reset max speed",     px, py); py += 15;
  text("[+/-] sensitivity",       px, py); py += 15;
  text("[R] reconnect",           px, py); py += 15;
  text("[S] screenshot",          px, py);
}

// =============================================================
//  ANALOG SPEEDOMETER
//  Uses liveSpeedKmh directly — needle is always exact
// =============================================================
void drawSpeedometer() {
  int   cx = gaugeX, cy = gaugeY, r = gaugeR;

  // Gauge dark face
  noStroke();
  fill(10, 10, 10);
  ellipse(cx, cy, r * 2, r * 2);

  // Thin outer rim
  stroke(0, 70, 0);
  strokeWeight(2);
  noFill();
  ellipse(cx, cy, r * 2, r * 2);

  // ── Coloured arc zones (0–50 green, 50–80 yellow, 80–100 red) ──
  int arcR = r - 18;
  strokeWeight(16);
  noFill();

  stroke(0, 200, 50);                                    // green  0–50
  arc(cx, cy, arcR*2, arcR*2, needleAngle(0),  needleAngle(50));

  stroke(220, 200, 0);                                   // yellow 50–80
  arc(cx, cy, arcR*2, arcR*2, needleAngle(50), needleAngle(80));

  stroke(220, 50, 0);                                    // red    80–100
  arc(cx, cy, arcR*2, arcR*2, needleAngle(80), needleAngle(SPEED_MAX_KMH));

  // ── Minor ticks in the expanded low-speed zone (5, 15, 25 km/h) ──
  stroke(130);
  strokeWeight(1.5);
  for (int ms : new int[]{5, 15, 25}) {
    float ang = needleAngle(ms);
    line(cx + cos(ang)*(r - 28), cy + sin(ang)*(r - 28),
         cx + cos(ang)*(r - 20), cy + sin(ang)*(r - 20));
  }

  // ── Main ticks + labels every 10 km/h ───────────────────
  for (int s = 0; s <= (int)SPEED_MAX_KMH; s += 10) {
    float ang   = needleAngle(s);
    boolean isBig = (s == 0 || s == 10 || s == 20 || s == 50 || s == 100);
    float   r1  = isBig ? r - 36 : r - 30;
    float   r2  = r - 20;

    stroke(isBig ? 240 : 180);
    strokeWeight(isBig ? 2.5 : 1.8);
    line(cx + cos(ang)*r1, cy + sin(ang)*r1,
         cx + cos(ang)*r2, cy + sin(ang)*r2);

    noStroke();
    fill(isBig ? color(230) : color(180));
    textSize(isBig ? 11 : 10);
    textAlign(CENTER, CENTER);
    float lR = r - 52;
    text(s, cx + cos(ang)*lR, cy + sin(ang)*lR);
  }
  textAlign(LEFT, BASELINE);

  // ── Needle (exact — no lerp, no lag) ────────────────────
  float ang = needleAngle(liveSpeedKmh);    // ← EXACT live value
  float nx  = cx + cos(ang) * (r - 28);
  float ny  = cy + sin(ang) * (r - 28);

  // Shadow
  stroke(0); strokeWeight(5);
  line(cx, cy, nx, ny);

  // Colour by zone
  color needleCol = liveSpeedKmh < 50
                  ? color(50, 255, 100)
                  : liveSpeedKmh < 80
                    ? color(255, 220, 0)
                    : color(255, 60, 20);

  stroke(needleCol); strokeWeight(3);
  line(cx, cy, nx, ny);

  // Back tail
  float backAng = ang + PI;
  stroke(needleCol); strokeWeight(2);
  line(cx, cy, cx + cos(backAng)*18, cy + sin(backAng)*18);

  // Centre cap
  noStroke(); fill(55, 55, 55);
  ellipse(cx, cy, 22, 22);
  fill(needleCol);
  ellipse(cx, cy, 12, 12);

  // Unit label (inside gauge, below centre)
  fill(150, 150, 150);
  textAlign(CENTER, CENTER); textSize(13);
  text(showKmh ? "km/h" : "m/s", cx, cy + 55);
  textAlign(LEFT, BASELINE);

  // Max-speed marker on the arc
  if (maxSpeedKmh > 0) {
    float maxAng = needleAngle(maxSpeedKmh);
    stroke(255, 100, 0, 200); strokeWeight(2);
    float mr1 = r - 20, mr2 = r - 8;
    line(cx + cos(maxAng)*mr1, cy + sin(maxAng)*mr1,
         cx + cos(maxAng)*mr2, cy + sin(maxAng)*mr2);
  }
  strokeWeight(1);
}

// =============================================================
//  DIGITAL SPEED DISPLAY
//  Zone: y = 370 to 520 — completely clear of the gauge above
//        and the speed bar below.  Needle and number use the
//        same liveSpeedKmh so they always match exactly.
// =============================================================
void drawDigitalDisplay() {
  int cx   = gaugeX;
  int topY = gaugeY + gaugeR + 15;   // = 192 + 168 + 15 = 375

  // ── Large speed number ───────────────────────────────────
  float dispVal = showKmh ? liveSpeedKmh : liveSpeedMps;
  String dispStr = nf(dispVal, 1, 1);

  color numCol = liveSpeedKmh < 50
               ? color(50, 255, 100)
               : liveSpeedKmh < 80
                 ? color(255, 220, 0)
                 : color(255, 60, 20);

  textAlign(CENTER, TOP);
  fill(numCol);
  textSize(64);
  text(dispStr, cx, topY);             // number sits at y=375, height ~64px → bottom ~439

  // ── Unit label ───────────────────────────────────────────
  fill(numCol, 210);
  textSize(20);
  text(showKmh ? "km/h" : "m/s", cx, topY + 68);   // y ≈ 443

  // ── Secondary info (Doppler freq + other unit) ───────────
  fill(80, 150, 80);
  textSize(13);
  if (showKmh) {
    text(nf(liveSpeedMps, 1, 2) + " m/s     " +
         nf(liveFreqHz, 3, 1) + " Hz Doppler", cx, topY + 93); // y ≈ 468
  } else {
    text(nf(liveSpeedKmh, 3, 1) + " km/h     " +
         nf(liveFreqHz, 3, 1) + " Hz Doppler", cx, topY + 93);
  }

  textAlign(LEFT, BASELINE);
  // Digital display block occupies y = 375 → ~480
  // Speed bar starts at y = 492 → clear gap of ~12px
}

// =============================================================
//  SPEED BAR
//  Zone: y = 492 to 516 — clearly below the digital display
// =============================================================
void drawSpeedBar() {
  int bx = 230, by = 494, bw = 780, bh = 20;

  // Label
  noStroke();
  fill(0, 150, 0);
  textSize(10); textAlign(LEFT, BASELINE);
  text("0", bx - 1, by - 5);
  textAlign(RIGHT, BASELINE);
  text(nf(SPEED_MAX_KMH, 0, 0) + " km/h", bx + bw + 1, by - 5);
  textAlign(CENTER, BASELINE);
  text("SPEED", bx + bw / 2, by - 5);
  textAlign(LEFT, BASELINE);

  // Background (dark zone fills: 0-50 green, 50-80 yellow, 80-100 red)
  noStroke();
  fill(0, 60, 15);  rect(bx,                              by, bw * 50 / SPEED_MAX_KMH,  bh, 4, 0, 0, 4);
  fill(50, 50, 0);  rect(bx + bw * 50 / SPEED_MAX_KMH,  by, bw * 30 / SPEED_MAX_KMH,  bh);
  fill(60, 15, 0);  rect(bx + bw * 80 / SPEED_MAX_KMH,  by, bw * 20 / SPEED_MAX_KMH,  bh, 0, 4, 4, 0);

  // Active fill — exact live value, no smoothing
  float fillW = constrain(bw * liveSpeedKmh / SPEED_MAX_KMH, 0, bw);
  color barFill = liveSpeedKmh < 50
                ? color(0, 230, 70)
                : liveSpeedKmh < 80
                  ? color(230, 210, 0)
                  : color(230, 55, 15);
  fill(barFill, 210);
  // Draw with correct corners based on position
  if (fillW < bw * 50 / SPEED_MAX_KMH)
    rect(bx, by, fillW, bh, 4, 0, 0, 4);
  else
    rect(bx, by, fillW, bh, 4);

  // Tick marks every 10 km/h
  stroke(0, 50, 0); strokeWeight(1);
  for (int s = 10; s < (int)SPEED_MAX_KMH; s += 10) {
    float tx = bx + bw * s / SPEED_MAX_KMH;
    line(tx, by, tx, by + bh);
  }

  // Max-speed marker
  if (maxSpeedKmh > 0) {
    float mx = bx + bw * maxSpeedKmh / SPEED_MAX_KMH;
    stroke(255, 100, 0); strokeWeight(2.5);
    line(mx, by - 5, mx, by + bh + 5);
    noStroke(); fill(255, 100, 0);
    textSize(9); textAlign(CENTER, TOP);
    text("MAX", mx, by + bh + 6);
    textAlign(LEFT, BASELINE);
  }
  strokeWeight(1);
}

// =============================================================
//  SPEED HISTORY GRAPH
//  Zone: y = 534 to 624  (height 90 — compact, leaves room for spectrum)
// =============================================================
void drawHistoryGraph() {
  int gx = 220, gy = 534, gw = 800, gh = 90;

  // Background
  noStroke(); fill(0, 15, 0);
  rect(gx, gy, gw, gh, 4);

  // Grid lines (horizontal, every 20 km/h)
  for (int s = 0; s <= (int)SPEED_MAX_KMH; s += 20) {
    float ly = map(s, 0, SPEED_MAX_KMH, gy + gh, gy);
    stroke(0, 55, 0); strokeWeight(1);
    line(gx, ly, gx + gw, ly);
    noStroke(); fill(0, 95, 0);
    textSize(10); textAlign(RIGHT, CENTER);
    text(s, gx - 5, ly);
    textAlign(LEFT, BASELINE);
  }

  // Subtle zone bands
  noStroke();
  float y0  = map(0,  0, SPEED_MAX_KMH, gy+gh, gy);
  float y50 = map(50, 0, SPEED_MAX_KMH, gy+gh, gy);
  float y80 = map(80, 0, SPEED_MAX_KMH, gy+gh, gy);
  float ytop= gy;
  fill(0, 70, 20, 25);  rect(gx, y50, gw, y0 - y50);
  fill(70, 70,  0, 25); rect(gx, y80, gw, y50 - y80);
  fill(70, 20,  0, 25); rect(gx, ytop, gw, y80 - ytop);

  // Title
  noStroke(); fill(0, 110, 0);
  textSize(11); textAlign(RIGHT, TOP);
  text("Speed history (~8 s)", gx + gw - 4, gy + 4);
  textAlign(LEFT, BASELINE);

  // ── Plot line ───────────────────────────────────────────
  for (int i = 0; i < HIST_LEN - 1; i++) {
    int   i0 = (histHead + i)     % HIST_LEN;
    int   i1 = (histHead + i + 1) % HIST_LEN;
    float s0 = histKmh[i0];
    float s1 = histKmh[i1];
    float x0 = map(i,     0, HIST_LEN - 1, gx, gx + gw);
    float x1 = map(i + 1, 0, HIST_LEN - 1, gx, gx + gw);
    float ly0 = constrain(map(s0, 0, SPEED_MAX_KMH, gy+gh, gy), gy, gy+gh);
    float ly1 = constrain(map(s1, 0, SPEED_MAX_KMH, gy+gh, gy), gy, gy+gh);
    color lc  = s0 < 50 ? color(0, 220, 80) : s0 < 80 ? color(220, 200, 0) : color(220, 60, 20);
    stroke(lc); strokeWeight(2);
    line(x0, ly0, x1, ly1);
  }

  // Current value dot at right edge
  float curY = constrain(map(liveSpeedKmh, 0, SPEED_MAX_KMH, gy+gh, gy), gy, gy+gh);
  noStroke(); fill(255, 255, 255, 190);
  ellipse(gx + gw, curY, 9, 9);

  // Border
  stroke(0, 70, 0); strokeWeight(1); noFill();
  rect(gx, gy, gw, gh, 4);
}

// =============================================================
//  DOPPLER FREQUENCY SPECTRUM
//  Zone: y = 644 to 784  (height 140)
//
//  X axis : Doppler frequency  0 – ~498 Hz
//           Each bar = one FFT bin ≈ 9.77 Hz wide
//  Y axis : Magnitude normalised 0–1000 by the firmware
//           (1000 = strongest bin anywhere in the full spectrum)
//
//  Colour legend:
//    Blue-green gradient → noise floor / low energy
//    Yellow             → moderate signal
//    Red                → dominant Doppler peak
//
//  Vertical white marker : current measured peak frequency
//  Walking ≈ 70–150 Hz, Running ≈ 300–500 Hz
// =============================================================
void drawDopplerSpectrum() {
  int gx = 220, gy = 648, gw = 800, gh = 130;

  // ── Background ─────────────────────────────────────────
  noStroke(); fill(0, 15, 5);
  rect(gx, gy, gw, gh, 4);

  // ── Horizontal grid (25% increments) ───────────────────
  stroke(0, 50, 20); strokeWeight(1);
  for (int pct : new int[]{250, 500, 750, 1000}) {
    float ly = map(pct, 0, 1000, gy + gh, gy);
    line(gx, ly, gx + gw, ly);
    noStroke(); fill(0, 80, 30); textSize(9); textAlign(RIGHT, CENTER);
    text(pct / 10 + "%", gx - 3, ly);
    textAlign(LEFT, BASELINE); stroke(0, 50, 20);
  }

  // ── Spectrum bars ───────────────────────────────────────
  float barW = (float) gw / SPEC_BINS;

  for (int i = 0; i < SPEC_BINS; i++) {
    float val  = specBins[i];           // 0–1000
    float barH = map(val, 0, 1000, 0, gh - 2);
    float bx   = gx + i * barW;
    float by   = gy + gh - barH;

    // Colour: blue-green (low) → yellow (medium) → red (peak)
    color barCol;
    if (val < 300) {
      barCol = lerpColor(color(0, 80, 60), color(0, 180, 100), val / 300.0);
    } else if (val < 700) {
      barCol = lerpColor(color(0, 180, 100), color(220, 200, 0), (val - 300) / 400.0);
    } else {
      barCol = lerpColor(color(220, 200, 0), color(255, 40, 10), (val - 700) / 300.0);
    }

    noStroke(); fill(barCol);
    rect(bx, by, barW - 1, barH);

    // Bright top cap on each bar
    fill(red(barCol) + 60, green(barCol) + 60, blue(barCol) + 60, 200);
    rect(bx, by, barW - 1, min(barH, 3));
  }

  // ── Vertical marker at current peak Doppler frequency ──
  if (liveFreqHz > 0) {
    float peakBinF = liveFreqHz / HZ_PER_BIN;    // fractional bin index
    float markerX  = gx + (peakBinF / SPEC_BINS) * gw;
    markerX = constrain(markerX, gx, gx + gw);

    // White dashed line
    stroke(255, 255, 255, 220); strokeWeight(1.5);
    for (int dy = 0; dy < gh; dy += 8) {
      if (dy % 16 < 8) line(markerX, gy + dy, markerX, min(gy + dy + 6, gy + gh));
    }

    // Label above the marker
    noStroke(); fill(255, 255, 220);
    textSize(10); textAlign(CENTER, BOTTOM);
    text(nf(liveFreqHz, 0, 1) + " Hz", markerX, gy - 2);
    textAlign(LEFT, BASELINE);
  }

  // ── X-axis frequency labels ────────────────────────────
  // Mark every 100 Hz with a tick and label
  int[] markHz  = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500};
  stroke(0, 80, 30); strokeWeight(1);
  for (int hz : markHz) {
    float lx = gx + (hz / (HZ_PER_BIN * SPEC_BINS)) * gw;
    if (lx > gx + gw) continue;
    line(lx, gy + gh, lx, gy + gh + 4);

    noStroke(); fill(0, 120, 60);
    boolean major = (hz % 100 == 0);
    textSize(major ? 10 : 9); textAlign(CENTER, TOP);
    if (major) {
      text(hz + " Hz", lx, gy + gh + 5);
    } else {
      text(hz, lx, gy + gh + 5);
    }
    textAlign(LEFT, BASELINE);
    stroke(0, 80, 30);
  }

  // Speed equivalents at key frequency marks
  int[] speedMarkHz = {100, 200, 300, 400};
  for (int hz : speedMarkHz) {
    float lx  = gx + (hz / (HZ_PER_BIN * SPEC_BINS)) * gw;
    float kmh = hz * KMPH_PER_HZ;
    noStroke(); fill(0, 90, 40);
    textSize(9); textAlign(CENTER, TOP);
    text(nf(kmh, 0, 0) + "k", lx, gy + gh + 18);
    textAlign(LEFT, BASELINE);
  }

  // ── Title and legend ───────────────────────────────────
  noStroke(); fill(0, 150, 60);
  textSize(11); textAlign(RIGHT, TOP);
  text("Doppler Spectrum  (0 – ~500 Hz)   |  bin width ≈ 9.8 Hz",
       gx + gw - 4, gy + 4);
  textAlign(LEFT, TOP);
  text(liveMotion ? "● SIGNAL" : "○ NOISE", gx + 6, gy + 4);
  textAlign(LEFT, BASELINE);

  // ── Border ─────────────────────────────────────────────
  stroke(0, 80, 30); strokeWeight(1); noFill();
  rect(gx, gy, gw, gh, 4);
}

// =============================================================
//  STATUS BAR   y = 800
// =============================================================
void drawStatusBar() {
  fill(0, 85, 0); textSize(10); textAlign(LEFT, BASELINE);
  text("HB100 Speed Radar v5.2  |  10.525 GHz  |  " +
       "0.01424 m/s per Hz  |  Threshold: " + nf(magThreshold, 0, 0) +
       "  |  GPIO34  |  LM324  |  FFT-512 @ 5 kHz  |  Spectrum 0–500 Hz",
       10, height - 8);
}

// =============================================================
//  SERIAL
// =============================================================
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = line.trim();
  if (line.startsWith("RADAR_SPEED_READY") ||
      line.startsWith("---") ||
      line.startsWith("#") ||
      line.length() < 5) return;

  String[] tok = split(line, ',');
  if (tok.length < 6) return;
  try {
    liveMs        = Long.parseLong(tok[0].trim());
    liveFreqHz    = Float.parseFloat(tok[1].trim());
    liveSpeedMps  = Float.parseFloat(tok[2].trim());
    liveSpeedKmh  = Float.parseFloat(tok[3].trim());
    liveMagnitude = Float.parseFloat(tok[4].trim());
    liveMotion    = tok[5].trim().equals("1");
    lastDataSys   = System.currentTimeMillis();

    // Parse the 51 spectrum bins (fields 6–56)
    if (tok.length >= 6 + SPEC_BINS) {
      for (int i = 0; i < SPEC_BINS; i++) {
        specBins[i] = Integer.parseInt(tok[6 + i].trim());
      }
    }
  } catch (Exception e) { /* skip malformed line */ }
}

// =============================================================
//  KEY CONTROLS
// =============================================================
void keyPressed() {
  switch (Character.toLowerCase(key)) {
    case 'm': showKmh = !showKmh;                                   break;
    case 'c': maxSpeedKmh = 0;                                       break;
    case 'r': openPort();                                            break;
    case 's': saveFrame("speed_radar_####.png");                     break;
    case '+': magThreshold = min(magThreshold + 100, 5000);         break;
    case '-': magThreshold = max(magThreshold - 100,  200);         break;
  }
}

// =============================================================
//  SERIAL CONNECTION
// =============================================================
void openPort() {
  try {
    if (port != null) { port.stop(); port = null; }
    port     = new Serial(this, SERIAL_PORT, BAUD_RATE);
    port.bufferUntil('\n');
    portOpen  = true;
  } catch (Exception e) {
    portOpen  = false;
    println("Serial error: " + e.getMessage());
  }
}
