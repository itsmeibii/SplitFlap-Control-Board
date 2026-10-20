/*
 * HUB75 test sketch — ESP32 + single 64x32 FM6124 panel
 * ------------------------------------------------------
 * Config matches what we landed on for your panel:
 *   - FM6124 driver init
 *   - G/B channels swapped vs library defaults
 *   - E disabled (1/16 scan, 32-row panel)
 *
 * Library: ESP32-HUB75-MatrixPanel-DMA  (by mrcodetastic)
 *   Install via Library Manager, or:
 *   https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA
 *   Also needs: Adafruit GFX, Adafruit BusIO
 *
 * Board: "ESP32 Dev Module"  (plain ESP32-D0WD, not S3)
 *
 * Panel must be plugged into J1 (input), not J2.
 */

#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define PANEL_CHAIN 1

MatrixPanel_I2S_DMA *display = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin order: r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk
  HUB75_I2S_CFG::i2s_pins pins = {
    25,   // R1
    27,   // G1   <-- swapped
    26,   // B1   <-- swapped
    14,   // R2
    13,   // G2   <-- swapped
    12,   // B2   <-- swapped
    23,   // A
    19,   // B
    5,    // C
    17,   // D
    -1,   // E    (disabled, 32-row panel)
    4,    // LAT
    15,   // OE
    16    // CLK
  };

  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN, pins);
  mxconfig.driver = HUB75_I2S_CFG::FM6124;   // required for this panel

  display = new MatrixPanel_I2S_DMA(mxconfig);
  if (!display->begin()) {
    Serial.println("Display begin() failed — check wiring/power.");
  }
  display->setBrightness8(90);   // 0-255
  display->clearScreen();
}

void loop() {
  // --- Pattern 1: RGB bars ---
  // Confirms color channel order. If a bar shows the wrong color,
  // the R/G/B swap in the pin struct above needs adjusting.
  colorBars();
  delay(2500);

  // --- Pattern 2: border + diagonal ---
  // A solid fill CANNOT reveal row-remap/banding issues; a line can.
  // If you see the 8-on/8-off banding here, this panel likely needs
  // VirtualMatrixPanel with FOUR_SCAN_32PX_HIGH.
  lineTest();
  delay(2500);
}

void colorBars() {
  display->clearScreen();
  int w = PANEL_RES_X / 3;
  display->fillRect(0,      0, w, PANEL_RES_Y, display->color565(255, 0, 0));   // R
  display->fillRect(w,      0, w, PANEL_RES_Y, display->color565(0, 255, 0));   // G
  display->fillRect(2 * w,  0, PANEL_RES_X - 2 * w, PANEL_RES_Y,
                    display->color565(0, 0, 255));                              // B
}

void lineTest() {
  display->clearScreen();
  uint16_t white = display->color565(255, 255, 255);

  // Border
  display->drawRect(0, 0, PANEL_RES_X, PANEL_RES_Y, white);

  // Diagonal — spans all rows so a scan/remap error is obvious
  for (int x = 0; x < PANEL_RES_X; x++) {
    int y = (x * (PANEL_RES_Y - 1)) / (PANEL_RES_X - 1);
    display->drawPixel(x, y, display->color565(255, 180, 0));
  }

  // Row markers every 8 px on the left edge — makes banding readable
  for (int y = 0; y < PANEL_RES_Y; y += 8) {
    display->drawFastHLine(0, y, 6, display->color565(0, 255, 255));
  }
}
