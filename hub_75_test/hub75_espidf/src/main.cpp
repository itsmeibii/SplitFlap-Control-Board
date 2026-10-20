/*
 * HUB75 test — ESP32 + single 64x32 FM6124 panel, ESP-IDF port of hub75_test.ino
 * ------------------------------------------------------------------------------
 *   - FM6124 driver init
 *   - G/B channels swapped vs library defaults
 *   - E disabled (1/16 scan, 32-row panel)
 *   - Same pin map as the Arduino sketch, so existing wiring is unchanged
 *
 * Library: ESP32-HUB75-MatrixPanel-DMA, vendored in lib/ and built with
 * -DNO_GFX (no Adafruit GFX). Without GFX there is no drawRect/drawLine, so
 * the test patterns below are built from fillRect / drawFastH/VLine / drawPixelRGB888.
 *
 * Panel must be plugged into J1 (input), not J2.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"

static const char *TAG = "hub75";

static constexpr int PANEL_RES_X = 64;
static constexpr int PANEL_RES_Y = 32;
static constexpr int PANEL_CHAIN = 1;

static MatrixPanel_I2S_DMA *display = nullptr;

static void colorBars()
{
  // Confirms color channel order. If a bar shows the wrong color,
  // the R/G/B swap in the pin struct needs adjusting.
  display->clearScreen();
  const int w = PANEL_RES_X / 3;
  display->fillRect(0,     0, w, PANEL_RES_Y, MatrixPanel_I2S_DMA::color565(255, 0, 0));   // R
  display->fillRect(w,     0, w, PANEL_RES_Y, MatrixPanel_I2S_DMA::color565(0, 255, 0));   // G
  display->fillRect(2 * w, 0, PANEL_RES_X - 2 * w, PANEL_RES_Y,
                    MatrixPanel_I2S_DMA::color565(0, 0, 255));                             // B
}

static void lineTest()
{
  // A solid fill CANNOT reveal row-remap/banding issues; a line can.
  // If you see 8-on/8-off banding here, this panel likely needs the
  // virtual-matrix FOUR_SCAN_32PX_HIGH mapping.
  display->clearScreen();
  const uint16_t white = MatrixPanel_I2S_DMA::color565(255, 255, 255);

  // Border
  display->drawFastHLine(0, 0, PANEL_RES_X, white);
  display->drawFastHLine(0, PANEL_RES_Y - 1, PANEL_RES_X, white);
  display->drawFastVLine(0, 0, PANEL_RES_Y, white);
  display->drawFastVLine(PANEL_RES_X - 1, 0, PANEL_RES_Y, white);

  // Diagonal — spans all rows so a scan/remap error is obvious
  for (int x = 0; x < PANEL_RES_X; x++) {
    const int y = (x * (PANEL_RES_Y - 1)) / (PANEL_RES_X - 1);
    display->drawPixelRGB888(x, y, 255, 180, 0);
  }

  // Row markers every 8 px on the left edge — makes banding readable
  for (int y = 0; y < PANEL_RES_Y; y += 8) {
    display->drawFastHLine(0, y, 6, MatrixPanel_I2S_DMA::color565(0, 255, 255));
  }
}

extern "C" void app_main(void)
{
  // Pin order: r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk
  HUB75_I2S_CFG::i2s_pins pins = {
    25,   // R1
    27,   // G1   <-- swapped
    26,   // B1   <-- swapped
    14,   // R2
    13,   // G2   <-- swapped
    12,   // B2   <-- swapped (GPIO12 is a strapping pin: high at reset = flash-voltage/boot failure)
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
    ESP_LOGE(TAG, "display->begin() failed — check wiring/power.");
  } else {
    ESP_LOGI(TAG, "display started (%dx%d)", PANEL_RES_X, PANEL_RES_Y);
  }
  display->setBrightness8(90);   // 0-255
  display->clearScreen();

  while (true) {
    ESP_LOGI(TAG, "pattern: color bars");
    colorBars();
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGI(TAG, "pattern: line test");
    lineTest();
    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}
