// 4.2 b/w GDEY042T81 https://www.good-display.com/product/386.html
// Uses SSD1683 controller.
// Note from GOODISPLAY: The GDEQ042T81 is fully compatible with GDEY042T81
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include <stdint.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include <string>
#include <epd.h>
#include <Adafruit_GFX.h>
#include <epdspi.h>
#include <gdew_colors.h>
#include <esp_timer.h>

// Controller: SSD1683

#define GDEY042T81_WIDTH 400
#define GDEY042T81_HEIGHT 300
#define GDEY042T81_BUFFER_SIZE (uint32_t(GDEY042T81_WIDTH) * uint32_t(GDEY042T81_HEIGHT) / 8)

#define GDEY042T81_8PIX_BLACK 0x00
#define GDEY042T81_8PIX_WHITE 0xFF

enum GDEY042T81_REFRESH_MODE {
  GDEY042T81_REFRESH_MODE_FULL,
  GDEY042T81_REFRESH_MODE_FAST,
  GDEY042T81_REFRESH_MODE_PARTIAL,
  GDEY042T81_REFRESH_MODE_4G,
};

class Gdey042T81 : public Epd
{
  public:
   
    Gdey042T81(EpdSpi& IO);
    uint8_t colors_supported = 1;
    
    uint8_t fastmode = 0; // With 1 it will not go to power off
    bool is_powered = false;

    void setRefreshMode(GDEY042T81_REFRESH_MODE mode);
    
    // EPD tests 
    void init(bool debug = false);

    void setMode(GDEY042T81_REFRESH_MODE mode);

    void hwInitFull();
    void hwInitFast();
    void hwInit4Gray();

    void drawPixel(int16_t x, int16_t y, uint16_t color);  // Override GFX own drawPixel method
    
    void fillScreen(uint16_t color);
    void update();
    void updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation = true);
    
    void updateWindow2(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation = true);
    void _logicalToNative2(int16_t lx, int16_t ly, int16_t& nx, int16_t& ny);


    void deepsleep();

  private:
    EpdSpi& IO;

    void _hwInitPartial();
    uint16_t _setPartialRamArea(uint16_t x, uint16_t y, uint16_t xe, uint16_t ye);
    bool _partial_mode = false;

    uint8_t _buffer1[GDEY042T81_BUFFER_SIZE];
    uint8_t _buffer2[GDEY042T81_BUFFER_SIZE];
    GDEY042T81_REFRESH_MODE _refresh_mode = GDEY042T81_REFRESH_MODE_FULL;

    void _wakeUp();
    void _sleep();
    void _waitBusy(const char* message);
    void _rotate(uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h);
    void _logicalToNative(int16_t lx, int16_t ly, int16_t& nx, int16_t& ny);

    void _hwPreInit();
    void _hwRAMInit();

    static const epd_init_4 epd_resolution;
};
