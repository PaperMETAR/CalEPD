#include "goodisplay/gdey042T81.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <algorithm> // Needed for std::min/max

// Constructor
Gdey042T81::Gdey042T81(EpdSpi& dio): 
  Adafruit_GFX(GDEY042T81_WIDTH, GDEY042T81_HEIGHT),
  Epd(GDEY042T81_WIDTH, GDEY042T81_HEIGHT), IO(dio)
{
  printf("Gdey042T81() constructor injects IO and extends Adafruit_GFX(%d,%d)\n",
  GDEY042T81_WIDTH, GDEY042T81_HEIGHT);  
}

//Initialize the display
void Gdey042T81::init(bool debug)
{
    debug_enabled = debug;
    _refresh_mode = GDEY042T81_REFRESH_MODE_FAST;
    if (debug_enabled) printf("Gdey042T81::init(%d) and reset EPD\n", debug);
    //Initialize the Epaper and reset it
    IO.init(4, debug); // 4MHz frequency, debug

    //Reset the display
    IO.reset(20);
    fillScreen(EPD_WHITE);
}

void Gdey042T81::setRefreshMode(GDEY042T81_REFRESH_MODE mode){
  _refresh_mode = mode;
  if (debug_enabled) printf("Gdey042T81::setRefreshMode(%d)\n", mode);
}

void Gdey042T81::_hwPreInit() {
  // TODO: Confirm with vendor. Comments in code seems to imply that this should always be done, even with 
  // faster redrawing.
  // if (fastmode == 0 && !is_powered) {
  IO.reset(10);
  _waitBusy("epd_wakeup reset");  //waiting for the electronic paper IC to release the idle signal
  is_powered = true;
  IO.cmd(0x12);     //SWRESET
  _waitBusy("epd_wakeup swreset");  //waiting for the electronic paper IC to release the idle signal
  // }

	IO.cmd(0x01); // Driver output control
	IO.data((GDEY042T81_HEIGHT-1)%256);
	IO.data((GDEY042T81_HEIGHT-1)/256);
	IO.data(0x00);

  // These commands are similar to 042Z96 also SSD controller brand
  if (_refresh_mode != GDEY042T81_REFRESH_MODE_4G) {
    IO.cmd(0x21);  // Display update control
    IO.data(0x40); // Bypass RAM content as 0.
    IO.data(0x00);
  }

  IO.cmd(0x3C); // board - BorderWavefrom
  IO.data(0x05);

  if (_refresh_mode == GDEY042T81_REFRESH_MODE_FAST) {
    IO.cmd(0x1A); // Write to temperature register
    IO.data(0x6E);
    IO.cmd(0x22); // Load temperature value
    IO.data(0x91);
    IO.cmd(0x20);
    _waitBusy("epd_load temp fast");
  } else if (_refresh_mode == GDEY042T81_REFRESH_MODE_4G) {
    IO.cmd(0x1A); // Write to temperature register
    IO.data(0x5A);
    IO.cmd(0x22); // Load temperature value
    IO.data(0x91);
    IO.cmd(0x20);
    _waitBusy("epd_load temp 4G");
  }

  IO.cmd(0x11);  // Data entry mode
  IO.data(0x01);

  IO.cmd(0x44);
  IO.data(0x00); // RAM x address start at 0
  IO.data(0x31); // RAM x address end at 31h(49+1)*8->400
  IO.cmd(0x45);
  IO.data(0x2B);   // RAM y address start at 12Bh
  IO.data(0x01);
  IO.data(0x00); // RAM y address end at 00h
  IO.data(0x00);

  IO.cmd(0x4E);
  IO.data(0x00);
  IO.cmd(0x4F);
  IO.data(0x2B);
  IO.data(0x01);
  _waitBusy("epd_wakeup RAM init");
}

void Gdey042T81::_hwRAMInit(){
}

void Gdey042T81::hwInitFull(){
}

void Gdey042T81::hwInitFast(){
}

void Gdey042T81::hwInit4Gray(){
}

void Gdey042T81::fillScreen(uint16_t color) {
  // Force white for non-4G mode and gray colors
  if (_refresh_mode != GDEY042T81_REFRESH_MODE_4G && (color == EPD_DARKGREY || color == EPD_LIGHTGREY)) {
    color = EPD_WHITE;
  }
  
  uint16_t buf1 = 0;
  uint16_t buf2 = 0;

  switch (color) {
    case EPD_DARKGREY: // Dark gray
      buf1 = 0x00;  // All bits 0
      buf2 = 0xFF;  // All bits 1
      break;
    case EPD_LIGHTGREY: // Light gray
      buf1 = 0xFF;  // All bits 1
      buf2 = 0x00;  // All bits 0
      break;
    case EPD_WHITE: // White
      buf1 = 0xFF;  // All bits 1
      buf2 = 0xFF;  // All bits 1
      break;
    default: // Black (case 0)
      buf1 = 0x00;  // All bits 0
      buf2 = 0x00;  // All bits 0
      break;
  }

  for (uint16_t x = 0; x < sizeof(_buffer1); x++) {
    _buffer1[x] = buf1;
    _buffer2[x] = buf2;
  }

  if (debug_enabled) printf("fillScreen(%x) 4G:%d buffer len:%d\n", 
                          color, (_refresh_mode == GDEY042T81_REFRESH_MODE_4G), sizeof(_buffer1));
}

void Gdey042T81::_wakeUp(){
  _hwPreInit();
}

void Gdey042T81::update()
{
  uint64_t startTime = esp_timer_get_time();
  _partial_mode = false;
  _wakeUp();
  
  // BLACK: Write RAM for black(0)/white (1)
  IO.cmd(0x24);
  // v2 SPI optimizing. Check: https://github.com/martinberlin/cale-idf/wiki/About-SPI-optimization
  uint16_t i = 0;
  uint16_t bufferLength = GDEY042T81_BUFFER_SIZE; // Remove +1 to prevent buffer overflow
  uint16_t bufferMaxSpi = 3000;
  uint8_t xbuf[bufferMaxSpi];

  IO.cmd(0x24);
  uint32_t bufindex = 0;
  for (i = 0; i < bufferLength; i++) {
      xbuf[bufindex] = _buffer1[i];
      bufindex++;
      // Flush SPI buffer when full or at the end
      if (bufindex == bufferMaxSpi || i == bufferLength - 1) {
          IO.data(xbuf, bufindex);  // Send actual number of bytes
          bufindex = 0;
      }
  }

  IO.cmd(0x26);
  bufindex = 0;
  for (i = 0; i < bufferLength; i++) {
      xbuf[bufindex] = _buffer2[i];
      bufindex++;
      // Flush SPI buffer when full or at the end
      if (bufindex == bufferMaxSpi || i == bufferLength - 1) {
          IO.data(xbuf, bufindex);  // Send actual number of bytes
          bufindex = 0;
      }
  }

  uint64_t endTime = esp_timer_get_time();
  
  IO.cmd(0x22);  // Display Update Control
  switch (_refresh_mode) {
    case GDEY042T81_REFRESH_MODE_FULL:
      IO.data(0xF7);
      break;
    case GDEY042T81_REFRESH_MODE_FAST:
      IO.data(0xC7);
      break;
    case GDEY042T81_REFRESH_MODE_PARTIAL:
      IO.data(0xFF);
      break;
    case GDEY042T81_REFRESH_MODE_4G:
      // TODO: Maybe there is an undocumented fast mode for 4G?
      IO.data(0xCF);  // 4G mode, full refresh
      break;
  }
  IO.cmd(0x20);  //Activate Display Update Sequence
  _waitBusy("update");
  uint64_t refreshTime= esp_timer_get_time();
  
  if (debug_enabled) {
      printf("\n\nSTATS (ms)\n%llu _wakeUp+send Buffer\n%llu refreshTime\n%llu total time in millis\n",
             (endTime - startTime) / 1000, (refreshTime - endTime) / 1000, (refreshTime - startTime) / 1000);
  }

  _sleep();
}

void Gdey042T81::_hwInitPartial() {
  IO.reset(10);
  _waitBusy("epd_wakeup reset");  //waiting for the electronic paper IC to release the idle signal
  is_powered = true;
  
  IO.cmd(0x3C); // BorderWavefrom
  IO.data(0x80);

  IO.cmd(0x21); // Display update control
  IO.data(0x00);
  IO.data(0x00);
}

// Helper based on this file's drawPixel transform logic
// Converts logical GFX coordinates to the display's native mirrored coordinates
void Gdey042T81::_logicalToNative2(int16_t lx, int16_t ly, int16_t& nx, int16_t& ny) {
    nx = lx;
    ny = ly;

    // Apply rotation transform first (like drawPixel)
    switch (getRotation()) {
        case 1: // 90
            swap(nx, ny);
            // Use GDEY042T81_WIDTH consistently as per drawPixel implementation
            nx = GDEY042T81_WIDTH - 1 - nx;
            break;
        case 2: // 180
            nx = GDEY042T81_WIDTH - 1 - nx;
            ny = GDEY042T81_HEIGHT - 1 - ny;
            break;
        case 3: // 270
            swap(nx, ny);
            // Use GDEY042T81_HEIGHT consistently as per drawPixel implementation
            ny = GDEY042T81_HEIGHT - 1 - ny;
            break;
         // case 0: no rotation transform
    }

    // Apply final mirroring transform (like drawPixel)
    // Uses GDEY042T81_WIDTH (width() in drawPixel context)
    nx = GDEY042T81_WIDTH - 1 - nx;
}

void Gdey042T81::_rotate(uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h) {
  ESP_LOGI(TAG, "rotate: x=%u y=%u w=%u h=%u rotation=%d", x, y, w, h, getRotation());
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      swap(w, h);
      x = GDEY042T81_WIDTH - x - w - 1;
      break;
    case 2:
      x = GDEY042T81_WIDTH - x - w - 1;
      y = GDEY042T81_HEIGHT - y - h - 1;
      ESP_LOGI(TAG, "rotate case 2: x=%u y=%u w=%u h=%u rot=%d", x, y, w, h, getRotation());
      break;
    case 3:
      swap(x, y);
      swap(w, h);
      y = GDEY042T81_HEIGHT - y - h - 1;
      break;
  }
  ESP_LOGI(TAG, "rotate: x=%u y=%u w=%u h=%u", x, y, w, h);
}

void Gdey042T81::updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation) {
    if (w == 0 || h == 0) {
      ESP_LOGE(TAG, "updateWindow: Width or height is 0, ignoring update: w=%u h=%u", w, h);
      return;
    }
    ESP_LOGI(TAG, "updateWindow incoming: x=%u y=%u w=%u h=%u rot=%d", x, y, w, h, getRotation());

    // Apply mirroring in X direction first - this is critical for correct coordinate mapping
    // Same mirroring as in drawPixel, applied to the region
    uint16_t mirrored_x = GDEY042T81_WIDTH - x - w - 1;
    x = mirrored_x;

    if ((x+w) > (GDEY042T81_WIDTH - 1)) {
      ESP_LOGE(TAG, "updateWindow: x+w > GDEY042T81_WIDTH, limiting w to max-width");
      w = GDEY042T81_WIDTH - 1 - x;
    }

    if ((y+h) > (GDEY042T81_HEIGHT - 1)) {
      ESP_LOGE(TAG, "updateWindow: y+h > GDEY042T81_HEIGHT, limiting h to max-height");
      h = GDEY042T81_HEIGHT - 1 - y;
    }

    ESP_LOGI(TAG, "updateWindow after mirroring: x=%u y=%u w=%u h=%u", x, y, w, h);
    
    // Apply rotation if needed - AFTER mirroring, to match drawPixel's sequence
    if (using_rotation) _rotate(x, y, w, h);
    ESP_LOGI(TAG, "updateWindow after rotation: x=%u y=%u w=%u h=%u rot=%d", x, y, w, h, getRotation());
    
    // Boundary checks
    if (x >= GDEY042T81_WIDTH) return;
    if (y >= GDEY042T81_HEIGHT) return;
    
    // Calculate boundaries, ensuring they're within display limits
    uint16_t xe = gx_uint16_min(GDEY042T81_WIDTH, x + w) - 1;
    uint16_t ye = gx_uint16_min(GDEY042T81_HEIGHT, y + h) - 1;
    
    // Convert to byte-aligned coordinates for X (display uses byte addressing for X)
    uint16_t xs_bx = x / 8;
    uint16_t xe_bx = (xe + 7) / 8;  // Proper ceil division for bytes
    
    ESP_LOGI(TAG, "updateWindow native: x=%u y=%u xe=%u ye=%u xs_bx=%u xe_bx=%u", x, y, xe, ye, xs_bx, xe_bx);

    // Initialize the display for partial update
    IO.reset(10);
    _waitBusy("epd_wakeup reset");
    IO.cmd(0x12); // SWRESET
    _waitBusy("epd_wakeup swreset");

    IO.cmd(0x01); // Driver output control
    IO.data((GDEY042T81_HEIGHT-1)%256);
    IO.data((GDEY042T81_HEIGHT-1)/256);
    IO.data(0x00);

    IO.cmd(0x11); // Data entry mode       
    IO.data(0x01);

    IO.cmd(0x21); // Display update control
    IO.data(0x00);
    IO.data(0x00);

    IO.cmd(0x3C); // BorderWavefrom
    IO.data(0x80);

    // Set X range (using byte-aligned coordinates)
    IO.cmd(0x44);
    IO.data(xs_bx);
    IO.data(xe_bx - 1);  // Hardware expects inclusive range end point

    // Set Y range (using pixel coordinates)
    IO.cmd(0x45);
    IO.data(y % 256);
    IO.data(y / 256);
    IO.data(ye % 256);
    IO.data(ye / 256);

    // Set current position (start position for data transfer)
    IO.cmd(0x4E);
    IO.data(xs_bx);
    IO.cmd(0x4F);
    IO.data((GDEY042T81_HEIGHT - 1 - y) % 256);
    IO.data((GDEY042T81_HEIGHT - 1 - y) / 256);

    // Prepare to transfer image data
    IO.cmd(0x24);  // Write RAM (black/white buffer)
    uint16_t bufferMaxSpi = 3000;
    uint8_t xbuf[bufferMaxSpi];
    uint32_t bufIndex = 0;

    // Calculate bytes per row once (constant for this display)
    const uint16_t bytes_per_row = GDEY042T81_WIDTH / 8;
    
    // Process the window data row by row and byte by byte
    for (int16_t y1 = y; y1 <= ye; y1++) {
        for (uint16_t x_byte = xs_bx; x_byte < xe_bx; x_byte++) {
            uint32_t idx = (uint32_t)y1 * bytes_per_row + x_byte;
            if (idx < GDEY042T81_BUFFER_SIZE) {
                xbuf[bufIndex++] = _buffer1[idx];
                if (bufIndex == bufferMaxSpi) {
                    IO.data(xbuf, bufIndex);
                    bufIndex = 0;
                }
            } else {
                ESP_LOGE(TAG, "Buffer index out of bounds: %lu", (unsigned long)idx);
            }
        }
    }
    
    // Send any remaining data
    if (bufIndex > 0) {
        IO.data(xbuf, bufIndex);
    }

    // Finalize the update
    IO.cmd(0x22);
    IO.data(0xFF); // Use partial update mode
    IO.cmd(0x20);
    _waitBusy("partial update");

    _sleep();
}

void Gdey042T81::updateWindow2(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation) {
    if (w == 0 || h == 0) return;
    ESP_LOGI(TAG, "updateWindow logical: x=%u y=%u w=%u h=%u rot=%d", x, y, w, h, getRotation());

    // Calculate the window coordinates in the display's native system
    int16_t native_x, native_y, native_w, native_h;

    // Transform the 4 corners of the logical box to native coordinates
    int16_t lx1 = x;
    int16_t ly1 = y;
    int16_t lx2 = x + w - 1;
    int16_t ly2 = y + h - 1;

    int16_t nx1, ny1, nx2, ny2, nx3, ny3, nx4, ny4;

    _logicalToNative(lx1, ly1, nx1, ny1); // Top-left
    _logicalToNative(lx2, ly1, nx2, ny2); // Top-right
    _logicalToNative(lx1, ly2, nx3, ny3); // Bottom-left
    _logicalToNative(lx2, ly2, nx4, ny4); // Bottom-right

    // Find the bounding box in native coordinates
    // Ensure results are treated as potentially negative during min/max
    native_x = std::min({nx1, nx2, nx3, nx4});
    native_y = std::min({ny1, ny2, ny3, ny4});
    int16_t native_x_max = std::max({nx1, nx2, nx3, nx4});
    int16_t native_y_max = std::max({ny1, ny2, ny3, ny4});

    // Clamp coordinates to be within display bounds before calculating w/h
    if (native_x < 0) native_x = 0;
    if (native_y < 0) native_y = 0;
    if (native_x >= GDEY042T81_WIDTH) native_x = GDEY042T81_WIDTH - 1;
    if (native_y >= GDEY042T81_HEIGHT) native_y = GDEY042T81_HEIGHT - 1;

    if (native_x_max < native_x) native_x_max = native_x;
    if (native_y_max < native_y) native_y_max = native_y;
    if (native_x_max >= GDEY042T81_WIDTH) native_x_max = GDEY042T81_WIDTH - 1;
    if (native_y_max >= GDEY042T81_HEIGHT) native_y_max = GDEY042T81_HEIGHT - 1;


    native_w = native_x_max - native_x + 1;
    native_h = native_y_max - native_y + 1;

    ESP_LOGI(TAG, "Native window: x=%u, y=%u, w=%u, h=%u", native_x, native_y, native_w, native_h);

    // Final check on native dimensions
    if (native_x >= GDEY042T81_WIDTH || native_y >= GDEY042T81_HEIGHT || native_w == 0 || native_h == 0) {
         ESP_LOGE(TAG, "Native window calculation resulted in zero size or out of bounds start: x=%d, y=%d, w=%d, h=%d",
            native_x, native_y, native_w, native_h);
         return;
    }


    // Convert NATIVE pixel coordinates/dimensions to byte-aligned values for hardware
    uint16_t x_start = native_x / 8;
    uint16_t x_end = (native_x + native_w - 1) / 8;
    uint16_t y_start = native_y;
    uint16_t y_end = native_y + native_h - 1;

    IO.reset(10);
    _waitBusy("epd_wakeup reset");

    IO.cmd(0x3C); // BorderWavefrom
    IO.data(0x80);

    IO.cmd(0x11); // Data entry mode
    IO.data(0x01);

    IO.cmd(0x21); // Display update control
    IO.data(0x00);
    IO.data(0x00);

    // Set X range (using native byte coords)
    IO.cmd(0x44);
    IO.data(x_start);
    IO.data(x_end);

    // Set Y range (using native pixel coords)
    IO.cmd(0x45);
    IO.data(y_start % 256);
    IO.data(y_start / 256);
    IO.data(y_end % 256);
    IO.data(y_end / 256);

    // Set current position counter (using native byte/pixel coords)
    IO.cmd(0x4E);
    IO.data(x_start);

    IO.cmd(0x4F);
    IO.data(y_start % 256);
    IO.data(y_start / 256);

    _waitBusy("partial window setup"); // Wait before sending data? Maybe helpful.

    // Define constants for buffer handling
    const uint16_t bytes_per_row = GDEY042T81_WIDTH / 8;
    const uint16_t bufferMaxSpi = 3000; // Keep buffer size reasonable
    uint8_t xbuf[bufferMaxSpi];
    uint32_t bufindex = 0;

    // --- Write _buffer1 (Command 0x24 - Black/White) ---
    // Send data corresponding to the NATIVE window (x_start..x_end, y_start..y_end)
    // Data comes from the framebuffer _buffer1, which should contain the correctly
    // transformed (rotated and mirrored) image data.
    IO.cmd(0x24);
    bufindex = 0;
    // Iterate through the NATIVE window rows
    for (uint16_t current_y = y_start; current_y <= y_end; ++current_y) {
        // Iterate through the NATIVE window bytes within the row
        for (uint16_t current_x_byte = x_start; current_x_byte <= x_end; ++current_x_byte) {
            // Calculate index in the full framebuffer _buffer1
            // Use native current_y, current_x_byte directly
            uint32_t current_index = (uint32_t)current_y * bytes_per_row + current_x_byte;
            if (current_index < GDEY042T81_BUFFER_SIZE) {
                 xbuf[bufindex++] = _buffer1[current_index];
                 if (bufindex == bufferMaxSpi) {
                     IO.data(xbuf, bufindex);
                     bufindex = 0;
                 }
            } else {
                ESP_LOGE(TAG, "Buffer1 index out of bounds: %lu for native y=%u, x_byte=%u",
                         (unsigned long)current_index, current_y, current_x_byte);
                 // Avoid continuing if out of bounds
                 bufindex = 0; // Clear partial buffer before jumping
                 goto send_remaining_partial; // Skip rest of the window data
            }
        }
    }
send_remaining_partial: // Label to jump to after error or normal completion
    // Send any remaining data in xbuf after loops complete or early exit
    if (bufindex > 0) {
        IO.data(xbuf, bufindex);
    }

    // Update display - follow reference code exactly for partial update
    IO.cmd(0x22);
    IO.data(0xFF); // Use partial update mode (command from EPD_Part_Update)
    IO.cmd(0x20);
    _waitBusy("partial update");

    _sleep();
}

void Gdey042T81::_waitBusy(const char* message){
  if (debug_enabled) {
    ESP_LOGI(TAG, "_waitBusy for %s", message);
  }
  int64_t time_since_boot = esp_timer_get_time();

  while (1){
    if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 0) break;
    vTaskDelay(1);
    if (esp_timer_get_time()-time_since_boot>2000000)
    {
      if (debug_enabled) ESP_LOGI(TAG, "Busy Timeout");
      break;
    }
  }
}

// Public method
void Gdey042T81::deepsleep() {
  _sleep();
}

void Gdey042T81::_sleep() {
  // TODO: Confirm with vendor. Comments in code seems to imply that this should always be done, even with 
  // faster redrawing.
  // if (!fastmode) {
  IO.cmd(0x10);
  IO.data(0x01);// power off
  // }
  is_powered = false;
}

void Gdey042T81::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) return;

  // MIRROR Issue. Swap X axis (For sure there is a smarter solution than this one)
  // Correct the mirroring calculation to map 0 -> width-1, etc.
  x = width() - 1 - x;

  // Check rotation, move pixel around if necessary
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      x = GDEY042T81_WIDTH - x - 1;
      break;
    case 2:
      x = GDEY042T81_WIDTH - x - 1;
      y = GDEY042T81_HEIGHT - y - 1;
      break;
    case 3:
      swap(x, y);
      y = GDEY042T81_HEIGHT - y - 1;
      break;
  }
  uint16_t i = x / 8 + y * GDEY042T81_WIDTH / 8;
  uint8_t mask = 0x80 >> (x & 7);

  // Non-4G mode only supports black and white
  if (_refresh_mode != GDEY042T81_REFRESH_MODE_4G && (color == EPD_DARKGREY || color == EPD_LIGHTGREY)) {
    color = EPD_BLACK;
  }

  switch (color) {
      case EPD_DARKGREY: // Dark gray
          _buffer1[i] = _buffer1[i] | mask;
          _buffer2[i] = _buffer2[i] & (0xFF ^ mask);
          break;
      case EPD_LIGHTGREY: // Light gray
          _buffer1[i] = _buffer1[i] & (0xFF ^ mask);
          _buffer2[i] = _buffer2[i] | mask;
          break;
      case EPD_WHITE: // White
          _buffer1[i] = _buffer1[i] | mask;
          _buffer2[i] = _buffer2[i] | mask;
          break;
      default: // Black (case 0)
          _buffer1[i] = _buffer1[i] & (0xFF ^ mask);
          _buffer2[i] = _buffer2[i] & (0xFF ^ mask);
          break;
  }
}
