/*****************************************************************************
* | File      	:	EPD_1in54b.c
* | Author      :   MH-ET LIVE
* | Function    :   Electronic paper driver
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2018-12-5
* | Info        :
* 1.Remove:ImageBuff[EPD_HEIGHT * EPD_WIDTH / 8]
* 2.Change:EPD_Display(UBYTE *Image)
*   Need to pass parameters: pointer to cached data
* 3.Change:
*   EPD_RST -> EPD_RST_PIN
*   EPD_DC -> EPD_DC_PIN
*   EPD_CS -> EPD_CS_PIN
*   EPD_BUSY -> EPD_BUSY_PIN
*
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "EPD_1in54b.h"
#include "Debug.h"

static void EPD_SendCommand(UBYTE Reg);
static void EPD_SendData(UBYTE Reg);
static void EPD_WaitUntilIdle(void);

const unsigned char lut_vcom0[] = {
    0x0E, 0x14, 0x01, 0x0A, 0x06, 0x04, 0x0A, 0x0A,
    0x0F, 0x03, 0x03, 0x0C, 0x06, 0x0A, 0x00
};

const unsigned char lut_w[] = {
    0x0E, 0x14, 0x01, 0x0A, 0x46, 0x04, 0x8A, 0x4A,
    0x0F, 0x83, 0x43, 0x0C, 0x86, 0x0A, 0x04
};

const unsigned char lut_b[] = {
    0x0E, 0x14, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A,
    0x0F, 0x83, 0x43, 0x0C, 0x06, 0x4A, 0x04
};

const unsigned char lut_g1[] = {
    0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A,
    0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04
};

const unsigned char lut_g2[] = {
    0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A,
    0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04
};

const unsigned char lut_vcom1[] = {
    0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char lut_red0[] = {
    0x83, 0x5D, 0x01, 0x81, 0x48, 0x23, 0x77, 0x77,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char lut_red1[] = {
    0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/******************************************************************************
function :  Set partial RAM area
parameter:
******************************************************************************/
void EPD_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    EPD_SendCommand(0x44);
    EPD_SendData(x / 8);
    EPD_SendData((x + w - 1) / 8);
    EPD_SendCommand(0x45);
    EPD_SendData(y % 256);
    EPD_SendData(y / 256);
    EPD_SendData((y + h - 1) % 256);
    EPD_SendData((y + h - 1) / 256);
    EPD_SendCommand(0x4e);
    EPD_SendData(x / 8);
    EPD_SendCommand(0x4f);
    EPD_SendData(y % 256);
    EPD_SendData(y / 256);
}

/******************************************************************************
function :	Software reset (reference code is from GxEPD2 codes
parameter:
******************************************************************************/
void EPD_Reset(void)
{
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    
    DEV_Delay_ms(10);       // 10ms according to specs
    EPD_SendCommand(0x12);  // SWRESET
    DEV_Delay_ms(10);       // 10ms according to specs
    EPD_SendCommand(0x01);  // Driver output control
    EPD_SendData(0xF9);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendCommand(0x11);  // Data entry mode
    EPD_SendData(0x03);
    EPD_SendCommand(0x3C);  // BorderWaveform
    EPD_SendData(0x05);
    EPD_SendCommand(0x18);  // Read built-in temperature sensor
    EPD_SendData(0x80);
    EPD_SendCommand(0x21);  // Display update control
    EPD_SendData(0x00);
    EPD_SendData(0x80);
    EPD_setPartialRamArea(0, 0, EPD_WIDTH, EPD_HEIGHT);

    // Power ON procedure
    EPD_SendCommand(0x22);
    EPD_SendData(0xf8);
    EPD_SendCommand(0x20);

    EPD_WaitUntilIdle();
}

void EPD_Update_Full()
{
  EPD_SendCommand(0x22);
  EPD_SendData(0xf7);
  EPD_SendCommand(0x20);
  EPD_WaitUntilIdle();//"_Update_Full", full_refresh_time);
}

void EPD_Update_Part()
{
  EPD_SendCommand(0x22);
  EPD_SendData(0xf7);
  EPD_SendCommand(0x20);
  //EPD_WaitUntilIdle("_Update_Part", partial_refresh_time);
  EPD_WaitUntilIdle();//"_Update_Part", 15000);
}

void EPD_clearScreen(uint8_t black_value, uint8_t color_value)
{
  //_initial_write = false; // initial full screen buffer clean done
  //EPD_Reset();  //_Init_Part();
  EPD_setPartialRamArea(0, 0, EPD_WIDTH, EPD_HEIGHT);
  EPD_SendCommand(0x24);
  for (uint32_t i = 0; i < (uint32_t)EPD_WIDTH * (uint32_t)EPD_HEIGHT / 8; i++)
  {
    EPD_SendData(black_value);
  }
  EPD_SendCommand(0x26);
  for (uint32_t i = 0; i < (uint32_t)EPD_WIDTH * (uint32_t)EPD_HEIGHT / 8; i++)
  {
    EPD_SendData(~color_value);
  }
  EPD_Update_Part();
}

void EPD_clearScreenWhite(uint8_t value)
{
  EPD_clearScreen(value, 0xFF);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void EPD_SendCommand(UBYTE Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void EPD_SendData(UBYTE Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	Wait until the busy_pin goes LOW
parameter:
******************************************************************************/
static void EPD_WaitUntilIdle(void)
{
    Debug("e-Paper busy\r\n");
    //while(1) {      //LOW: busy, HIGH: idle
    while(1) {      //LOW: idle, HIGH: busy
        if(DEV_Digital_Read(EPD_BUSY_PIN) == 0)
            break;
    }
    DEV_Delay_ms(200);
    Debug("e-Paper busy release\r\n");
}

/******************************************************************************
function :	Set the look-up black and white tables
parameter:
******************************************************************************/
#if 0
static void EPD_SetLutBw(void)
{
    UWORD count;
    EPD_SendCommand(0x20);         //g vcom
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_vcom0[count]);
    }
    EPD_SendCommand(0x21);        //g ww --
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_w[count]);
    }
    EPD_SendCommand(0x22);         //g bw r
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_b[count]);
    }
    EPD_SendCommand(0x23);         //g wb w
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_g1[count]);
    }
    EPD_SendCommand(0x24);         //g bb b
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_g2[count]);
    }
}
#endif

/******************************************************************************
function :	Set the look-up red tables
parameter:
******************************************************************************/
#if 0
static void EPD_SetLutRed(void)
{
    UWORD count;
    EPD_SendCommand(0x25);
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_vcom1[count]);
    }
    EPD_SendCommand(0x26);
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_red0[count]);
    }
    EPD_SendCommand(0x27);
    for(count = 0; count < 15; count++) {
        EPD_SendData(lut_red1[count]);
    }
}
#endif

/******************************************************************************
function :	Initialize the e-Paper register
parameter:
******************************************************************************/
UBYTE EPD_Init(void)
{
    EPD_Reset();

#if 0
    EPD_SendCommand(POWER_SETTING);
    EPD_SendData(0x07);
    EPD_SendData(0x00);
    EPD_SendData(0x08);
    EPD_SendData(0x00);
    EPD_SendCommand(BOOSTER_SOFT_START);
    EPD_SendData(0x07);
    EPD_SendData(0x07);
    EPD_SendData(0x07);
    EPD_SendCommand(POWER_ON);

    EPD_WaitUntilIdle();

    EPD_SendCommand(PANEL_SETTING);
    EPD_SendData(0xcf);
    EPD_SendCommand(VCOM_AND_DATA_INTERVAL_SETTING);
    EPD_SendData(0xF0);
    EPD_SendCommand(PLL_CONTROL);
    EPD_SendData(0x39);
    EPD_SendCommand(TCON_RESOLUTION);  //set x and y
    EPD_SendData(0xC8);            //x
    EPD_SendData(0x00);            //y High eight
    EPD_SendData(0xC8);            //y Low eight
    EPD_SendCommand(VCM_DC_SETTING_REGISTER); //VCOM
    EPD_SendData(0x0E);

    EPD_SetLutBw();
    EPD_SetLutRed();
#endif

    return 0;
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void EPD_Clear(void)
{
    UWORD Width, Height;
    Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    Height = EPD_HEIGHT;

    //send black data
    EPD_SendCommand(DATA_START_TRANSMISSION_1);
    DEV_Delay_ms(2);
    for(UWORD i = 0; i < Height; i++) {
        for(UWORD i = 0; i < Width; i++) {
            EPD_SendData(0xFF);
            EPD_SendData(0xFF);
        }
    }
    DEV_Delay_ms(2);

    //send red data
    EPD_SendCommand(DATA_START_TRANSMISSION_2);
    DEV_Delay_ms(2);
    for(UWORD i = 0; i < Height; i++) {
        for(UWORD i = 0; i < Width; i++) {
            EPD_SendData(0xFF);
        }
    }
    DEV_Delay_ms(2);

    EPD_SendCommand(DISPLAY_REFRESH);
    EPD_WaitUntilIdle();
}

/******************************************************************************
function :	Sends the image buffer in RAM to e-Paper and displays
parameter:
******************************************************************************/
void EPD_Display(const UBYTE *blackimage, const UBYTE *redimage)
{
    UBYTE Temp = 0x00;
    UWORD Width, Height;
    Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    Height = EPD_HEIGHT;

    EPD_SendCommand(DATA_START_TRANSMISSION_1);
    for (UWORD j = 0; j < Height; j++) {
        for (UWORD i = 0; i < Width; i++) {
            Temp = 0x00;
            for (int bit = 0; bit < 4; bit++) {
                if ((blackimage[i + j * Width] & (0x80 >> bit)) != 0) {
                    Temp |= 0xC0 >> (bit * 2);
                }
            }
            EPD_SendData(Temp);
            Temp = 0x00;
            for (int bit = 4; bit < 8; bit++) {
                if ((blackimage[i + j * Width] & (0x80 >> bit)) != 0) {
                    Temp |= 0xC0 >> ((bit - 4) * 2);
                }
            }
            EPD_SendData(Temp);
        }
    }
    DEV_Delay_ms(2);

    EPD_SendCommand(DATA_START_TRANSMISSION_2);
    for (UWORD j = 0; j < Height; j++) {
        for (UWORD i = 0; i < Width; i++) {
            EPD_SendData(redimage[i + j * Width]);
        }
    }
    DEV_Delay_ms(2);
    
    //Display refresh
    EPD_SendCommand(DISPLAY_REFRESH);
    EPD_WaitUntilIdle();

}

/******************************************************************************
function :	Enter sleep mode
parameter:
******************************************************************************/
void EPD_Sleep(void)
{
    EPD_SendCommand(VCOM_AND_DATA_INTERVAL_SETTING);
    EPD_SendData(0x17);
    EPD_SendCommand(VCM_DC_SETTING_REGISTER);         //to solve Vcom drop
    EPD_SendData(0x00);
    EPD_SendCommand(POWER_SETTING);         //power setting
    EPD_SendData(0x02);        //gate switch to external
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_WaitUntilIdle();
    EPD_SendCommand(POWER_OFF);         //power off
}




#if 0
GxEPD2_213_Z98c::GxEPD2_213_Z98c(int16_t cs, int16_t dc, int16_t rst, int16_t busy) :
  GxEPD2_EPD(cs, dc, rst, busy, HIGH, 30000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate)
{
}

void GxEPD2_213_Z98c::clearScreen(uint8_t value)
{
  clearScreen(value, 0xFF);
}

void GxEPD2_213_Z98c::clearScreen(uint8_t black_value, uint8_t color_value)
{
  _initial_write = false; // initial full screen buffer clean done
  _Init_Part();
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);
  _writeCommand(0x24);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(black_value);
  }
  _writeCommand(0x26);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(~color_value);
  }
  _Update_Part();
}

void GxEPD2_213_Z98c::writeScreenBuffer(uint8_t value)
{
  writeScreenBuffer(value, 0xFF);
}

void GxEPD2_213_Z98c::writeScreenBuffer(uint8_t black_value, uint8_t color_value)
{
  _initial_write = false; // initial full screen buffer clean done
  _Init_Part();
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);
  _writeCommand(0x24);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(black_value);
  }
  _writeCommand(0x26);
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _writeData(~color_value);
  }
}

void GxEPD2_213_Z98c::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, NULL, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_213_Z98c::writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  int16_t wb = (w + 7) / 8; // width bytes, bitmaps are padded
  x -= x % 8; // byte boundary
  w = wb * 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(0x24);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data = 0xFF;
      if (black)
      {
        // use wb, h of bitmap for index!
        int16_t idx = mirror_y ? j + dx / 8 + ((h - 1 - (i + dy))) * wb : j + dx / 8 + (i + dy) * wb;
        if (pgm)
        {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
          data = pgm_read_byte(&black[idx]);
#else
          data = black[idx];
#endif
        }
        else
        {
          data = black[idx];
        }
        if (invert) data = ~data;
      }
      _writeData(data);
    }
  }
  _writeCommand(0x26);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data = 0xFF;
      if (color)
      {
        // use wb, h of bitmap for index!
        int16_t idx = mirror_y ? j + dx / 8 + ((h - 1 - (i + dy))) * wb : j + dx / 8 + (i + dy) * wb;
        if (pgm)
        {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
          data = pgm_read_byte(&color[idx]);
#else
          data = color[idx];
#endif
        }
        else
        {
          data = color[idx];
        }
        if (invert) data = ~data;
      }
      _writeData(~data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_213_Z98c::writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                     int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, NULL, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_213_Z98c::writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                     int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  if ((w_bitmap < 0) || (h_bitmap < 0) || (w < 0) || (h < 0)) return;
  if ((x_part < 0) || (x_part >= w_bitmap)) return;
  if ((y_part < 0) || (y_part >= h_bitmap)) return;
  int16_t wb_bitmap = (w_bitmap + 7) / 8; // width bytes, bitmaps are padded
  x_part -= x_part % 8; // byte boundary
  w = w_bitmap - x_part < w ? w_bitmap - x_part : w; // limit
  h = h_bitmap - y_part < h ? h_bitmap - y_part : h; // limit
  x -= x % 8; // byte boundary
  w = 8 * ((w + 7) / 8); // byte boundary, bitmaps are padded
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_using_partial_mode) _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(0x24);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb_bitmap, h_bitmap of bitmap for index!
      int16_t idx = mirror_y ? x_part / 8 + j + dx / 8 + ((h_bitmap - 1 - (y_part + i + dy))) * wb_bitmap : x_part / 8 + j + dx / 8 + (y_part + i + dy) * wb_bitmap;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&black[idx]);
#else
        data = black[idx];
#endif
      }
      else
      {
        data = black[idx];
      }
      if (invert) data = ~data;
      _writeData(data);
    }
  }
  _writeCommand(0x26);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data = 0xFF;
      if (color)
      {
        // use wb_bitmap, h_bitmap of bitmap for index!
        int16_t idx = mirror_y ? x_part / 8 + j + dx / 8 + ((h_bitmap - 1 - (y_part + i + dy))) * wb_bitmap : x_part / 8 + j + dx / 8 + (y_part + i + dy) * wb_bitmap;
        if (pgm)
        {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
          data = pgm_read_byte(&color[idx]);
#else
          data = color[idx];
#endif
        }
        else
        {
          data = color[idx];
        }
        if (invert) data = ~data;
      }
      _writeData(~data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_213_Z98c::writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1)
  {
    writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_213_Z98c::drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_213_Z98c::drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_213_Z98c::drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(black, color, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_213_Z98c::drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(black, color, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_213_Z98c::drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeNative(data1, data2, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_213_Z98c::refresh(bool partial_update_mode)
{
  if (partial_update_mode) refresh(0, 0, WIDTH, HEIGHT);
  else _Update_Full();
}

void GxEPD2_213_Z98c::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
  // intersection with screen
  int16_t w1 = x < 0 ? w + x : w; // reduce
  int16_t h1 = y < 0 ? h + y : h; // reduce
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  w1 = x1 + w1 < int16_t(WIDTH) ? w1 : int16_t(WIDTH) - x1; // limit
  h1 = y1 + h1 < int16_t(HEIGHT) ? h1 : int16_t(HEIGHT) - y1; // limit
  if ((w1 <= 0) || (h1 <= 0)) return;
  // make x1, w1 multiple of 8
  w1 += x1 % 8;
  if (w1 % 8 > 0) w1 += 8 - w1 % 8;
  x1 -= x1 % 8;
  _Init_Part();
  _setPartialRamArea(x1, y1, w1, h1);
  _Update_Part();
}

void GxEPD2_213_Z98c::powerOff()
{
  _PowerOff();
}

void GxEPD2_213_Z98c::hibernate()
{
  _PowerOff();
  if (_rst >= 0)
  {
    _writeCommand(0x10); // deep sleep mode
    _writeData(0x1);     // enter deep sleep
    _hibernating = true;
  }
}

void GxEPD2_213_Z98c::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  _writeCommand(0x44);
  _writeData(x / 8);
  _writeData((x + w - 1) / 8);
  _writeCommand(0x45);
  _writeData(y % 256);
  _writeData(y / 256);
  _writeData((y + h - 1) % 256);
  _writeData((y + h - 1) / 256);
  _writeCommand(0x4e);
  _writeData(x / 8);
  _writeCommand(0x4f);
  _writeData(y % 256);
  _writeData(y / 256);
}

void GxEPD2_213_Z98c::_PowerOn()
{
  if (!_power_is_on)
  {
    _writeCommand(0x22);
    _writeData(0xf8);
    _writeCommand(0x20);
    _waitWhileBusy("_PowerOn", power_on_time);
  }
  _power_is_on = true;
}

void GxEPD2_213_Z98c::_PowerOff()
{
  if (_power_is_on)
  {
    _writeCommand(0x22);
    _writeData(0x83);
    _writeCommand(0x20);
    _waitWhileBusy("_PowerOff", power_off_time);
  }
  _power_is_on = false;
}

void GxEPD2_213_Z98c::_InitDisplay()
{
  if (_hibernating) _reset();
  delay(10); // 10ms according to specs
  _writeCommand(0x12);  //SWRESET
  delay(10); // 10ms according to specs
  _writeCommand(0x01); //Driver output control
  _writeData(0xF9);
  _writeData(0x00);
  _writeData(0x00);
  _writeCommand(0x11); //data entry mode
  _writeData(0x03);
  _writeCommand(0x3C); //BorderWavefrom
  _writeData(0x05);
  _writeCommand(0x18); //Read built-in temperature sensor
  _writeData(0x80);
  _writeCommand(0x21); //  Display update control
  _writeData(0x00);
  _writeData(0x80);
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);
}

void GxEPD2_213_Z98c::_Init_Full()
{
  _InitDisplay();
  _PowerOn();
}

void GxEPD2_213_Z98c::_Init_Part()
{
  _InitDisplay();
  _PowerOn();
}

void GxEPD2_213_Z98c::_Update_Full()
{
  _writeCommand(0x22);
  _writeData(0xf7);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Full", full_refresh_time);
}

void GxEPD2_213_Z98c::_Update_Part()
{
  _writeCommand(0x22);
  _writeData(0xf7);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Part", partial_refresh_time);
}
#endif
