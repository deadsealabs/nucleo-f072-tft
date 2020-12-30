#include "tft.h"
#include "stm32f0xx_hal.h"
#include "string.h"
#include "functions.h"
#include "user_setting.h"
#include "stdlib.h"

void PIN_INPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void PIN_OUTPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#define RD_ACTIVE  HAL_GPIO_WritePin(RD_PORT, RD_PIN, GPIO_PIN_RESET)
#define RD_IDLE    HAL_GPIO_WritePin(RD_PORT, RD_PIN, GPIO_PIN_SET)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  HAL_GPIO_WritePin(WR_PORT, WR_PIN, GPIO_PIN_RESET)
#define WR_IDLE    HAL_GPIO_WritePin(WR_PORT, WR_PIN, GPIO_PIN_SET)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
#define CD_COMMAND HAL_GPIO_WritePin(CD_PORT, CD_PIN, GPIO_PIN_RESET)
#define CD_DATA    HAL_GPIO_WritePin(CD_PORT, CD_PIN, GPIO_PIN_SET)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)
#define CS_IDLE    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET)
#define RESET_IDLE    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)

#define WR_ACTIVE2  {WR_ACTIVE; WR_ACTIVE;}
#define WR_ACTIVE4  {WR_ACTIVE2; WR_ACTIVE2;}
#define WR_ACTIVE8  {WR_ACTIVE4; WR_ACTIVE4;}
#define RD_ACTIVE2  {RD_ACTIVE; RD_ACTIVE;}
#define RD_ACTIVE4  {RD_ACTIVE2; RD_ACTIVE2;}
#define RD_ACTIVE8  {RD_ACTIVE4; RD_ACTIVE4;}
#define RD_ACTIVE16 {RD_ACTIVE8; RD_ACTIVE8;}
#define WR_IDLE2  {WR_IDLE; WR_IDLE;}
#define WR_IDLE4  {WR_IDLE2; WR_IDLE2;}
#define RD_IDLE2  {RD_IDLE; RD_IDLE;}
#define RD_IDLE4  {RD_IDLE2; RD_IDLE2;}

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }         //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE   //PWLR=TRDL=150ns

#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; WR_IDLE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; } // read 250ns after RD_ACTIVE goes low
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }

uint16_t _width    = WIDTH;
uint16_t _height   = HEIGHT;

uint16_t width(void)
{ return _width; }

uint16_t height(void)
{ return _height; }

void pushColors16b(uint16_t * block, int16_t n, uint8_t first);
void pushColors8b(uint8_t * block, int16_t n, uint8_t first);
void pushColors4n(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend);
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h);

void setReadDir (void);
void setWriteDir (void);

static uint8_t done_reset;

static uint16_t color565_to_555(uint16_t color) {
    return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01));  //lose Green LSB, extend Blue LSB
}

static uint8_t color565_to_r(uint16_t color) {
    return ((color & 0xF800) >> 8);  // transform to rrrrrxxx
}

static uint8_t color565_to_g(uint16_t color) {
    return ((color & 0x07E0) >> 3);  // transform to ggggggxx
}
static uint8_t color565_to_b(uint16_t color) {
    return ((color & 0x001F) << 3);  // transform to bbbbbxxx
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }

uint16_t readPixel(int16_t x, int16_t y) { uint16_t color; readGRAM(x, y, &color, 1, 1); return color; }

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags);

static void write24(uint16_t color);

static void writecmddata(uint16_t cmd, uint16_t dat);

void WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);

static void init_table(const void *table, int16_t size);

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block);

void pushCommand(uint16_t cmd, uint8_t * block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void);

uint16_t readReg(uint16_t reg, int8_t index);

uint32_t readReg32(uint16_t reg);

uint32_t readReg40(uint16_t reg);

uint8_t cursor_y  =0, cursor_x    = 0;
uint8_t textsize  = 1;
uint16_t textcolor =0xffff,  textbgcolor = 0xFFFF;
uint8_t wrap      = true;
uint8_t _cp437    = false;
uint8_t rotation  = 0;

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

uint16_t  _lcd_xor, _lcd_capable;

uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;

void setReadDir (void)
{
	PIN_INPUT(D0_PORT, D0_PIN);
	PIN_INPUT(D1_PORT, D1_PIN);
	PIN_INPUT(D2_PORT, D2_PIN);
	PIN_INPUT(D3_PORT, D3_PIN);
	PIN_INPUT(D4_PORT, D4_PIN);
	PIN_INPUT(D5_PORT, D5_PIN);
	PIN_INPUT(D6_PORT, D6_PIN);
	PIN_INPUT(D7_PORT, D7_PIN);
}

void setWriteDir (void)
{
	PIN_OUTPUT(D0_PORT, D0_PIN);
	PIN_OUTPUT(D1_PORT, D1_PIN);
	PIN_OUTPUT(D2_PORT, D2_PIN);
	PIN_OUTPUT(D3_PORT, D3_PIN);
	PIN_OUTPUT(D4_PORT, D4_PIN);
	PIN_OUTPUT(D5_PORT, D5_PIN);
	PIN_OUTPUT(D6_PORT, D6_PIN);
	PIN_OUTPUT(D7_PORT, D7_PIN);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
	uint8_t isconst = flags & 1;
	uint8_t isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) {
        WriteCmd(cmd);
    }

    if (!isconst && !isbigend) {
        uint16_t *block16 = (uint16_t*)block;
        while (n-- > 0) {
            color = *block16++;
            write16(color);
        }
    } else

    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);

        write16(color);
    }
    CS_IDLE;
}

static void write24(uint16_t color)
{
    uint8_t r = color565_to_r(color);
    uint8_t g = color565_to_g(color);
    uint8_t b = color565_to_b(color);
    write8(r);
    write8(g);
    write8(b);
}

static void writecmddata(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        write8(u8);
    }
    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{

    uint8_t *p = (uint8_t *) table, dat[24];            //R61526 has GAMMA[22]

    while (size > 0)
    {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8)
        {
            delay(len);
            len = 0;
        }
        else
        {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}

void reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
	WriteCmdData(0xB0, 0x0000);
}

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
    READ_8(ret);
    READ_8(lo);
    return (ret << 8) | lo;
}

uint16_t readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    delay(1);    //1us should be adequate

    do { ret = read16bits(); }while (--index >= 0);
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}

void tft_init(uint16_t ID)
{
    int16_t *p16;               //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    _lcd_xor = 0;

    _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS; //Red 3.5", Blue 3.5"
	static const uint8_t ILI9486_regValues[]  = {
		0xC0, 2, 0x0d, 0x0d,        //Power Control 1 [0E 0E]
		0xC1, 2, 0x43, 0x00,        //Power Control 2 [43 00]
		0xC2, 1, 0x00,      //Power Control 3 [33]
		0xC5, 4, 0x00, 0x48, 0x00, 0x48,    //VCOM  Control 1 [00 40 00 40]
		0xB4, 1, 0x00,      //Inversion Control [00]
		0xB6, 3, 0x02, 0x02, 0x3B,  // Display Function Control [02 02 3B]
		// 3.2 TM  3.2 Inch Initial Code not bad
		0xE0, 15, 0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00,
		0xE1, 15, 0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00,
	};
	table8_ads = ILI9486_regValues, table_size = sizeof(ILI9486_regValues);
	p16 = (int16_t *) &_height;
	*p16 = 320;
	p16 = (int16_t *) &_width;
	*p16 = 480;

    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);
    if (table8_ads != NULL) {
        static const uint8_t reset_off[]  = {
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
            0x28, 0,            //Display Off
            0x3A, 1, 0x55,      //Pixel read=565, write=565.
        };
        static const uint8_t wake_on[]  = {
			0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 150,
            0x29, 0,            //Display On
        };
		init_table(&reset_off, sizeof(reset_off));
	    init_table(table8_ads, table_size);   //can change PIXFMT
		init_table(&wake_on, sizeof(wake_on));
    }
    setRotation(0);             //PORTRAIT
    invertDisplay(false);
}

uint16_t readID(void)
{
    uint16_t ret = readReg32(0xD3);
    return ret;
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    if (_lcd_ID == 0x1602) _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0) {
        if (!(_lcd_capable & MIPI_DCS_REV1)) {
            WriteCmdData(_MC, x + col);
            WriteCmdData(_MP, y + row);
        }
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        if (_lcd_capable & READ_NODUMMY) {
            ;
        } else if ((_lcd_capable & MIPI_DCS_REV1) || _lcd_ID == 0x1289) {
            READ_8(r);
        } else {
            READ_16(dummy);
        }
		if (_lcd_ID == 0x1511) READ_8(r);   //extra dummy for R61511
        while (n)
        {
            if (_lcd_capable & READ_24BITS)
            {
                READ_8(r);
                READ_8(g);
                READ_8(b);
                if (_lcd_capable & READ_BGR)
                    ret = color565(b, g, r);
                else
                    ret = color565(r, g, b);
            } else
            {
                READ_16(ret);
                if (_lcd_capable & READ_LOWHIGH)
                    ret = (ret >> 8) | (ret << 8);
                if (_lcd_capable & READ_BGR)
                    ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
            }

            *block++ = ret;
            n--;
            if (!(_lcd_capable & AUTO_READINC))
                break;
        }
        if (++col >= w) {
            col = 0;
            if (++row >= h)
                row = 0;
        }
        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void setRotation(uint8_t r)
{
   uint16_t GS, SS_v, ORG, REV = _lcd_rev;
   uint8_t val, d[3];
   rotation = r & 3;           // just perform the operation ourselves on the protected variables
   _width = (rotation & 1) ? HEIGHT : WIDTH;
   _height = (rotation & 1) ? WIDTH : HEIGHT;
   switch (rotation) {
   case 0:                    //PORTRAIT:
       val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
       break;
   case 1:                    //LANDSCAPE: 90 degrees
       val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
       break;
   case 2:                    //PORTRAIT_REV: 180 degrees
       val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
       break;
   case 3:                    //LANDSCAPE_REV: 270 degrees
       val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
       break;
   }
   if (_lcd_capable & INVERT_GS)
       val ^= 0x80;
   if (_lcd_capable & INVERT_SS)
       val ^= 0x40;
   if (_lcd_capable & INVERT_RGB)
       val ^= 0x08;
   if (_lcd_capable & MIPI_DCS_REV1) {
     common_MC:
       _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
     common_BGR:
       WriteCmdParamN(0x36, 1, &val);
       _lcd_madctl = val;
   }
   else {
	   _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
	   GS = (val & 0x80) ? (1 << 15) : 0;
	   WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
	 common_SS:
	   SS_v = (val & 0x40) ? (1 << 8) : 0;
	   WriteCmdData(0x01, SS_v);     // set Driver Output Control
	 common_ORG:
	   ORG = (val & 0x20) ? (1 << 3) : 0;
	   if (val & 0x08)
		   ORG |= 0x1000;  //BGR
	   _lcd_madctl = ORG | 0x0030;
	   WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
   }
   if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
       uint16_t x;
       x = _MC, _MC = _MP, _MP = x;
       x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
       x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
   }
   setAddrWindow(0, 0, width() - 1, height() - 1);
   vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void drawPixel(int16_t x, int16_t y, uint16_t color)
{
   // MCUFRIEND just plots at edge if you try to write outside of the box:
   if (x < 0 || y < 0 || x >= width() || y >= height())
       return;
   setAddrWindow(x, y, x, y);
   WriteCmdData(_MW, color);
}

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
   if (_lcd_capable & MIPI_DCS_REV1) {
       WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
       WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
   } else {
       WriteCmdData(_MC, x);
       WriteCmdData(_MP, y);
       if (!(x == x1 && y == y1)) {  //only need MC,MP for drawPixel
           if (_lcd_capable & XSA_XEA_16BIT) {
               if (rotation & 1)
                   y1 = y = (y1 << 8) | y;
               else
                   x1 = x = (x1 << 8) | x;
           }
           WriteCmdData(_SC, x);
           WriteCmdData(_SP, y);
           WriteCmdData(_EC, x1);
           WriteCmdData(_EP, y1);
       }
   }
}

void vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;

	if (_lcd_ID == 0x9327) bfa += 32;

	if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position

	if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;

    if (_lcd_capable & MIPI_DCS_REV1) {
        uint8_t d[6];           // for multi-byte parameters
        d[0] = top >> 8;        //TFA
        d[1] = top;
        d[2] = scrollines >> 8; //VSA
        d[3] = scrollines;
        d[4] = bfa >> 8;        //BFA
        d[5] = bfa;
        WriteCmdParamN(0x33, 6, d);
		d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(0x37, 2, d);
		if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
			WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
		}
		return;
    }

	// 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
	WriteCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
	WriteCmdData(0x6A, vsp);        //VL#
}

void pushColors16b(uint16_t * block, int16_t n, uint8_t first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}

void pushColors8b(uint8_t * block, int16_t n, uint8_t first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}

void pushColors4n(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void fillScreen(uint16_t color)
{
    fillRect(0, 0, _width, _height, color);
}

void invertDisplay(uint8_t i)
{
    uint8_t val;
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;

    if (_lcd_capable & MIPI_DCS_REV1)
    {
		WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
        return;
    }

    WriteCmdData(0x61, _lcd_rev);
}

void  drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	fillRect(x, y, 1, h, color);
}
void  drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	fillRect(x, y, w, 1, color);
}

void writePixel(int16_t x, int16_t y, uint16_t color)
{
    drawPixel(x, y, color);
}

void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            writePixel(y0, x0, color);
        } else {
            writePixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if(x0 == x1){
        if(y0 > y1) _swap_int16_t(y0, y1);
        drawFastVLine(x0, y0, y1 - y0 + 1, color);
    } else if(y0 == y1){
        if(x0 > x1) _swap_int16_t(x0, x1);
        drawFastHLine(x0, y0, x1 - x0 + 1, color);
    } else {
        writeLine(x0, y0, x1, y1, color);
    }
}

void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    writePixel(x0  , y0+r, color);
    writePixel(x0  , y0-r, color);
    writePixel(x0+r, y0  , color);
    writePixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        writePixel(x0 + x, y0 + y, color);
        writePixel(x0 - x, y0 + y, color);
        writePixel(x0 + x, y0 - y, color);
        writePixel(x0 - x, y0 - y, color);
        writePixel(x0 + y, y0 + x, color);
        writePixel(x0 - y, y0 + x, color);
        writePixel(x0 + y, y0 - x, color);
        writePixel(x0 - y, y0 - x, color);
    }
}

void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            writePixel(x0 + x, y0 + y, color);
            writePixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            writePixel(x0 + x, y0 - y, color);
            writePixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            writePixel(x0 - y, y0 + x, color);
            writePixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            writePixel(x0 - y, y0 - x, color);
            writePixel(x0 - x, y0 - y, color);
        }
    }
}

void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    drawFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}

void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) drawFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) drawFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) drawFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) drawFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y+h-1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x+w-1, y, h, color);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (h-- > 0) {
        end = w;

        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
        setAddrWindow(0, 0, width() - 1, height() - 1);
}


void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    drawFastHLine(x+r  , y    , w-2*r, color); // Top
    drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
    drawFastVLine(x    , y+r  , h-2*r, color); // Left
    drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    drawCircleHelper(x+r    , y+r    , r, 1, color);
    drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
    drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}


void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    fillRect(x+r, y, w-2*r, h, color);
    // draw four corners
    fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}


void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}


void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        drawFastHLine(a, y0, b-a+1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if(a > b) _swap_int16_t(a,b);
        drawFastHLine(a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if(a > b) _swap_int16_t(a,b);
        drawFastHLine(a, y, b-a+1, color);
    }
}


/********************************* TESTS  *********************************************/

void testFillScreen()
{
    fillScreen(BLACK);
    fillScreen(RED);
    fillScreen(GREEN);
    fillScreen(BLUE);
    fillScreen(BLACK);
}

void testLines(uint16_t color)
{
    int           x1, y1, x2, y2,
                  w = width(),
                  h = height();

    fillScreen(BLACK);

    x1 = y1 = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) drawLine(x1, y1, x2, y2, color);

    fillScreen(BLACK);

    x1    = w - 1;
    y1    = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) drawLine(x1, y1, x2, y2, color);

    fillScreen(BLACK);

    x1    = 0;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) drawLine(x1, y1, x2, y2, color);

    fillScreen(BLACK);

    x1    = w - 1;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) drawLine(x1, y1, x2, y2, color);

}

void testFastLines(uint16_t color1, uint16_t color2)
{
    int           x, y, w = width(), h = height();

    fillScreen(BLACK);
    for (y = 0; y < h; y += 5) drawFastHLine(0, y, w, color1);
    for (x = 0; x < w; x += 5) drawFastVLine(x, 0, h, color2);
}

void testRects(uint16_t color) {
    int           n, i, i2,
                  cx = width()  / 2,
                  cy = height() / 2;

    fillScreen(BLACK);
    n     = min(width(), height());
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        drawRect(cx - i2, cy - i2, i, i, color);
    }

}

void testFilledRects(uint16_t color1, uint16_t color2)
{
    int           n, i, i2,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    fillScreen(BLACK);
    n = min(width(), height());
    for (i = n; i > 0; i -= 6) {
        i2    = i / 2;

        fillRect(cx - i2, cy - i2, i, i, color1);

        drawRect(cx - i2, cy - i2, i, i, color2);
    }
}

void testFilledCircles(uint8_t radius, uint16_t color)
{
    int x, y, w = width(), h = height(), r2 = radius * 2;

    fillScreen(BLACK);
    for (x = radius; x < w; x += r2) {
        for (y = radius; y < h; y += r2) {
            fillCircle(x, y, radius, color);
        }
    }
}

void testCircles(uint8_t radius, uint16_t color)
{
    int           x, y, r2 = radius * 2,
                        w = width()  + radius,
                        h = height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            drawCircle(x, y, radius, color);
        }
    }
}

void testTriangles() {
    int           n, i, cx = width()  / 2 - 1,
                        cy = height() / 2 - 1;

    fillScreen(BLACK);
    n     = min(cx, cy);
    for (i = 0; i < n; i += 5) {
        drawTriangle(
            cx    , cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            color565(0, 0, i));
    }
}

void testFilledTriangles()
{
    int i;
    int cx = width()  / 2 - 1;
    int cy = height() / 2 - 1;

    fillScreen(BLACK);

    for (i = min(cx, cy); i > 10; i -= 5)
    {
        fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, color565(0, i, i));
        drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, color565(i, i, 0));
    }
}

void testRoundRects()
{
    int w, i, i2, red, step;
	int cx = width()  / 2 - 1;
	int cy = height() / 2 - 1;

    fillScreen(BLACK);
    w = min(width(), height());
    red = 0;
    step = (256 * 6) / w;

    for (i = 0; i < w; i += 6)
    {
        i2 = i / 2;
        red += step;
        drawRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(red, 0, 0));
    }
}

void testFilledRoundRects()
{
    int i, i2, green, step;
    int cx = width()  / 2 - 1;
	int cy = height() / 2 - 1;

    fillScreen(BLACK);
    green = 256;
    step = (256 * 6) / min(width(), height());

    for (i = min(width(), height()); i > 20; i -= 6)
    {
        i2 = i / 2;
        green -= step;
        fillRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(0, green, 0));
    }
}

void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	{ // Custom font

        // Character is assumed previously filtered by write() to eliminate
        // newlines, returns, non-printable characters, etc.  Calling
        // drawChar() directly with 'bad' characters of font may cause mayhem!

        c -= (uint8_t)pgm_read_byte(&gfxFont->first);
        GFXglyph *glyph  = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
        uint8_t  *bitmap = (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);

        uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
        uint8_t  w  = pgm_read_byte(&glyph->width),
                 h  = pgm_read_byte(&glyph->height);
        int8_t   xo = pgm_read_byte(&glyph->xOffset),
                 yo = pgm_read_byte(&glyph->yOffset);
        uint8_t  xx, yy, bits = 0, bit = 0;
        int16_t  xo16 = 0, yo16 = 0;

        if(size > 1)
        {
            xo16 = xo;
            yo16 = yo;
        }

        for(yy=0; yy<h; yy++)
        {
            for(xx=0; xx<w; xx++)
            {
                if(!(bit++ & 7))
                {
                    bits = pgm_read_byte(&bitmap[bo++]);
                }

                if(bits & 0x80)
                {
                    if(size == 1)
                    {
                        writePixel(x+xo+xx, y+yo+yy, color);
                    }
                    else
                    {
                        fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size, size, size, color);
                    }
                }
                bits <<= 1;
            }
        }
    } // End classic vs custom font
}

size_t write(uint8_t c)
{
	if(c == '\n')
	{
		cursor_x  = 0;
		cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
	}
	else if(c != '\r')
	{
		uint8_t first = pgm_read_byte(&gfxFont->first);

		if((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last)))
		{
			GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);
			uint8_t w = pgm_read_byte(&glyph->width);
			uint8_t h = pgm_read_byte(&glyph->height);

			if((w > 0) && (h > 0))
			{ // Is there an associated bitmap?
				int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);

				if(wrap && ((cursor_x + textsize * (xo + w)) > _width))
				{
					cursor_x  = 0;
					cursor_y += (int16_t)textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
				}

				drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
			}

			cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize;
		}
	}

    return 1;
}

void setFont(const GFXfont *f)
{
    if(f)
    {            // Font struct pointer passed in?
        if(!gfxFont)
        { // And no current font struct?
            // Switching from classic to new font behavior.
            // Move cursor pos down 6 pixels so it's on baseline.
            cursor_y += 6;
        }
    }
    else if(gfxFont)
    { 	// NULL passed.  Current font struct defined?
        // Switching from new to classic font behavior.
        // Move cursor pos up 6 pixels so it's at top-left of char.
        cursor_y -= 6;
    }

    gfxFont = (GFXfont *)f;
}

void charBounds(char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy)
{
    if(gfxFont)
    {
        if(c == '\n')
        {
            *x  = 0;    // Reset x to zero, advance y by one line
            *y += textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        }
        else if(c != '\r')
        { // Not a carriage return; is normal char
        	uint8_t first = pgm_read_byte(&gfxFont->first);
        	uint8_t last  = pgm_read_byte(&gfxFont->last);

        	if((c >= first) && (c <= last))
        	{ // Char present in this font?
        		GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);

        		uint8_t gw = pgm_read_byte(&glyph->width);
        		uint8_t gh = pgm_read_byte(&glyph->height);
        		uint8_t xa = pgm_read_byte(&glyph->xAdvance);

        		int8_t  xo = pgm_read_byte(&glyph->xOffset);
        		uint8_t yo = pgm_read_byte(&glyph->yOffset);

                if(wrap && ((*x+(((int16_t)xo+gw)*textsize)) > _width))
                {
                    *x  = 0; // Reset x to zero, advance y by one line
                    *y += textsize * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }

                int16_t ts = (int16_t) textsize;
                uint8_t x1 = *x + xo * ts;
                uint8_t y1 = *y + yo * ts;
                uint8_t x2 = x1 + gw * ts - 1;
                uint8_t y2 = y1 + gh * ts - 1;

                if(x1 < *minx) *minx = x1;
                if(y1 < *miny) *miny = y1;
                if(x2 > *maxx) *maxx = x2;
                if(y2 > *maxy) *maxy = y2;

                *x += xa * ts;
            }
        }
    } else { // Default font

        if(c == '\n') {                     // Newline?
            *x  = 0;                        // Reset x to zero,
            *y += textsize * 8;             // advance y one line
            // min/max x/y unchaged -- that waits for next 'normal' character
        } else if(c != '\r') {  // Normal char; ignore carriage returns
            if(wrap && ((*x + textsize * 6) > _width)) { // Off right?
                *x  = 0;                    // Reset x to zero,
                *y += textsize * 8;         // advance y one line
            }
            int x2 = *x + textsize * 6 - 1, // Lower-right pixel of char
                y2 = *y + textsize * 8 - 1;
            if(x2 > *maxx) *maxx = x2;      // Track max x, y
            if(y2 > *maxy) *maxy = y2;
            if(*x < *minx) *minx = *x;      // Track min x, y
            if(*y < *miny) *miny = *y;
            *x += textsize * 6;             // Advance x one char
        }
    }
}

void getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h)
{
    uint8_t c; // Current character

    *x1 = x;
    *y1 = y;
    *w  = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while((c = *str++))
    {
        charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }

    if(maxx >= minx)
    {
        *x1 = minx;
        *w  = maxx - minx + 1;
    }

    if(maxy >= miny)
    {
        *y1 = miny;
        *h  = maxy - miny + 1;
    }
}


void printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
	setFont(f);
	textcolor = txtcolor;
	textsize = (txtsize > 0) ? txtsize : 1;
	setCursor(0, row);
	while (*str) write (*str++);
}

void printstr (uint8_t *str)
{
	while (*str) write (*str++);
}

void setTextWrap(uint8_t w)
{
	wrap = w;
}

void setTextColor (uint16_t color)
{
	textcolor = color;
}

void setTextSize (uint8_t size)
{
	textsize = size;
}

void setCursor(int16_t x, int16_t y)
{
	cursor_x = x;
	cursor_y = y;
}

uint8_t getRotation (void)
{
	return rotation;
}

void scrollup (uint16_t speed)
{
     uint16_t maxscroll;
     if (getRotation() & 1)
	 {
    	 maxscroll = width();
	 }
     else
     {
    	 maxscroll = height();
     }

     for (uint16_t i = 1; i <= maxscroll; i++)
     {
    	 vertScroll(0, maxscroll, i);
         if (speed < 655)
		 {
        	 delay(speed*100);
		 }
         else
		 {
        	 HAL_Delay(speed);
		 }
     }

}

void scrolldown (uint16_t speed)
{
	uint16_t maxscroll;
	if (getRotation() & 1)
	{
		maxscroll = width();
	}
	else
	{
		maxscroll = height();
	}

	for (uint16_t i = 1; i <= maxscroll; i++)
	{
		vertScroll(0, maxscroll, 0 - (int16_t)i);
		if (speed < 655)
		{
			delay(speed*100);
		}
		else
		{
			HAL_Delay(speed);
		}
	}
}
