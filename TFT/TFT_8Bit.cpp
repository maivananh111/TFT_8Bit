/*
 * TFT_8Bit.cpp
 *
 *  Created on: Sep 10, 2021
 *      Author: A315-56
 */

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "TFT_8Bit.h"
#include "Setting.h"


void PIN_LOW (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	GPIOx -> BSRR |= ((1<<GPIO_Pin)<<16);
}
void PIN_HIGH (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	GPIOx -> BSRR |= (1<<GPIO_Pin);
}
void PIN_INPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void PIN_OUTPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//#define RD_ACTIVE  PIN_LOW(CMD_PORT, RD_PIN)
//#define RD_IDLE    PIN_HIGH(CMD_PORT, RD_PIN)
//#define RD_OUTPUT  PIN_OUTPUT(CMD_PORT, RD_PIN)
//#define WR_ACTIVE  PIN_LOW(CMD_PORT, WR_PIN)
//#define WR_IDLE    PIN_HIGH(CMD_PORT, WR_PIN)
//#define WR_OUTPUT  PIN_OUTPUT(CMD_PORT, WR_PIN)
//#define CD_COMMAND PIN_LOW(CMD_PORT, CD_PIN)
//#define CD_DATA    PIN_HIGH(CMD_PORT, CD_PIN)
//#define CD_OUTPUT  PIN_OUTPUT(CMD_PORT, CD_PIN)
//#define CS_ACTIVE  PIN_LOW(CMD_PORT, CS_PIN)
//#define CS_IDLE    PIN_HIGH(CMD_PORT, CS_PIN)
//#define CS_OUTPUT  PIN_OUTPUT(CMD_PORT, CS_PIN)
//#define RESET_ACTIVE  PIN_LOW(CMD_PORT, RESET_PIN)
//#define RESET_IDLE    PIN_HIGH(CMD_PORT, RESET_PIN)
//#define RESET_OUTPUT  PIN_OUTPUT(CMD_PORT, RESET_PIN)
#define RD_ACTIVE     PIN_LOW(CMD_PORT, RD_N)
#define RD_IDLE       PIN_HIGH(CMD_PORT, RD_N)
#define RD_OUTPUT     PIN_OUTPUT(CMD_PORT, RD_PIN)
#define WR_ACTIVE     PIN_LOW(CMD_PORT, WR_N)
#define WR_IDLE       PIN_HIGH(CMD_PORT, WR_N)
#define WR_OUTPUT     PIN_OUTPUT(CMD_PORT, WR_PIN)
#define CD_COMMAND    PIN_LOW(CMD_PORT, CD_N)
#define CD_DATA       PIN_HIGH(CMD_PORT, CD_N)
#define CD_OUTPUT     PIN_OUTPUT(CMD_PORT, CD_PIN)
#define CS_ACTIVE     PIN_LOW(CMD_PORT, CS_N)
#define CS_IDLE       PIN_HIGH(CMD_PORT, CS_N)
#define CS_OUTPUT     PIN_OUTPUT(CMD_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(CMD_PORT, RESET_N)
#define RESET_IDLE    PIN_HIGH(CMD_PORT, RESET_N)
#define RESET_OUTPUT  PIN_OUTPUT(CMD_PORT, RESET_PIN)



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

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#define write8(x)     { write_8(x);WR_STROBE;}
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; } // read 250ns after RD_ACTIVE goes low
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

void CTL_INIT()       {
//	RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT;
	CMD_PORT -> MODER |=   (1<<(CS_N*2))
						 |(1<<(CD_N*2))
						 |(1<<(WR_N*2))
						 |(1<<(RD_N*2))
						 |(1<<(RESET_N*2));
	CMD_PORT -> OTYPER &=~ (1<<CS_N)
			             |(1<<CD_N)
						 |(1<<WR_N)
						 |(1<<RD_N)
						 |(1<<RESET_N);
	CMD_PORT -> OSPEEDR |= (1<<(CS_N*2))|(1<<(CS_N*2 + 1))
			             |(1<<(CD_N*2))|(1<<(CD_N*2 + 1))
						 |(1<<(WR_N*2))|(1<<(WR_N*2 + 1))
						 |(1<<(RD_N*2))|(1<<(RD_N*2 + 1))
						 |(1<<(RESET_N*2))|(1<<(RESET_N*2 + 1));
	CMD_PORT -> PUPDR  &=~ (1<<(CS_N*2))|(1<<(CS_N*2 + 1))
						 |(1<<(CD_N*2))|(1<<(CD_N*2 + 1))
						 |(1<<(WR_N*2))|(1<<(WR_N*2 + 1))
						 |(1<<(RD_N*2))|(1<<(RD_N*2 + 1))
						 |(1<<(RESET_N*2))|(1<<(RESET_N*2 + 1));
}
#define WriteCmd(x)  { CD_COMMAND; write16(x); CD_DATA; }
#define WriteData(x) { write16(x); }

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F

uint16_t WIDTH, HEIGHT;
TIM_HandleTypeDef *Timer;

TFT_HandleTypeDef::TFT_HandleTypeDef(uint16_t x, uint16_t y, TIM_HandleTypeDef *TFT_Timer){
	WIDTH = x;
	HEIGHT = y;
	Timer = TFT_Timer;
}

static void delay (uint16_t time){
	__HAL_TIM_SET_COUNTER(Timer, 0);
	while ((__HAL_TIM_GET_COUNTER(Timer))<time);
}

uint16_t _width    = WIDTH;
uint16_t _height   = HEIGHT;

uint16_t TFT_HandleTypeDef::width(void){ return _width; }
uint16_t TFT_HandleTypeDef::height(void){ return _height; }

void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void setReadDir (void);
void setWriteDir (void);
static uint8_t done_reset, is8347, is555, is9797;
static uint16_t color565_to_555(uint16_t color) {return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01));}
static uint16_t color555_to_565(uint16_t color) {return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); }
static uint8_t color565_to_r(uint16_t color) {return ((color & 0xF800) >> 8); }
static uint8_t color565_to_g(uint16_t color) {return ((color & 0x07E0) >> 3); }
static uint8_t color565_to_b(uint16_t color) {return ((color & 0x001F) << 3);  }

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
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

uint8_t wrap      = true;
uint8_t _cp437    = false;
uint8_t rotation  = 0;
const G_Font *gfxFont;
uint8_t W, H, VAL;
uint16_t  _lcd_xor, _lcd_capable;
uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;

void setReadDir (void){
	GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = D0_PIN|D1_PIN|D2_PIN|D3_PIN|D4_PIN|D5_PIN|D6_PIN|D7_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(DATA_PORT, &GPIO_InitStruct);
}
void setWriteDir (void){
//	GPIO_InitTypeDef GPIO_InitStruct;
//	  GPIO_InitStruct.Pin = D0_PIN|D1_PIN|D2_PIN|D3_PIN|D4_PIN|D5_PIN|D6_PIN|D7_PIN;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	  HAL_GPIO_Init(DATA_PORT, &GPIO_InitStruct);
	DATA_PORT -> MODER |=       (1<<(D0_N*2))
							 |(1<<(D1_N*2))
							 |(1<<(D2_N*2))
							 |(1<<(D3_N*2))
							 |(1<<(D4_N*2))
							 |(1<<(D5_N*2))
							 |(1<<(D6_N*2))
							 |(1<<(D7_N*2));
	DATA_PORT -> OTYPER &=~     (1<<D0_N)
				             |(1<<D1_N)
							 |(1<<D2_N)
							 |(1<<D3_N)
							 |(1<<D4_N)
							 |(1<<D5_N)
							 |(1<<D6_N)
							 |(1<<D7_N);
	DATA_PORT -> OSPEEDR |=     (1<<(D0_N*2))|(1<<(D0_N*2 + 1))
				             |(1<<(D1_N*2))|(1<<(D1_N*2 + 1))
							 |(1<<(D2_N*2))|(1<<(D2_N*2 + 1))
							 |(1<<(D3_N*2))|(1<<(D3_N*2 + 1))
							 |(1<<(D4_N*2))|(1<<(D4_N*2 + 1))
							 |(1<<(D5_N*2))|(1<<(D5_N*2 + 1))
							 |(1<<(D6_N*2))|(1<<(D6_N*2 + 1))
							 |(1<<(D7_N*2))|(1<<(D7_N*2 + 1));
	DATA_PORT -> PUPDR  &=~     (1<<(D0_N*2))|(1<<(D0_N*2 + 1))
							 |(1<<(D1_N*2))|(1<<(D1_N*2 + 1))
							 |(1<<(D2_N*2))|(1<<(D2_N*2 + 1))
							 |(1<<(D3_N*2))|(1<<(D3_N*2 + 1))
							 |(1<<(D4_N*2))|(1<<(D4_N*2 + 1))
							 |(1<<(D5_N*2))|(1<<(D5_N*2 + 1))
							 |(1<<(D6_N*2))|(1<<(D6_N*2 + 1))
							 |(1<<(D7_N*2))|(1<<(D7_N*2 + 1));
}
static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags){
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
#if defined(SUPPORT_9488_555)
        if (is555) color = color565_to_555(color);
#endif
        if (is9797) write24(color); else
        write16(color);
    }
    CS_IDLE;
}
static void write24(uint16_t color){
    uint8_t r = color565_to_r(color);
    uint8_t g = color565_to_g(color);
    uint8_t b = color565_to_b(color);
    write8(r);
    write8(g);
    write8(b);
}
static void writecmddata(uint16_t cmd, uint16_t dat){
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}
static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block){
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        write8(u8);
        if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
    }
    CS_IDLE;
}
static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4){
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}
static void init_table(const void *table, int16_t size){
    uint8_t *p = (uint8_t *) table, dat[24];            //R61526 has GAMMA[22]
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8){
            delay(len);
            len = 0;
        }else{
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}
static void init_table16(const void *table, int16_t size){
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
			writecmddata(cmd, d);                      //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}
void reset(void){
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
	WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}
static uint16_t read16bits(void){
    uint16_t ret;
    uint8_t lo;
    READ_8(ret);
    READ_8(lo);
    return (ret << 8) | lo;
}
uint16_t readReg(uint16_t reg, int8_t index){
    uint16_t ret;
    uint8_t lo;
    if (!done_reset)
        reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    delay(1);    //1us should be adequate
    //    READ_16(ret);
    do { ret = read16bits(); }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}
uint32_t readReg32(uint16_t reg){
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}
uint32_t readReg40(uint16_t reg){
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}
uint16_t TFT_HandleTypeDef::ReadID(void){
	HAL_TIM_Base_Start(Timer);
    uint16_t ret, ret2;
    uint8_t msb;
    ret = readReg(0,0);           //forces a reset() if called before begin()
    if (ret == 0x5408)          //the SPFD5408 fails the 0xD3D3 test.
        return 0x5408;
    if (ret == 0x5420)          //the SPFD5420 fails the 0xD3D3 test.
        return 0x5420;
    if (ret == 0x8989)          //SSD1289 is always 8989
        return 0x1289;
    ret = readReg(0x67,0);        //HX8347-A
    if (ret == 0x4747)
        return 0x8347;
    ret = readReg32(0xA1);      //SSD1963: [01 57 61 01]
    if (ret == 0xf000){          //S6D05A1
        return 0x05a1;}
    if (ret == 0x6101)
        return 0x1963;
    if (ret == 0xFFFF)          //R61526: [xx FF FF FF]
        return 0x1526;          //subsequent begin() enables Command Access
	ret = readReg40(0xBF);
	if (ret == 0x8357)          //HX8357B: [xx 01 62 83 57 FF]
        return 0x8357;
	if (ret == 0x9481)          //ILI9481: [xx 02 04 94 81 FF]
        return 0x9481;
    if (ret == 0x1511)          //?R61511: [xx 02 04 15 11] not tested yet
        return 0x1511;
    if (ret == 0x1520)          //?R61520: [xx 01 22 15 20]
        return 0x1520;
    if (ret == 0x1526)          //?R61526: [xx 01 22 15 26]
        return 0x1526;
    if (ret == 0x1581)          //R61581:  [xx 01 22 15 81]
        return 0x1581;
    if (ret == 0x1400)          //?RM68140:[xx FF 68 14 00] not tested yet
        return 0x6814;
    ret = readReg32(0xD4);
    if (ret == 0x5310)          //NT35310: [xx 01 53 10]
        return 0x5310;
    ret = readReg32(0xD7);
    if (ret == 0x8031)          //weird unknown from BangGood [xx 20 80 31] PrinceCharles
        return 0x8031;
    ret = readReg40(0xEF);      //ILI9327: [xx 02 04 93 27 FF]
    if (ret == 0x9327)
        return 0x9327;
    ret = readReg32(0xFE) >> 8; //weird unknown from BangGood [04 20 53]
    if (ret == 0x2053)
        return 0x2053;
    uint32_t ret32 = readReg32(0x04);
    msb = ret32 >> 16;
    ret = ret32;
    if (msb == 0x00 && ret == 0x8000) { //HX8357-D [xx 00 80 00]
#if 1
        uint8_t cmds[] = {0xFF, 0x83, 0x57};
        pushCommand(0xB9, cmds, 3);
        msb = readReg(0xD0,0);
        if (msb == 0x99) return 0x0099; //HX8357-D from datasheet
        if (msb == 0x90)        //HX8357-C undocumented
#endif
            return 0x9090;      //BIG CHANGE: HX8357-D was 0x8357
    }
    if (ret == 0x1526)          //R61526 [xx 06 15 26] if I have written NVM
        return 0x1526;          //subsequent begin() enables Command Access
	if (ret == 0x89F0)          //ST7735S: [xx 7C 89 F0]
        return 0x7735;
	if (ret == 0x8552)          //ST7789V: [xx 85 85 52]
        return 0x7789;
    if (ret == 0xAC11)          //?unknown [xx 61 AC 11]
        return 0xAC11;
    ret32 = readReg32(0xD3);      //[xx 91 63 00]
    ret = ret32 >> 8;
    if (ret == 0x9163) return ret;
    ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    msb = ret >> 8;
    if (msb == 0x93 || msb == 0x94 || msb == 0x98 || msb == 0x77 || msb == 0x16)
        return ret;             //0x9488, 9486, 9340, 9341, 7796
    if (ret == 0x00D3 || ret == 0xD3D3)
        return ret;             //16-bit write-only bus
	return readReg(0,0);          //0154, 7783, 9320, 9325, 9335, B505, B509
}

void TFT_HandleTypeDef::ReadGRAM(int16_t x, int16_t y, uint16_t block[], int16_t w, int16_t h){
	uint8_t r, g, b;
	uint16_t color;
	setAddrWindow(x, y, x+w-1, y+h-1);
	CS_ACTIVE;
	WriteCmd(0x2E);
	setReadDir();
	READ_8(r);
    for(uint32_t i=0; i<(w*h);i++){
		READ_8(g);
		READ_8(b);
		color = (g<<8)|b;
		block[i] = color;
    }
    RD_IDLE;
	CS_IDLE;
	setWriteDir();
	setAddrWindow(0, 0, width() - 1, height() - 1);
}

void TFT_HandleTypeDef::setRotation(uint8_t r){
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
   if (_lcd_ID == 0x05A1) {  //
		GS = (val & 0x80) ? (1 << 1) : 0;   //MY
	   // SS = (val & 0x20) ? (1 << 2) : 0;   //MX
		uint8_t DA[19];
		DA[0]=0x3B;
		DA[1]=0x33;
		DA[2]=0x03;
		DA[3]=0x0C;
		DA[4]=0x08;
		DA[5]=0x08;
		DA[6]=0x08;
		DA[7]=0x00;
		DA[8]=0x08;
		DA[9]=0x08;
		DA[10]=0x00;
		DA[11]=GS|SS_v;
		DA[12]=0x00;
		DA[13]=0x00;
		DA[14]=0x33;
		DA[15]=0x0C;
		DA[16]=0x08;
		DA[17]=0x0C;
		DA[18]=0x08;
		WriteCmdParamN(0xF2, 19, DA);
		goto common_MC;
	}
   if (_lcd_capable & INVERT_GS)
       val ^= 0x80;
   if (_lcd_capable & INVERT_SS)
       val ^= 0x40;
   if (_lcd_capable & INVERT_RGB)
       val ^= 0x08;
   if (_lcd_capable & MIPI_DCS_REV1) {
       if (_lcd_ID == 0x6814) {  //.kbv my weird 0x9486 might be 68140
           GS = (val & 0x80) ? (1 << 6) : 0;   //MY
           SS_v = (val & 0x40) ? (1 << 5) : 0;   //MX
           val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
           d[0] = 0;
           d[1] = GS | SS_v | 0x02;      //MY, MX
           d[2] = 0x3B;
           WriteCmdParamN(0xB6, 3, d);
           goto common_MC;
       }
       else if (_lcd_ID == 0x1963 || _lcd_ID == 0x9481 || _lcd_ID == 0x1511) {
           if (val & 0x80)
               val |= 0x01;    //GS
           if ((val & 0x40))
               val |= 0x02;    //SS
           if (_lcd_ID == 0x1963) val &= ~0xC0;
           if (_lcd_ID == 0x9481) val &= ~0xD0;
           if (_lcd_ID == 0x1511) {
               val &= ~0x10;   //remove ML
               val |= 0xC0;    //force penguin 180 rotation
           }
//            val &= (_lcd_ID == 0x1963) ? ~0xC0 : ~0xD0; //MY=0, MX=0 with ML=0 for ILI9481
           goto common_MC;
      }
       else if (is8347) {
           _MC = 0x02, _MP = 0x06, _MW = 0x22, _SC = 0x02, _EC = 0x04, _SP = 0x06, _EP = 0x08;
           if (_lcd_ID == 0x0065) {             //HX8352-B
               val |= 0x01;    //GS=1
               if ((val & 0x10)) val ^= 0xD3;  //(ML) flip MY, MX, ML, SS, GS
               if (r & 1) _MC = 0x82, _MP = 0x80;
               else _MC = 0x80, _MP = 0x82;
           }
           if (_lcd_ID == 0x5252) {             //HX8352-A
               val |= 0x02;   //VERT_SCROLLON
               if ((val & 0x10)) val ^= 0xD4;  //(ML) flip MY, MX, SS. GS=1
           }
			goto common_BGR;
       }
     common_MC:
       _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
     common_BGR:
       WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
       _lcd_madctl = val;
//	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
   }
   // cope with 9320 variants
   else {
       switch (_lcd_ID) {
#if defined(SUPPORT_9225)
       case 0x9225:
           _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
           _MC = 0x20, _MP = 0x21, _MW = 0x22;
           GS = (val & 0x80) ? (1 << 9) : 0;
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           WriteCmdData(0x01, GS | SS_v | 0x001C);       // set Driver Output Control
           goto common_ORG;
#endif
#if defined(SUPPORT_0139) || defined(SUPPORT_0154)
#ifdef SUPPORT_0139
       case 0x0139:
           _SC = 0x46, _EC = 0x46, _SP = 0x48, _EP = 0x47;
           goto common_S6D;
#endif
#ifdef SUPPORT_0154
       case 0x0154:
           _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
           goto common_S6D;
#endif
         common_S6D:
           _MC = 0x20, _MP = 0x21, _MW = 0x22;
           GS = (val & 0x80) ? (1 << 9) : 0;
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           // S6D0139 requires NL = 0x27,  S6D0154 NL = 0x28
           WriteCmdData(0x01, GS | SS_v | ((_lcd_ID == 0x0139) ? 0x27 : 0x28));
           goto common_ORG;
#endif
       case 0x5420:
       case 0x7793:
       case 0x9326:
		case 0xB509:
           _MC = 0x200, _MP = 0x201, _MW = 0x202, _SC = 0x210, _EC = 0x211, _SP = 0x212, _EP = 0x213;
           GS = (val & 0x80) ? (1 << 15) : 0;
			uint16_t NL;
			NL = ((432 / 8) - 1) << 9;
           if (_lcd_ID == 0x9326 || _lcd_ID == 0x5420) NL >>= 1;
           WriteCmdData(0x400, GS | NL);
           goto common_SS;
       default:
           _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
           GS = (val & 0x80) ? (1 << 15) : 0;
           WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
         common_SS:
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           WriteCmdData(0x01, SS_v);     // set Driver Output Control
         common_ORG:
           ORG = (val & 0x20) ? (1 << 3) : 0;
#ifdef SUPPORT_8230
           if (_lcd_ID == 0x8230) {    // UC8230 has strange BGR and READ_BGR behaviour
               if (rotation == 1 || rotation == 2) {
                   val ^= 0x08;        // change BGR bit for LANDSCAPE and PORTRAIT_REV
               }
           }
#endif
           if (val & 0x08)
               ORG |= 0x1000;  //BGR
           _lcd_madctl = ORG | 0x0030;
           WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
           break;
#ifdef SUPPORT_1289
       case 0x1289:
           _MC = 0x4E, _MP = 0x4F, _MW = 0x22, _SC = 0x44, _EC = 0x44, _SP = 0x45, _EP = 0x46;
           if (rotation & 1)
               val ^= 0xD0;    // exchange Landscape modes
           GS = (val & 0x80) ? (1 << 14) : 0;    //called TB (top-bottom), CAD=0
           SS_v = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
           ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
           _lcd_drivOut = GS | SS_v | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
           if (val & 0x08)
               _lcd_drivOut |= 0x0800; //BGR
           WriteCmdData(0x01, _lcd_drivOut);   // set Driver Output Control
           if (is9797) WriteCmdData(0x11, ORG | 0x4C30); else  // DFM=2, DEN=1, WM=1, TY=0
           WriteCmdData(0x11, ORG | 0x6070);   // DFM=3, EN=0, TY=1
           break;
#endif
		}
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
void TFT_HandleTypeDef::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1){
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (rotation == 2) y += OFFSET_9327, y1 += OFFSET_9327;
	    if (rotation == 3) x += OFFSET_9327, x1 += OFFSET_9327;
   }
#endif
#if 1
   if (_lcd_ID == 0x1526 && (rotation & 1)) {
		int16_t dx = x1 - x, dy = y1 - y;
		if (dy == 0) { y1++; }
		else if (dx == 0) { x1 += dy; y1 -= dy; }
   }
#endif
   if (_lcd_capable & MIPI_DCS_REV1) {
       WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
       WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
       if (is8347 && _lcd_ID == 0x0065) {             //HX8352-B has separate _MC, _SC
           uint8_t d[2];
           d[0] = x >> 8; d[1] = x;
           WriteCmdParamN(_MC, 2, d);                 //allows !MV_AXIS to work
           d[0] = y >> 8; d[1] = y;
           WriteCmdParamN(_MP, 2, d);
       }
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

void TFT_HandleTypeDef::vertScroll(int16_t top, int16_t scrollines, int16_t offset){
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (rotation == 2 || rotation == 3) top += OFFSET_9327;
    }
#endif
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
        WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
		d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
		if (is8347) {
		    d[0] = (offset != 0) ? (_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
			WriteCmdParamN(_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
		} else if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
			WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
		}
		return;
    }
    switch (_lcd_ID) {
    case 0x7783:
        WriteCmdData(0x61, _lcd_rev);   //!NDL, !VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
#ifdef SUPPORT_0139
    case 0x0139:
        WriteCmdData(0x07, 0x0213 | (_lcd_rev << 2));  //VLE1=1, GON=1, REV=x, D=3
        WriteCmdData(0x41, vsp);  //VL# check vsp
        break;
#endif
#if defined(SUPPORT_0154) || defined(SUPPORT_9225)  //thanks tongbajiel
    case 0x9225:
	case 0x0154:
        WriteCmdData(0x31, sea);        //SEA
        WriteCmdData(0x32, top);        //SSA
        WriteCmdData(0x33, vsp - top);  //SST
        break;
#endif
#ifdef SUPPORT_1289
    case 0x1289:
        WriteCmdData(0x41, vsp);        //VL#
        break;
#endif
	case 0x5420:
    case 0x7793:
	case 0x9326:
	case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //VLE, REV
        WriteCmdData(0x404, vsp);       //VL#
        break;
    default:
        WriteCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
    }
}
void TFT_HandleTypeDef::invertDisplay(uint8_t i){
    uint8_t val;
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (is8347) {
            if (_lcd_ID == 0x8347 || _lcd_ID == 0x5252) // HX8347-A, HX5352-A
			    val = _lcd_rev ? 6 : 2;       //INVON id bit#2,  NORON=bit#1
            else val = _lcd_rev ? 8 : 10;     //HX8347-D, G, I: SCROLLON=bit3, INVON=bit1
            WriteCmdParamN(0x01, 1, &val);
        } else
            WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
        return;
    }
    switch (_lcd_ID) {
#ifdef SUPPORT_0139
    case 0x0139:
#endif
    case 0x9225:                                        //REV is in reg(0x07) like Samsung
    case 0x0154:
        WriteCmdData(0x07, 0x13 | (_lcd_rev << 2));     //.kbv kludge
        break;
#ifdef SUPPORT_1289
    case 0x1289:
        _lcd_drivOut &= ~(1 << 13);
        if (_lcd_rev)
            _lcd_drivOut |= (1 << 13);
        WriteCmdData(0x01, _lcd_drivOut);
        break;
#endif
	case 0x5420:
    case 0x7793:
    case 0x9326:
	case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //.kbv kludge VLE
        break;
    default:
        WriteCmdData(0x61, _lcd_rev);
        break;
    }
}
void TFT_HandleTypeDef::drawPixel(int16_t x, int16_t y, uint16_t color){
   if (x < 0 || y < 0 || x >= width() || y >= height())
       return;
#if defined(SUPPORT_9488_555)
   if (is555) color = color565_to_555(color);
#endif
   setAddrWindow(x, y, x, y);
   if (is9797) { CS_ACTIVE; WriteCmd(_MW); write24(color); CS_IDLE;} else
   WriteCmdData(_MW, color);
}
uint16_t TFT_HandleTypeDef::ReadPixel(int16_t x, int16_t y){
	uint16_t GRAM[1];
	ReadGRAM(x, y, GRAM, 1, 1);
	return GRAM[0];
}
#ifdef USE_GUI/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void TFT_HandleTypeDef::pushColors(uint16_t * block, int16_t n, uint8_t first){
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void TFT_HandleTypeDef::pushColors(uint8_t * block, int16_t n, uint8_t first){
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void TFT_HandleTypeDef::pushColors(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend){
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}
void TFT_HandleTypeDef::fillScreen(uint16_t color){
    fillRect(0, 0, _width, _height, color);
}

void  TFT_HandleTypeDef::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){
	fillRect(x, y, 1, h, color);
}
void  TFT_HandleTypeDef::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color){
	fillRect(x, y, w, 1, color);
}
void TFT_HandleTypeDef::writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
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
            drawPixel(y0, x0, color);
        } else {
            drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}
void TFT_HandleTypeDef::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
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
void TFT_HandleTypeDef::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color){
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    drawPixel(x0  , y0+r, color);
    drawPixel(x0  , y0-r, color);
    drawPixel(x0+r, y0  , color);
    drawPixel(x0-r, y0  , color);
    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        drawPixel(x0 + x, y0 + y, color);
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 - x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        drawPixel(x0 - y, y0 + x, color);
        drawPixel(x0 + y, y0 - x, color);
        drawPixel(x0 - y, y0 - x, color);
    }
}

void TFT_HandleTypeDef::drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color){
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
            drawPixel(x0 + x, y0 + y, color);
            drawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            drawPixel(x0 + x, y0 - y, color);
            drawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            drawPixel(x0 - y, y0 + x, color);
            drawPixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            drawPixel(x0 - y, y0 - x, color);
            drawPixel(x0 - x, y0 - y, color);
        }
    }
}
void TFT_HandleTypeDef::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color){
    drawFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}
void TFT_HandleTypeDef::fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color){
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;
    delta++;
    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
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
void TFT_HandleTypeDef::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y+h-1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x+w-1, y, h, color);
}

void TFT_HandleTypeDef::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    int16_t end;
#if defined(SUPPORT_9488_555)
    if (is555) color = color565_to_555(color);
#endif
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
    if(hi == lo){
  	  write8(hi);
  	  write8(lo);
  	  CD_DATA; // Cho chan RS = 1
  	  for(uint32_t y=0; y<(w*h); y++){
  	    WR_ACTIVE;  // WR 0
  	    WR_IDLE; // WR 1
  	    WR_ACTIVE;  // WR 0
  		WR_IDLE; // WR 1
  	  }
    }else{
    while (h-- > 0) {
        end = w;
#if defined(SUPPORT_1289)
        if (is9797) {
             uint8_t r = color565_to_r(color);
             uint8_t g = color565_to_g(color);
             uint8_t b = color565_to_b(color);
             do {
                 write8(r);
                 write8(g);
                 write8(b);
             } while (--end != 0);
        } else
#endif
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
     }
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
    setAddrWindow(0, 0, width() - 1, height() - 1);
}
void TFT_HandleTypeDef::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
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
void TFT_HandleTypeDef::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    fillRect(x+r, y, w-2*r, h, color);
    fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}
void TFT_HandleTypeDef::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}
void TFT_HandleTypeDef::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
    int16_t a, b, y, last;
    if (y0 > y1) { _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);}
    if (y1 > y2) { _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);}
    if (y0 > y1) { _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);}
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
void TFT_HandleTypeDef::scrollup (uint16_t speed){
     uint16_t maxscroll;
     if (getRotation() & 1) maxscroll = width();
     else maxscroll = height();
     for (uint16_t i = 1; i <= maxscroll; i++){
          vertScroll(0, maxscroll, i);
         if (speed < 655) delay(speed*100);
         else HAL_Delay(speed);
     }
}
void TFT_HandleTypeDef::scrolldown (uint16_t speed){
	uint16_t maxscroll;
	if (getRotation() & 1) maxscroll = width();
	     else maxscroll = height();
	for (uint16_t i = 1; i <= maxscroll; i++){
		vertScroll(0, maxscroll, 0 - (int16_t)i);
		if (speed < 655) delay(speed*100);
		else HAL_Delay(speed);
	}
}
#endif
#ifdef USE_PRINT
void TFT_HandleTypeDef::WriteByteBack(uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, uint8_t Char){
   for(uint8_t i=0; i<8; i++){
		if(Char&(0x01)) {
			write8(color>>8);
			write8(color);
		}
		 else {
			 write8(B_color>>8);
			 write8(B_color);
		 }
		 Char>>=1;
	 }
}
void TFT_HandleTypeDef::WriteByteNoBack(uint16_t x, uint16_t y, uint16_t color, uint8_t Char){
   for(uint8_t i=0; i<8; i++){
		if(Char&(0x01)) {
           drawPixel(x, y+i, color);
		}
		 Char>>=1;
	 }
}
void TFT_HandleTypeDef::WriteNByteBack(uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, uint8_t Char){
	setAddrWindow(x, y, x, y+8);
	CS_ACTIVE;
	WriteCmd(_MW);
   for(uint8_t i=0; i<8; i++){
		if(Char&(0x80)) {
			write8(color>>8);
			write8(color);
		}
		 else {
			write8(B_color>>8);
			write8(B_color);
		 }
		 Char<<=1;
	 }
   CS_IDLE;
   if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
	setAddrWindow(0, 0, width() - 1, height() - 1);
}
void TFT_HandleTypeDef::WriteNByteNoBack(uint16_t x, uint16_t y, uint16_t color, uint8_t Char){
   for(uint8_t i=7; i>=0; i--){
		if(Char&(0x80)) {
           drawPixel(x, y+i, color);
		}
		 Char = Char<<1;
	 }
}
void TFT_HandleTypeDef::WriteCharBack(uint8_t numWrite, uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, const uint8_t *font, uint16_t num){
	setAddrWindow(x, y, x, y+(numWrite*8));
	CS_ACTIVE;
	WriteCmd(_MW);
	for(uint8_t i = 0; i < numWrite; i++){
		WriteByteBack(x, y+i*8, color, B_color, font[num+i]);
	}
   CS_IDLE;
   if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
	setAddrWindow(0, 0, width() - 1, height() - 1);
}
void TFT_HandleTypeDef::WriteCharNoBack(uint8_t numWrite, uint16_t x, uint16_t y, uint16_t color, const uint8_t *font, uint16_t num){
	for(uint8_t i = 0; i < numWrite; i++){
		WriteByteNoBack(x, y+i*8, color, font[num+i]);
	}
}
void TFT_HandleTypeDef::drawSmallBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, bool back, uint16_t color, uint16_t B_color, uint16_t w, uint16_t h){
	uint8_t VAL = (h%2==0)? h/8 : (h/8)+1;
	for (uint16_t z = 0; z < W; z++){
	 if(back) WriteCharBack(VAL, x+z, y, color, B_color, bitmap, (z*VAL));
	 else     WriteCharNoBack(VAL, x+z, y, color, bitmap, (z*VAL));
	}
}
uint8_t TFT_HandleTypeDef::UTF8_GetAddr(char *utf8_char, char *char_offset){
	*char_offset=1;
	 if((*utf8_char)<UTF8_char_start2) //nếu đây là kí tự trong bản ascii
	  return (*utf8_char);
	 else {             //nếu đây là kí tự tiếng Việt có đấu
	  uint32_t utf8_value=0;
	  unsigned char temp = 0xF0 & (*utf8_char);
	  if(temp == 0xC0) {//loại utf-8 2 byte
	    *char_offset=2;
	    utf8_value= (*utf8_char) << 8;
	    utf8_value|=*(utf8_char+1);
	  }
	  else if(temp == 0xE0) {   //loại utf-8 3 byte
	    *char_offset = 3;
	    utf8_value= (*utf8_char) << 16;
	    utf8_value|=*(utf8_char+1) << 8;
	    utf8_value|=*(utf8_char+2);
	  }
	  else if(temp == 0xF0) {   //loại utf-8 4 byte
	    *char_offset = 4;
	    utf8_value= (*utf8_char) << 24;
	    utf8_value|=*(utf8_char+1) << 16;
	    utf8_value|=*(utf8_char+2) << 8;
	    utf8_value|=*(utf8_char+3);
	  }
	  for(uint8_t i = 0; i < UTF8_table_size; i++){
	      if(utf8_value == UTF8_Table[i]){
	            if(i<UTF8_char_end1)
	                return i+1;
	            else
	                return i + UTF8_char_start2 - UTF8_char_end1;
	        }
	  }
	  return '?';
	 }
}
void TFT_HandleTypeDef::draw_Viet_Char(uint16_t x, uint16_t y, const G_Font *font, bool back, uint16_t color, uint16_t B_color, uint8_t Num){
	uint16_t i = x, j = y;
	gfxFont = font;
	W = gfxFont -> w;
	H = gfxFont -> h;
	VAL = gfxFont -> byte;
	  uint8_t ascii = Num;
	  int char_start  = (ascii * VAL * W) + 1 + ascii;
		for (uint16_t z = 0; z < W; z++){
		 if(back) WriteCharBack(VAL, i+z, j, color, B_color, gfxFont -> bitmap, (char_start + z*VAL));
		 else     WriteCharNoBack(VAL, i+z, j, color, gfxFont -> bitmap, (char_start + z*VAL));
		}
 }
void TFT_HandleTypeDef::drawChar(uint16_t x, uint16_t y, const G_Font *font, bool back, uint16_t color, uint16_t B_color, char Char){
	uint16_t i = x, j = y;
	gfxFont = font;
	W = gfxFont -> w;
	H = gfxFont -> h;
	VAL = gfxFont -> byte;
  uint8_t ascii = Char - 32;
  int char_start  = (ascii * VAL * W) + 1 + ascii;
    for (uint16_t z = 0; z < W; z++){
     if(back) WriteCharBack(VAL, i+z, j, color, B_color, gfxFont -> bitmap, (char_start + z*VAL));
     else     WriteCharNoBack(VAL, i+z, j, color, gfxFont -> bitmap, (char_start + z*VAL));
    }
 }
void TFT_HandleTypeDef::print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, char *str) {
	uint16_t i = x;
	unsigned char type = *str;
	gfxFont = font;
	W = gfxFont -> w;
	H = gfxFont -> h;
  if(type >= 128) x = x - (W+(W/2));
  while(*str){
	  if(x > (width()-W/2)){
	  	  		  y += (H+5);
	  	  		  x = i;
	  	  }
	  drawChar(x, y, font, back, N_Color, B_Color, *str++);

      if (type >= 128) {x = x + W-(W/2);}
      else {x = x + W;};
	}
}
void TFT_HandleTypeDef::Viet_print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, char *str) {
	uint16_t i = x;
	gfxFont = font;
	W = gfxFont -> w;
	H = gfxFont -> h;
	char offset = 0;
	uint8_t UTF8_addr;
	  while(*str){
		  if(x > (width()-W/2)){y += (H+5); x = i; }
		  UTF8_addr = UTF8_GetAddr(str, &offset);
		  draw_Viet_Char(x, y, font, back, N_Color, B_Color, UTF8_addr);
		  str+=offset;
          x = x + W;
		}
}
void TFT_HandleTypeDef::print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, float Num) {
	char c[10];
	sprintf(c, "%5.3f", Num);
    print(x, y, font, back, N_Color,B_Color, c);
}
void TFT_HandleTypeDef::print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, long Num) {
	 char c[20];
     print(x, y, font, back, N_Color,B_Color, itoa(Num,c,10));
}

#endif
#ifdef DRAW_BMP
void TFT_HandleTypeDef::drawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, bool back, uint16_t color, uint16_t B_color, uint16_t w, uint16_t h) {
	 for (int16_t j=0; j<h; j++) {
	    for (int16_t i=0; i<w; i++ ) {
	      if (pgm_read_byte(bitmap + i + (j/8)*w) & (1<<(j%8))) {
	        drawPixel(x+i, y+j, color);
	      }
	      else if(back){
	    	drawPixel(x+i, y+j, B_color);
	      }
	    }
	  }
}
void TFT_HandleTypeDef::drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t* bitmap, uint16_t w, uint16_t h){
#if defined(SUPPORT_9488_555)
//    if (is555) color = color565_to_555(color);
#endif
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    for (uint16_t i=0; i<h; i++ ) {
      for (uint16_t j=0; j<w; j++) {
          write8(bitmap[j+i*w]>>8);
          write8(bitmap[j+i*w]);
      }
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
       setAddrWindow(0, 0, width() - 1, height() - 1);
}
#endif
#ifdef USE_EXTERNAL_FLASH
	void TFT_HandleTypeDef::drawRGBbitmapInFlash(W25Q w25q, uint32_t Sector_OffSet_Addr, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
		 uint32_t addr = Sector_OffSet_Addr;
		 uint16_t block_height = (4096/2)/w;  // Chieu cao 1 khoi pixel
		 uint16_t num_block = h/block_height;// So luong khoi pixel
		 uint8_t buf[block_height*w*2];
		 setAddrWindow(x, y, x + w - 1, y + h - 1);
		 CS_ACTIVE;
		 WriteCmd(_MW);
		  for(uint16_t j=0; j<num_block; j++){
					w25q.ReadSector(buf, addr, 0, block_height*w*2);
					addr = addr + 1;
					for(uint16_t i=0; i<block_height*w; i++) {
						  write8(buf[2*i+1]);
						  write8(buf[2*i]);
					}
		   }
		CS_IDLE;
		if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (rotation & 1)))
		setAddrWindow(0, 0, width() - 1, height() - 1);
	}
#endif
#ifdef USE_SD_CARD
	void TFT_HandleTypeDef::drawBitmapFile(char *s, uint16_t x,uint16_t y, uint16_t s_x,uint16_t s_y, uint16_t Width, uint16_t Height, uint16_t file_Width, uint16_t file_Height){
		 uint16_t i,h;
		 uint32_t index = 0, width = Width, height = Height, bitpixel = 0;
		 FRESULT res;
		 FIL MyFile;
		 UINT bytesread;
		 uint32_t tmpcolor;
		 uint16_t color[width];
		 uint8_t *bufbmp = NULL;
				if( f_open(&MyFile, s, FA_READ) != FR_OK){
					char x[40];
					strcpy(x, "ERROR Open file ");
					strcat(x, s);
					print(10, 10, &Font_Terminal8x12, true, RED, BLACK, x);
				}
				else{
					bufbmp = (uint8_t *)malloc(100);
					res = f_read(&MyFile, bufbmp, 90, &bytesread);
						index=bufbmp[10];
						index|=bufbmp[11]<<8;
						index|=bufbmp[12]<<16;
						index|=bufbmp[13]<<24;
	//					width=bufbmp[18];
	//					width|=bufbmp[19]<<8;
	//					width|=bufbmp[20]<<16;
	//					width|=bufbmp[21]<<24;
	//					height=bufbmp[22];
	//					height|=bufbmp[23]<<8;
	//					height|=bufbmp[24]<<16;
	//					height|=bufbmp[25]<<24;
						bitpixel=bufbmp[28];
						bitpixel|=bufbmp[29]<<8;
						for(h = s_y; h < height+s_y; h++){
							for(i = s_x; i < width+s_x; i++){
							  f_lseek(&MyFile, index + ((file_Height-h-1) * file_Width*3) + (i*3));
							  res = f_read(&MyFile, bufbmp, 3, &bytesread);
								tmpcolor=bufbmp[0];
								tmpcolor|=bufbmp[1]<<8;
								tmpcolor|=bufbmp[2]<<16;
								color[i-s_x] = (uint16_t)(((tmpcolor & 0x00F80000)>>8)|((tmpcolor & 0x0000FC00)>>5)|((tmpcolor & 0x000000F8)>>3));
							}
						  drawRGBBitmap(x, y+h-s_y, color, width, 1);
						}
						f_close(&MyFile);
					free(bufbmp);
				}
	}
  #ifdef USE_EXTERNAL_FLASH
	void TFT_HandleTypeDef::CopyBmpFileToFlash(W25Q w25q, char *s, uint32_t Sector_OffSet_Addr, uint16_t s_x,uint16_t s_y, uint16_t Width, uint16_t Height, uint16_t file_Width, uint16_t file_Height){
		/*1 Sector co 4096 byte => 1 Sector luu tru 2048 pixel*/
		 uint16_t i,h;
		 uint32_t index = 0, width = Width, height = Height, bitpixel = 0;
		 FRESULT res;
		 FIL MyFile;
		 UINT bytesread;
		 uint32_t tmpcolor, addr = Sector_OffSet_Addr;
		 uint16_t color;
		 uint16_t counter = 0, step = 0;
		 uint8_t *bufbmp = NULL;
	     uint16_t block_height = (4096/2)/width;
		 uint8_t flash_Byte[block_height*width*2+1];
				if(f_open(&MyFile, s, FA_READ) != FR_OK){
				} else{
					bufbmp = (uint8_t *)malloc(100);
					res = f_read(&MyFile, bufbmp, 90, &bytesread);
						index=bufbmp[10];
						index|=bufbmp[11]<<8;
						index|=bufbmp[12]<<16;
						index|=bufbmp[13]<<24;
						bitpixel=bufbmp[28];
						bitpixel|=bufbmp[29]<<8;
						for(h = s_y; h < height+s_y; h++){
							for(i = s_x; i < width+s_x; i++){
							  f_lseek(&MyFile, index + ((file_Height-h-1) * file_Width*3) + (i*3));
							  res = f_read(&MyFile, bufbmp, 3, &bytesread);
								tmpcolor=bufbmp[0];
								tmpcolor|=bufbmp[1]<<8;
								tmpcolor|=bufbmp[2]<<16;
								color = (uint16_t)(((tmpcolor & 0x00F80000)>>8)|((tmpcolor & 0x0000FC00)>>5)|((tmpcolor & 0x000000F8)>>3));
								flash_Byte[2*counter] = color & 0xFF;
								flash_Byte[2*counter+1] = (color>>8) & 0xFF;
								counter++;
								if(counter == (block_height*width)){
								    w25q.EraseSector(addr);
									w25q.WriteSector(flash_Byte, addr, 0, block_height*width*2+1);
									addr = addr + 1;
									counter = 0;
									step = step + 1;
								}
							}
						}
					  f_close(&MyFile);
					free(bufbmp);
				}
	}
  #endif
#endif
void TFT_HandleTypeDef::DataWrite(uint16_t data){
    write8(data>>8);
    write8(data);
}
void TFT_HandleTypeDef::DataWriteS(uint16_t *data, uint32_t size){
	for(uint32_t i=0; i<size; i++){
		write8(data[i]>>8);
		write8(data[i]);
	}
}
void TFT_HandleTypeDef::WriteEnable(void){
    WriteCmd(_MW);
}
void TFT_HandleTypeDef::Enable(void){
	CS_ACTIVE;
}
void TFT_HandleTypeDef::Disable(void){
	CS_IDLE;
}

uint8_t TFT_HandleTypeDef::getRotation (void){return rotation;}
void TFT_HandleTypeDef::Init(uint16_t ID){
	int dummy=0;
    int16_t *p16;               //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    _lcd_xor = 0;
    switch (_lcd_ID = ID) {
#ifdef SUPPORT_0139
    case 0x0139:
        _lcd_capable = REV_SCREEN | XSA_XEA_16BIT;    //remove AUTO_READINC
        static const uint16_t S6D0139_regValues[]  = {
            0x0000, 0x0001,     //Start oscillator
            0x0011, 0x1a00,     //Power Control 2
            0x0014, 0x2020,     //Power Control 4
            0x0010, 0x0900,     //Power Control 1
            0x0013, 0x0040,     //Power Control 3
            0x0013, 0x0060,     //Power Control 3
            0x0013, 0x0070,     //Power Control 3
            0x0011, 0x1a04,     //Power Control 2
            0x0010, 0x2f00,     //Power Control 1
            0x0001, 0x0127,     //Driver Control: SM=0, GS=0, SS=1, 240x320
            0x0002, 0x0100,     //LCD Control:  (.kbv was 0700) FLD=0, BC= 0, EOR=1
            0x0003, 0x1030,     //Entry Mode:    TR1=0, DFM=0, BGR=1, I_D=3
            0x0007, 0x0000,     //Display Control: everything off
            0x0008, 0x0303,     //Blank Period:  FP=3, BP=3
            0x0009, 0x0000,     //f.k.
            0x000b, 0x0000,     //Frame Control:
            0x000c, 0x0000,     //Interface Control: system i/f
            0x0040, 0x0000,     //Scan Line
            0x0041, 0x0000,     //Vertical Scroll Control
            0x0007, 0x0014,     //Display Control: VLE1=0, SPT=0, GON=1, REV=1, D=0 (halt)
            0x0007, 0x0016,     //Display Control: VLE1=0, SPT=0, GON=1, REV=1, D=2 (blank)
            0x0007, 0x0017,     //Display Control: VLE1=0, SPT=0, GON=1, REV=1, D=3 (normal)
//            0x0007, 0x0217,     //Display Control: VLE1=1, SPT=0, GON=1, REV=1, D=3
        };
        init_table16(S6D0139_regValues, sizeof(S6D0139_regValues));
        break;
#endif

#ifdef SUPPORT_0154
    case 0x0154:
        _lcd_capable = AUTO_READINC | REV_SCREEN;
        static const uint16_t S6D0154_regValues[]  = {
            0x0011, 0x001A,
            0x0012, 0x3121,     //BT=3, DC1=1, DC2=2, DC3=1
            0x0013, 0x006C,     //GVD=108
            0x0014, 0x4249,     //VCM=66, VML=73

            0x0010, 0x0800,     //SAP=8
            TFTLCD_DELAY, 10,
            0x0011, 0x011A,     //APON=0, PON=1, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x031A,     //APON=0, PON=3, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x071A,     //APON=0, PON=7, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x0F1A,     //APON=0, PON=15, AON=0, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 10,
            0x0011, 0x0F3A,     //APON=0, PON=15, AON=1, VCI1_EN=1, VC=10
            TFTLCD_DELAY, 30,

            0x0001, 0x0128,
            0x0002, 0x0100,
            0x0003, 0x1030,
            0x0007, 0x1012,
            0x0008, 0x0303,
            0x000B, 0x1100,
            0x000C, 0x0000,
            0x000F, 0x1801,
            0x0015, 0x0020,

               0x0050,0x0101,
               0x0051,0x0603,
               0x0052,0x0408,
               0x0053,0x0000,
               0x0054,0x0605,
               0x0055,0x0406,
               0x0056,0x0303,
               0x0057,0x0303,
               0x0058,0x0010,
               0x0059,0x1000,

            0x0007, 0x0012,     //GON=1, REV=0, D=2
            TFTLCD_DELAY, 40,
            0x0007, 0x0013,     //GON=1, REV=0, D=3
            0x0007, 0x0017,     //GON=1, REV=1, D=3 DISPLAY ON
        };
        init_table16(S6D0154_regValues, sizeof(S6D0154_regValues));

        break;
#endif

#ifdef SUPPORT_05A1
    	case 0x05A1:
            _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
            static const uint8_t S6D05A1_regValues_max[]  = {        // Atmel MaxTouch
                0xF0, 2, 0x5A,0x5A,
				0xF1, 2, 0x5A,	0x5A,
				0xF2,19,0x3B,0x33,0x03,0x0C,0x08,0x08,0x08,0x00,0x08,0x08,0x00,0x00,0x00,0x00,0x33, 0x0C,0x08, 0x0C,0x08,
				0xF4,14,	0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	0x04,0x70,0x03,0x04,0x70,0x03,
				0xF5,12, 0x00,	0x46,0x70,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x46,0x70,
				0xF6,8,0x03,0x00,0x08,0x03,0x03,0x00,0x03,0x00,
				0xF7,5,0x00,0x80,0x10,0x02,0x00,
				0xF8,2,0x11,0x00,
				0xF9, 1,0x14,
				0xFA, 16,0x33,0x07,0x04,0x1A,0x18,0x1C,0x24,0x1D,0x26,0x28,0x2F,0x2E,0x00,0x00,0x00,0x00,
				0xFB, 16,0x33,0x03,0x00,0x2E,0x2F,0x28,0x26,0x1D,0x24,0x1C,0x18,0x1A,0x04,0x00,0x00,0x00,
				0xF9, 1,0x12,
				0xFA, 16,0x36,0x07,0x04,0x1C,0x1C,0x23,0x28,0x1C,0x25,0x26,0x2E,0x2B,0x00,0x00,0x00,0x00,
				0xFB, 16,0x33,0x06,0x00,0x2B,0x2E,0x26,0x25,0x1C,0x28,0x23,0x1C,0x1C,0x04,0x00,0x00,0x00,
				0xF9, 1,0x11,
				0xFA, 16,0x33,0x07,0x04,0x30,0x32,0x34,0x35,0x11,0x1D,0x20,0x28,0x20,0x00,0x00,0x00,0x00,
				0xFB, 16,0x33,0x03,0x00,0x20,0x28,0x20,0x1D,0x11,0x35,0x34,0x32,0x30,0x04,0x00,0x00,0x00,
				0x44,2,0x00,0x01,
				0x2A,4,0x00,0x00,0x01,0x3F,
				0x2B,4,0x00,0x00,0x01,0xDF,
				0x36, 1,0x08,
				0x3A, 1,0x55,
				0x11,0,
				TFTLCD_DELAY8, 120,
				0x29,0,
            };
            init_table(S6D05A1_regValues_max, sizeof(S6D05A1_regValues_max));
            p16 = (int16_t *) & _height;
            *p16 = 480;
            p16 = (int16_t *) & _width;
            *p16 = 320;
            break;
    #endif

#ifdef SUPPORT_1289
    case 0x9797:
        is9797 = 1;
//        _lcd_capable = 0 | XSA_XEA_16BIT | REV_SCREEN | AUTO_READINC | READ_24BITS;
// deliberately set READ_BGR to disable Software Scroll in graphictest_kbv example
        _lcd_capable = 0 | XSA_XEA_16BIT | REV_SCREEN | AUTO_READINC | READ_24BITS | READ_BGR;
        _lcd_ID = 0x1289;
        goto common_1289;
    case 0x1289:
        _lcd_capable = 0 | XSA_XEA_16BIT | REV_SCREEN | AUTO_READINC;
      common_1289:
	    dummy=1;
        // came from MikroElektronika library http://www.hmsprojects.com/tft_lcd.html
        static const uint16_t SSD1289_regValues[]  = {
            0x0000, 0x0001,
            0x0003, 0xA8A4,
            0x000C, 0x0000,
            0x000D, 0x000A,     // VRH=10
            0x000E, 0x2B00,
            0x001E, 0x00B7,
            0x0001, 0x2B3F,     // setRotation() alters
            0x0002, 0x0600,     // B_C=1, EOR=1
            0x0010, 0x0000,
            0x0011, 0x6070,     // setRotation() alters
            0x0005, 0x0000,
            0x0006, 0x0000,
            0x0016, 0xEF1C,
            0x0017, 0x0003,
            0x0007, 0x0233,
            0x000B, 0x0000,
            0x000F, 0x0000,
            0x0030, 0x0707,
            0x0031, 0x0204,
            0x0032, 0x0204,
            0x0033, 0x0502,
            0x0034, 0x0507,
            0x0035, 0x0204,
            0x0036, 0x0204,
            0x0037, 0x0502,
            0x003A, 0x0302,
            0x003B, 0x0302,
            0x0023, 0x0000,
            0x0024, 0x0000,
            0x0025, 0x8000,
        };
        init_table16(SSD1289_regValues, sizeof(SSD1289_regValues));
        break;
#endif

    case 0x1511:                // Unknown from Levy
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1;   //extra read_8(dummy)
        static const uint8_t R61511_regValues[]  = {
			0xB0, 1, 0x00,       //Command Access Protect
        };
        table8_ads = R61511_regValues, table_size = sizeof(R61511_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;

    case 0x1520:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t R61520_regValues[]  = {
            0xB0, 1, 0x00,      //Command Access Protect
            0xC0, 1, 0x0A,      //DM=1, BGR=1
        };
        table8_ads = R61520_regValues, table_size = sizeof(R61520_regValues);
        break;

	case 0x1526:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t R61526_regValues[]  = {
            0xB0, 1, 0x03,      //Command Access
            0xE2, 1, 0x3F,      //Command Write Access
            0xC0, 1, 0x22,      //REV=0, BGR=1, SS=0
            0xE2, 1, 0x00,      //Command Write Protect
			0x3A, 1, 0x55,
        };
        table8_ads = R61526_regValues, table_size = sizeof(R61526_regValues);
        break;

#ifdef SUPPORT_1580
    case 0x1580:
        _lcd_capable = 0 | REV_SCREEN | READ_BGR | INVERT_GS | READ_NODUMMY; //thanks vanhan123
        static const uint16_t R61580_regValues[]  = {  //from MCHIP Graphics Lib drvTFT001.c
            // Synchronization after reset
            TFTLCD_DELAY, 2,
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,

            // Setup display
            0x00A4, 0x0001,          // CALB=1
            TFTLCD_DELAY, 2,
            0x0060, 0xA700,          // Driver Output Control
            0x0008, 0x0808,          // Display Control BP=8, FP=8
            0x0030, 0x0111,          // y control
            0x0031, 0x2410,          // y control
            0x0032, 0x0501,          // y control
            0x0033, 0x050C,          // y control
            0x0034, 0x2211,          // y control
            0x0035, 0x0C05,          // y control
            0x0036, 0x2105,          // y control
            0x0037, 0x1004,          // y control
            0x0038, 0x1101,          // y control
            0x0039, 0x1122,          // y control
            0x0090, 0x0019,          // 80Hz
            0x0010, 0x0530,          // Power Control
            0x0011, 0x0237,          //DC1=2, DC0=3, VC=7
//            0x0011, 0x17B0,          //DC1=7, DC0=3, VC=0 ?b12 ?b7 vanhan123
            0x0012, 0x01BF,          //VCMR=1, PSON=1, PON=1, VRH=15
//            0x0012, 0x013A,          //VCMR=1, PSON=1, PON=1, VRH=10 vanhan123
            0x0013, 0x1300,          //VDV=19
            TFTLCD_DELAY, 100,

            0x0001, 0x0100,
            0x0002, 0x0200,
            0x0003, 0x1030,
            0x0009, 0x0001,
            0x000A, 0x0008,
            0x000C, 0x0001,
            0x000D, 0xD000,
            0x000E, 0x0030,
            0x000F, 0x0000,
            0x0020, 0x0000,
            0x0021, 0x0000,
            0x0029, 0x0077,
            0x0050, 0x0000,
            0x0051, 0xD0EF,
            0x0052, 0x0000,
            0x0053, 0x013F,
            0x0061, 0x0001,
            0x006A, 0x0000,
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x005F,
            0x0093, 0x0701,
            0x0007, 0x0100,
        };
        static const uint16_t R61580_DEM240320C[]  = { //from DEM 240320C TMH-PW-N
            0x00, 0x0000,
            0x00, 0x0000,
            TFTLCD_DELAY, 100,
            0x00, 0x0000,
            0x00, 0x0000,
            0x00, 0x0000,
            0x00, 0x0000,
            0xA4, 0x0001,
            TFTLCD_DELAY, 100,
            0x60, 0xA700,
            0x08, 0x0808,
            /******************************************/
            //Gamma Setting:
            0x30, 0x0203,
            0x31, 0x080F,
            0x32, 0x0401,
            0x33, 0x050B,
            0x34, 0x3330,
            0x35, 0x0B05,
            0x36, 0x0005,
            0x37, 0x0F08,
            0x38, 0x0302,
            0x39, 0x3033,
            /******************************************/
            //Power Setting:
            0x90, 0x0018, //80Hz
            0x10, 0x0530, //BT,AP
            0x11, 0x0237, //DC1,DC0,VC
            0x12, 0x01BF,
            0x13, 0x1000, //VCOM
            TFTLCD_DELAY, 200,
            /******************************************/
            0x01, 0x0100,
            0x02, 0x0200,
            0x03, 0x1030,
            0x09, 0x0001,
            0x0A, 0x0008,
            0x0C, 0x0000,
            0x0D, 0xD000,

            0x0E, 0x0030,
            0x0F, 0x0000,
            0x20, 0x0000, //H Start
            0x21, 0x0000, //V Start
            0x29, 0x002E,
            0x50, 0x0000,
            0x51, 0x00EF,
            0x52, 0x0000,
            0x53, 0x013F,
            0x61, 0x0001,
            0x6A, 0x0000,
            0x80, 0x0000,
            0x81, 0x0000,
            0x82, 0x005F,
            0x93, 0x0701,
            /******************************************/
            0x07, 0x0100,
            TFTLCD_DELAY, 100,
        };
        init_table16(R61580_DEM240320C, sizeof(R61580_DEM240320C));
//        init_table16(R61580_regValues, sizeof(R61580_regValues));
        break;
#endif

#if defined(SUPPORT_1963) && USING_16BIT_BUS
    case 0x1963:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | READ_NODUMMY | INVERT_SS | INVERT_RGB;
        // from NHD 5.0" 8-bit
        static const uint8_t SSD1963_NHD_50_regValues[]  = {
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    // Lock PLL
            (0xB0), 7, 0x08, 0x80, 0x03, 0x1F, 0x01, 0xDF, 0x00,        //LCD SPECIFICATION
            (0xF0), 1, 0x03,    //was 00 pixel data interface
            //            (0x3A), 1, 0x60,        // SET R G B format = 6 6 6
            (0xE2), 3, 0x1D, 0x02, 0x54,        //PLL multiplier, set PLL clock to 120M
            (0xE6), 3, 0x02, 0xFF, 0xFF,        //PLL setting for PCLK, depends on resolution
            (0xB4), 8, 0x04, 0x20, 0x00, 0x58, 0x80, 0x00, 0x00, 0x00,  //HSYNC
            (0xB6), 7, 0x02, 0x0D, 0x00, 0x20, 0x01, 0x00, 0x00,        //VSYNC
            (0x13), 0,          //Enter Normal mode
            (0x38), 0,          //Exit Idle mode
        };
        // from NHD 7.0" 8-bit
        static const uint8_t SSD1963_NHD_70_regValues[]  = {
            (0xE2), 3, 0x1D, 0x02, 0x04,        //PLL multiplier, set PLL clock to 120M
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    // Lock PLL
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 120,
            (0xB0), 7, 0x08, 0x80, 0x03, 0x1F, 0x01, 0xDF, 0x00,        //LCD SPECIFICATION
            (0xF0), 1, 0x03,    //was 00 pixel data interface
            //            (0x3A), 1, 0x60,        // SET R G B format = 6 6 6
            (0xE6), 3, 0x0F, 0xFF, 0xFF,        //PLL setting for PCLK, depends on resolution
            (0xB4), 8, 0x04, 0x20, 0x00, 0x58, 0x80, 0x00, 0x00, 0x00,  //HSYNC
            (0xB6), 7, 0x02, 0x0D, 0x00, 0x20, 0x01, 0x00, 0x00,        //VSYNC
            (0x13), 0,          //Enter Normal mode
            (0x38), 0,          //Exit Idle mode
        };
        // from UTFTv2.81 initlcd.h
        static const uint8_t SSD1963_800_regValues[]  = {
            (0xE2), 3, 0x1E, 0x02, 0x54,        //PLL multiplier, set PLL clock to 120M
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    //
            TFTLCD_DELAY8, 10,
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 100,
            (0xE6), 3, 0x03, 0xFF, 0xFF,        //PLL setting for PCLK, depends on resolution
            (0xB0), 7, 0x24, 0x00, 0x03, 0x1F, 0x01, 0xDF, 0x00,        //LCD SPECIFICATION
            //            (0xB0), 7, 0x24, 0x00, 0x03, 0x1F, 0x01, 0xDF, 0x2D,        //LCD SPECIFICATION
            (0xB4), 8, 0x03, 0xA0, 0x00, 0x2E, 0x30, 0x00, 0x0F, 0x00,  //HSYNC
            (0xB6), 7, 0x02, 0x0D, 0x00, 0x10, 0x10, 0x00, 0x08,        //VSYNC
            (0xBA), 1, 0x0F,    //GPIO[3:0] out 1
            (0xB8), 2, 0x07, 0x01,      //GPIO3=input, GPIO[2:0]=output
            (0xF0), 1, 0x03,    //pixel data interface
            TFTLCD_DELAY8, 1,
            0x28, 0,            //Display Off
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 100,
            0x29, 0,            //Display On
            (0xBE), 6, 0x06, 0xF0, 0x01, 0xF0, 0x00, 0x00,      //set PWM for B/L
            (0xD0), 1, 0x0D,
        };
        // from UTFTv2.82 initlcd.h
        static const uint8_t SSD1963_800NEW_regValues[]  = {
            (0xE2), 3, 0x1E, 0x02, 0x54,        //PLL multiplier, set PLL clock to 120M
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    //
            TFTLCD_DELAY8, 10,
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 100,
            (0xE6), 3, 0x03, 0xFF, 0xFF,        //PLL setting for PCLK, depends on resolution
            (0xB0), 7, 0x24, 0x00, 0x03, 0x1F, 0x01, 0xDF, 0x00,        //LCD SPECIFICATION
            (0xB4), 8, 0x03, 0xA0, 0x00, 0x2E, 0x30, 0x00, 0x0F, 0x00,  //HSYNC HT=928, HPS=46, HPW=48, LPS=15
            (0xB6), 7, 0x02, 0x0D, 0x00, 0x10, 0x10, 0x00, 0x08,        //VSYNC VT=525, VPS=16, VPW=16, FPS=8
            (0xBA), 1, 0x0F,    //GPIO[3:0] out 1
            (0xB8), 2, 0x07, 0x01,      //GPIO3=input, GPIO[2:0]=output
            (0xF0), 1, 0x03,    //pixel data interface
            TFTLCD_DELAY8, 1,
            0x28, 0,            //Display Off
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 100,
            0x29, 0,            //Display On
            (0xBE), 6, 0x06, 0xF0, 0x01, 0xF0, 0x00, 0x00,      //set PWM for B/L
            (0xD0), 1, 0x0D,
        };
        // from UTFTv2.82 initlcd.h
        static const uint8_t SSD1963_800ALT_regValues[]  = {
            (0xE2), 3, 0x23, 0x02, 0x04,        //PLL multiplier, set PLL clock to 120M
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    //
            TFTLCD_DELAY8, 10,
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 100,
            (0xE6), 3, 0x04, 0x93, 0xE0,        //PLL setting for PCLK, depends on resolution
            (0xB0), 7, 0x00, 0x00, 0x03, 0x1F, 0x01, 0xDF, 0x00,        //LCD SPECIFICATION
            (0xB4), 8, 0x03, 0xA0, 0x00, 0x2E, 0x30, 0x00, 0x0F, 0x00,  //HSYNC HT=928, HPS=46, HPW=48, LPS=15
            (0xB6), 7, 0x02, 0x0D, 0x00, 0x10, 0x10, 0x00, 0x08,        //VSYNC VT=525, VPS=16, VPW=16, FPS=8
            (0xBA), 1, 0x0F,    //GPIO[3:0] out 1
            (0xB8), 2, 0x07, 0x01,      //GPIO3=input, GPIO[2:0]=output
            (0xF0), 1, 0x03,    //pixel data interface
            TFTLCD_DELAY8, 1,
            0x28, 0,            //Display Off
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 100,
            0x29, 0,            //Display On
            (0xBE), 6, 0x06, 0xF0, 0x01, 0xF0, 0x00, 0x00,      //set PWM for B/L
            (0xD0), 1, 0x0D,
        };
        // from UTFTv2.82 initlcd.h
        static const uint8_t SSD1963_480_regValues[]  = {
            (0xE2), 3, 0x23, 0x02, 0x54,        //PLL multiplier, set PLL clock to 120M
            (0xE0), 1, 0x01,    // PLL enable
            TFTLCD_DELAY8, 10,
            (0xE0), 1, 0x03,    //
            TFTLCD_DELAY8, 10,
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 100,
            (0xE6), 3, 0x01, 0x1F, 0xFF,        //PLL setting for PCLK, depends on resolution
            (0xB0), 7, 0x20, 0x00, 0x01, 0xDF, 0x01, 0x0F, 0x00,        //LCD SPECIFICATION
            (0xB4), 8, 0x02, 0x13, 0x00, 0x08, 0x2B, 0x00, 0x02, 0x00,  //HSYNC
            (0xB6), 7, 0x01, 0x20, 0x00, 0x04, 0x0C, 0x00, 0x02,        //VSYNC
            (0xBA), 1, 0x0F,    //GPIO[3:0] out 1
            (0xB8), 2, 0x07, 0x01,      //GPIO3=input, GPIO[2:0]=output
            (0xF0), 1, 0x03,    //pixel data interface
            TFTLCD_DELAY8, 1,
            0x28, 0,            //Display Off
            0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 100,
            0x29, 0,            //Display On
            (0xBE), 6, 0x06, 0xF0, 0x01, 0xF0, 0x00, 0x00,      //set PWM for B/L
            (0xD0), 1, 0x0D,
        };
//        table8_ads = SSD1963_480_regValues, table_size = sizeof(SSD1963_480_regValues);
        table8_ads = SSD1963_800_regValues, table_size = sizeof(SSD1963_800_regValues);
//        table8_ads = SSD1963_NHD_50_regValues, table_size = sizeof(SSD1963_NHD_50_regValues);
//        table8_ads = SSD1963_NHD_70_regValues, table_size = sizeof(SSD1963_NHD_70_regValues);
//        table8_ads = SSD1963_800NEW_regValues, table_size = sizeof(SSD1963_800NEW_regValues);
//        table8_ads = SSD1963_800ALT_regValues, table_size = sizeof(SSD1963_800ALT_regValues);
        p16 = (int16_t *) & height;
        *p16 = 480;
        p16 = (int16_t *) & width;
        *p16 = 800;
        break;
#endif

#ifdef SUPPORT_4532
//Support for LG Electronics LGDP4532 (also 4531 i guess) by Leodino v1.0 2-Nov-2016
//based on data by waveshare and the datasheet of LG Electronics
//My approach to get it working: the parameters by waveshare did no make it function allright
//I started with remming lines to see if this helped. Basically the stuff in range 41-93
//gives problems.
//The other lines that are REMmed give no problems, but it seems default values are OK as well.
case 0x4532:    // thanks Leodino
	_lcd_capable = 0 | REV_SCREEN;  // | INVERT_GS;
	static const uint16_t LGDP4532_regValues[]  = {
		0x0000,0x0001, //Device code read
		0x0010,0x0628, //Power control 1 SAP[2:0] BT[3:0] AP[2:0] DK DSTB SLP
		0x0012,0x0006, //Power control 3 PON VRH[3:0]
		//0x0013,0x0A32, //Power control 4 VCOMG VDV[4:0] VCM[6:0]
		0x0011,0x0040, //Power control 2; DC1[2:0] DC0[2:0] VC[2:0]
		//0x0015,0x0050, //Regulator control RSET RI[2:0] RV[2:0] RCONT[2:0]
		0x0012,0x0016, //Power control 3 PON VRH[3:0]
		TFTLCD_DELAY,50,
		0x0010,0x5660, //Power control 1 SAP[2:0] BT[3:0] AP[2:0] DK DSTB SLP
		TFTLCD_DELAY,50,
		//0x0013,0x2A4E, //Power control 4 VCOMG VDV[4:0] VCM[6:0]
		//0x0001,0x0100, //Driver output control SM SS
		//0x0002,0x0300, //LCD Driving Wave Control
		//0x0003,0x1030, //Entry mode TRI DFM  BGR  ORG I/D[1:0] AM
		//0x0007,0x0202, //Display Control 1 PTDE[1:0] BASEE GON DTE COL D[1:0]
		TFTLCD_DELAY,50,
		//0x0008,0x0202, //Display Control 2 FP[3:0] BP[3:0] front and back porch (blank period at begin and end..)
		//0x000A,0x0000, //Test Register 1 (RA0h)
		//Gamma adjustment
		0x0030,0x0000,
		0x0031,0x0402,
		0x0032,0x0106,
		0x0033,0x0700,
		0x0034,0x0104,
		0x0035,0x0301,
		0x0036,0x0707,
		0x0037,0x0305,
		0x0038,0x0208,
		0x0039,0x0F0B,
		TFTLCD_DELAY,50,
		//some of this stuff in range 41-93 really throws things off....
		//0x0041,0x0002,
		//0x0060,0x2700, //Driver Output Control (R60h)
		//0x0061,0x0001, //Base Image Display Control (R61h)
		//0x0090,0x0119,   //Panel Interface Control 1 (R90h) DIVI[1:0]  RTNI[4:0]
		//0x0092,0x010A,  //Panel Interface Control 2 (R92h)  NOWI[2:0] EQI2[1:0] EQI1[1:0]
		//0x0093,0x0004, //Panel Interface Control 3 (R93h) MCPI[2:0]
		//0x00A0,0x0100, //Test Register 1 (RA0h)
		TFTLCD_DELAY,50,
		0x0007,0x0133, //Display Control 1 PTDE[1:0] BASEE GON DTE COL D[1:0]
		TFTLCD_DELAY,50,
		//0x00A0,0x0000, //Test Register 1 (RA0h)
		};
	init_table16(LGDP4532_regValues, sizeof(LGDP4532_regValues));
	break;
#endif

#ifdef SUPPORT_4535
    case 0x4535:
        _lcd_capable = 0 | REV_SCREEN;  // | INVERT_GS;
        static const uint16_t LGDP4535_regValues[]  = {
            0x0015, 0x0030,     // Set the internal vcore voltage
            0x009A, 0x0010,     // Start internal OSC
            0x0011, 0x0020,     // set SS and SM bit
            0x0010, 0x3428,     // set 1 line inversion
            0x0012, 0x0002,     // set GRAM write direction and BGR=1
            0x0013, 0x1038,     // Resize register
            TFTLCD_DELAY, 40,
            0x0012, 0x0012,     // set the back porch and front porch
            TFTLCD_DELAY, 40,
            0x0010, 0x3420,     // set non-display area refresh cycle ISC[3:0]
            0x0013, 0x3045,     // FMARK function
            TFTLCD_DELAY, 70,
            0x0030, 0x0000,     // RGB interface setting
            0x0031, 0x0402,     // Frame marker Position
            0x0032, 0x0307,     // RGB interface polarity
            0x0033, 0x0304,     // SAP, BT[3:0], AP, DSTB, SLP, STB
            0x0034, 0x0004,     // DC1[2:0], DC0[2:0], VC[2:0]
            0x0035, 0x0401,     // VREG1OUT voltage
            0x0036, 0x0707,     // VDV[4:0] for VCOM amplitude
            0x0037, 0x0305,     // SAP, BT[3:0], AP, DSTB, SLP, STB
            0x0038, 0x0610,     // DC1[2:0], DC0[2:0], VC[2:0]
            0x0039, 0x0610,     // VREG1OUT voltage
            0x0001, 0x0100,     // VDV[4:0] for VCOM amplitude
            0x0002, 0x0300,     // VCM[4:0] for VCOMH
            0x0003, 0x1030,     // GRAM horizontal Address
            0x0008, 0x0808,     // GRAM Vertical Address
            0x000A, 0x0008,
            0x0060, 0x2700,     // Gate Scan Line
            0x0061, 0x0001,     // NDL,VLE, REV
            0x0090, 0x013E,
            0x0092, 0x0100,
            0x0093, 0x0100,
            0x00A0, 0x3000,
            0x00A3, 0x0010,
            0x0007, 0x0001,
            0x0007, 0x0021,
            0x0007, 0x0023,
            0x0007, 0x0033,
            0x0007, 0x0133,
        };
        init_table16(LGDP4535_regValues, sizeof(LGDP4535_regValues));
        break;
#endif

    case 0x5310:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | INVERT_SS | INVERT_RGB | READ_24BITS;
        static const uint8_t NT35310_regValues[]  = {        //
            TFTLCD_DELAY8, 10,    //just some dummy
        };
        table8_ads = NT35310_regValues, table_size = sizeof(NT35310_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;

#ifdef SUPPORT_68140
    case 0x6814:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS;
		static const uint8_t RM68140_regValues_max[]  = {        //
            0x3A, 1, 0x55,      //Pixel format .kbv my Mega Shield
        };
        table8_ads = RM68140_regValues_max, table_size = sizeof(RM68140_regValues_max);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;
#endif

#ifdef SUPPORT_7735
    case 0x7735:                //
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN | READ_24BITS;
        static const uint8_t  table7735S[] = {
            //  (COMMAND_BYTE), n, data_bytes....
            0xB1, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C] FRMCTR1 if GM==11
            0xB2, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C]
            0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, // [05 3C 3C 05 3C 3C]
            0xB4, 1, 0x07,              // [07] INVCTR Column inversion
            //ST7735XR Power Sequence
            0xC0, 3, 0xA2, 0x02, 0x84,  // [A8 08 84] PWCTR1
            0xC1, 1, 0xC5,              // [C0]
            0xC2, 2, 0x0A, 0x00,        // [0A 00]
            0xC3, 2, 0x8A, 0x2A,        // [8A 26]
            0xC4, 2, 0x8A, 0xEE,        // [8A EE]
            0xC5, 1, 0x0E,              // [05] VMCTR1 VCOM
        };
        table8_ads = table7735S, table_size = sizeof(table7735S);   //
        p16 = (int16_t *) & height;
        *p16 = 160;
        p16 = (int16_t *) & width;
        *p16 = 128;
        break;
#endif

#ifdef SUPPORT_7781
    case 0x7783:
        _lcd_capable = AUTO_READINC | REV_SCREEN | INVERT_GS;
        static const uint16_t ST7781_regValues[]  = {
            0x00FF, 0x0001,     //can we do 0xFF
            0x00F3, 0x0008,
            //  LCD_Write_COM(0x00F3,

            0x00, 0x0001,
            0x0001, 0x0100,     // Driver Output Control Register (R01h)
            0x0002, 0x0700,     // LCD Driving Waveform Control (R02h)
            0x0003, 0x1030,     // Entry Mode (R03h)
            0x0008, 0x0302,
            0x0009, 0x0000,
            0x0010, 0x0000,     // Power Control 1 (R10h)
            0x0011, 0x0007,     // Power Control 2 (R11h)
            0x0012, 0x0000,     // Power Control 3 (R12h)
            0x0013, 0x0000,     // Power Control 4 (R13h)
            TFTLCD_DELAY, 50,
            0x0010, 0x14B0,     // Power Control 1 SAP=1, BT=4, APE=1, AP=3
            TFTLCD_DELAY, 10,
            0x0011, 0x0007,     // Power Control 2 VC=7
            TFTLCD_DELAY, 10,
            0x0012, 0x008E,     // Power Control 3 VCIRE=1, VRH=14
            0x0013, 0x0C00,     // Power Control 4 VDV=12
            0x0029, 0x0015,     // NVM read data 2 VCM=21
            TFTLCD_DELAY, 10,
            0x0030, 0x0000,     // Gamma Control 1
            0x0031, 0x0107,     // Gamma Control 2
            0x0032, 0x0000,     // Gamma Control 3
            0x0035, 0x0203,     // Gamma Control 6
            0x0036, 0x0402,     // Gamma Control 7
            0x0037, 0x0000,     // Gamma Control 8
            0x0038, 0x0207,     // Gamma Control 9
            0x0039, 0x0000,     // Gamma Control 10
            0x003C, 0x0203,     // Gamma Control 13
            0x003D, 0x0403,     // Gamma Control 14
            0x0060, 0xA700,     // Driver Output Control (R60h) .kbv was 0xa700
            0x0061, 0x0001,     // Driver Output Control (R61h)
            0x0090, 0X0029,     // Panel Interface Control 1 (R90h)

            // Display On
            0x0007, 0x0133,     // Display Control (R07h)
            TFTLCD_DELAY, 50,
        };
        init_table16(ST7781_regValues, sizeof(ST7781_regValues));
        break;
#endif

    case 0x7789:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t ST7789_regValues[]  = {
            (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
            (0xB7), 1, 0x35,    //GCTRL: Gate Control [35]
            (0xBB), 1, 0x2B,    //VCOMS: VCOM setting VCOM=1.175 [20] VCOM=0.9
            (0xC0), 1, 0x04,    //LCMCTRL: LCM Control [2C]
            (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
            (0xC3), 1, 0x11,    //VRHS: VRH Set VAP=4.4, VAN=-4.4 [0B]
            (0xC4), 1, 0x20,    //VDVS: VDV Set [20]
            (0xC6), 1, 0x0F,    //FRCTRL2: Frame Rate control in normal mode [0F]
            (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
            (0xE0), 14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,     //PVGAMCTRL: Positive Voltage Gamma control
            (0xE1), 14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,     //NVGAMCTRL: Negative Voltage Gamma control
        };
        static const uint8_t ST7789_regValues_arcain6[]  = {
            (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
            (0xB7), 1, 0x35,    //GCTRL: Gate Control [35]
            (0xBB), 1, 0x35,    //VCOMS: VCOM setting VCOM=??? [20] VCOM=0.9
            (0xC0), 1, 0x2C,    //LCMCTRL: LCM Control [2C]
            (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
            (0xC3), 1, 0x13,    //VRHS: VRH Set VAP=???, VAN=-??? [0B]
            (0xC4), 1, 0x20,    //VDVS: VDV Set [20]
            (0xC6), 1, 0x0F,    //FRCTRL2: Frame Rate control in normal mode [0F]
            (0xCA), 1, 0x0F,    //REGSEL2 [0F]
            (0xC8), 1, 0x08,    //REGSEL1 [08]
            (0x55), 1, 0x90,    //WRCACE  [00]
            (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
            (0xE0), 14, 0xD0, 0x00, 0x06, 0x09, 0x0B, 0x2A, 0x3C, 0x55, 0x4B, 0x08, 0x16, 0x14, 0x19, 0x20,     //PVGAMCTRL: Positive Voltage Gamma control
            (0xE1), 14, 0xD0, 0x00, 0x06, 0x09, 0x0B, 0x29, 0x36, 0x54, 0x4B, 0x0D, 0x16, 0x14, 0x21, 0x20,     //NVGAMCTRL: Negative Voltage Gamma control
        };
        table8_ads = ST7789_regValues, table_size = sizeof(ST7789_regValues); //
        break;

    case 0x8031:      //Unknown BangGood thanks PrinceCharles
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | REV_SCREEN;
        static const uint8_t FK8031_regValues[]  = {
            // 0xF2:8.2 = SM, 0xF2:8.0 = REV. invertDisplay(), vertScroll() do not work
            0xF2,11, 0x16, 0x16, 0x03, 0x08, 0x08, 0x08, 0x08, 0x10, 0x04, 0x16, 0x16, // f.k. 0xF2:8.2 SM=1
            0xFD, 3, 0x11, 0x02, 0x35,     //f.k 0xFD:1.1 creates contiguous scan lins
        };
        table8_ads = FK8031_regValues, table_size = sizeof(FK8031_regValues);
        break;

#ifdef SUPPORT_8347D
    case 0x4747:       //HX8347-D
        _lcd_capable = REV_SCREEN | MIPI_DCS_REV1 | MV_AXIS | INVERT_SS | AUTO_READINC | READ_24BITS;
		goto common_8347DGI;
    case 0x6767:       //HX8367-A
	case 0x7575:       //HX8347-G
	case 0x9595:       //HX8347-I
        _lcd_capable = REV_SCREEN | MIPI_DCS_REV1 | MV_AXIS;
      common_8347DGI:
		is8347 = 1;
        static const uint8_t HX8347G_2_regValues[]  = {
            0xEA, 2, 0x00, 0x20,        //PTBA[15:0]
            0xEC, 2, 0x0C, 0xC4,        //STBA[15:0]
            0xE8, 1, 0x38,      //OPON[7:0]
            0xE9, 1, 0x10,      //OPON1[7:0]
            0xF1, 1, 0x01,      //OTPS1B
            0xF2, 1, 0x10,      //GEN
            //Gamma 2.2 Setting
            0x40, 13, 0x01, 0x00, 0x00, 0x10, 0x0E, 0x24, 0x04, 0x50, 0x02, 0x13, 0x19, 0x19, 0x16,
            0x50, 14, 0x1B, 0x31, 0x2F, 0x3F, 0x3F, 0x3E, 0x2F, 0x7B, 0x09, 0x06, 0x06, 0x0C, 0x1D, 0xCC,
            //Power Voltage Setting
            0x1B, 1, 0x1B,      //VRH=4.65V
            0x1A, 1, 0x01,      //BT (VGH~15V,VGL~-10V,DDVDH~5V)
            0x24, 1, 0x2F,      //VMH(VCOM High voltage ~3.2V)
            0x25, 1, 0x57,      //VML(VCOM Low voltage -1.2V)
            //****VCOM offset**///
            0x23, 1, 0x88,      //for Flicker adjust //can reload from OTP
            //Power on Setting
            0x18, 1, 0x34,      //I/P_RADJ,N/P_RADJ, Normal mode 60Hz
            0x19, 1, 0x01,      //OSC_EN='1', start Osc
            0x01, 1, 0x00,      //DP_STB='0', out deep sleep
            0x1F, 1, 0x88,      // GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
            TFTLCD_DELAY8, 5,
            0x1F, 1, 0x80,      // GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
            TFTLCD_DELAY8, 3,
            0x1F, 1, 0x90,      // GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
            TFTLCD_DELAY8, 5,
            0x1F, 1, 0xD0,      // GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
            TFTLCD_DELAY8, 5,
            //262k/65k color selection
            0x17, 1, 0x05,      //default 0x06 262k color // 0x05 65k color
            //SET PANEL
            0x36, 1, 0x00,      //SS_P, GS_P,REV_P,BGR_P
            //Display ON Setting
            0x28, 1, 0x38,      //GON=1, DTE=1, D=1000
            TFTLCD_DELAY8, 40,
            0x28, 1, 0x3F,      //GON=1, DTE=1, D=1100

            0x16, 1, 0x18,
        };
        init_table(HX8347G_2_regValues, sizeof(HX8347G_2_regValues));
        break;
#endif

#ifdef SUPPORT_8352A
	case 0x5252:       //HX8352-A
        _lcd_capable = MIPI_DCS_REV1 | MV_AXIS;
        is8347 = 1;
        static const uint8_t HX8352A_regValues[]  = {
            0x83, 1, 0x02,      //Test Mode: TESTM=1
            0x85, 1, 0x03,      //VDD ctl  : VDC_SEL=3 [05]
            0x8B, 1, 0x01,      //VGS_RES 1: RES_VGS1=1
            0x8C, 1, 0x93,      //VGS_RES 2: RES_VGS2=1, anon=0x13 [93]
            0x91, 1, 0x01,      //PWM control: SYNC=1
            0x83, 1, 0x00,      //Test Mode: TESTM=0
            //Gamma  Setting
            0x3E, 12, 0xB0, 0x03, 0x10, 0x56, 0x13, 0x46, 0x23, 0x76, 0x00, 0x5E, 0x4F, 0x40,
            //Power Voltage Setting
            0x17, 1, 0x91,      //OSC   1: RADJ=9, OSC_EN=1 [F0]
            0x2B, 1, 0xF9,      //Cycle 1: N_DC=F9 [BE]
            TFTLCD_DELAY8, 10,
            0x1B, 1, 0x14,      //Power 3: BT=1, ??=1, AP=0 [42]
            0x1A, 1, 0x11,      //Power 2: VC3=1, VC1=1 [05]
            0x1C, 1, 0x06,      //Power 4: VRH=6 [0D]
            0x1F, 1, 0x42,      //VCOM   : VCM=42 [55]
            TFTLCD_DELAY8, 20,
            0x19, 1, 0x0A,      //Power 1: DK=1, VL_TR1=1 [09]
            0x19, 1, 0x1A,      //Power 1: PON=1, DK=1, VL_TR1=1 [09]
            TFTLCD_DELAY8, 40,
            0x19, 1, 0x12,      //Power 1: PON=1, DK=1, STB=1 [09]
            TFTLCD_DELAY8, 40,
            0x1E, 1, 0x27,      //Power 6: VCOMG=1, VDV=7 [10]
            TFTLCD_DELAY8, 100,
            //Display ON Setting
            0x24, 1, 0x60,      //Display 2: PT=1, GON=1 [A0]
            0x3D, 1, 0x40,      //Source 1: N_SAP=40 [C0]
            0x34, 1, 0x38,      //Cycle 10: EQS=0x38 [38]
            0x35, 1, 0x38,      //Cycle 11: EQP=0x38 [38]
            0x24, 1, 0x38,      //Display 2: GON=1 D=2 [A0]
            TFTLCD_DELAY8, 40,
            0x24, 1, 0x3C,      //Display 2: GON=1 D=3 [A0]
            0x16, 1, 0x1C,      //Memaccess: GS=1, BGR=1, SS=1
            0x01, 1, 0x06,      //Disp Mode: INVON=1, NORON=1 [02]
            0x55, 1, 0x06,      //SM_PANEL=0, SS_PANEL=0, GS_PANEL=1, REV_PANEL=1, BGR_PANEL=0
        };
        init_table(HX8352A_regValues, sizeof(HX8352A_regValues));
        p16 = (int16_t *) & height;
        *p16 = 400;
        break;
#endif

#ifdef SUPPORT_8352B
    case 0x0065:       //HX8352-B
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | REV_SCREEN;
        is8347 = 1;
        static const uint8_t HX8352B_regValues[]  = {
            // Register setting for EQ setting
            0xe5, 1, 0x10,      //
            0xe7, 1, 0x10,      //
            0xe8, 1, 0x48,      //
            0xec, 1, 0x09,      //
            0xed, 1, 0x6c,      //
            // Power on Setting
            0x23, 1, 0x6F,      //VMF
            0x24, 1, 0x57,      //VMH
            0x25, 1, 0x71,      //VML
            0xE2, 1, 0x18,      //
            0x1B, 1, 0x15,      //VRH
            0x01, 1, 0x00,      //
            0x1C, 1, 0x03,      //AP=3
            // Power on sequence
            0x19, 1, 0x01,      //OSCEN=1
            TFTLCD_DELAY8, 5,
            0x1F, 1, 0x8C,      //GASEN=1, DK=1, XDK=1
            0x1F, 1, 0x84,      //GASEN=1, XDK=1
            TFTLCD_DELAY8, 10,
            0x1F, 1, 0x94,      //GASEN=1, PON=1, XDK=1
            TFTLCD_DELAY8, 10,
            0x1F, 1, 0xD4,      //GASEN=1, VCOMG=1, PON=1, XDK=1
            TFTLCD_DELAY8, 5,
            // Gamma Setting
            0x40, 13, 0x00, 0x2B, 0x29, 0x3E, 0x3D, 0x3F, 0x24, 0x74, 0x08, 0x06, 0x07, 0x0D, 0x17,
            0x50, 13, 0x00, 0x02, 0x01, 0x16, 0x14, 0x3F, 0x0B, 0x5B, 0x08, 0x12, 0x18, 0x19, 0x17,
            0x5D, 1, 0xFF,      //

            0x16, 1, 0x08,      //MemoryAccess BGR=1
            0x28, 1, 0x20,      //GON=1
            TFTLCD_DELAY8, 40,
            0x28, 1, 0x38,      //GON=1, DTE=1, D=2
            TFTLCD_DELAY8, 40,
            0x28, 1, 0x3C,      //GON=1, DTE=1, D=3

            0x02, 2, 0x00, 0x00,     //SC
            0x04, 2, 0x00, 0xEF,     //EC
            0x06, 2, 0x00, 0x00,     //SP
            0x08, 2, 0x01, 0x8F,     //EP

            0x80, 2, 0x00, 0x00,     //CAC
            0x82, 2, 0x00, 0x00,     //RAC
            0x17, 1, 0x05,      //COLMOD = 565

        };
        init_table(HX8352B_regValues, sizeof(HX8352B_regValues));
        p16 = (int16_t *) & height;
        *p16 = 400;
        break;
#endif

#ifdef SUPPORT_8347A
    case 0x8347:
        _lcd_capable = REV_SCREEN | MIPI_DCS_REV1 | MV_AXIS;
        // AN.01 The reference setting of CMO 3.2” Panel
        static const uint8_t HX8347A_CMO32_regValues[]  = {
            //  VENDOR Gamma for 3.2"
            (0x46), 12, 0xA4, 0x53, 0x00, 0x44, 0x04, 0x67, 0x33, 0x77, 0x12, 0x4C, 0x46, 0x44,
            // Display Setting
            (0x01), 1, 0x06,    // IDMON=0, INVON=1, NORON=1, PTLON=0
            (0x16), 1, 0x48,    // MY=0, MX=0, MV=0, ML=1, BGR=0, TEON=0
            (0x23), 3, 0x95, 0x95, 0xFF,        // N_DC=1001 0101, PI_DC=1001 0101, I_DC=1111 1111

            (0x27), 4, 0x02, 0x02, 0x02, 0x02,  // N_BP=2, N_FP=2, PI_BP=2, PI_FP=2
            (0x2C), 2, 0x02, 0x02,      // I_BP=2, I_FP=2

            (0x3a), 4, 0x01, 0x01, 0xF0, 0x00,  // N_RTN=0, N_NW=1, P_RTN=0, P_NW=1, I_RTN=15, I_NW=0, DIV=0
            TFTLCD_DELAY8, 5,
            (0x35), 2, 0x38, 0x78,      // EQS=38h, EQP=78h
            (0x3E), 1, 0x38,    // SON=38h
            (0x40), 2, 0x0F, 0xF0,      // GDON=0Fh, GDOFF
            // Power Supply Setting
            (0x19), 1, 0x49,    // CADJ=0100, CUADJ=100, OSD_EN=1 ,60Hz
            (0x93), 1, 0x0F,    // RADJ=1111, 100%
            TFTLCD_DELAY8, 5,
            (0x20), 1, 0x40,    // BT=0100
            (0x1D), 3, 0x07, 0x00, 0x04,        // VC1=7, VC3=0, VRH=??
            //VCOM SETTING for 3.2"
            (0x44), 2, 0x4D, 0x11,      // VCM=100 1101, VDV=1 0001
            TFTLCD_DELAY8, 10,
            (0x1C), 1, 0x04,    // AP=100
            TFTLCD_DELAY8, 20,
            (0x1B), 1, 0x18,    // GASENB=0, PON=0, DK=1, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 40,
            (0x1B), 1, 0x10,    // GASENB=0, PON=1, DK=0, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 40,
            (0x43), 1, 0x80,    //set VCOMG=1
            TFTLCD_DELAY8, 100,
            // Display ON Setting
            (0x90), 1, 0x7F,    // SAP=0111 1111
            (0x26), 1, 0x04,    //GON=0, DTE=0, D=01
            TFTLCD_DELAY8, 40,
            (0x26), 1, 0x24,    //GON=1, DTE=0, D=01
            (0x26), 1, 0x2C,    //GON=1, DTE=0, D=11
            TFTLCD_DELAY8, 40,
            (0x26), 1, 0x3C,    //GON=1, DTE=1, D=11
            // INTERNAL REGISTER SETTING
            (0x57), 1, 0x02,    // TEST_Mode=1: into TEST mode
            (0x55), 1, 0x00,    // VDC_SEL=000, VDDD=1.95V
            (0xFE), 1, 0x5A,    // For ESD protection
            (0x57), 1, 0x00,    // TEST_Mode=0: exit TEST mode
        };
        // AN.01 The reference setting of CMO 2.4” Panel
        static const uint8_t HX8347A_CMO24_regValues[]  = {
            //  VENDOR Gamma for 2.4"
            (0x46), 12, 0x94, 0x41, 0x00, 0x33, 0x23, 0x45, 0x44, 0x77, 0x12, 0xCC, 0x46, 0x82,
            // Display Setting
            (0x01), 1, 0x06,    // IDMON=0, INVON=1, NORON=1, PTLON=0
            (0x16), 1, 0x48,    // MY=0, MX=0, MV=0, ML=1, BGR=0, TEON=0
            (0x23), 3, 0x95, 0x95, 0xFF,        // N_DC=1001 0101, PI_DC=1001 0101, I_DC=1111 1111

            (0x27), 4, 0x02, 0x02, 0x02, 0x02,  // N_BP=2, N_FP=2, PI_BP=2, PI_FP=2
            (0x2C), 2, 0x02, 0x02,      // I_BP=2, I_FP=2

            (0x3a), 4, 0x01, 0x01, 0xF0, 0x00,  // N_RTN=0, N_NW=1, P_RTN=0, P_NW=1, I_RTN=15, I_NW=0, DIV=0
            TFTLCD_DELAY8, 5,
            (0x35), 2, 0x38, 0x78,      // EQS=38h, EQP=78h
            (0x3E), 1, 0x38,    // SON=38h
            (0x40), 2, 0x0F, 0xF0,      // GDON=0Fh, GDOFF
            // Power Supply Setting
            (0x19), 1, 0x49,    // CADJ=0100, CUADJ=100, OSD_EN=1 ,60Hz
            (0x93), 1, 0x0F,    // RADJ=1111, 100%
            TFTLCD_DELAY8, 5,
            (0x20), 1, 0x40,    // BT=0100
            (0x1D), 3, 0x07, 0x00, 0x04,        // VC1=7, VC3=0, VRH=??
            //VCOM SETTING for 2.4"
            (0x44), 2, 0x40, 0x12,      // VCM=100 0000, VDV=1 0001
            TFTLCD_DELAY8, 10,
            (0x1C), 1, 0x04,    // AP=100
            TFTLCD_DELAY8, 20,
            (0x1B), 1, 0x18,    // GASENB=0, PON=0, DK=1, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 40,
            (0x1B), 1, 0x10,    // GASENB=0, PON=1, DK=0, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 40,
            (0x43), 1, 0x80,    //set VCOMG=1
            TFTLCD_DELAY8, 100,
            // Display ON Setting
            (0x90), 1, 0x7F,    // SAP=0111 1111
            (0x26), 1, 0x04,    //GON=0, DTE=0, D=01
            TFTLCD_DELAY8, 40,
            (0x26), 1, 0x24,    //GON=1, DTE=0, D=01
            (0x26), 1, 0x2C,    //GON=1, DTE=0, D=11
            TFTLCD_DELAY8, 40,
            (0x26), 1, 0x3C,    //GON=1, DTE=1, D=11
            // INTERNAL REGISTER SETTING
            (0x57), 1, 0x02,    // TEST_Mode=1: into TEST mode
            (0x55), 1, 0x00,    // VDC_SEL=000, VDDD=1.95V
            (0xFE), 1, 0x5A,    // For ESD protection
            (0x57), 1, 0x00,    // TEST_Mode=0: exit TEST mode
        };
        static const uint8_t HX8347A_ITDB02_regValues[]  = {
            //  VENDOR Gamma ITDB02 same as CMO32.   Delays are shorter than AN01
            (0x46), 12, 0xA4, 0x53, 0x00, 0x44, 0x04, 0x67, 0x33, 0x77, 0x12, 0x4C, 0x46, 0x44,
            // Display Setting
            (0x01), 1, 0x06,    // IDMON=0, INVON=1, NORON=1, PTLON=0
            (0x16), 1, 0xC8,    // MY=0, MX=0, MV=0, ML=1, BGR=0, TEON=0 .itead
            (0x23), 3, 0x95, 0x95, 0xFF,        // N_DC=1001 0101, PI_DC=1001 0101, I_DC=1111 1111

            (0x27), 4, 0x02, 0x02, 0x02, 0x02,  // N_BP=2, N_FP=2, PI_BP=2, PI_FP=2
            (0x2C), 2, 0x02, 0x02,      // I_BP=2, I_FP=2

            (0x3a), 4, 0x01, 0x00, 0xF0, 0x00,  // N_RTN=0, N_NW=1, P_RTN=0, ?? P_NW=1, I_RTN=15, I_NW=0, DIV=0 .itead
            TFTLCD_DELAY8, 5,
            (0x35), 2, 0x38, 0x78,      // EQS=38h, EQP=78h
            (0x3E), 1, 0x38,    // SON=38h
            (0x40), 2, 0x0F, 0xF0,      // GDON=0Fh, GDOFF
            // Power Supply Setting
            (0x19), 1, 0x49,    // CADJ=0100, CUADJ=100, OSD_EN=1 ,60Hz
            (0x93), 1, 0x0F,    // RADJ=1111, 100%
            TFTLCD_DELAY8, 5,
            (0x20), 1, 0x40,    // BT=0100
            (0x1D), 3, 0x07, 0x00, 0x04,        // VC1=7, VC3=0, VRH=??
            //VCOM SETTING for ITDB02
            (0x44), 2, 0x4D, 0x0E,      // VCM=101 0000  4D, VDV=1 0001 .itead
            TFTLCD_DELAY8, 5,
            (0x1C), 1, 0x04,    // AP=100
            TFTLCD_DELAY8, 5,
            (0x1B), 1, 0x18,    // GASENB=0, PON=0, DK=1, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 5,
            (0x1B), 1, 0x10,    // GASENB=0, PON=1, DK=0, XDK=0, VLCD_TRI=0, STB=0
            TFTLCD_DELAY8, 5,
            (0x43), 1, 0x80,    //set VCOMG=1
            TFTLCD_DELAY8, 5,
            // Display ON Setting
            (0x90), 1, 0x7F,    // SAP=0111 1111
            (0x26), 1, 0x04,    //GON=0, DTE=0, D=01
            TFTLCD_DELAY8, 5,
            (0x26), 1, 0x24,    //GON=1, DTE=0, D=01
            (0x26), 1, 0x2C,    //GON=1, DTE=0, D=11
            TFTLCD_DELAY8, 5,
            (0x26), 1, 0x3C,    //GON=1, DTE=1, D=11
            // INTERNAL REGISTER SETTING for ITDB02
            (0x57), 1, 0x02,    // TEST_Mode=1: into TEST mode
            (0x95), 1, 0x01,    // SET DISPLAY CLOCK AND PUMPING CLOCK TO SYNCHRONIZE .itead
            (0x57), 1, 0x00,    // TEST_Mode=0: exit TEST mode
        };
        static const uint8_t HX8347A_NHD_regValues[]  = {
            //Gamma Setting NHD
            (0x46), 12, 0x94, 0x41, 0x00, 0x33, 0x23, 0x45, 0x44, 0x77, 0x12, 0xCC, 0x46, 0x82,
            (0x01), 1, 0x06,    //Display Mode [06]
            (0x16), 1, 0xC8,    //MADCTL [00] MY=1, MX=1, BGR=1
//            (0x70), 1, 0x05,    //Panel [06] 16-bit
            (0x23), 3, 0x95, 0x95, 0xFF,        //Cycle Control 1-3 [95 95 FF]
            (0x27), 4, 0x02, 0x02, 0x02, 0x02,  //Display Control 2-5 [02 02 02 02]
            (0x2C), 2, 0x02, 0x02,      //Display Control 6-7 [02 02]
            (0x3A), 4, 0x01, 0x01, 0xF0, 0x00,  //Cycle Control 1-4 [01 01 F0 00]
            TFTLCD_DELAY8, 80,
            (0x35), 2, 0x38, 0x78,      //Display Control 9-10 [09 09] EQS=56, EQP=120
            (0x3E), 1, 0x38,    //Cycle Control 5 [38]
            (0x40), 1, 0x0F,    //Cycle Control 6 [03]  GDON=15
            (0x41), 1, 0xF0,    //Cycle Control 14 [F8] GDOF=248

            (0x19), 1, 0x2D,    //OSC Control 1 [86] CADJ=2, CUADJ=6, OSCEN=1
            (0x93), 1, 0x06,    //SAP Idle mode [00] ???  .nhd
            TFTLCD_DELAY8, 80,
            (0x20), 1, 0x40,    //Power Control 6 [40]
            (0x1D), 3, 0x07, 0x00, 0x04,        //Power Control 3-5 [04 00 06] VC=7
            (0x44), 2, 0x3C, 0x12,      //VCOM Control 2-3 [5A 11] VCM=60, VDV=18
            TFTLCD_DELAY8, 80,
            (0x1C), 1, 0x04,    //Power Control 2 [04]
            TFTLCD_DELAY8, 80,
            (0x43), 1, 0x80,    //VCOM Control 1 [80]
            TFTLCD_DELAY8, 80,
            (0x1B), 1, 0x08,    //Power Control 1 [00] DK=1
            TFTLCD_DELAY8, 80,
            (0x1B), 1, 0x10,    //Power Control 1 [00] PON=1
            TFTLCD_DELAY8, 80,
            (0x90), 1, 0x7F,    //Display Control 8 [0A]
            (0x26), 1, 0x04,    //Display Control 1 [A0] D=1
            TFTLCD_DELAY8, 80,
            (0x26), 1, 0x24,    //Display Control 1 [A0] GON=1, D=1
            (0x26), 1, 0x2C,    //Display Control 1 [A0] GON=1, D=3
            TFTLCD_DELAY8, 80,
            (0x26), 1, 0x3C,    //Display Control 1 [A0] GON=1, DTE=1, D=3
            (0x57), 1, 0x02,    //?
            (0x55), 1, 0x00,    //?
            (0x57), 1, 0x00,    //?
        };
        // Atmel ASF code uses VCOM2-3: 0x38, 0x12. 50ms delays and no TEST mode changes.
        init_table(HX8347A_NHD_regValues, sizeof(HX8347A_NHD_regValues));
        //        init_table(HX8347A_CMO32_regValues, sizeof(HX8347A_CMO32_regValues));
        //        init_table(HX8347A_CMO24_regValues, sizeof(HX8347A_CMO24_regValues));
        //        init_table(HX8347A_ITDB02_regValues, sizeof(HX8347A_ITDB02_regValues));
        //        init_table(HX8347G_2_regValues, sizeof(HX8347G_2_regValues));
        break;
#endif

    case 0x8357:                //BIG CHANGE: HX8357-B is now 0x8357
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN;
        goto common_8357;
    case 0x9090:                //BIG CHANGE: HX8357-D was 0x8357
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN | READ_24BITS;
      common_8357:
	  	dummy=1;
        static const uint8_t HX8357C_regValues[]  = {
            TFTLCD_DELAY8, 1,  //dummy table
        };
        table8_ads = HX8357C_regValues, table_size = sizeof(HX8357C_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;

    case 0x0099:                //HX8357-D matches datasheet
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN | READ_24BITS;
        static const uint8_t HX8357_99_regValues[]  = {
            (0xB9), 3, 0xFF, 0x83, 0x57,   // SETEXTC
            TFTLCD_DELAY8, 150,
            TFTLCD_DELAY8, 150,
            (0xB6), 1, 0x25,   // SETCOM [4B 00] -2.5V+37*0.02V=-1.76V [-1.00V]
            (0xC0), 6, 0x50, 0x50, 0x01, 0x3C, 0x1E, 0x08,  // SETSTBA [73 50 00 3C C4 08]
            (0xB4), 7, 0x02, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x78, // SETCYC [02 40 00 2A 2A 0D 96]
#ifdef SUPPORT_8357D_GAMMA
            // HX8357D_SETGAMMA [0B 0C 11 1D 25 37 43 4B 4E 47 41 39 35 31 2E 21 1C 1D 1D 26 31 44 4E 56 44 3F 39 33 31 2E 28 1D E0 01]
            (0xE0),34, 0x02, 0x0A, 0x11, 0x1D, 0x23, 0x35, 0x41, 0x4B, 0x4B, 0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A, 0x11, 0x1D, 0x23, 0x35, 0x41, 0x4B, 0x4B, 0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x00, 0x01,
#endif
        };
        table8_ads = HX8357_99_regValues, table_size = sizeof(HX8357_99_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;

#ifdef SUPPORT_8230
    case 0x8230:    //thanks Auzman
        _lcd_capable = 0 | REV_SCREEN | INVERT_GS | INVERT_RGB | READ_BGR;
        static const uint16_t UC8230_regValues[]  = {
            //After pin Reset wait at least 100ms
            TFTLCD_DELAY, 100, //at least 100ms
            0x0046, 0x0002, //MTP Disable
            0x0010, 0x1590, //SAP=1, BT=5, APE=1, AP=1
            0x0011, 0x0227, //DC1=2, DC0=2, VC=7
            0x0012, 0x80ff, //P5VMD=1, PON=7, VRH=15
            0x0013, 0x9c31, //VDV=28, VCM=49
            TFTLCD_DELAY, 10, //at least 10ms
            0x0002, 0x0300, //set N-line = 1
            0x0003, 0x1030, //set GRAM writing direction & BGR=1
            0x0060, 0xa700, //GS; gate scan: start position & drive line Q'ty
            0x0061, 0x0001, //REV, NDL, VLE
            /*--------------------Gamma control------------------------*/
            0x0030, 0x0303,
            0x0031, 0x0303,
            0x0032, 0x0303,
            0x0033, 0x0300,
            0x0034, 0x0003,
            0x0035, 0x0303,
            0x0036, 0x1400,
            0x0037, 0x0303,
            0x0038, 0x0303,
            0x0039, 0x0303,
            0x003a, 0x0300,
            0x003b, 0x0003,
            0x003c, 0x0303,
            0x003d, 0x1400,
            //-----------------------------------------------------------//
            0x0020, 0x0000, //GRAM horizontal address
            0x0021, 0x0000, //GRAM vertical address
            //************** Partial Display control*********************//
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x0000,
            0x0083, 0x0000,
            0x0084, 0x0000,
            0x0085, 0x0000,
            //-----------------------------------------------------------//
            0x0092, 0x0200,
            0x0093, 0x0303,
            0x0090, 0x0010, //set clocks/Line
            0x0000, 0x0001,
            TFTLCD_DELAY, 200, // Delay 200 ms
            0x0007, 0x0173, //Display on setting
        };
        init_table16(UC8230_regValues, sizeof(UC8230_regValues));
        break;
#endif

#ifdef SUPPORT_9163
    case 0x9163:                //
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t  table9163C[] = {
            //  (COMMAND_BYTE), n, data_bytes....
            0x26, 1, 0x02,       // [01] GAMMASET use CURVE=1, 2, 4, 8
            0xB1, 2, 0x08, 0x02, // [0E 14] FRMCTR1 if GM==011 61.7Hz
            0xB4, 1, 0x07,       // [02] INVCTR
            0xB8, 1, 0x01,       // [00] GSCTRL
            0xC0, 2, 0x0A, 0x02, // [0A 05] PWCTR1 if LCM==10
            0xC1, 1, 0x02,       // [07] PWCTR2
            0xC5, 2, 0x50, 0x63, // [43 4D] VMCTR1
            0xC7, 1, 0,          // [40] VCOMOFFS
        };
        table8_ads = table9163C, table_size = sizeof(table9163C);   //
        p16 = (int16_t *) & height;
        *p16 = 160;
        p16 = (int16_t *) & width;
        *p16 = 128;
        break;
#endif

#ifdef SUPPORT_9225
#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE                (0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1              (0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL                (0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1             (0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2             (0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3             (0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4             (0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5             (0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING           (0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address END Position   HEA
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 (0x37u)  // Horizontal Address START Position HSA
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address END Position     VEA
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address START Position   VSA
#define ILI9225_GAMMA_CTRL1             (0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             (0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             (0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             (0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             (0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             (0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             (0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

#define ILI9225C_INVOFF  0x20
#define ILI9225C_INVON   0x21

    case 0x6813:
    case 0x9226:
        _lcd_ID = 0x9225;                //fall through
    case 0x9225:
        _lcd_capable = REV_SCREEN | READ_BGR;     //thanks tongbajiel
        static const uint16_t ILI9225_regValues[]  = {
            /* Start Initial Sequence */
            /* Set SS bit and direction output from S528 to S1 */
            ILI9225_POWER_CTRL1, 0x0000, // Set SAP,DSTB,STB
            ILI9225_POWER_CTRL2, 0x0000, // Set APON,PON,AON,VCI1EN,VC
            ILI9225_POWER_CTRL3, 0x0000, // Set BT,DC1,DC2,DC3
            ILI9225_POWER_CTRL4, 0x0000, // Set GVDD
            ILI9225_POWER_CTRL5, 0x0000, // Set VCOMH/VCOML voltage
            TFTLCD_DELAY, 40,

            // Power-on sequence
            ILI9225_POWER_CTRL2, 0x0018, // Set APON,PON,AON,VCI1EN,VC
            ILI9225_POWER_CTRL3, 0x6121, // Set BT,DC1,DC2,DC3
            ILI9225_POWER_CTRL4, 0x006F, // Set GVDD   /*007F 0088 */
            ILI9225_POWER_CTRL5, 0x495F, // Set VCOMH/VCOML voltage
            ILI9225_POWER_CTRL1, 0x0800, // Set SAP,DSTB,STB
            TFTLCD_DELAY, 10,
            ILI9225_POWER_CTRL2, 0x103B, // Set APON,PON,AON,VCI1EN,VC
            TFTLCD_DELAY, 50,

            ILI9225_DRIVER_OUTPUT_CTRL, 0x011C, // set the display line number and display direction
            ILI9225_LCD_AC_DRIVING_CTRL, 0x0100, // set 1 line inversion
            ILI9225_ENTRY_MODE, 0x1030, // set GRAM write direction and BGR=1.
            ILI9225_DISP_CTRL1, 0x0000, // Display off
            ILI9225_BLANK_PERIOD_CTRL1, 0x0808, // set the back porch and front porch
            ILI9225_FRAME_CYCLE_CTRL, 0x1100, // set the clocks number per line
            ILI9225_INTERFACE_CTRL, 0x0000, // CPU interface
            ILI9225_OSC_CTRL, 0x0D01, // Set Osc  /*0e01*/
            ILI9225_VCI_RECYCLING, 0x0020, // Set VCI recycling
            ILI9225_RAM_ADDR_SET1, 0x0000, // RAM Address
            ILI9225_RAM_ADDR_SET2, 0x0000, // RAM Address

            /* Set GRAM area */
            ILI9225_GATE_SCAN_CTRL, 0x0000,
            ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB,
            ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000,
            ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000,
            ILI9225_PARTIAL_DRIVING_POS1, 0x00DB,
            ILI9225_PARTIAL_DRIVING_POS2, 0x0000,
            ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF,
            ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000,
            ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB,
            ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000,

            /* Set GAMMA curve */
            ILI9225_GAMMA_CTRL1, 0x0000,
            ILI9225_GAMMA_CTRL2, 0x0808,
            ILI9225_GAMMA_CTRL3, 0x080A,
            ILI9225_GAMMA_CTRL4, 0x000A,
            ILI9225_GAMMA_CTRL5, 0x0A08,
            ILI9225_GAMMA_CTRL6, 0x0808,
            ILI9225_GAMMA_CTRL7, 0x0000,
            ILI9225_GAMMA_CTRL8, 0x0A00,
            ILI9225_GAMMA_CTRL9, 0x0710,
            ILI9225_GAMMA_CTRL10, 0x0710,

            ILI9225_DISP_CTRL1, 0x0012,
            TFTLCD_DELAY, 50,
            ILI9225_DISP_CTRL1, 0x1017,
        };
        init_table16(ILI9225_regValues, sizeof(ILI9225_regValues));
        p16 = (int16_t *) & height;
        *p16 = 220;
        p16 = (int16_t *) & width;
        *p16 = 176;
        break;
#endif

    case 0x0001:
        _lcd_capable = 0 | REV_SCREEN | INVERT_GS; //no RGB bug. thanks Ivo_Deshev
        goto common_9320;
    case 0x5408:
        _lcd_capable = 0 | REV_SCREEN | READ_BGR; //Red 2.4" thanks jorgenv, Ardlab_Gent
//        _lcd_capable = 0 | REV_SCREEN | READ_BGR | INVERT_GS; //Blue 2.8" might be different
        goto common_9320;
    case 0x1505:                //R61505 thanks Ravi_kanchan2004. R61505V, R61505W different
    case 0x9320:
        _lcd_capable = 0 | REV_SCREEN | READ_BGR;
      common_9320:
	    dummy=1;
        static const uint16_t ILI9320_regValues[]  = {
            0x00e5, 0x8000,
            0x0000, 0x0001,
            0x0001, 0x100,
            0x0002, 0x0700,
            0x0003, 0x1030,
            0x0004, 0x0000,
            0x0008, 0x0202,
            0x0009, 0x0000,
            0x000A, 0x0000,
            0x000C, 0x0000,
            0x000D, 0x0000,
            0x000F, 0x0000,
            //-----Power On sequence-----------------------
            0x0010, 0x0000,
            0x0011, 0x0007,
            0x0012, 0x0000,
            0x0013, 0x0000,
            TFTLCD_DELAY, 50,
            0x0010, 0x17B0,  //SAP=1, BT=7, APE=1, AP=3
            0x0011, 0x0007,  //DC1=0, DC0=0, VC=7
            TFTLCD_DELAY, 10,
            0x0012, 0x013A,  //VCMR=1, PON=3, VRH=10
            TFTLCD_DELAY, 10,
            0x0013, 0x1A00,  //VDV=26
            0x0029, 0x000c,  //VCM=12
            TFTLCD_DELAY, 10,
            //-----Gamma control-----------------------
            0x0030, 0x0000,
            0x0031, 0x0505,
            0x0032, 0x0004,
            0x0035, 0x0006,
            0x0036, 0x0707,
            0x0037, 0x0105,
            0x0038, 0x0002,
            0x0039, 0x0707,
            0x003C, 0x0704,
            0x003D, 0x0807,
            //-----Set RAM area-----------------------
            0x0060, 0xA700,     //GS=1
            0x0061, 0x0001,
            0x006A, 0x0000,
            0x0021, 0x0000,
            0x0020, 0x0000,
            //-----Partial Display Control------------
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x0000,
            0x0083, 0x0000,
            0x0084, 0x0000,
            0x0085, 0x0000,
            //-----Panel Control----------------------
            0x0090, 0x0010,
            0x0092, 0x0000,
            0x0093, 0x0003,
            0x0095, 0x0110,
            0x0097, 0x0000,
            0x0098, 0x0000,
            //-----Display on-----------------------
            0x0007, 0x0173,
            TFTLCD_DELAY, 50,
        };
        init_table16(ILI9320_regValues, sizeof(ILI9320_regValues));
        break;
    case 0x6809:
        _lcd_capable = 0 | REV_SCREEN | INVERT_GS | AUTO_READINC;
        goto common_93x5;
    case 0x9328:
    case 0x9325:
        _lcd_capable = 0 | REV_SCREEN | INVERT_GS;
        goto common_93x5;
    case 0x9331:
    case 0x9335:
        _lcd_capable = 0 | REV_SCREEN;
      common_93x5:
	    dummy=1;
        static const uint16_t ILI9325_regValues[]  = {
            0x00E5, 0x78F0,     // set SRAM internal timing
            0x0001, 0x0100,     // set Driver Output Control
            0x0002, 0x0200,     // set 1 line inversion
            0x0003, 0x1030,     // set GRAM write direction and BGR=1.
            0x0004, 0x0000,     // Resize register
            0x0005, 0x0000,     // .kbv 16bits Data Format Selection
            0x0008, 0x0207,     // set the back porch and front porch
            0x0009, 0x0000,     // set non-display area refresh cycle ISC[3:0]
            0x000A, 0x0000,     // FMARK function
            0x000C, 0x0000,     // RGB interface setting
            0x000D, 0x0000,     // Frame marker Position
            0x000F, 0x0000,     // RGB interface polarity
            // ----------- Power On sequence ----------- //
            0x0010, 0x0000,     // SAP, BT[3:0], AP, DSTB, SLP, STB
            0x0011, 0x0007,     // DC1[2:0], DC0[2:0], VC[2:0]
            0x0012, 0x0000,     // VREG1OUT voltage
            0x0013, 0x0000,     // VDV[4:0] for VCOM amplitude
            0x0007, 0x0001,
            TFTLCD_DELAY, 200,  // Dis-charge capacitor power voltage
            0x0010, 0x1690,     // SAP=1, BT=6, APE=1, AP=1, DSTB=0, SLP=0, STB=0
            0x0011, 0x0227,     // DC1=2, DC0=2, VC=7
            TFTLCD_DELAY, 50,   // wait_ms 50ms
            0x0012, 0x000D,     // VCIRE=1, PON=0, VRH=5
            TFTLCD_DELAY, 50,   // wait_ms 50ms
            0x0013, 0x1200,     // VDV=28 for VCOM amplitude
            0x0029, 0x000A,     // VCM=10 for VCOMH
            0x002B, 0x000D,     // Set Frame Rate
            TFTLCD_DELAY, 50,   // wait_ms 50ms
            0x0020, 0x0000,     // GRAM horizontal Address
            0x0021, 0x0000,     // GRAM Vertical Address
            // ----------- Adjust the Gamma Curve ----------//

            0x0030, 0x0000,
            0x0031, 0x0404,
            0x0032, 0x0003,
            0x0035, 0x0405,
            0x0036, 0x0808,
            0x0037, 0x0407,
            0x0038, 0x0303,
            0x0039, 0x0707,
            0x003C, 0x0504,
            0x003D, 0x0808,

            //------------------ Set GRAM area ---------------//
            0x0060, 0x2700,     // Gate Scan Line GS=0 [0xA700]
            0x0061, 0x0001,     // NDL,VLE, REV .kbv
            0x006A, 0x0000,     // set scrolling line
            //-------------- Partial Display Control ---------//
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x0000,
            0x0083, 0x0000,
            0x0084, 0x0000,
            0x0085, 0x0000,
            //-------------- Panel Control -------------------//
            0x0090, 0x0010,
            0x0092, 0x0000,
            0x0007, 0x0133,     // 262K color and display ON
        };
        init_table16(ILI9325_regValues, sizeof(ILI9325_regValues));
        break;

#if defined(SUPPORT_9326_5420)
	case 0x5420:
    case 0x9326:
        _lcd_capable = REV_SCREEN | READ_BGR;
        static const uint16_t ILI9326_CPT28_regValues[]  = {
//************* Start Initial Sequence **********//
         0x0702, 0x3008,     //  Set internal timing, don’t change this value
         0x0705, 0x0036,     //  Set internal timing, don’t change this value
         0x070B, 0x1213,     //  Set internal timing, don’t change this value
         0x0001, 0x0100,     //  set SS and SM bit
         0x0002, 0x0100,     //  set 1 line inversion
         0x0003, 0x1030,     //  set GRAM write direction and BGR=1.
         0x0008, 0x0202,     //  set the back porch and front porch
         0x0009, 0x0000,     //  set non-display area refresh cycle ISC[3:0]
         0x000C, 0x0000,     //  RGB interface setting
         0x000F, 0x0000,     //  RGB interface polarity
//*************Power On sequence ****************//
         0x0100, 0x0000,     //  SAP, BT[3:0], AP, DSTB, SLP, STB
         0x0102, 0x0000,     //  VREG1OUT voltage
         0x0103, 0x0000,   // VDV[4:0] for VCOM amplitude
        TFTLCD_DELAY, 200,   // Dis-charge capacitor power voltage
         0x0100, 0x1190,   // SAP, BT[3:0], AP, DSTB, SLP, STB
         0x0101, 0x0227,   // DC1[2:0], DC0[2:0], VC[2:0]
        TFTLCD_DELAY, 50,   // Delay 50ms
         0x0102, 0x01BD,   // VREG1OUT voltage
        TFTLCD_DELAY, 50,   // Delay 50ms
         0x0103, 0x2D00,   // VDV[4:0] for VCOM amplitude
         0x0281, 0x000E,   // VCM[5:0] for VCOMH
        TFTLCD_DELAY, 50,   //
         0x0200, 0x0000,   // GRAM horizontal Address
         0x0201, 0x0000,   // GRAM Vertical Address
// ----------- Adjust the Gamma Curve ----------//
         0x0300, 0x0000,   //
         0x0301, 0x0707,   //
         0x0302, 0x0606,   //
         0x0305, 0x0000,   //
         0x0306, 0x0D00,   //
         0x0307, 0x0706,   //
         0x0308, 0x0005,   //
         0x0309, 0x0007,   //
         0x030C, 0x0000,   //
         0x030D, 0x000A,   //
//------------------ Set GRAM area ---------------//
         0x0400, 0x3100,     //  Gate Scan Line 400 lines
         0x0401, 0x0001,     //  NDL,VLE, REV
         0x0404, 0x0000,     //  set scrolling line
//-------------- Partial Display Control ---------//
         0x0500, 0x0000,     // Partial Image 1 Display Position
         0x0501, 0x0000,     // Partial Image 1 RAM Start/End Address
         0x0502, 0x0000,     // Partial Image 1 RAM Start/End Address
         0x0503, 0x0000,     // Partial Image 2 Display Position
         0x0504, 0x0000,     // Partial Image 2 RAM Start/End Address
         0x0505, 0x0000,     // Partial Image 2 RAM Start/End Address
//-------------- Panel Control -------------------//
         0x0010, 0x0010,     // DIVI[1:0];RTNI[4:0]
         0x0011, 0x0600,     // NOWI[2:0];SDTI[2:0]
         0x0020, 0x0002,     // DIVE[1:0];RTNE[5:0]
         0x0007, 0x0173,     //  262K color and display ON
		 };
        init_table16(ILI9326_CPT28_regValues, sizeof(ILI9326_CPT28_regValues));
        p16 = (int16_t *) & height;
        *p16 = 400;
        p16 = (int16_t *) & width;
        *p16 = 240;
        break;
#endif

    case 0x9327:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS;
        static const uint8_t ILI9327_regValues[]  = {
            0xB0, 1, 0x00,      //Disable Protect for cmds B1-DF, E0-EF, F0-FF
            //            0xE0, 1, 0x20,      //NV Memory Write [00]
            //            0xD1, 3, 0x00, 0x71, 0x19,  //VCOM control [00 40 0F]
            //            0xD0, 3, 0x07, 0x01, 0x08,  //Power Setting [07 04 8C]
            0xC1, 4, 0x10, 0x10, 0x02, 0x02,    //Display Timing [10 10 02 02]
            0xC0, 6, 0x00, 0x35, 0x00, 0x00, 0x01, 0x02,        //Panel Drive [00 35 00 00 01 02 REV=0,GS=0,SS=0
            0xC5, 1, 0x04,      //Frame Rate [04]
            0xD2, 2, 0x01, 0x04,        //Power Setting [01 44]
            //            0xC8, 15, 0x04, 0x67, 0x35, 0x04, 0x08, 0x06, 0x24, 0x01, 0x37, 0x40, 0x03, 0x10, 0x08, 0x80, 0x00,
            //            0xC8, 15, 0x00, 0x77, 0x77, 0x04, 0x04, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
            0xCA, 1, 0x00,      //DGC LUT ???
            0xEA, 1, 0x80,      //3-Gamma Function Enable
            //                     0xB0, 1, 0x03,      //Enable Protect
        };
        table8_ads = ILI9327_regValues, table_size = sizeof(ILI9327_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 400;
        p16 = (int16_t *) & _width;
        *p16 = 240;
        break;
    case 0x1602:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS; //does not readGRAM
        static const uint8_t XX1602_regValues[]  = {
            0xB8, 1, 0x01,      //GS [00]
            0xC0, 1, 0x0E,      //??Power [0A]
        };
        table8_ads = XX1602_regValues, table_size = sizeof(XX1602_regValues);
        break;

    case 0x2053:    //weird from BangGood
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | REV_SCREEN | READ_BGR;
		goto common_9329;
    case 0xAC11:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | REV_SCREEN; //thanks viliam
		goto common_9329;
    case 0x9302:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS;
		goto common_9329;
    case 0x9338:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
		goto common_9329;
    case 0x9329:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | INVERT_SS | REV_SCREEN;
	  common_9329:
	    dummy=1;
        static const uint8_t ILI9329_regValues[]  = {
//            0xF6, 3, 0x01, 0x01, 0x00,  //Interface Control needs EXTC=1 MX_EOR=1, TM=0, RIM=0
//            0xB6, 3, 0x0A, 0x82, 0x27,  //Display Function [0A 82 27]
//            0xB7, 1, 0x06,      //Entry Mode Set [06]
            0x36, 1, 0x00,      //Memory Access [00] pointless but stops an empty array
        };
        table8_ads = ILI9329_regValues, table_size = sizeof(ILI9329_regValues);
        break;

    case 0x9340:                //ILI9340 thanks Ravi_kanchan2004.
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | REV_SCREEN;
        goto common_9341;
    case 0x9341:
      common_9341:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t ILI9341_regValues_2_4[]  = {        // BOE 2.4"
            0xF6, 3, 0x01, 0x01, 0x00,  //Interface Control needs EXTC=1 MV_EOR=0, TM=0, RIM=0
            0xCF, 3, 0x00, 0x81, 0x30,  //Power Control B [00 81 30]
            0xED, 4, 0x64, 0x03, 0x12, 0x81,    //Power On Seq [55 01 23 01]
            0xE8, 3, 0x85, 0x10, 0x78,  //Driver Timing A [04 11 7A]
            0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,      //Power Control A [39 2C 00 34 02]
            0xF7, 1, 0x20,      //Pump Ratio [10]
            0xEA, 2, 0x00, 0x00,        //Driver Timing B [66 00]
            0xB0, 1, 0x00,      //RGB Signal [00]
            0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
            //            0xB6, 2, 0x0A, 0xA2, 0x27, //Display Function [0A 82 27 XX]    .kbv SS=1
            0xB4, 1, 0x00,      //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
            0xC0, 1, 0x21,      //Power Control 1 [26]
            0xC1, 1, 0x11,      //Power Control 2 [00]
            0xC5, 2, 0x3F, 0x3C,        //VCOM 1 [31 3C]
            0xC7, 1, 0xB5,      //VCOM 2 [C0]
            0x36, 1, 0x48,      //Memory Access [00]
            0xF2, 1, 0x00,      //Enable 3G [02]
            0x26, 1, 0x01,      //Gamma Set [01]
            0xE0, 15, 0x0f, 0x26, 0x24, 0x0b, 0x0e, 0x09, 0x54, 0xa8, 0x46, 0x0c, 0x17, 0x09, 0x0f, 0x07, 0x00,
            0xE1, 15, 0x00, 0x19, 0x1b, 0x04, 0x10, 0x07, 0x2a, 0x47, 0x39, 0x03, 0x06, 0x06, 0x30, 0x38, 0x0f,
        };
        static const uint8_t ILI9341_regValues_ada[]  = {        // Adafruit_TFTLCD only works with EXTC=0
            //                     0xF6, 3, 0x00, 0x01, 0x00,  //Interface Control needs EXTC=1 TM=0, RIM=0
            //            0xF6, 3, 0x01, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=1, RIM=1
            0xF6, 3, 0x09, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=0, RIM=1
            0xB0, 1, 0x40,      //RGB Signal [40] RCM=2
            0xB4, 1, 0x00,      //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
            0xC0, 1, 0x23,      //Power Control 1 [26]
            0xC1, 1, 0x10,      //Power Control 2 [00]
            0xC5, 2, 0x2B, 0x2B,        //VCOM 1 [31 3C]
            0xC7, 1, 0xC0,      //VCOM 2 [C0]
            0x36, 1, 0x88,      //Memory Access [00]
            0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
            0xB7, 1, 0x07,      //Entry Mode [00]
        };
        table8_ads = ILI9341_regValues_2_4, table_size = sizeof(ILI9341_regValues_2_4);   //
        break;
#if defined(SUPPORT_9342)
    case 0x9342:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS | INVERT_GS | REV_SCREEN;
        static const uint8_t ILI9342_regValues_CPT24[]  = {     //CPT 2.4"
            (0xB9), 3, 0xFF, 0x93, 0x42, //[00 00 00]
            (0xC0), 2, 0x1D, 0x0A,    //[26 09]
            (0xC1), 1, 0x02,          //[10]
            (0xC5), 2, 0x2F, 0x2F,    //[31 3C]
            (0xC7), 1, 0xC3,          //[C0]
            (0xB8), 1, 0x0B,          //[07]
            (0xE0), 15, 0x0F, 0x33, 0x30, 0x0C, 0x0F, 0x08, 0x5D, 0x66, 0x4A, 0x07, 0x13, 0x05, 0x1B, 0x0E, 0x08,
            (0xE1), 15, 0x08, 0x0E, 0x11, 0x02, 0x0E, 0x02, 0x24, 0x33, 0x37, 0x03, 0x0A, 0x09, 0x26, 0x33, 0x0F,
        };
        static const uint8_t ILI9342_regValues_Tianma23[]  = {     //Tianma 2.3"
            (0xB9), 3, 0xFF, 0x93, 0x42,
            (0xC0), 2, 0x1D, 0x0A,
            (0xC1), 1, 0x01,
            (0xC5), 2, 0x2C, 0x2C,
            (0xC7), 1, 0xC6,
            (0xB8), 1, 0x09,
            (0xE0), 15, 0x0F, 0x26, 0x21, 0x07, 0x0A, 0x03, 0x4E, 0x62, 0x3E, 0x0B, 0x11, 0x00, 0x08, 0x02, 0x00,
            (0xE1), 15, 0x00, 0x19, 0x1E, 0x03, 0x0E, 0x03, 0x30, 0x23, 0x41, 0x03, 0x0B, 0x07, 0x2F, 0x36, 0x0F,
        };
        static const uint8_t ILI9342_regValues_HSD23[]  = {     //HSD 2.3"
            (0xB9), 3, 0xFF, 0x93, 0x42,
            (0xC0), 2, 0x1D, 0x0A,
            (0xC1), 1, 0x02,
            (0xC5), 2, 0x2F, 0x27,
            (0xC7), 1, 0xA4,
            (0xB8), 1, 0x0B,
            (0xE0), 15, 0x0F, 0x24, 0x21, 0x0C, 0x0F, 0x06, 0x50, 0x75, 0x3F, 0x07, 0x12, 0x05, 0x11, 0x0B, 0x08,
            (0xE1), 15, 0x08, 0x1D, 0x20, 0x02, 0x0E, 0x04, 0x31, 0x24, 0x42, 0x03, 0x0B, 0x09, 0x30, 0x36, 0x0F,
        };
        table8_ads = ILI9342_regValues_CPT24, table_size = sizeof(ILI9342_regValues_CPT24);   //
        //        table8_ads = ILI9342_regValues_Tianma23, table_size = sizeof(ILI9342_regValues_Tianma23);   //
        //        table8_ads = ILI9342_regValues_HSD23, table_size = sizeof(ILI9342_regValues_HSD23);   //
        p16 = (int16_t *) & height;
        *p16 = 240;
        p16 = (int16_t *) & width;
        *p16 = 320;
        break;
#endif
    case 0x1581:                        //no BGR in MADCTL.  set BGR in Panel Control
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS; //thanks zdravke
		goto common_9481;
    case 0x9481:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_BGR;
	  common_9481:
	    dummy=1;
        static const uint8_t ILI9481_regValues[]  = {    // Atmel MaxTouch
            0xB0, 1, 0x00,              // unlocks E0, F0
            0xB3, 4, 0x02, 0x00, 0x00, 0x00, //Frame Memory, interface [02 00 00 00]
			0xB4, 1, 0x00,              // Frame mode [00]
			0xD0, 3, 0x07, 0x42, 0x18,  // Set Power [00 43 18] x1.00, x6, x3
            0xD1, 3, 0x00, 0x07, 0x10,  // Set VCOM  [00 00 00] x0.72, x1.02
            0xD2, 2, 0x01, 0x02,        // Set Power for Normal Mode [01 22]
            0xD3, 2, 0x01, 0x02,        // Set Power for Partial Mode [01 22]
            0xD4, 2, 0x01, 0x02,        // Set Power for Idle Mode [01 22]
            0xC0, 5, 0x12, 0x3B, 0x00, 0x02, 0x11, //Panel Driving BGR for 1581 [10 3B 00 02 11]
            0xC1, 3, 0x10, 0x10, 0x88,  // Display Timing Normal [10 10 88]
			0xC5, 1, 0x03,      //Frame Rate [03]
			0xC6, 1, 0x02,      //Interface Control [02]
            0xC8, 12, 0x00, 0x32, 0x36, 0x45, 0x06, 0x16, 0x37, 0x75, 0x77, 0x54, 0x0C, 0x00,
			0xCC, 1, 0x00,      //Panel Control [00]
        };
        static const uint8_t ILI9481_CPT29_regValues[]  = {    // 320x430
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x42, 0x1C,  // Set Power [00 43 18]
            0xD1, 3, 0x00, 0x02, 0x0F,  // Set VCOM  [00 00 00] x0.695, x1.00
            0xD2, 2, 0x01, 0x11,        // Set Power for Normal Mode [01 22]
            0xC0, 5, 0x10, 0x35, 0x00, 0x02, 0x11,      //Set Panel Driving [10 3B 00 02 11]
            0xC5, 1, 0x03,      //Frame Rate [03]
            0xC8, 12, 0x00, 0x30, 0x36, 0x45, 0x04, 0x16, 0x37, 0x75, 0x77, 0x54, 0x0F, 0x00,
            0xE4, 1, 0xA0,
            0xF0, 1, 0x01,
            0xF3, 2, 0x02, 0x1A,
        };
        static const uint8_t ILI9481_PVI35_regValues[]  = {    // 320x480
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x41, 0x1D,  // Set Power [00 43 18]
            0xD1, 3, 0x00, 0x2B, 0x1F,  // Set VCOM  [00 00 00] x0.900, x1.32
            0xD2, 2, 0x01, 0x11,        // Set Power for Normal Mode [01 22]
            0xC0, 5, 0x10, 0x3B, 0x00, 0x02, 0x11,      //Set Panel Driving [10 3B 00 02 11]
            0xC5, 1, 0x03,      //Frame Rate [03]
            0xC8, 12, 0x00, 0x14, 0x33, 0x10, 0x00, 0x16, 0x44, 0x36, 0x77, 0x00, 0x0F, 0x00,
            0xE4, 1, 0xA0,
            0xF0, 1, 0x01,
            0xF3, 2, 0x40, 0x0A,
        };
        static const uint8_t ILI9481_AUO317_regValues[]  = {    // 320x480
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x40, 0x1D,  // Set Power [00 43 18]
            0xD1, 3, 0x00, 0x18, 0x13,  // Set VCOM  [00 00 00] x0.805, x1.08
            0xD2, 2, 0x01, 0x11,        // Set Power for Normal Mode [01 22]
            0xC0, 5, 0x10, 0x3B, 0x00, 0x02, 0x11,      //Set Panel Driving [10 3B 00 02 11]
            0xC5, 1, 0x03,      //Frame Rate [03]
            0xC8, 12, 0x00, 0x44, 0x06, 0x44, 0x0A, 0x08, 0x17, 0x33, 0x77, 0x44, 0x08, 0x0C,
            0xE4, 1, 0xA0,
            0xF0, 1, 0x01,
        };
        static const uint8_t ILI9481_CMO35_regValues[]  = {    // 320480
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x41, 0x1D,  // Set Power [00 43 18] 07,41,1D
            0xD1, 3, 0x00, 0x1C, 0x1F,  // Set VCOM  [00 00 00] x0.825, x1.32 1C,1F
            0xD2, 2, 0x01, 0x11,        // Set Power for Normal Mode [01 22]
            0xC0, 5, 0x10, 0x3B, 0x00, 0x02, 0x11,      //Set Panel Driving [10 3B 00 02 11]
            0xC5, 1, 0x03,      //Frame Rate [03]
			0xC6, 1, 0x83,
            0xC8, 12, 0x00, 0x26, 0x21, 0x00, 0x00, 0x1F, 0x65, 0x23, 0x77, 0x00, 0x0F, 0x00,
            0xF0, 1, 0x01,		//?
            0xE4, 1, 0xA0,      //?SETCABC on Himax
            0x36, 1, 0x48,      //Memory Access [00]
            0xB4, 1, 0x11,
        };
        static const uint8_t ILI9481_RGB_regValues[]  = {    // 320x480
            0xB0, 1, 0x00,
            0xD0, 3, 0x07, 0x41, 0x1D,  // SETPOWER [00 43 18]
            0xD1, 3, 0x00, 0x2B, 0x1F,  // SETVCOM  [00 00 00] x0.900, x1.32
            0xD2, 2, 0x01, 0x11,        // SETNORPOW for Normal Mode [01 22]
            0xC0, 6, 0x10, 0x3B, 0x00, 0x02, 0x11, 0x00,     //SETPANEL [10 3B 00 02 11]
            0xC5, 1, 0x03,      //SETOSC Frame Rate [03]
            0xC6, 1, 0x80,      //SETRGB interface control
			0xC8, 12, 0x00, 0x14, 0x33, 0x10, 0x00, 0x16, 0x44, 0x36, 0x77, 0x00, 0x0F, 0x00,
            0xF3, 2, 0x40, 0x0A,
            0xF0, 1, 0x08,
            0xF6, 1, 0x84,
            0xF7, 1, 0x80,
            0x0C, 2, 0x00, 0x55, //RDCOLMOD
			0xB4, 1, 0x00,      //SETDISPLAY
//			0xB3, 4, 0x00, 0x01, 0x06, 0x01,  //SETGRAM simple example
			0xB3, 4, 0x00, 0x01, 0x06, 0x30,  //jpegs example
        };
        table8_ads = ILI9481_regValues, table_size = sizeof(ILI9481_regValues);
//        table8_ads = ILI9481_CPT29_regValues, table_size = sizeof(ILI9481_CPT29_regValues);
//        table8_ads = ILI9481_PVI35_regValues, table_size = sizeof(ILI9481_PVI35_regValues);
//        table8_ads = ILI9481_AUO317_regValues, table_size = sizeof(ILI9481_AUO317_regValues);
//        table8_ads = ILI9481_CMO35_regValues, table_size = sizeof(ILI9481_CMO35_regValues);
//        table8_ads = ILI9481_RGB_regValues, table_size = sizeof(ILI9481_RGB_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;
    case 0x9486:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS; //Red 3.5", Blue 3.5"
//        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | REV_SCREEN; //old Red 3.5"
        static const uint8_t ILI9486_regValues[]  = {
/*
            0xF2, 9, 0x1C, 0xA3, 0x32, 0x02, 0xB2, 0x12, 0xFF, 0x12, 0x00,        //f.k
            0xF1, 2, 0x36, 0xA4,        //
            0xF8, 2, 0x21, 0x04,        //
            0xF9, 2, 0x00, 0x08,        //
*/
            0xC0, 2, 0x0d, 0x0d,        //Power Control 1 [0E 0E]
            0xC1, 2, 0x43, 0x00,        //Power Control 2 [43 00]
            0xC2, 1, 0x00,      //Power Control 3 [33]
            0xC5, 4, 0x00, 0x48, 0x00, 0x48,    //VCOM  Control 1 [00 40 00 40]
            0xB4, 1, 0x00,      //Inversion Control [00]
            0xB6, 3, 0x02, 0x02, 0x3B,  // Display Function Control [02 02 3B]
#define GAMMA9486 4
#if GAMMA9486 == 0
            // default GAMMA terrible
#elif GAMMA9486 == 1
            // GAMMA f.k.	bad
            0xE0, 15, 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00,
            0xE1, 15, 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f,
#elif GAMMA9486 == 2
            // 1.2 CPT 3.5 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x1B, 0x18, 0x0B, 0x0E, 0x09, 0x47, 0x94, 0x35, 0x0A, 0x13, 0x05, 0x08, 0x03, 0x00,
			0xE1, 15, 0x0F, 0x3A, 0x37, 0x0B, 0x0C, 0x05, 0x4A, 0x24, 0x39, 0x07, 0x10, 0x04, 0x27, 0x25, 0x00,
#elif GAMMA9486 == 3
            // 2.2 HSD 3.5 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00,
			0xE1, 15, 0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
#elif GAMMA9486 == 4
            // 3.2 TM  3.2 Inch Initial Code not bad
			0xE0, 15, 0x0F, 0x21, 0x1C, 0x0B, 0x0E, 0x08, 0x49, 0x98, 0x38, 0x09, 0x11, 0x03, 0x14, 0x10, 0x00,
			0xE1, 15, 0x0F, 0x2F, 0x2B, 0x0C, 0x0E, 0x06, 0x47, 0x76, 0x37, 0x07, 0x11, 0x04, 0x23, 0x1E, 0x00,
#elif GAMMA9486 == 5
            // 4.2 WTK 3.5 Inch Initial Code too white
			0xE0, 15, 0x0F, 0x10, 0x08, 0x05, 0x09, 0x05, 0x37, 0x98, 0x26, 0x07, 0x0F, 0x02, 0x09, 0x07, 0x00,
			0xE1, 15, 0x0F, 0x38, 0x36, 0x0D, 0x10, 0x08, 0x59, 0x76, 0x48, 0x0A, 0x16, 0x0A, 0x37, 0x2F, 0x00,
#endif
        };
        table8_ads = ILI9486_regValues, table_size = sizeof(ILI9486_regValues);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;
    case 0x7796:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS;   //thanks to safari1
        goto common_9488;
    case 0x9487:                //with thanks to Charlyf
    case 0x9488:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
      common_9488:
	    dummy=1;
        static const uint8_t ILI9488_regValues_max[]  = {        // Atmel MaxTouch
            0xC0, 2, 0x10, 0x10,        //Power Control 1 [0E 0E]
            0xC1, 1, 0x41,      //Power Control 2 [43]
            0xC5, 4, 0x00, 0x22, 0x80, 0x40,    //VCOM  Control 1 [00 40 00 40]
            0x36, 1, 0x68,      //Memory Access [00]
            0xB0, 1, 0x00,      //Interface     [00]
            0xB1, 2, 0xB0, 0x11,        //Frame Rate Control [B0 11]
            0xB4, 1, 0x02,      //Inversion Control [02]
            0xB6, 3, 0x02, 0x02, 0x3B,  // Display Function Control [02 02 3B] .kbv NL=480
            0xB7, 1, 0xC6,      //Entry Mode      [06]
            0x3A, 1, 0x55,      //Interlace Pixel Format [XX]
            0xF7, 4, 0xA9, 0x51, 0x2C, 0x82,    //Adjustment Control 3 [A9 51 2C 82]
        };
        table8_ads = ILI9488_regValues_max, table_size = sizeof(ILI9488_regValues_max);
        p16 = (int16_t *) & _height;
        *p16 = 480;
        p16 = (int16_t *) & _width;
        *p16 = 320;
        break;
    case 0xB505:                //R61505V
    case 0xC505:                //R61505W
        _lcd_capable = 0 | REV_SCREEN | READ_LOWHIGH;
        static const uint16_t R61505V_regValues[]  = {
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0001,
            0x00A4, 0x0001,     //CALB=1
            TFTLCD_DELAY, 10,
            0x0060, 0x2700,     //NL
            0x0008, 0x0808,     //FP & BP
            0x0030, 0x0214,     //Gamma settings
            0x0031, 0x3715,
            0x0032, 0x0604,
            0x0033, 0x0E16,
            0x0034, 0x2211,
            0x0035, 0x1500,
            0x0036, 0x8507,
            0x0037, 0x1407,
            0x0038, 0x1403,
            0x0039, 0x0020,
            0x0090, 0x0015,     //DIVI & RTNI
            0x0010, 0x0410,     //BT=4,AP=1
            0x0011, 0x0237,     //DC1=2,DC0=3, VC=7
            0x0029, 0x0046,     //VCM1=70
            0x002A, 0x0046,     //VCMSEL=0,VCM2=70
            // Sleep mode IN sequence
            0x0007, 0x0000,
            //0x0012, 0x0000,   //PSON=0,PON=0
            // Sleep mode EXIT sequence
            0x0012, 0x0189,     //VCMR=1,PSON=0,PON=0,VRH=9
            0x0013, 0x1100,     //VDV=17
            TFTLCD_DELAY, 150,
            0x0012, 0x01B9,     //VCMR=1,PSON=1,PON=1,VRH=9 [018F]
            0x0001, 0x0100,     //SS=1 Other mode settings
            0x0002, 0x0200,     //BC0=1--Line inversion
            0x0003, 0x1030,
            0x0009, 0x0001,     //ISC=1 [0000]
            0x000A, 0x0000,     // [0000]
            //            0x000C, 0x0001,   //RIM=1 [0000]
            0x000D, 0x0000,     // [0000]
            0x000E, 0x0030,     //VEM=3 VCOM equalize [0000]
            0x0061, 0x0001,
            0x006A, 0x0000,
            0x0080, 0x0000,
            0x0081, 0x0000,
            0x0082, 0x005F,
            0x0092, 0x0100,
            0x0093, 0x0701,
            TFTLCD_DELAY, 80,
            0x0007, 0x0100,     //BASEE=1--Display On
        };
        init_table16(R61505V_regValues, sizeof(R61505V_regValues));
        break;

#if defined(SUPPORT_B509_7793)
    case 0x7793:
    case 0xB509:
        _lcd_capable = REV_SCREEN;
        static const uint16_t R61509V_regValues[]  = {
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            TFTLCD_DELAY, 15,
            0x0400, 0x6200,     //NL=0x31 (49) i.e. 400 rows
            0x0008, 0x0808,
            //gamma
            0x0300, 0x0C00,
            0x0301, 0x5A0B,
            0x0302, 0x0906,
            0x0303, 0x1017,
            0x0304, 0x2300,
            0x0305, 0x1700,
            0x0306, 0x6309,
            0x0307, 0x0C09,
            0x0308, 0x100C,
            0x0309, 0x2232,

            0x0010, 0x0016,     //69.5Hz         0016
            0x0011, 0x0101,
            0x0012, 0x0000,
            0x0013, 0x0001,

            0x0100, 0x0330,     //BT,AP
            0x0101, 0x0237,     //DC0,DC1,VC
            0x0103, 0x0D00,     //VDV
            0x0280, 0x6100,     //VCM
            0x0102, 0xC1B0,     //VRH,VCMR,PSON,PON
            TFTLCD_DELAY, 50,

            0x0001, 0x0100,
            0x0002, 0x0100,
            0x0003, 0x1030,     //1030
            0x0009, 0x0001,
            0x000C, 0x0000,
            0x0090, 0x8000,
            0x000F, 0x0000,

            0x0210, 0x0000,
            0x0211, 0x00EF,
            0x0212, 0x0000,
            0x0213, 0x018F,     //432=01AF,400=018F
            0x0500, 0x0000,
            0x0501, 0x0000,
            0x0502, 0x005F,     //???
            0x0401, 0x0001,     //REV=1
            0x0404, 0x0000,
            TFTLCD_DELAY, 50,

            0x0007, 0x0100,     //BASEE
            TFTLCD_DELAY, 50,

            0x0200, 0x0000,
            0x0201, 0x0000,
        };
        init_table16(R61509V_regValues, sizeof(R61509V_regValues));
        p16 = (int16_t *) & _height;
        *p16 = 400;
        break;
#endif

#ifdef SUPPORT_9806
    case 0x9806:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        // from ZinggJM
        static const uint8_t ILI9806_regValues[]  = {
            (0xFF), 3, /* EXTC Command Set enable register*/ 0xFF, 0x98, 0x06,
            (0xBA), 1, /* SPI Interface Setting*/0xE0,
            (0xBC), 21, /* GIP 1*/0x03, 0x0F, 0x63, 0x69, 0x01, 0x01, 0x1B, 0x11, 0x70, 0x73, 0xFF, 0xFF, 0x08, 0x09, 0x05, 0x00, 0xEE, 0xE2, 0x01, 0x00, 0xC1,
            (0xBD), 8, /* GIP 2*/0x01, 0x23, 0x45, 0x67, 0x01, 0x23, 0x45, 0x67,
            (0xBE), 9, /* GIP 3*/0x00, 0x22, 0x27, 0x6A, 0xBC, 0xD8, 0x92, 0x22, 0x22,
            (0xC7), 1, /* Vcom*/0x1E,
            (0xED), 3, /* EN_volt_reg*/0x7F, 0x0F, 0x00,
            (0xC0), 3, /* Power Control 1*/0xE3, 0x0B, 0x00,
            (0xFC), 1, 0x08,
            (0xDF), 6, /* Engineering Setting*/0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
            (0xF3), 1, /* DVDD Voltage Setting*/0x74,
            (0xB4), 3, /* Display Inversion Control*/0x00, 0x00, 0x00,
            (0xF7), 1, /* 480x854*/0x81,
            (0xB1), 3, /* Frame Rate*/0x00, 0x10, 0x14,
            (0xF1), 3, /* Panel Timing Control*/0x29, 0x8A, 0x07,
            (0xF2), 4, /*Panel Timing Control*/0x40, 0xD2, 0x50, 0x28,
            (0xC1), 4, /* Power Control 2*/0x17, 0x85, 0x85, 0x20,
            (0xE0), 16, 0x00, 0x0C, 0x15, 0x0D, 0x0F, 0x0C, 0x07, 0x05, 0x07, 0x0B, 0x10, 0x10, 0x0D, 0x17, 0x0F, 0x00,
            (0xE1), 16, 0x00, 0x0D, 0x15, 0x0E, 0x10, 0x0D, 0x08, 0x06, 0x07, 0x0C, 0x11, 0x11, 0x0E, 0x17, 0x0F, 0x00,
            (0x35), 1, /*Tearing Effect ON*/0x00,
        };
        table8_ads = ILI9806_regValues, table_size = sizeof(ILI9806_regValues);
        p16 = (int16_t *) & height;
        *p16 = 480;
        p16 = (int16_t *) & width;
        *p16 = 854;
        break;
#endif
    default:
        p16 = (int16_t *) & _width;
        *p16 = 0;       //error value for width
        break;
    }
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
#if defined(SUPPORT_9488_555)
    if (_lcd_ID == 0x9488) {
		is555 = 0;
		drawPixel(0, 0, 0xFFE0);
		if (ReadPixel(0, 0) == 0xFF1F) {
			uint8_t pixfmt = 0x06;
			pushCommand(0x3A, &pixfmt, 1);
			_lcd_capable &= ~READ_24BITS;
			is555 = 1;
		}
	}
#endif
}
