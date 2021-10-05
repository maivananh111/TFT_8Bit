/*
 * TFT_8Bit.h
 *
 *  Created on: Sep 10, 2021
 *      Author: A315-56
 */

#ifndef INC_TFT_8BIT_H_
#define INC_TFT_8BIT_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "Font.h"

#define USE_GUI
#define USE_PRINT
#define DRAW_BMP
//#define USE_EXTERNAL_FLASH
//#define USE_SD_CARD

#ifdef USE_EXTERNAL_FLASH
  #include "W25Qxx.h"
#endif
#ifdef USE_SD_CARD
  #include "fatfs.h"
#endif

class TFT_HandleTypeDef {
  public:
	TFT_HandleTypeDef(uint16_t x, uint16_t y, TIM_HandleTypeDef *TFT_Timer);
	uint16_t ReadID(void);
	uint16_t width(void);
	uint16_t height(void);
	uint8_t  getRotation (void);

	void Init(uint16_t ID);
	void DataWrite(uint16_t data);
	void DataWriteS(uint16_t *data, uint32_t size);
	void WriteEnable(void);
	void Enable(void);
	void Disable(void);
	void setRotation(uint8_t r);
	void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
	void ReadGRAM(int16_t x, int16_t y, uint16_t block[], int16_t w, int16_t h);
	void vertScroll(int16_t top, int16_t scrollines, int16_t offset);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
    uint16_t ReadPixel(int16_t x, int16_t y);
	void invertDisplay(uint8_t i);

	#ifdef USE_GUI

	    uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
		void pushColors(uint16_t * block, int16_t n, uint8_t first);
		void pushColors(uint8_t * block, int16_t n, uint8_t first);
		void pushColors(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend);
		void Clear(uint16_t color);
		void fillScreen(uint16_t color);
		void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
		void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
		void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
		void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
		void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
		void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
		void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
		void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
		void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
		void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
		void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
		void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
		void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
		void scrollup (uint16_t speed);
		void scrolldown (uint16_t speed);
	#endif
	#ifdef DRAW_BMP
			void drawRGBBitmap(uint16_t x, uint16_t y, const uint16_t* bitmap, uint16_t w, uint16_t h);
			void drawBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, bool back, uint16_t color, uint16_t B_color, uint16_t w, uint16_t h);
	#endif
	#ifdef USE_PRINT
		void drawSmallBitmap(uint16_t x, uint16_t y, const uint8_t* bitmap, bool back, uint16_t color, uint16_t B_color, uint16_t w, uint16_t h);
		void drawChar(uint16_t x, uint16_t y, const G_Font *font, bool back, uint16_t color, uint16_t B_color, char Char);
		void print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, char *str);
		void print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, float Num);
		void print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, long Num);
		void draw_Viet_Char(uint16_t x, uint16_t y, const G_Font *font, bool back, uint16_t color, uint16_t B_color, uint8_t Num);
		void Viet_print(uint16_t x, uint16_t y, const G_Font *font, bool back, int16_t N_Color, int16_t B_Color, char *str);
	#endif
	#ifdef USE_EXTERNAL_FLASH
		void drawRGBbitmapInFlash(W25Q w25q, uint32_t Sector_OffSet_Addr, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
		#ifdef USE_SD_CARD
			void CopyBmpFileToFlash(W25Q w25q, char *s, uint32_t Sector_OffSet_Addr, uint16_t s_x,uint16_t s_y, uint16_t Width, uint16_t Height, uint16_t file_Width, uint16_t file_Height);
			void drawBitmapFile(char *s, uint16_t x,uint16_t y, uint16_t s_x,uint16_t s_y, uint16_t Width, uint16_t Height, uint16_t file_Width, uint16_t file_Height);
		#endif
	#endif

  private:
	#ifdef USE_PRINT
		uint8_t UTF8_GetAddr(char *utf8_char, char *char_offset);
		void  writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
		void  WriteByteBack(uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, uint8_t Char);
		void  WriteByteNoBack(uint16_t x, uint16_t y, uint16_t color, uint8_t Char);
		void  WriteNByteBack(uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, uint8_t Char);
		void  WriteNByteNoBack(uint16_t x, uint16_t y, uint16_t color, uint8_t Char);
		void  WriteCharBack(uint8_t numWrite, uint16_t x, uint16_t y, uint16_t color, uint16_t B_color, const uint8_t *font, uint16_t num);
		void  WriteCharNoBack(uint8_t numWrite, uint16_t x, uint16_t y, uint16_t color, const uint8_t *font, uint16_t num);
	#endif
};
#endif /* INC_TFT_8BIT_H_ */
