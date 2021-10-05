/*
 * Setting.h
 *
 *  Created on: Sep 10, 2021
 *      Author: A315-56
 */

#ifndef INC_SETTING_H_
#define INC_SETTING_H_

#include "stdint.h"

#define DATA_PORT GPIOB
#define CMD_PORT  GPIOA

#define RD_PIN     GPIO_PIN_3
#define WR_PIN     GPIO_PIN_2
#define CD_PIN     GPIO_PIN_1
#define CS_PIN     GPIO_PIN_0
#define RESET_PIN  GPIO_PIN_4

#define RD_N  3
#define WR_N  2
#define CD_N  1
#define CS_N  0
#define RESET_N 4

//#define D0_PIN GPIO_PIN_0
//#define D1_PIN GPIO_PIN_1
//#define D2_PIN GPIO_PIN_2
//#define D3_PIN GPIO_PIN_3
//#define D4_PIN GPIO_PIN_4
//#define D5_PIN GPIO_PIN_5
//#define D6_PIN GPIO_PIN_6
//#define D7_PIN GPIO_PIN_7
//
//
//#define D0_N 0
//#define D1_N 1
//#define D2_N 2
//#define D3_N 3
//#define D4_N 4
//#define D5_N 5
//#define D6_N 6
//#define D7_N 7
//
///****************** delay in microseconds ***********************/
//
//inline void write_8(uint8_t d) { \
//   DATA_PORT->BSRR = 0x00ff << 16; \
//   DATA_PORT->BSRR = (((d) & (1<<0))) \
//				   | (((d) & (1<<1))) \
//				   | (((d) & (1<<2))) \
//				   | (((d) & (1<<3))) \
//				   | (((d) & (1<<4))) \
//				   | (((d) & (1<<5))) \
//				   | (((d) & (1<<6))) \
//				   | (((d) & (1<<7))); \
//    }
//
//uint8_t read_8() {
//	 return          ((DATA_PORT->IDR & (1<<0))) \
//				   | ((DATA_PORT->IDR & (1<<1))) \
//				   | ((DATA_PORT->IDR & (1<<2))) \
//				   | ((DATA_PORT->IDR & (1<<3))) \
//				   | ((DATA_PORT->IDR & (1<<4))) \
//				   | ((DATA_PORT->IDR & (1<<5))) \
//				   | ((DATA_PORT->IDR & (1<<6))) \
//				   | ((DATA_PORT->IDR & (1<<7)));
//}
#define D0_PIN GPIO_PIN_7
#define D1_PIN GPIO_PIN_6
#define D2_PIN GPIO_PIN_5
#define D3_PIN GPIO_PIN_4
#define D4_PIN GPIO_PIN_3
#define D5_PIN GPIO_PIN_2
#define D6_PIN GPIO_PIN_1
#define D7_PIN GPIO_PIN_0


#define D0_N 7
#define D1_N 6
#define D2_N 5
#define D3_N 4
#define D4_N 3
#define D5_N 2
#define D6_N 1
#define D7_N 0

/****************** delay in microseconds ***********************/

inline void write_8(uint8_t d) { \
   GPIOB->BSRR = 0x00ff << 16; \
   GPIOB->BSRR = (((d) & (1<<0)) << 7) \
               | (((d) & (1<<1)) << 5) \
			   | (((d) & (1<<2)) << 3) \
			   | (((d) & (1<<3)) << 1) \
			   | (((d) & (1<<4)) >> 1) \
			   | (((d) & (1<<5)) >> 3) \
			   | (((d) & (1<<6)) >> 5) \
			   | (((d) & (1<<7)) >> 7); \
    }

uint8_t read_8() {
	 return          ((GPIOB->IDR & (1<<0)) << 7) \
				   | ((GPIOB->IDR & (1<<1)) << 5) \
				   | ((GPIOB->IDR & (1<<2)) << 3) \
				   | ((GPIOB->IDR & (1<<3)) << 1) \
				   | ((GPIOB->IDR & (1<<4)) >> 1) \
				   | ((GPIOB->IDR & (1<<5)) >> 3) \
				   | ((GPIOB->IDR & (1<<6)) >> 5) \
				   | ((GPIOB->IDR & (1<<7)) >> 7);
}

/********************* For 180 MHz *****************************/
//#define WRITE_DELAY { WR_ACTIVE8; }
//#define READ_DELAY  { RD_ACTIVE16;}
/************************** For 72 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { RD_ACTIVE;  }
/************************** For 84 MHZ ****************************/
#define WRITE_DELAY {}
#define READ_DELAY  { RD_ACTIVE4; }
/************************** For 100 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE2; }
//#define READ_DELAY  { RD_ACTIVE4; }
/************************** For 216 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE8; WR_ACTIVE8; } //216MHz
//#define IDLE_DELAY  { WR_IDLE4;WR_IDLE4; }
//#define READ_DELAY  { RD_ACTIVE16;RD_ACTIVE16;RD_ACTIVE16;}
/************************** For 48 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { }
/*****************************  DEFINES FOR DIFFERENT TFTs   ****************************************************/
#define Color_555
#define SUPPORT_05A1                //for S6D05A1
//#define SUPPORT_0139              //S6D0139 +280 bytes
#define SUPPORT_0154                //S6D0154 +320 bytes
//#define SUPPORT_1289              //SSD1289,SSD1297 (ID=0x9797) +626 bytes, 0.03s
#define SUPPORT_1580              //R61580 Untested
//#define SUPPORT_1963              //only works with 16BIT bus anyway
//#define SUPPORT_4532              //LGDP4532 +120 bytes.  thanks Leodino
#define SUPPORT_4535                //LGDP4535 +180 bytes
#define SUPPORT_68140               //RM68140 +52 bytes defaults to PIXFMT=0x55
//#define SUPPORT_7735
#define SUPPORT_7781                //ST7781 +172 bytes
//#define SUPPORT_8230              //UC8230 +118 bytes
//#define SUPPORT_8347D             //HX8347-D, HX8347-G, HX8347-I, HX8367-A +520 bytes, 0.27s
//#define SUPPORT_8347A             //HX8347-A +500 bytes, 0.27s
//#define SUPPORT_8352A             //HX8352A +486 bytes, 0.27s
//#define SUPPORT_8352B             //HX8352B
//#define SUPPORT_8357D_GAMMA       //monster 34 byte
//#define SUPPORT_9163              //
//#define SUPPORT_9225              //ILI9225-B, ILI9225-G ID=0x9225, ID=0x9226, ID=0x6813 +380 bytes
//#define SUPPORT_9326_5420         //ILI9326, SPFD5420 +246 bytes
//#define SUPPORT_9342              //costs +114 bytes
//#define SUPPORT_9806              //UNTESTED
#define SUPPORT_9488_555            //costs +230 bytes, 0.03s / 0.19s
#define SUPPORT_B509_7793           //R61509, ST7793 +244 bytes
#define OFFSET_9327 32              //costs about 103 bytes, 0.08s


#endif /* INC_SETTING_H_ */
