#include "lcd_lv_driver.h"
#include "TFT_8Bit.h"
#include "stdlib.h"

static lv_disp_drv_t *LastDriver;
uint16_t tbuf[100];
extern TFT_HandleTypeDef lcd;

void LCD_flush(lv_disp_drv_t * drv, const lv_area_t * area,  lv_color_t * color_map){
		LastDriver=drv;
		lcd.setAddrWindow((uint16_t)area->x1, (uint16_t)area->y1, (uint16_t)area->x2, (uint16_t)area->y2);
		lcd.Enable();
		lcd.WriteEnable();
		uint32_t size = (area->x2 - area->x1 +1) * (area->y2 - area->y1 +1);
		lcd.DataWriteS((uint16_t *)color_map, size);
		lcd.Disable();
		lcd.setAddrWindow(0, 0, lcd.width(), lcd.height());
		lv_disp_flush_ready(LastDriver);
}






