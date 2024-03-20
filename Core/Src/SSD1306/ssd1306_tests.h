#ifndef __SSD1306_TEST_H__
#define __SSD1306_TEST_H__

#include <_ansi.h>

_BEGIN_STD_C

void ssd1306_TestBorder(I2C_HandleTypeDef *hi2c);
void ssd1306_TestFonts1(I2C_HandleTypeDef *hi2c);
void ssd1306_TestFonts2(I2C_HandleTypeDef *hi2c);
void ssd1306_TestFPS(I2C_HandleTypeDef *hi2c);
void ssd1306_TestAll(I2C_HandleTypeDef *hi2c);
void ssd1306_TestLine(I2C_HandleTypeDef *hi2c);
void ssd1306_TestRectangle(I2C_HandleTypeDef *hi2cid);
void ssd1306_TestRectangleFill(I2C_HandleTypeDef *hi2c);
void ssd1306_TestCircle(I2C_HandleTypeDef *hi2c);
void ssd1306_TestArc(I2C_HandleTypeDef *hi2c);
void ssd1306_TestPolyline(I2C_HandleTypeDef *hi2c);
void ssd1306_TestDrawBitmap(I2C_HandleTypeDef *hi2c);
void ssd1306_info(I2C_HandleTypeDef *hi2c, int acc_x, int acc_y, int acc_z, int gyz_x, int gyz_y, int gyz_z, float tem, float gas);
void ssd1306_wait(I2C_HandleTypeDef *hi2c);

_END_STD_C

#endif // __SSD1306_TEST_H__
