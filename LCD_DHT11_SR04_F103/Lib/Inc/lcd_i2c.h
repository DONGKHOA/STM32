#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "main.h"
#include "stdio.h"
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t LCD_Columns;
    uint8_t LCD_Rows;
    uint8_t Slave_address;
    uint8_t LCD_Backlight_val;
    uint8_t LCD_Display_Option;
} LCD_I2C_HandleTypeDef;

// initialization LCD
void LCD_I2C_Init(LCD_I2C_HandleTypeDef *p_LCD, I2C_HandleTypeDef *p_hi2c, uint8_t p_col, uint8_t p_row, uint8_t p_Slave_Address);

void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_LCD, uint8_t p_col, uint8_t p_row); // set cursor in the display

void LCD_Send_cmd(LCD_I2C_HandleTypeDef *p_LCD, char p_cmd);   // send commands to the LCD
void LCD_Send_data(LCD_I2C_HandleTypeDef *p_LCD, char p_data); // send data to the LCD
void LCD_Send_String(LCD_I2C_HandleTypeDef *p_LCD, char *str); // send string to the LCD
void LCD_Set_Clear(LCD_I2C_HandleTypeDef *p_LCD);
#endif /* LCD_I2C_H_ */
