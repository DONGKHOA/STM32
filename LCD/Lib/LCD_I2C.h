#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "main.h"
#include "stdio.h"


// commands
#define	LCD_CLEARDISPLAY				0x01
#define LCD_DISPLAYCONTROL 				0x08
#define LCD_ENTRYMODESET 				0x06
#define LCD_FUNCTIONSET 				0x28

// display
#define LCD_CURSORLLINE					0x80


// flags for display on/off control
#define LCD_DISPLAYON					0x04
#define LCD_DISPLAYOFF 					0x00
#define LCD_CURSORON 					0x02
#define LCD_CURSOROFF 					0x00
#define LCD_BLINKON 					0x01
#define LCD_BLINKOFF 					0x00

// flags for backlight control
#define LCD_BACKLIGHT 					0x08
#define LCD_NOBACKLIGHT 				0x00

#define En 								0b00000100  // Enable bit
#define Rw 								0b00000010  // Read/Write bit
#define Rs 								0b00000001  // Register select bit

typedef struct
{
	I2C_HandleTypeDef* hi2c;
	uint8_t LCD_Columns;
	uint8_t LCD_Rows;
	uint8_t Slave_address;
	uint8_t LCD_Backlight_val;
	uint8_t LCD_Display_Option;
} LCD_I2C_HandleTypeDef;

extern uint8_t p_data[4];
extern char p_data_H, p_data_L;

// initialization LCD
void LCD_I2C_Init(LCD_I2C_HandleTypeDef *p_LCD, I2C_HandleTypeDef *p_hi2c, uint8_t p_col, uint8_t p_row, uint8_t p_Slave_Address);
void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_LCD, uint8_t p_col, uint8_t p_row);	// set cursor in the display


void LCD_Send_cmd(LCD_I2C_HandleTypeDef *p_LCD, char p_cmd);	// send commands to the LCD
void LCD_Send_data(LCD_I2C_HandleTypeDef *p_LCD, char p_data);	// send data to the LCD
void LCD_Send_String (LCD_I2C_HandleTypeDef *p_LCD, char *str);	// send string to the LCD
void LCD_Set_Clear(LCD_I2C_HandleTypeDef *p_LCD);
#endif
