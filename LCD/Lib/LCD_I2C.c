#include "LCD_I2C.h"

void LCD_Send_cmd(LCD_I2C_HandleTypeDef *p_LCD, char p_cmd)
{
	char p_data_H, p_data_L;
	uint8_t p_data[4];
	p_data_H = (p_cmd & 0xF0);
	p_data_L = (((p_cmd << 4) & 0xF0));

	p_data[0] = p_data_H |0x0C;
	p_data[1] = p_data_H |0X08;
	p_data[2] = p_data_L |0X0C;
	p_data[3] = p_data_L |0X08;

	HAL_I2C_Master_Transmit(p_LCD->hi2c, p_LCD->Slave_address,(uint8_t *) p_data, 4, 100);
}
void LCD_Send_data(LCD_I2C_HandleTypeDef *p_LCD, char p_data)
{
	char p_data_H, p_data_L;
	uint8_t p_data_Buf[4];
	p_data_H = (p_data & 0xF0);
	p_data_L = (((p_data << 4) & 0xF0));

	p_data_Buf[0] = p_data_H |0X0D;
	p_data_Buf[1] = p_data_H |0X09;
	p_data_Buf[2] = p_data_L |0X0D;
	p_data_Buf[3] = p_data_L |0X09;

	HAL_I2C_Master_Transmit(p_LCD->hi2c, p_LCD->Slave_address,(uint8_t *) p_data_Buf, 4, 100);
}
void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_LCD, uint8_t p_col, uint8_t p_row)
{
	uint8_t t_row_Offets[] = {0x00, 0x40, 0x14, 0x54};
	if(p_row > p_LCD->LCD_Rows) p_row = p_LCD->LCD_Rows - 1;
	LCD_Send_cmd(p_LCD, LCD_CURSORLLINE | (p_col + t_row_Offets[p_row]));
}
void LCD_I2C_Init(LCD_I2C_HandleTypeDef *p_LCD, I2C_HandleTypeDef *p_hi2c, uint8_t p_col, uint8_t p_row, uint8_t p_Slave_Address)
{
	p_LCD->Slave_address = p_Slave_Address;
	p_LCD->LCD_Backlight_val = LCD_BACKLIGHT;
	p_LCD->LCD_Columns = p_col;
	p_LCD->LCD_Rows = p_row;
	p_LCD->hi2c = p_hi2c;
	p_LCD->LCD_Display_Option = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;


	//4 bit initialization
	HAL_Delay(50); 	//waiting for > 40ms
	LCD_Send_cmd(p_LCD, 0x30);
	HAL_Delay(5);  // waiting for >4.1ms
	LCD_Send_cmd(p_LCD, 0x30);
	HAL_Delay(1);  // waiting for >100us
	LCD_Send_cmd(p_LCD, 0x30);
	HAL_Delay(10);
	LCD_Send_cmd(p_LCD, 0x20);	//4 bit mode
	HAL_Delay(10);

	 //display initialization
	LCD_Send_cmd(p_LCD, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_cmd(p_LCD, 0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off		HAL_Delay(1);
	HAL_Delay(1);
	LCD_Send_cmd(p_LCD, 0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LCD_Send_cmd(p_LCD, 0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_cmd(p_LCD, LCD_DISPLAYCONTROL | p_LCD->LCD_Display_Option); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

}

void LCD_Send_String (LCD_I2C_HandleTypeDef *p_LCD, char *str)
{
	while (*str)LCD_Send_data(p_LCD, *str++);
}
void LCD_Set_Clear(LCD_I2C_HandleTypeDef *p_LCD)
{
	LCD_Send_cmd(p_LCD, LCD_CLEARDISPLAY);
	HAL_Delay(2);
	LCD_Set_Cursor(p_LCD, 0, 0);
}
