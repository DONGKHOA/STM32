#include "lcd_i2c.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_DISPLAYCONTROL 0x08
#define LCD_ENTRYMODESET 0x06
#define LCD_FUNCTIONSET 0x28

// display
#define LCD_CURSORLLINE 0x80

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100 // Enable bit
#define Rw 0b00000010 // Read/Write bit
#define Rs 0b00000001 // Register select bit

/**
 * The function takes a command byte and splits it into two nibbles, then sends the nibbles to the LCD
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 * @param p_cmd The command to be sent to the LCD.
 */
void LCD_Send_cmd(LCD_I2C_HandleTypeDef *p_LCD, char p_cmd)
{
    char p_data_H, p_data_L;
    uint8_t p_data[4];
    p_data_H = (p_cmd & 0xF0);
    p_data_L = (((p_cmd << 4) & 0xF0));

    p_data[0] = p_data_H | 0x0C;
    p_data[1] = p_data_H | 0X08;
    p_data[2] = p_data_L | 0X0C;
    p_data[3] = p_data_L | 0X08;

    HAL_I2C_Master_Transmit(p_LCD->hi2c, p_LCD->Slave_address, (uint8_t *)p_data, 4, 100);
}
/**
 * The function takes a pointer to a struct that contains a pointer to an I2C handle and an I2C slave
 * address. The function then takes a character and splits it into two 4-bit nibbles, then sends the
 * nibbles to the LCD
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 * @param p_data The data to be sent to the LCD.
 */
/**
 * The function takes a pointer to a struct that contains a pointer to an I2C handle and an I2C slave
 * address. The function then takes a character and splits it into two 4-bit nibbles, then sends the
 * nibbles to the LCD
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 * @param p_data The data to be sent to the LCD.
 */
void LCD_Send_data(LCD_I2C_HandleTypeDef *p_LCD, char p_data)
{
    char p_data_H, p_data_L;
    uint8_t p_data_Buf[4];
    p_data_H = (p_data & 0xF0);
    p_data_L = (((p_data << 4) & 0xF0));

    p_data_Buf[0] = p_data_H | 0X0D;
    p_data_Buf[1] = p_data_H | 0X09;
    p_data_Buf[2] = p_data_L | 0X0D;
    p_data_Buf[3] = p_data_L | 0X09;

    HAL_I2C_Master_Transmit(p_LCD->hi2c, p_LCD->Slave_address, (uint8_t *)p_data_Buf, 4, 100);
}
/**
 * The function takes the LCD handle, the column and row number as parameters and sets the cursor to
 * the specified position
 *
 * @param p_LCD This is the LCD_I2C_HandleTypeDef structure that was created in the LCD_Init()
 * function.
 * @param p_col The column you want to set the cursor to.
 * @param p_row The row you want to set the cursor to.
 */
void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_LCD, uint8_t p_col, uint8_t p_row)
{
    uint8_t t_row_Offets[] = {0x00, 0x40, 0x14, 0x54};
    if (p_row > p_LCD->LCD_Rows)
        p_row = p_LCD->LCD_Rows - 1;
    LCD_Send_cmd(p_LCD, LCD_CURSORLLINE | (p_col + t_row_Offets[p_row]));
}
/**
 * The function initializes the LCD by sending the initialization commands to the LCD
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 * @param p_hi2c This is the I2C handle that you created in your main.c file.
 * @param p_col number of columns on the LCD
 * @param p_row number of rows of the LCD
 * @param p_Slave_Address The I2C address of the LCD.
 */
void LCD_I2C_Init(LCD_I2C_HandleTypeDef *p_LCD, I2C_HandleTypeDef *p_hi2c, uint8_t p_col, uint8_t p_row, uint8_t p_Slave_Address)
{
    p_LCD->Slave_address = p_Slave_Address;
    p_LCD->LCD_Backlight_val = LCD_BACKLIGHT;
    p_LCD->LCD_Columns = p_col;
    p_LCD->LCD_Rows = p_row;
    p_LCD->hi2c = p_hi2c;
    p_LCD->LCD_Display_Option = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

    // 4 bit initialization
    HAL_Delay(50); // waiting for > 40ms
    LCD_Send_cmd(p_LCD, 0x30);
    HAL_Delay(5); // waiting for >4.1ms
    LCD_Send_cmd(p_LCD, 0x30);
    HAL_Delay(1); // waiting for >100us
    LCD_Send_cmd(p_LCD, 0x30);
    HAL_Delay(10);
    LCD_Send_cmd(p_LCD, 0x20); // 4 bit mode
    HAL_Delay(10);

    // display initialization
    LCD_Send_cmd(p_LCD, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    LCD_Send_cmd(p_LCD, 0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off		HAL_Delay(1);
    HAL_Delay(1);
    LCD_Send_cmd(p_LCD, 0x01); // clear display
    HAL_Delay(1);
    HAL_Delay(1);
    LCD_Send_cmd(p_LCD, 0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    LCD_Send_cmd(p_LCD, LCD_DISPLAYCONTROL | p_LCD->LCD_Display_Option); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

/**
 * The function takes a pointer to a string and sends each character to the LCD
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 * @param str The string to be sent to the LCD
 */
void LCD_Send_String(LCD_I2C_HandleTypeDef *p_LCD, char *str)
{
    while (*str)
        LCD_Send_data(p_LCD, *str++);
}
/**
 * This function clears the display and sets the cursor to the top left corner
 *
 * @param p_LCD pointer to the LCD_I2C_HandleTypeDef structure
 */
void LCD_Set_Clear(LCD_I2C_HandleTypeDef *p_LCD)
{
    LCD_Send_cmd(p_LCD, LCD_CLEARDISPLAY);
    HAL_Delay(2);
    LCD_Set_Cursor(p_LCD, 0, 0);
}