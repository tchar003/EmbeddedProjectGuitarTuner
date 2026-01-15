#include "lcd_driver.h"
#include "main.h"

// I2C address of LCD (7-bit shifted)
#define LCD_ADDR     (0x3C << 1)
#define LCD_CMD      0x00
#define LCD_DATA     0x40

LCD_ControlTypeDef lcd = {
    .cursor_position = 0x00,
    .line1_base = 0x00,
    .line2_base = 0x40,
    .LCD_Buffer = {0}
};

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief  Sends command signal followed by command code
  * @param  cmd: refer to https://newhavendisplay.com/content/specs/NHD-C0220BiZ-FSW-FBW-3V3M.pdf for list of instruction codes
  * @retval None
  * @note
  */
static void LCD_SendCommand(uint8_t cmd) {
    uint8_t data[2] = {LCD_CMD, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data, 2, HAL_MAX_DELAY);
}

/**
  * @brief  Performs a hardware reset of the LCD using the RST pin.
  * @note   The RST pin is pulled low for 10ms, then high for 10ms to ensure a clean reset.
  * @retval None
  */
void LCD_Reset(void) {
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

/**
  * @brief  Initializes the LCD hardware and software state.
  * @note   Sends the recommended initialization sequence to the LCD controller.
  *         Resets the display, cursor position, and internal software buffer.
  * @retval None
  */
void LCD_Init(void) {
    LCD_Reset();
    HAL_Delay(40);

    lcd.cursor_position = 0x00;
    memset(lcd.LCD_Buffer, ' ', 40);
    lcd.LCD_Buffer[40] = '\0';

    LCD_SendCommand(0x38); HAL_Delay(10);
    LCD_SendCommand(0x39); HAL_Delay(10);
    LCD_SendCommand(0x14);
    LCD_SendCommand(0x78);
    LCD_SendCommand(0x5E);
    LCD_SendCommand(0x6D);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x01);
    LCD_SendCommand(0x06);
    HAL_Delay(10);
}

/**
  * @brief  Sets the LCD cursor to the specified row and column.
  * @param  row: Row index (0 = first line, 1 = second line)
  * @param  col: Column index (0 to 19)
  * @note   Out-of-bounds values will be clamped to the displayable area.
  * @retval None
  */
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? lcd.line1_base + col : lcd.line2_base + col;
    if (addr > 0x53) addr = 0x53;					  // Clamp to end of line 2
    else if (addr > 0x13 && addr < 0x40) addr = 0x13; // Clamp to end of line 1
    lcd.cursor_position = addr;

    /* Set DDRAM address in address counter
     * 0 0 | 1 AC6 AC5 AC4 AC3 AC2 AC1 AC0
     *
     * 1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 	| Character (2x20)
     * 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13  | (Line 1)
     * 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 50 51 52 53	| (Line 2)
     */

    LCD_SendCommand(0x80 | addr);
}

/**
  * @brief  Calculates the column offset needed to center a string of given length.
  * @param  len: Length of the string to center (should be ≤ 20)
  * @retval The starting column for centered text (0–19)
  */
uint8_t Center_Offset(size_t len){
	if (len >= 20) return 0;
	return (20 - len - 1) / 2;
}

/**
  * @brief  Sends a string to the LCD, centered on the specified line.
  * @param  str: Pointer to a null-terminated ASCII string (maximum 20 characters used)
  * @param  line_sel: Line selection (0 = first line + automatic wrapping, 1 = only first line, 2 = only second line)
  * @note   The string will be centered on the chosen line; longer strings will be truncated.
  * @retval None
  */
void LCD_SendString(const char *str, uint8_t line_sel) {
	LCD_SendCommand(0x01);
	HAL_Delay(2);

	size_t len = strlen(str);
    if (len == 0) return;


	if (line_sel <= 1)
	{
		LCD_SetCursor(0,Center_Offset(len));
	}
	else
	{
		LCD_SetCursor(1,Center_Offset(len));

	}


	// Transmit buffer
	uint8_t buf_size = (len <= 21) ? len + 1 : 21; // Clamp to display length 20 + 1 control byte
    uint8_t buf[buf_size];
    buf[0] = LCD_DATA;
    memcpy(&buf[1], str, buf_size - 1);


    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, buf, buf_size, HAL_MAX_DELAY);



    //Calls function again to wrap data onto second line of display
    if (len > 21 && line_sel == 0) {
    	char buf2[len - 20];
    	memcpy(buf2, str + 20, len);


    	LCD_SendString(buf2, 2);
    }

    HAL_Delay(10);
}

