/*
 * lcd_driver.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Tyler Chargualaf
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Structure to track software state of the LCD.
  * @note   Maintains cursor position, base addresses for each line, and a shadow buffer
  *         representing the current display content.
  */
typedef struct {
    uint8_t cursor_position;   /**< Current DDRAM address (0x00–0x53) */
    uint8_t line1_base;        /**< DDRAM base address for line 1 (0x00) */
    uint8_t line2_base;        /**< DDRAM base address for line 2 (0x40) */
    char LCD_Buffer[41];       /**< Shadow buffer: 40 characters plus null terminator */
} LCD_ControlTypeDef;

extern LCD_ControlTypeDef lcd;

void LCD_Init(void);
void LCD_Reset(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_SendString(const char *str, uint8_t line_sel);
uint8_t Center_Offset(size_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_LCD_DRIVER_H_ */
