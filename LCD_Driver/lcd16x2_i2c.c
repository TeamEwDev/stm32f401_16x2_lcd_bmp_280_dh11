/*
 * lcd16x2_i2c.c
 *
 *  Created on: May 27, 2023
 *      Author: Embedded Wala
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "lcd16x2_i2c.h"
#include "stdbool.h"

/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef *lcd16x2I2cHandle;
static bool backLightEnable = true;

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Send a command to the LCD.
 * @param cmd: LCD command to be sent.
 * @retval None.
 */
void LCD_Send_Cmd(uint8_t cmd)
{
    uint8_t dataU, dataL;
    uint8_t dataUTx[2], dataLTx[2];
    uint8_t backLight = backLightEnable ? BACKLIGHT : 0;

    /*Store upper nibble*/
    dataU = (cmd & 0xF0);

    /*Store lower nibble*/
    dataL = ((cmd << 4) & 0xF0);

    /* Construct upper byte-> compatible for LCD*/
    dataUTx[0] = dataU | backLight | PIN_EN;
    dataUTx[1] = dataU | backLight;

    /* Construct lower byte-> compatible for LCD*/
    dataLTx[0] = dataL | backLight | PIN_EN;
    dataLTx[1] = dataL | backLight;

    /* Transmit upper nibble using I2C APIs*/
    if (HAL_I2C_IsDeviceReady(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
        HAL_I2C_Master_Transmit(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, dataUTx, 2, 100);

    /*Provide a delay */
    HAL_Delay(5);

    /* Transmit lower nibble using I2C APIs*/
    if (HAL_I2C_IsDeviceReady(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
        HAL_I2C_Master_Transmit(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, dataLTx, 2, 100);

    /*Provide a delay */
    HAL_Delay(5);
}

/**
 * @brief Send data to the LCD.
 * @param data: Data to be sent to the LCD.
 * @retval None.
 */
void LCD_Send_Data(uint8_t data)
{
    uint8_t dataU, dataL;
    uint8_t dataUTx[2], dataLTx[2];
    uint8_t backLight = backLightEnable ? BACKLIGHT : 0;

    /*Store upper nibble*/
    dataU = (data & 0xF0);

    /*Store lower nibble*/
    dataL = ((data << 4) & 0xF0);

    /* Construct upper byte-> compatible for LCD*/
    dataUTx[0] = dataU | backLight | PIN_EN | PIN_RS;
    dataUTx[1] = dataU | backLight | PIN_RS;

    /* Construct lower byte-> compatible for LCD*/
    dataLTx[0] = dataL | backLight | PIN_EN | PIN_RS;
    dataLTx[1] = dataL | backLight | PIN_RS;

    /* Transmit upper nibble using I2C APIs*/
    if (HAL_I2C_IsDeviceReady(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
        HAL_I2C_Master_Transmit(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, dataUTx, 2, 100);

    /*Provide a delay */
    HAL_Delay(1);

    /* Transmit lower nibble using I2C APIs*/
    if (HAL_I2C_IsDeviceReady(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, 1, 10) == HAL_OK)
        HAL_I2C_Master_Transmit(lcd16x2I2cHandle, I2C_SLAVE_ADDRESS, dataLTx, 2, 100);

    /*Provide a delay */
    HAL_Delay(5);
}

/**
 * @brief Initialize the LCD.
 * @param lcd16x2I2cHandleParam: Pointer to the I2C handle for the LCD.
 * @retval None.
 */
void LCD_Init(I2C_HandleTypeDef *lcd16x2I2cHandleParam)
{
    lcd16x2I2cHandle = lcd16x2I2cHandleParam;
    /* Wait for 15ms */
    HAL_Delay(15);

    /*Function Set - As per HD44780U*/
    LCD_Send_Cmd(LCD_FUNCTION_SET1);

    /*Function Set -As per HD44780U*/
    LCD_Send_Cmd(LCD_FUNCTION_SET2);

    /*Set 4bit mode and 2 lines */
    LCD_Send_Cmd(LCD_4BIT_2LINE_MODE);

    /*Display on, cursor off*/
    LCD_Send_Cmd(0x0C);

    /* SET Row1 and Col1 (1st Line) */
    LCD_Send_Cmd(0x80);

    /*Clear Display*/
    LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
}

/**
 * @brief Control the backlight of the LCD.
 * @param enable: Set to true to enable backlight, false to disable.
 * @retval None.
 */
void LCD_Enable_BackLight(bool enable)
{
    backLightEnable = enable;
}

/**
 * @brief Set the cursor position on the LCD.
 * @param row: Row number (0 or 1).
 * @param col: Column number (0 to 15).
 * @retval None.
 */
void LCD_Set_Cursor(uint8_t row, uint8_t col)
{
    uint8_t maskData;
    maskData = (col) & 0x0F;
    if (row == 0)
    {
        maskData |= (0x80);
        LCD_Send_Cmd(maskData);
    }
    else
    {
        maskData |= (0xc0);
        LCD_Send_Cmd(maskData);
    }
}

/**
 * @brief Send a string to the LCD.
 * @param str: Pointer to the string to be sent.
 * @retval None.
 */
void LCD_Send_String(char *str)
{
    while (*str)
    {
        LCD_Send_Data(*str++);
    }
}

/**
 * @brief Clear the screen and then display the given string on the LCD.
 * @param str: Pointer to the string to be displayed.
 * @retval None.
 */
void LCD_Clear_Then_Display(char *str)
{
    LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
    LCD_Send_String(str);
}

/**
 * @brief Send a string to the LCD at the specified position.
 * @param str: Pointer to the string to be sent.
 * @param row: Row number (0 or 1).
 * @param col: Column number (0 to 15).
 * @retval None.
 */
void LCD_Send_String_At_Pos(char *str, uint8_t row, uint8_t col)
{
    LCD_Set_Cursor(row, col);
    LCD_Send_String(str);
}

/**
 * @brief Display a string on the first line of the LCD.
 * @param str: Pointer to the string to be displayed.
 * @retval None.
 */
void LCD_Send_String_On_Line1(char *str)
{
    LCD_Send_Cmd(LCD_SET_ROW1_COL1);
    LCD_Send_String(str);
}


/**
 * @brief Display a string on the second line of the LCD.
 * @param str: Pointer to the string to be displayed.
 * @retval None.
 */
void LCD_Send_String_On_Line2(char *str)
{
    LCD_Send_Cmd(LCD_SET_ROW2_COL1);
    LCD_Send_String(str);
}

/**
 * @brief Display long messages of any size on the LCD, scrolling as needed.
 * @param string: Pointer to the long message string.
 * @retval None.
 */
void LCD_Display_Long_Message(char *string)
{
    int i = 0, count = 1, j = 0;
    /*Clear display and Set position to Line1 start*/
    LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
    LCD_Send_Cmd(LCD_SET_ROW1_COL1);

    while (string[i] != '\0')
    {
        LCD_Send_Data(string[i]);

        /*If we reach 1st Line end, then goto 2nd line start*/
        if (j >= 15 && (count % 2 == 1))
        {
            count++;
            LCD_Send_Cmd(LCD_SET_ROW2_COL1);
        }

        /*If we reach second line end, clear display start from line1 again*/
        if (j >= 31 && (count % 2 == 0))
        {
            count++;
            j = 0;
            LCD_Send_Cmd(LCD_CLEAR_DISPLAY);
            LCD_Send_Cmd(LCD_SET_ROW1_COL1);
        }

        HAL_Delay(100);
        i++;
        j++;
    }
}
