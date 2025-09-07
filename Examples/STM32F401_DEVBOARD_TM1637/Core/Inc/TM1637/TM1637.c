/*
 * TM1637.c
 *
 *  Created on: Mar 29, 2024
 *      Author: Am0k
 */

#include <TM1637/TM1637.h>

//  --0x01--
// |        |
//0x20     0x02
// |        |
//  --0x40- -
// |        |
//0x10     0x04
// |        |
//  --0x08--

GPIO_InitTypeDef GPIO_InitStruct = { 0 };


uint8_t char2segments (char c)
{
    switch (c)
    {
            case 'A':                                                       // A
                return 0x77;
            case 'b':                                                       // b
                return 0x7c;
            case 'C':                                                       // C
                return 0x39;
            case 'd':                                                       // d
                return 0x5e;
            case 'E':                                                       // E
                return 0x79;
            case 'F':                                                       // F
                return 0x71;
            case '_':                                                       // _
                return 0x08;
            case '^':                                                       // ¯
                return 0x01;
            case '-':                                                       // -
                return 0x40;
            case '*':                                                       // °
                return 0x63;
            case ' ':                                                       // space
                return 0x00;
            case 'c':                                                       // top 'c'
                return 0x61;
            case 'U':                                                       // 'U'
                return 0b00111110;
    }
    return 0;
}

static int8_t tube_tab[] = { 0x3f, 0x06, 0x5b, 0x4f,       //0~9,A,b,C,d,E,F
                             0x66, 0x6d, 0x7d, 0x07,
                             0x7f, 0x6f, 0x77, 0x7c,
                             0x39, 0x5e, 0x79, 0x71 };

void _GPIO_Init ( GPIO_TypeDef *PORT, uint16_t PIN, bool OUTPUT)
{
    HAL_GPIO_WritePin (PORT, PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = PIN;
    if (OUTPUT)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    }
    else
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init (PORT, &GPIO_InitStruct);
}

void TM1637_init (TM1637_t *Handle, GPIO_TypeDef *CLK_PORT, uint16_t CLK_PIN, GPIO_TypeDef *DIO_PORT, uint16_t DIO_PIN)
{
    Handle->CLK_PORT = CLK_PORT;
    Handle->DIO_PORT = DIO_PORT;
    Handle->CLK_PIN = CLK_PIN;
    Handle->DIO_PIN = DIO_PIN;

    _GPIO_Init (Handle->CLK_PORT, Handle->CLK_PIN, true);
    _GPIO_Init (Handle->DIO_PORT, Handle->DIO_PIN, true);

    TM1637_clearDisplay (Handle);
}

int TM1637_writeByte (TM1637_t *Handle, int8_t wr_data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_RESET);

        if (wr_data & 0x01)
        {
            HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_RESET);
        }

        wr_data >>= 1;
        HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_SET);

    _GPIO_Init (Handle->DIO_PORT, Handle->DIO_PIN, false);

    TM1637_bitDelay ();
    uint8_t ack = HAL_GPIO_ReadPin (Handle->DIO_PORT, Handle->DIO_PIN);

    if (ack == 0)
    {
        _GPIO_Init (Handle->DIO_PORT, Handle->DIO_PIN, true);
        HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_RESET);
    }

    TM1637_bitDelay ();

    _GPIO_Init (Handle->DIO_PORT, Handle->DIO_PIN, true);

    TM1637_bitDelay ();

    return ack;
}

void TM1637_bitDelay (void)
{
    HAL_Delay (1);
}

void TM1637_start (TM1637_t *Handle)
{
    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_RESET);
}

void TM1637_stop (TM1637_t *Handle)
{
    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (Handle->CLK_PORT, Handle->CLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin (Handle->DIO_PORT, Handle->DIO_PIN, GPIO_PIN_SET);
}

void TM1637_display (TM1637_t *Handle, int8_t disp_data[])
{
    int8_t seg_data[NUMBER_OF_DIGITS];
    uint8_t i;

    for (i = 0; i < NUMBER_OF_DIGITS; i++)
    {
        seg_data[i] = disp_data[i];
    }

    TM1637_coding (Handle, seg_data);
    TM1637_start (Handle);                                              // Start signal sent to TM1637 from MCU
    TM1637_writeByte (Handle, ADDR_AUTO);                                 // Command1: Set data
    TM1637_stop (Handle);
    TM1637_start (Handle);
    TM1637_writeByte (Handle, Handle->cmd_set_addr);                              // Command2: Set address (automatic address adding)

    for (i = 0; i < NUMBER_OF_DIGITS; i++)
    {
        TM1637_writeByte (Handle, seg_data[i]);                           // Transfer display data (8 bits x num_of_digits)
    }

    TM1637_stop (Handle);
    TM1637_start (Handle);
    TM1637_writeByte (Handle, Handle->cmd_disp_ctrl);                             // Control display
    TM1637_stop (Handle);
}

void TM1637_displaySymbol (TM1637_t *Handle, uint8_t bit_addr, int8_t disp_data)
{
    int8_t seg_data = TM1637_codingSymbol (Handle, disp_data);
    TM1637_start (Handle);                                              // Start signal sent to TM1637 from MCU
    TM1637_writeByte (Handle, ADDR_FIXED);                                // Command1: Set data
    TM1637_stop (Handle);
    TM1637_start (Handle);
    TM1637_writeByte (Handle, bit_addr | 0xc0);                           // Command2: Set data (fixed address)
    TM1637_writeByte (Handle, seg_data);                                  // Transfer display data 8 bits
    TM1637_stop (Handle);
    TM1637_start (Handle);
    TM1637_writeByte (Handle, Handle->cmd_disp_ctrl);                             // Control display
    TM1637_stop (Handle);
}

void TM1637_displayInt (TM1637_t *Handle, int16_t value)
{
    if (value > 9999 || value < -999)
        return;
    bool negative = false;
    int8_t digits[4];
    if (value < 0)
    {
        negative = true;
        value = -value;
    }
    digits[0] = value / 1000;                              // Thousands
    uint16_t b = digits[0] * 1000;
    digits[1] = (value - b) / 100;                         // Hundreds
    b += digits[1] * 100;
    digits[2] = (value - b) / 10;                          // Dozens
    b += digits[2] * 10;
    digits[3] = value - b;                                 // Units

    if (!negative)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if (digits[i] == 0)
                digits[i] = ' ';
            else
                break;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if (digits[i] == 0)
            {
                if (digits[i + 1] == 0)
                {
                    digits[i] = ' ';
                }
                else
                {
                    digits[i] = '-';
                    break;
                }
            }
        }
    }
    TM1637_display (Handle, digits);
}

void TM1637_displayFloat(TM1637_t *Handle, float value, uint8_t precision)
{
    if (precision > 3)
        precision = 3;
    if (precision)
    {
        for (uint8_t i = precision; i > 0; i--)
        {
            value = value * 10;
        }
    }
    if (value > 9999 || value < -999)
            return;
    bool negative = false;
    int8_t digits[4];
    if (value < 0)
    {
        negative = true;
        value = -value;
    }
    digits[0] = value / 1000;                              // Thousands
    uint16_t b = digits[0] * 1000;
    digits[1] = (value - b) / 100;                         // Hundreds
    b += digits[1] * 100;
    digits[2] = (value - b) / 10;                          // Dozens
    b += digits[2] * 10;
    digits[3] = value - b;                                 // Units

    digits[3 - precision] = digits[3-precision] | 0x80;

    if (!negative)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if (digits[i] == 0)
                digits[i] = ' ';
            else
                break;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            if (digits[i] == 0)
            {
                if (digits[i + 1] == 0)
                {
                    digits[i] = ' ';
                }
                else
                {
                    digits[i] = '-';
                    break;
                }
            }
        }
    }
    TM1637_display (Handle, digits);
}

void TM1637_displayTemp (TM1637_t *Handle, int16_t value)
{
    Handle->_PointFlag = false;
    int8_t digits[4];
    if (value < 0)
    {
        if (value < -99)
        {
            digits[0] = '_';
            digits[1] = '_';
            digits[2] = '_';
            digits[3] = '*';
        }
        else
        {
            digits[0] = '-';
            if (value < -9)
            {
                digits[1] = - value / 10;
                digits[2] = - value % 10;
                digits[3] = '*';
            }
            else
            {
                digits[1] = - value;
                digits[2] = '*';
                digits[3] = 'C';
            }
        }
    }
    else
    {
        if (value > 999)
        {
            digits[0] = '^';
            digits[1] = '^';
            digits[2] = '^';
            digits[3] = '*';
        }
        else
        {
            if (value > 99)
            {
                digits[0] = value / 100;                   // Hundreds
                uint16_t b = digits[0] * 100;
                digits[1] = (value - b) / 10;              // Dozens
                b += digits[1] * 10;
                digits[2] = value - b;                     // Units
                digits[3] = '*';
            }
            else
            {
                digits[0] = value / 10;
                if (digits[0] == 0)
                    digits[0] = ' ';
                digits[1] = value % 10;
                digits[2] = '*';
                digits[3] = 'C';
            }
        }
    }
    TM1637_display (Handle, digits);
}

void TM1637_displayTime (TM1637_t *Handle, uint32_t value)
{
    if (value > 1440 || value < 0)
        return;
    int8_t digits[4];
    int8_t hours = value / 60;
    int8_t minutes = value % 60;
    digits[0] = hours / 10;
    digits[1] = hours % 10;
    digits[2] = minutes / 10;
    digits[3] = minutes % 10;
    TM1637_display (Handle, digits);
}

void TM1637_displaySpec (TM1637_t *Handle, int16_t value, uint8_t symbol)
{
    Handle->_PointFlag = false;
    int8_t digits[4];
    if (value > 999 || value < -99)
        return;
    if (value < 0)
    {
        digits[0] = '-';
        digits[1] = - value / 10;
        digits[2] = - value % 10;
    }
    else
    {
        digits[0] = value / 100;                   // Hundreds
        uint16_t b = digits[0] * 100;
        digits[1] = (value - b) / 10;              // Dozens
        b += digits[1] * 10;
        digits[2] = value - b;                     // Units
     }
    digits[3] = symbol;
    TM1637_display (Handle, digits);
}

void TM1637_clearDisplay (TM1637_t *Handle)
{
    TM1637_displaySymbol (Handle, 0x00, 0x7f);
    TM1637_displaySymbol (Handle, 0x01, 0x7f);
    TM1637_displaySymbol (Handle, 0x02, 0x7f);
    TM1637_displaySymbol (Handle, 0x03, 0x7f);
}

void TM1637_set (TM1637_t *Handle, uint8_t brightness)
{
    Handle->cmd_set_data = 0x40;
    Handle->cmd_set_addr = 0xc0;
    //Set the brightness and it takes effect the next time it displays.
    Handle->cmd_disp_ctrl = 0x88 + brightness;
}

// Whether to light the clock point ":".
// To take effect the next time it displays.
void TM1637_point (TM1637_t *Handle, bool PointFlag)
{
    Handle->_PointFlag = PointFlag;
}

void TM1637_coding (TM1637_t *Handle, int8_t disp_data[])
{
    for (uint8_t i = 0; i < NUMBER_OF_DIGITS; i++)
    {
        disp_data[i] = TM1637_codingSymbol(Handle, disp_data[i]);
    }
}

int8_t TM1637_codingSymbol (TM1637_t *Handle, int8_t disp_data)
{
    bool point = false;
    if (disp_data & 0x80)
    {
        point = true;
        disp_data = (disp_data & 0x7f);
    }
    if (disp_data == 0x7f)
    {
        disp_data = 0x00;    // Clear digit
    }
    else if ((disp_data >= 0) && (disp_data < (int)(sizeof(tube_tab) / sizeof(*tube_tab))))
    {
        disp_data = tube_tab[disp_data];
    }
    else if (disp_data >= '0' && disp_data <= '9')
    {
        disp_data = tube_tab[(int)(disp_data) - 48]; // char to int (char "0" = ASCII 48)
    }
    else
    {
        disp_data = char2segments (disp_data);
    }
    if (point)
        disp_data += 0x80;
    else
        disp_data += Handle->_PointFlag == POINT_ON ? 0x80 : 0;

    return disp_data;
}
