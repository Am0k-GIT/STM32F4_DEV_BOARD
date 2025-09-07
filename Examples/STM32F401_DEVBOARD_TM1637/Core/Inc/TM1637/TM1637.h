/*
 * TM1637.h
 *
 *  Created on: Mar 29, 2024
 *      Author: Am0k
 *
 */

#ifndef TM1637_h
#define TM1637_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

/*******************Definitions for TM1637*********************/
#define ADDR_AUTO 0x40
#define ADDR_FIXED 0x44
#define NUMBER_OF_DIGITS 4
#define STARTADDR 0xc0
/*****Definitions for the clock point of the digit tube *******/
#define POINT_ON 1
#define POINT_OFF 0
/**************Definitions for brightness**********************/
#define BRIGHT_DARKEST 0
#define BRIGHT_TYPICAL 2
#define BRIGHTEST 7

typedef struct {
    GPIO_TypeDef *CLK_PORT;
    GPIO_TypeDef *DIO_PORT;
    uint16_t CLK_PIN;
    uint16_t DIO_PIN;
    uint8_t cmd_set_data;
    uint8_t cmd_set_addr;
    uint8_t cmd_disp_ctrl;
    bool _PointFlag;
    uint8_t clkpin;
    uint8_t datapin;
} TM1637_t;

void TM1637_init (TM1637_t *Handle, GPIO_TypeDef *CLK_PORT, uint16_t CLK_PIN, GPIO_TypeDef *DIO_PORT, uint16_t DIO_PIN);                                                          // To clear the display
int TM1637_writeByte (TM1637_t *Handle, int8_t wr_data);                                            // Write 8bit data to tm1637
void TM1637_start (TM1637_t *Handle);                                                         // Send start bits
void TM1637_stop (TM1637_t *Handle);                                                          // Send stop bits
void TM1637_display (TM1637_t *Handle, int8_t DispData[]);
void TM1637_displaySymbol (TM1637_t *Handle, uint8_t BitAddr, int8_t DispData);
void TM1637_displayInt (TM1637_t *Handle, int16_t value);
void TM1637_displayFloat (TM1637_t *Handle, float value, uint8_t precision);
void TM1637_displayTemp (TM1637_t *Handle, int16_t value);
void TM1637_displayTime (TM1637_t *Handle, uint32_t value);
void TM1637_displaySpec (TM1637_t *Handle, int16_t value, uint8_t symbol);
void TM1637_clearDisplay (TM1637_t *Handle);
void TM1637_set (TM1637_t *Handle, uint8_t brightness);       //To take effect the next time it displays.
void TM1637_point (TM1637_t *Handle, bool PointFlag);                                               //whether to light the clock point ":".
void TM1637_coding (TM1637_t *Handle, int8_t DispData[]);
int8_t TM1637_codingSymbol (TM1637_t *Handle, int8_t DispData);
void TM1637_bitDelay (void);

#ifdef __cplusplus
}
#endif

#endif
