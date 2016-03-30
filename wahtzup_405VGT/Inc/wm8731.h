#ifndef __WM8731_H
#define __WM8731_H
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define WM8731_NSS_PORT GPIOB
#define WM8731_NSS_PIN GPIO_PIN_11
void codec_init( SPI_HandleTypeDef* hspi );

#endif