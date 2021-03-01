# STM32_HC05
This is a PCB board designed in KiCAD which implements a STM32F746VGT6 microcontroller exposing the following pins and internal connections. All of the GPIO pins support timer peripheral output, for a more detailed view of the potential PIN function, check the STM32F7 datasheet using name as a reference. Please read [STM32F7 Reference page](https://www.st.com/en/microcontrollers-microprocessors/stm32f7-series.html) and [Getting Started Guide](https://www.st.com/resource/en/user_manual/dm00180213-getting-started-with-stm32cubef7-mcu-package-for-stm32f7-series-stmicroelectronics.pdf).

This board includes a [HC-05 Bluetooth module](https://components101.com/wireless/hc-05-bluetooth-module)  which exposes a serial port interface over bluetooth as a direct link with the microcontroller. The bluetooth module is switched on/off with a transistor switch controlled by a microcontroller pin. 

## Board Design
![PCB Design](/IMG1.PNG "PCB Design")

## Pinout
![Pinout](/IMG2.PNG "Pinout")

## Pin Function

| Name            | Pin     | Extra Functions   |
| :-------------: |:-------:| :-------------:   |
| PA0             | 22      | ADC0, UART4_TX, USART2_CTS |
| PA1             | 23      | ADC1, UART4_RX, USART2_RTS |
| PA2             | 24      | ADC2, USART2_TX            |
| PA3             | 25      | ADC3, USART2_RX            |
| PA4             | 28      | ADC4, USART2_CLK           |
| PA5             | 29      | ADC5, SPI1_CLK             |
| PA6             | 30      | ADC6, SPI1_MISO            |
| PA7             | 31      | ADC7, SPI1_MOSI            |
| PB0             | 34      | ADC8                       |
| PB1             | 35      | ADC9                       |
| PB10            | 46      | I2C2_CLK                   |
| PB11            | 47      | I2C2_SDA                   |
| PD8             | 55      | USART3_TX                  |
| PD9             | 56      | USART3_RX                  |
| PD10            | 57      | USART3_CLK                 |
| PD11            | 58      | USART3_CTS                 |
| PD12            | 59      | USART3_RTS                 |
| PD13            | 60      |                            |
| PD14            | 61      |                            |
| PD15            | 62      |                            |
| PD0             | 81      | Green LED                  |
| PD1             | 82      | Red LED                    |
| PD2             | 83      | Blue LED                   |
| PD3             | 84      | Orange LED                 |
| PD4             | 85      | Yellow LED                 |
| PC6             | 63      | HC-05 UART RX              |
| PC7             | 64      | HC-05 UART TX              |
| PC8             | 65      | HC-05 BT KEY               |
| PE12            | 42      | Bluetooth Enable           |

## Programming Interface
The programming and power port is located on the upper left section of the board. The flashing debugging must be performed using a ST-Link/v2 debugger or STM32FX evaluation board (Discovery, Nucleo, etc ...). LEDs are connected to GPIO (PD0-4), bluetooth module is connected to UART6 port and the bluetooth module on/off actuation is driven by GPIO PE12. Further information on programming and the specific pin functions can be found on the STM32F7 datasheet, and the HC-05 bluetooth module reference guide.

# Code Examples
The following 4 files form the required interface to program and operate the Bluetooth module via the UART peripheral of the STM32F7.
## bluetooth.h
```C
#ifndef __bluetooth_H
#define __bluetooth_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include "gpio.h"

void BT_isAlive(void);
void BT_setNameMixcell(void);
void BT_clearPaired(void);
void BT_setMaster(void);
void BT_setSlave(void);
void BT_setCOM(void);
void BT_Reset(void);	
void BT_Restore(void);	
void BT_SendTest(void);
void BT_CMode(void);
void BT_Init_UART(void);
	 
#ifdef __cplusplus
}
#endif
#endif /*__ bluetooth_H */
```
## bluetooth.c
```C
#include "bluetooth.h"

extern UART_HandleTypeDef UART;

extern char data_rx[32];
extern char rx_data[8];			/* Buffer used for UART */

void BT_isAlive(void)
{
	char data_tx[4] = "AT\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 4, 1000);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 1000);
}

void BT_setName(void)
{
	// TODO Set Bluetooth name: MY_NAME.
	char data_tx[17] = "AT+NAME=MY_NAME\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 17, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}

void BT_setMaster(void)
{
	char data_tx[11] = "AT+ROLE=1\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 11, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}

void BT_setSlave(void)
{
	char data_tx[11] = "AT+ROLE=0\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 11, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}

void BT_Reset(void)
{
	char data_tx[10] = "AT+RESET\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 10, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}

void BT_Restore(void)
{
	char data_tx[9] = "AT+ORGL\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 9, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 32, 500);
}


void BT_clearPaired(void)
{
	char data_tx[10] = "AT+RMAAD\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 10, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}

void BT_setCOM(void)
{
	char data_tx[20] = "AT+UART=115200,0,0\r\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 20, 500);
	HAL_UART_Receive(&UART, (uint8_t *) data_rx, 24, 500);
}
```
## usart.h
```C
/**
  ******************************************************************************
  * File Name          : UART.h
  * Date               : 13/04/2015 14:34:07
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "stdlib.h"
#include "bluetooth.h"
	 
void MX_USART_UART_Init(void);
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart);
void receive_UART_action(void);
void send_UART_copy(void);
void send_back(char rx_data[8]);
void send_alive(void);

/* HC-05 BT driver. */
void BT_Config(void);
void BT_UART(void);
void send_data(void);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
```
## usart.c
```C
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
#include "stdlib.h"
#include <string.h>

UART_HandleTypeDef UART;

char data_rx[32] = "";
extern char rx_data[8];			/* Buffer used for UART */
extern uint16_t INFO_TO_SEND[1024]; 	/* Buffer used for ADC */

/* USART6 init function */
void MX_USART_UART_Init(void)
{
  UART.Instance = USART6;
  UART.Init.BaudRate = 38400;
	UART.Init.WordLength = UART_WORDLENGTH_8B;
  UART.Init.StopBits = UART_STOPBITS_1;
  UART.Init.Parity = UART_PARITY_NONE;
  UART.Init.Mode = UART_MODE_TX_RX;
  UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART.Init.OverSampling = UART_OVERSAMPLING_16;
	UART.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  UART.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&UART);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART_MspInit 0 */

  /* USER CODE END USART_MspInit 0 */
    /* Peripheral clock enable */
		__GPIOC_CLK_ENABLE();
    __USART6_CLK_ENABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
  /* System interrupt init*/
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
void BT_Init_UART()
{
	BT_UART();
}

void BT_Config(void)
{
	BT_Reset();
	HAL_Delay(1000);
	BT_isAlive();
	HAL_Delay(1000);
	BT_Restore();
	HAL_Delay(1000);
	BT_setName();
	HAL_Delay(1000);
	BT_setCOM();
	HAL_Delay(1000);
	BT_clearPaired();
	HAL_Delay(1000);
}

void BT_SendTest(void)
{
	BT_isAlive();
}

void BT_UART(void)
{
	HAL_UART_DeInit(&UART);
	
  UART.Instance = USART6;
  UART.Init.BaudRate = 115200;
  UART.Init.WordLength = UART_WORDLENGTH_8B;
  UART.Init.StopBits = UART_STOPBITS_1;
  UART.Init.Parity = UART_PARITY_NONE;
  UART.Init.Mode = UART_MODE_TX_RX;
  UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UART);
}


/* TX SECTION */
void send_alive()
{
	char data_tx[9] = "SM_ALIVE\n";
	memset(data_rx, 0x00, 32);
	HAL_UART_Transmit(&UART, (uint8_t *) data_tx, 9, 1000);
	receive_UART_action();
}

void send_data(void)
{
	// TODO: INFO_TO_SEND would be the char array to be send. 
	uint8_t * data;
	HAL_UART_Transmit(&UART, (uint8_t *)(INFO_TO_SEND), 16384, 10000);
}

void send_back(char rx_data[8])
{
		HAL_UART_Transmit(&UART, (uint8_t *)rx_data, 8, 1000);		
}


/* RX SECTION */
void receive_UART_action()
{
	int i = 0;
	while (i < 8)
	{
		rx_data[i] = 0x00;
		i++;
	}
	HAL_UART_Receive_IT(&UART, (uint8_t *)rx_data, 8);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		// TODO: Reception complete.
}
```

# Success Stories
- [Sensing Cell-Culture Assays with Low-Cost Circuitry](https://www.nature.com/articles/s41598-018-27295-3)
- [An Empirical-Mathematical Approach for Calibration and Fitting Cell-Electrode Electrical Models in Bioimpedance Tests](https://www.mdpi.com/1424-8220/18/7/2354)
- [Remote Cell Growth Sensing Using Self-Sustained Bio-Oscillations](https://www.mdpi.com/1424-8220/18/8/2550)
- [Data-Analytics Modeling of Electrical Impedance Measurements for Cell Culture Monitoring](https://www.mdpi.com/1424-8220/19/21/4639)
- [Electrical Modeling of the Growth and Differentiation of Skeletal Myoblasts Cell Cultures for Tissue Engineering](https://www.mdpi.com/1424-8220/20/11/3152)
