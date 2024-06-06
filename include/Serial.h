#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <cstring>
#include <stdio.h>
#include <string>

namespace stm32f4{
    namespace serial{
        inline void Transmit(UART_HandleTypeDef* uart, const char* message){
            HAL_UART_Transmit(uart, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
        }

        class Uart{
        public:
            UART_HandleTypeDef configure(){
                UART_HandleTypeDef uart1;
                uart1.Instance = USART1;
                uart1.Init.BaudRate = 115200;
                uart1.Init.WordLength = UART_WORDLENGTH_8B;
                uart1.Init.StopBits = UART_STOPBITS_1;
                uart1.Init.Parity = UART_PARITY_NONE;
                uart1.Init.Mode = UART_MODE_TX_RX;
                uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
                uart1.Init.OverSampling = UART_OVERSAMPLING_16;

                GPIO_InitTypeDef GPIO_InitStruct = {0};

                    // Enable GPIO Clocks
                __HAL_RCC_GPIOA_CLK_ENABLE();
                __HAL_RCC_USART1_CLK_ENABLE();

                // UART TX GPIO pin configuration
                GPIO_InitStruct.Pin = GPIO_PIN_9; // PA9 for USART1_TX
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
                
                // UART RX GPIO pin configuration
                GPIO_InitStruct.Pin = GPIO_PIN_10; // PA10 for USART1_RX
                GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
                HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

                return uart1;
            }
        };
    };
};