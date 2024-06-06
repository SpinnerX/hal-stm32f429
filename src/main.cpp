#include "Serial.h"
#include <chrono>

constexpr uint32_t cs_pin = GPIO_PIN_6;
GPIO_TypeDef* cs_gpio_port = GPIOB;
constexpr uint32_t dc_pin = GPIO_PIN_7;
GPIO_TypeDef* dc_gpio_port = GPIOB;

/**
 * 
 * For setting STM32F4 pins you do GPIO_PIN_RESET = LOW, GPIO_PIN_SET = Hight
 * 
 * 
 * 
 * 
*/

void set_cs_low(){
  HAL_GPIO_WritePin(cs_gpio_port, cs_pin, GPIO_PIN_RESET); // setting our cs pin to low
}

void set_cs_high(){
  HAL_GPIO_WritePin(cs_gpio_port, cs_pin, GPIO_PIN_SET); // setting our cs pin to high
}


#define LCD_CS_HIGH()    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_CS_LOW() HAL_GPIO_WritePin(LCD_CS_GPIO_PORT)
#define LCD_DC_LOW()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RESET_LOW()  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET)
#define LCD_RESET_HIGH() HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET)
SPI_HandleTypeDef spi1;
UART_HandleTypeDef uart;

void WriteCommand(){
  LCD_DC_LOW();
  set_cs_low();
  // HAL_SPI_Transmit(&uart, &command, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart, &command, 1, HAL_MAX_DELAY);
  LCD_CS_HIGH();
}

int main(){
  using namespace std::chrono_literals;

  stm32f4::serial::Uart uart1;
  uart = uart1.configure();
  spi1.Instance = SPI1;
  spi1.Init.Mode = SPI_MODE_MASTER;
  spi1.Init.Direction = SPI_DIRECTION_2LINES;
  spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  spi1.Init.NSS = SPI_NSS_SOFT;
  spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi1.Init.CRCPolynomial = 10;


  if(HAL_UART_Init(&uart) != HAL_OK){
    return -2;
  }
  // serial::uart::Transmit(&uart1, data.c_str());
  int index = 0;
  while(index < 11){
    std::string data = "Transmitting data through UART!";
    data += " index = " + std::to_string(index++) + "\n";
    stm32f4::serial::Transmit(&uart, data.c_str());
  }

  return 0;
}