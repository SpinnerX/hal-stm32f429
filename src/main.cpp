#include "Serial.h"
#include <chrono>

constexpr uint32_t cs_pin = GPIO_PIN_6;
GPIO_TypeDef* cs_gpio_port = GPIOB;

constexpr uint32_t dc_pin = GPIO_PIN_7;
GPIO_TypeDef* dc_gpio_port = GPIOB;

constexpr uint32_t reset_pin = GPIO_PIN_8;
GPIO_TypeDef* reset_gpio_pin = GPIOB;

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

void set_dc_low(){
  HAL_GPIO_WritePin(dc_gpio_port, dc_pin, GPIO_PIN_RESET);
}

void set_dc_high(){
  HAL_GPIO_WritePin(dc_gpio_port, dc_pin, GPIO_PIN_SET);
}

void set_reset_low(){
  HAL_GPIO_WritePin(reset_gpio_pin, reset_pin, GPIO_PIN_RESET);
}

void set_reset_high(){
  HAL_GPIO_WritePin(reset_gpio_pin, reset_pin, GPIO_PIN_SET);
}


// #define LCD_CS_HIGH()    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
// #define LCD_CS_LOW() HAL_GPIO_WritePin(LCD_CS_GPIO_PORT)
// #define LCD_DC_LOW()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
// #define LCD_DC_HIGH()    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
// #define LCD_RESET_LOW()  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET)
// #define LCD_RESET_HIGH() HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET)
SPI_HandleTypeDef spi1;
UART_HandleTypeDef uart;

void WriteCommand(uint8_t command){
  // LCD_DC_LOW();
  set_dc_low();
  set_cs_low();
  // HAL_SPI_Transmit(&uart, &command, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart, &command, 1, HAL_MAX_DELAY);
  // LCD_CS_HIGH();
  set_cs_high();
}

void LCD_WriteData(uint8_t data){
    set_cs_high();
    set_cs_low();
    HAL_SPI_Transmit(&spi1, &data, 1, HAL_MAX_DELAY);
    set_cs_high();
}

void InitializeSequence(){
  // Initial sequence for ILI9341 (Example)
  // WriteCommand(0x01); // Software reset
  // HAL_Delay(100);

  // WriteCommand(0x28); // Display OFF

  // WriteCommand(0xCF); // Power control B
  // LCD_WriteData(0x00);
  // LCD_WriteData(0x81); // Enable factor
  // LCD_WriteData(0x30);

  // WriteCommand(0xED); // Power on sequence control
  // LCD_WriteData(0x64);
  // LCD_WriteData(0x03);
  // LCD_WriteData(0x12);
  // LCD_WriteData(0x81);

  // // Add more initialization commands here as needed

  // WriteCommand(0x29); // Display ON
  // Reset the LCD
  set_reset_low();
  HAL_Delay(20); // 20ms delay
  LCD_RESET_HIGH();
  HAL_Delay(150); // 150ms delay

  // ILI9341 initial sequence (Basic Example)
  LCD_WriteCommand(0x01); // Software reset
  HAL_Delay(100);

  LCD_WriteCommand(0x28); // Display OFF

  // Power control A
  LCD_WriteCommand(0xCB);
  LCD_WriteData(0x39);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x00);
  LCD_WriteData(0x34);
  LCD_WriteData(0x02);

  // Power control B
  LCD_WriteCommand(0xCF);
  LCD_WriteData(0x00);
  LCD_WriteData(0xC1);
  LCD_WriteData(0x30);

  // Driver timing control A
  LCD_WriteCommand(0xE8);
  LCD_WriteData(0x85);
  LCD_WriteData(0x00);
  LCD_WriteData(0x78);

  // Power on sequence control
  LCD_WriteCommand(0xED);
  LCD_WriteData(0x64);
  LCD_WriteData(0x03);
  LCD_WriteData(0x12);
  LCD_WriteData(0x81);

  // Pump ratio control
  LCD_WriteCommand(0xF7);
  LCD_WriteData(0x20);

  // Driver timing control B
  LCD_WriteCommand(0xEA);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);

  // Power control, VRH[5:0]
  LCD_WriteCommand(0xC0);
  LCD_WriteData(0x23);

  // Power control, SAP[2:0]; BT[3:0]
  LCD_WriteCommand(0xC1);
  LCD_WriteData(0x10);

  LCD_WriteCommand(0xC5);  // VCM control 1
  LCD_WriteData(0x3E);
  LCD_WriteData(0x28);

  LCD_WriteCommand(0xC7);  // VCM control 2
  LCD_WriteData(0x86);

  // Memory Access Control
  LCD_WriteCommand(0x36);
  LCD_WriteData(0x48);

  // Pixel Format Set
  LCD_WriteCommand(0x3A);
  LCD_WriteData(0x55);

  // Frame Rate Control
  LCD_WriteCommand(0xB1);
  LCD_WriteData(0x00);
  LCD_WriteData(0x18);

  // Display Function Control
  LCD_WriteCommand(0xB6);
  LCD_WriteData(0x08);
  LCD_WriteData(0x82);
  LCD_WriteData(0x27);

  // 3Gamma Function Disable
  LCD_WriteCommand(0xF2);
  LCD_WriteData(0x00);

  // Gamma curve selected
  LCD_WriteCommand(0x26);
  LCD_WriteData(0x01);

  LCD_WriteCommand(0xE0);  // Set Gamma
  LCD_WriteData(0x0F);
  LCD_WriteData(0x31);
  LCD_WriteData(0x2B);
  LCD_WriteData(0x0C);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x08);
  LCD_WriteData(0x4E);
  LCD_WriteData(0xF1);
  LCD_WriteData(0x37);
  LCD_WriteData(0x07);
  LCD_WriteData(0x10);
  LCD_WriteData(0x03);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x09);
  LCD_WriteData(0x00);

  LCD_WriteCommand(0xE1);  // Set Gamma
  LCD_WriteData(0x00);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x14);
  LCD_WriteData(0x03);
  LCD_WriteData(0x11);
  LCD_WriteData(0x07);
  LCD_WriteData(0x31);
  LCD_WriteData(0xC1);

  LCD_WriteData(0x48);
  LCD_WriteData(0x08);
  LCD_WriteData(0x0F);
  LCD_WriteData(0x0C);
  LCD_WriteData(0x31);
  LCD_WriteData(0x36);
  LCD_WriteData(0x0F);

  // Exit Sleep
  LCD_WriteCommand(0x11);
  HAL_Delay(120);

  // Display ON
  LCD_WriteCommand(0x29);
}

void LCD_SetAddress(uint16_t x, uint16_t y, uint16_t w, uint16_t h){
    WriteCommand(0x2A); // Column address set
    LCD_WriteData((x >> 8) & 0xFF);
    LCD_WriteData(x & 0xFF);
    LCD_WriteData(((x + w - 1) >> 8) & 0xFF);
    LCD_WriteData((x + w - 1) & 0xFF);

    WriteCommand(0x2B); // Page address set
    LCD_WriteData((y >> 8) & 0xFF);
    LCD_WriteData(y & 0xFF);
    LCD_WriteData(((y + h - 1) >> 8) & 0xFF);
    LCD_WriteData((y + h - 1) & 0xFF);

    WriteCommand(0x2C); // Memory write
}

  void LCD_DrawPixel(uint16_t x, uint16_t y, uint32_t color){
    LCD_SetAddress(x, y, 1, 1);

    LCD_WriteData((color >> 8) & 0xFF); // High byte
    LCD_WriteData(color & 0xFF); // Low byte
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

  InitializeSequence();
  // serial::uart::Transmit(&uart1, data.c_str());
  int index = 0;
  while(index < 11){
    LCD_DrawPixel(10, 10, 0xFF0000);
    std::string data = "Transmitting data through UART!";
    data += " index = " + std::to_string(index++) + "\n";
    stm32f4::serial::print(&uart, data.c_str());
  }

  return 0;
}