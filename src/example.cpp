/*

// Initializng our pins for communication
#define LCD_CS_PIN       GPIO_PIN_6
#define LCD_CS_GPIO_PORT GPIOB
#define LCD_DC_PIN       GPIO_PIN_7
#define LCD_DC_GPIO_PORT GPIOB
#define LCD_RESET_PIN    GPIO_PIN_8
#define LCD_RESET_GPIO_PORT GPIOB

// Setting our pins to being high and low
#define LCD_CS_LOW()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)

#define LCD_CS_HIGH()    HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()    HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RESET_LOW()  HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_RESET)
#define LCD_RESET_HIGH() HAL_GPIO_WritePin(LCD_RESET_GPIO_PORT, LCD_RESET_PIN, GPIO_PIN_SET)

// Initializing SPI for 
SPI_HandleTypeDef hspi1;

*/