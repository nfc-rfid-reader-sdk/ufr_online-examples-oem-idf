#ifndef CONFIG_H
#define CONFIG_H

#include "esp32_digital_led_lib.h"

#define HIGH 1
#define LOW 0
#define OUTPUT GPIO_MODE_OUTPUT

#ifndef __cplusplus
#define nullptr  NULL
#endif

char *TAG = "DEMO";

#define BUFFER_SIZE 2048

uint32_t USB_UART_BAUD = 115200;
#define USB_UART_DATA_BITS UART_DATA_8_BITS
#define USB_UART_PARITY UART_PARITY_DISABLE
#define USB_UART_STOP_BITS UART_STOP_BITS_1
#define USB_UART_TIMEOUT 1000
#define USB_UART_ONE_BYTE_TIMEOUT 5

uint32_t UART1_BAUD = 115200;
#define UART1_DATA_BITS UART_DATA_8_BITS
#define UART1_PARITY UART_PARITY_DISABLE
#define UART1_STOP_BITS UART_STOP_BITS_1
#define UART1_RXPIN 34
#define UART1_TXPIN 16
#define UART1_RESETPIN 32
#define UART1_TIMEOUT 1000
#define UART1_ONE_BYTE_TIMEOUT 2

uint32_t UART2_BAUD = 115200;
#define UART2_DATA_BITS UART_DATA_8_BITS
#define UART2_PARITY UART_PARITY_DISABLE
#define UART2_STOP_BITS UART_STOP_BITS_1
#define UART2_RXPIN 35
#define UART2_TXPIN 23
#define UART2_RESETPIN 32
#define UART2_TIMEOUT 1000
#define UART2_ONE_BYTE_TIMEOUT 2

strand_t LED[] = {{.rmtChannel = 1, .gpioNum = 13, .ledType = LED_WS2812B_V3, .brightLimit = 1, .numPixels =  2, .pixels = nullptr, ._stateVars = nullptr}};

static void initialise_rgb();
static void initialise_uart();
static void set_led(uint8_t red1, uint8_t green1, uint8_t blue1, uint8_t red2, uint8_t green2, uint8_t blue2);
static int reader_ui_signal(uint8_t light, uint8_t beep);
static int card_id_ex(uint8_t *uid, uint8_t *uid_size, uint8_t *card_type);
int sendCommand(uint8_t *data, uint8_t *ext_read_len);
static uint8_t calculate_checksum(uint8_t *data, int len);
static uint8_t check_checksum(uint8_t *data, uint16_t len);
static int ufr_uart_write(uint8_t *data, int len);
static int ufr_uart_read(uint8_t *data, int len);
static void card_id_ex_task(void *pvParameters);
#endif
