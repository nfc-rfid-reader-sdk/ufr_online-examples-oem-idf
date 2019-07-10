/* uFR Online OEM demo */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "ESP_LOG.h"
#include <string.h>
#include "online.h"

void app_main()
{
    /* Initialise RGB LED */
 	initialise_rgb();

    /* Initialise UART */
    initialise_uart();

    vTaskDelay(300 / portTICK_PERIOD_MS);

    /* Set both RGB LEDs to white (75, 75, 75) */
    set_led(75,75,75,75,75,75);

    /* Create card_id_ex_task */
	xTaskCreate(card_id_ex_task, "card_id_ex_task", BUFFER_SIZE, NULL, 5, NULL);


}

/* Card UID polling (200ms) */
static void card_id_ex_task(void *pvParameters)
{
    int status;
    uint8_t uid[10];
    uint8_t temp_uid[10];
    uint8_t uid_size, card_type;  

    while (1)
    {
        status = card_id_ex(uid, &uid_size, &card_type);
        if(status == 0x00)
        {
            /* Check if last UID is different from current UID */
            if(memcmp(uid, temp_uid, uid_size) != 0)
            {
                /* If card is present wirte UID on LOG output (USB or UART_0) */
                ESP_LOG_BUFFER_HEX("UID", uid, uid_size);

                /* Beep on card present */
                reader_ui_signal(0, 1);

                /* Turn green LED on card present for 500ms */
                set_led(0,75,0,0,75,0);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                set_led(75,75,75,75,75,75);
            }
             memcpy(temp_uid, uid, uid_size);
        }
        else
        {
            memset(uid, 0xff, 10);
            memset(temp_uid, 0x00, 10);
        }
               
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);    
}


/* Initialise UART communication */
static void initialise_uart()
{

	//USB_UART
	uart_config_t usb_uart_config = {
		.baud_rate = 115200,
		.data_bits = USB_UART_DATA_BITS,
		.parity = USB_UART_PARITY,
		.stop_bits = USB_UART_STOP_BITS,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
	uart_param_config(UART_NUM_0, &usb_uart_config);
	uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUFFER_SIZE, 0, 0, NULL, 0));

	//UART1
	uart_config_t uart1_config = {
		.baud_rate = 115200,
		.data_bits = UART1_DATA_BITS,
		.parity = UART1_PARITY,
		.stop_bits = UART1_STOP_BITS,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
	uart_param_config(UART_NUM_1, &uart1_config);
	uart_set_pin(UART_NUM_1, UART1_TXPIN, UART1_RXPIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUFFER_SIZE, 0, 0, NULL, 0));

	vTaskDelay(50 / portTICK_PERIOD_MS);

	//UART RESET
	gpio_pad_select_gpio(GPIO_NUM_32);
	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_32, 1);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	gpio_set_level(GPIO_NUM_32, 0);

	uint8_t rx_buffer[4];
	uart_read_bytes(UART_NUM_1, rx_buffer, 4, 50 / portTICK_RATE_MS);

}

/* Initialise RGB LED driver */
static void initialise_rgb()
{
	gpio_pad_select_gpio(13);
	gpio_set_direction(13, GPIO_MODE_OUTPUT);
	gpio_set_level(13, LOW);
	if (digitalLeds_initStrands(LED, 1))
	{
		   ESP_LOGE(TAG, "LED Init error");
	}
}

/* Set LED color */
static void set_led(uint8_t red1, uint8_t green1, uint8_t blue1, uint8_t red2, uint8_t green2, uint8_t blue2)
{
    LED[0].pixels[1] = pixelFromRGB(red1, green1, blue1);
	LED[0].pixels[0] = pixelFromRGB(red2, green2, blue2);
	digitalLeds_updatePixels(&LED[0]);
}

/* send uFR Nano UI Signal on UART1 */
static int reader_ui_signal(uint8_t light, uint8_t beep)
{
	uint8_t buff[7] = { 0x55,  0x26, 0xAA, 0, 0, 0, 0 };
	uint8_t ext_len;

	buff[4] = light;
	buff[5] = beep;

	return sendCommand(buff, &ext_len);
}

/* Read card UID */
static int card_id_ex(uint8_t *uid, uint8_t *uid_size, uint8_t *card_type)
{
	uint8_t buff[20] = { 0x55, 0x2C, 0xAA, 0, 0, 0, 0 };
	uint8_t ext_len;

	int status = sendCommand(buff, &ext_len);
    if(status != 0)
    {
        return status;
    }

    *card_type = buff[4];
    *uid_size = buff[5];

    if ((status = ufr_uart_read(buff, ext_len)) != 0)
    {
		return status;
    }

    if ((status = check_checksum(buff, ext_len)) != 0)
    {
		return status;
    } 

    memset(uid, 0, 10);
    memcpy(uid, &buff[0], *uid_size);

    return 0;
}

/* Send CMD to UART1 */
int sendCommand(uint8_t *data, uint8_t *ext_read_len)
{
	uint8_t command = data[1];
	int status;

	uart_flush(UART_NUM_1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
	calculate_checksum(data, 7);
    
	if ((status = ufr_uart_write(data, 7)) != 0)
    {
		return status;
    }

	if ((status = ufr_uart_read(data, 7)) != 0)
    {
		return status;
    }

	if ((status = check_checksum(data, 7)) != 0)
    {
		return status;
    }    

	if ((data[0] == 0xEC) && (data[2] == 0xCE))
    {
		return data[1];
    }

	if ((data[1] != command) || (((data[0] != 0xDE) || (data[2] != 0xED)) && ((data[0] != 0xAC) || (data[2] != 0xCA))))
    {
        return -1;
    }

	*ext_read_len = data[3];
	return 0;
}

/* Calculate checksum */
static uint8_t calculate_checksum(uint8_t *data, int len)
{
	uint8_t sum = data[0];

	for (uint16_t i = 1; i < (len - 1); i++)
	{
		sum ^= data[i];
	}
    data[len -1] = sum + 0x07;
	return sum + 0x07;
}

/* Check checksum */
static uint8_t check_checksum(uint8_t *data, uint16_t len)
{
	uint8_t sum = data[0];

	for (uint16_t i = 1; i < (len - 1); i++)
	{
		sum ^= data[i];
	}
	sum += 0x07;

    if(sum == data[len - 1])
    {
        return 0;
    }

    return -1;
}

/* Write byte array to UART1 */
static int ufr_uart_write(uint8_t *data, int len)
{
    int status = uart_write_bytes(UART_NUM_1, (const char *)data, len);

    if(status == len)
    {
        return 0;
    }

    return -1;
}

/* Read byte array from UART1 */
static int ufr_uart_read(uint8_t *data, int len)
{
    int status = uart_read_bytes(UART_NUM_1, data, len, UART1_TIMEOUT / portTICK_RATE_MS);

    if(status == len)
    {
        return 0;
    }

    return -1;
}



