/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include"sensor/sensor.h"


void init_uart_sensor(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, LD2410_BUFFER_SIZE * 3, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    uint8_t data[LD2410_BUFFER_SIZE + 10];
    uint8_t state = 0;
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 30, 100/portTICK_PERIOD_MS);
        if (rxBytes > 0 )
        {
            read_frame_data(data);
            taskYIELD();
            // parse_data_frame_(radar_data_frame_);
            uint16_t d = moving_target_distance();
            uint16_t s = stationary_target_distance();
            printf("movingTargetDistance: %d\n", d);
            //printf("stationaryTargetDistance: %d\n", s);
            if ( d < 200 && d > 60 )                      // turn on led when target is in range from 60 to 200
            {
                state = 1;
                gpio_set_level(BLINK_GPIO, state);
                //vTaskDelay(100/portTICK_PERIOD_MS);
                //state = !state;

            }else{
                state = 0;
                gpio_set_level(BLINK_GPIO, state);
            }
            //vTaskDelay(50/portTICK_PERIOD_MS);
        }

    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    init_uart_sensor();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 5, NULL, 2, NULL);
        //    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}
