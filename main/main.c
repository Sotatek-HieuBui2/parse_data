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


#define LD2410_MAX_FRAME_LENGTH 40
#ifndef LD2410_BUFFER_SIZE
#define LD2410_BUFFER_SIZE 256
#endif

#define TXD_PIN (17)
#define RXD_PIN (18)
#define BLINK_GPIO 15

#if 1
typedef struct {
    const uint8_t* data;
    uint16_t length;

	uint8_t radar_data_frame_[LD2410_MAX_FRAME_LENGTH];				//Store the incoming data from the radar, to check it's in a valid format
	uint8_t radar_data_frame_position_;							//Where in the frame we are currently writing
	bool frame_started_;									//Whether a frame is currently being read
	bool ack_frame_;										//Whether the incoming frame is LIKELY an ACK frame
	bool waiting_for_ack_;									//Whether a command has just been sent
	uint8_t target_type_;
	uint16_t moving_target_distance_;
	uint8_t moving_target_energy_;
	uint16_t stationary_target_distance_;
	uint8_t stationary_target_energy_;
    uint16_t last_valid_frame_length;
}frameData;
    frameData frame_data;

bool presenceDetected()
{
	return frame_data.target_type_ != 0;
}

bool stationaryTargetDetected()
{
	if((frame_data.target_type_ & 0x02) && frame_data.stationary_target_distance_ > 0 && frame_data.stationary_target_energy_ > 0)
	{
		return true;
	}
	return false;
}

uint16_t stationaryTargetDistance()
{
	//if(stationary_target_energy_ > 0)
	{
		return frame_data.stationary_target_distance_;
	}
	//return 0;
}

uint8_t stationaryTargetEnergy()
{
	//if(stationary_target_distance_ > 0)
	{
		return frame_data.stationary_target_energy_;
	}
	//return 0;
}

bool movingTargetDetected()
{
	if((frame_data.target_type_ & 0x01) && frame_data.moving_target_distance_ > 0 && frame_data.moving_target_energy_ > 0)
	{
		return true;
	}
	return false;
}

uint16_t movingTargetDistance()
{
	//if(moving_target_energy_ > 0)
	{
		return frame_data.moving_target_distance_;
	}
	//return 0;
}

uint8_t movingTargetEnergy() {
    if (frame_data.moving_target_energy_ > 100) {
        return 100;  // limit target energy is 100
    }
    return frame_data.moving_target_energy_;  // show target energy in range from 0 to 100
}


bool check_frame_end_(uint8_t *rada) {
    if (frame_data.ack_frame_) {
        return (frame_data.radar_data_frame_[0] == 0xFD &&
                frame_data.radar_data_frame_[1] == 0xFC &&
                frame_data.radar_data_frame_[2] == 0xFB &&
                frame_data.radar_data_frame_[3] == 0xFA &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 4] == 0x04 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 3] == 0x03 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 2] == 0x02 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 1] == 0x01);
    } else {
        return (frame_data.radar_data_frame_[0] == 0xF4 &&
                frame_data.radar_data_frame_[1] == 0xF3 &&
                frame_data.radar_data_frame_[2] == 0xF2 &&
                frame_data.radar_data_frame_[3] == 0xF1 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 4] == 0xF8 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 3] == 0xF7 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 2] == 0xF6 &&
                frame_data.radar_data_frame_[frame_data.radar_data_frame_position_ - 1] == 0xF5);
    }
}

bool parse_data_frame_(uint8_t *radar_data_frame_) {
    uint16_t intra_frame_data_length = radar_data_frame_[4] | (radar_data_frame_[5] << 8);

    // check length of data frame is valid index
    if (frame_data.radar_data_frame_position_ != intra_frame_data_length + 10) {
        return false;
    }

    // check certain bytes to authorize data frame
    if (frame_data.radar_data_frame_[6] == 0x02 && frame_data.radar_data_frame_[7] == 0xAA &&
        frame_data.radar_data_frame_[17] == 0x55 && frame_data.radar_data_frame_[18] == 0x00) {

        frame_data.target_type_ = frame_data.radar_data_frame_[8];

        // extract distance and energy of target
        frame_data.stationary_target_distance_ = *(uint16_t*)(&frame_data.radar_data_frame_[9]);
        frame_data.moving_target_distance_ = *(uint16_t*)(&frame_data.radar_data_frame_[15]);
        frame_data.stationary_target_energy_ = frame_data.radar_data_frame_[14];
        frame_data.moving_target_energy_ = frame_data.radar_data_frame_[11];

        frame_data.last_valid_frame_length = frame_data.radar_data_frame_position_;  // check valid length of frame
        return true;
    }

    return false;  // valid frame
}
bool read_frame_(uint8_t *data) {
    for (size_t i = 0; i < 23; i++)
    {  // Corrected to pass byte_read by reference
        // If the frame has not started, check for the frame start
        if (!frame_data.frame_started_)
        {
            if (data[i] == 0xF4 || data[i] == 0xFD)
            {
                frame_data.radar_data_frame_[0] = data[i];
                frame_data.radar_data_frame_position_ = 1;
                frame_data.frame_started_ = true;
                frame_data.ack_frame_ = (data[i] == 0xFD);  // Determine the type of frame
            }
        }
         else
        {
            // Continue accumulating the frame bytes
            frame_data.radar_data_frame_[frame_data.radar_data_frame_position_++] = data[i];
            // After reading at least 8 bytes, verify the frame length
            if (frame_data.radar_data_frame_position_ == 8)
            {
                uint16_t intra_frame_data_length = frame_data.radar_data_frame_[4] | (frame_data.radar_data_frame_[5] << 8);

                // Check if the frame length exceeds the maximum allowed
                if (intra_frame_data_length + 10 > LD2410_MAX_FRAME_LENGTH)
                    {
                    frame_data.frame_started_ = false;
                    frame_data.radar_data_frame_position_ = 0;
                      printf("hello");
                    continue;  // Skip this frame
                }
            }
            if (frame_data.radar_data_frame_position_ >= 8 && check_frame_end_(frame_data.radar_data_frame_)) {
                frame_data.frame_started_ = false;  // Reset state for the next frame

                // Process the frame (command or data)
                if (frame_data.ack_frame_) {
                    return false;
                } else {
                    return parse_data_frame_(frame_data.radar_data_frame_);
                }
            }

            // Check if the frame is complete
        }

    }
     return false;  // No complete frame was found
}

#endif
void init(void)
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
            read_frame_(data);
            taskYIELD();
            // parse_data_frame_(radar_data_frame_);
            uint16_t d = movingTargetDistance();
            uint16_t s = stationaryTargetDistance();
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
    frame_data.radar_data_frame_position_ = 0;
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 5, NULL, 2, NULL);
        //    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}
