#ifndef SENSOR
#define SENSOR

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "stdint.h"


#define LD2410_MAX_FRAME_LENGTH 40
#ifndef LD2410_BUFFER_SIZE
#define LD2410_BUFFER_SIZE 256
#endif

#define TXD_PIN (17)
#define RXD_PIN (18)
#define BLINK_GPIO 15

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
}frame_data_t;

// kiem tra co su hien dien cua con nguoi hay khong
bool presence_detected();

/**
 * @brief tra ve true neu co doi tuong tinh, false neu nguoc lai
 *
 */
bool stationary_target_detected();

// tra ve khoang cach den doi tuong tinh duoc phat hien
uint16_t stationary_target_distance();

// tra ve nang luong cua doi tuong
uint8_t stationary_target_energy();

// tra ve true neu co doi tuong di chuyen, false neu nguoc lai
bool moving_target_detected();

// tra ve khoang cach den doi tuong dang di chuyen
uint16_t moving_target_distance();

// tra ve nang luong cua doi tuong di chuyen
uint8_t moving_target_energy();

/**
 * @brief kiem tra tinh toan ven cua khung data
 * @param radar khung du lieu cua cam bien
 *
 * @return  true neu khung hoan chinh, false neu co thieu hoac sai sot
 */
bool check_frame_end_(uint8_t *radar);

/**
 * @brief dich du lieu tho cua cam bien
 *
 * @param radar_data_frame_ khung du lieu tho cua cam bien
 * @return true neu dich thanh cong, false neu khong thanh cong
 */
bool parse_data_frame_(uint8_t *radar_data_frame_);

/**
 * @brief read data
 *
 * @param data truyen vapvkhi abc
 * @return true neu doc hoan chinh 1 khung data, false neu khong thanh cong
 */
bool read_frame_data(uint8_t *data);


#endif
