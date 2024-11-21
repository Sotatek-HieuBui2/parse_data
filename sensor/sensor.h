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

bool presenceDetected();
bool stationaryTargetDetected();
uint16_t stationaryTargetDistance();
uint8_t stationaryTargetEnergy();
bool movingTargetDetected();
uint16_t movingTargetDistance();
uint8_t movingTargetEnergy();
bool check_frame_end_(uint8_t *rada);
bool parse_data_frame_(uint8_t *radar_data_frame_);
bool read_frame_(uint8_t *data);
