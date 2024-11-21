#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "sensor.h"

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
