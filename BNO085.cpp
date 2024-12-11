#include "BNO085.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_timer.h>

static const char* TAG = "BNO085";

BNO085::BNO085() {
    for (int i = 0; i < 6; i++) {
        _sequence_number[i] = 0;
    }
    _q_i = _q_j = _q_k = _q_real = _quat_accuracy = 0;
    _lax = _lay = _laz = _linear_accel_accuracy = 0;
    _gravx = _gravy = _gravz = _gravity_accuracy = 0;
    _int_pin = GPIO_NUM_NC;
    _state = BNO085_STATE_UNINITIALIZED;
    _last_error = BNO085_SUCCESS;
}

bool BNO085::begin(i2c_port_t i2c_port, gpio_num_t int_pin) {
    _i2c_port = i2c_port;
    _int_pin = int_pin;
    _state = BNO085_STATE_UNINITIALIZED;
    _last_error = BNO085_SUCCESS;

    if (_int_pin != GPIO_NUM_NC) {
        gpio_set_direction(_int_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(_int_pin, GPIO_PULLUP_ONLY);
    }

    soft_reset();

    //Wait for BNO085 to send initialisation complete packet
    if (!wait_for_initialization(5000)) {
        _state = BNO085_STATE_ERROR;
        _last_error = BNO085_ERROR_BOOT_TIMEOUT;
        ESP_LOGE(TAG, "BNO085 failed to initialise within timeout");
        return false;
    }

    //Verify communication by requesting product ID
    uint8_t prod_id_req[] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0};
    if (!send_packet(SHTP_CHANNEL_CONTROL, 2, prod_id_req)) {
        _state = BNO085_STATE_ERROR;
        _last_error = BNO085_ERROR_I2C_COMM;
        ESP_LOGE(TAG, "Failed to send Product ID request");
        return false;
    }

    if (receive_packet()) {
        if (_shtp_data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
            _state = BNO085_STATE_INITIALIZED;
            ESP_LOGI(TAG, "BNO085 initialised successfully");
            return true;
        }
    }

    _state = BNO085_STATE_ERROR;
    _last_error = BNO085_ERROR_INVALID_ID;
    ESP_LOGE(TAG, "Invalid Product ID response");
    return false;
}

bool BNO085::wait_for_initialization(uint32_t timeout_ms) {
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        if (receive_packet()) {
            if (_shtp_header[2] == SHTP_CHANNEL_EXECUTABLE && _shtp_data[0] == 1) {
                ESP_LOGI(TAG, "BNO085 boot complete - Unsolicited Initialised received");
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void BNO085::soft_reset() {
    uint8_t reset_command[] = {1};
    send_packet(SHTP_CHANNEL_EXECUTABLE, 1, reset_command);
    ESP_LOGI(TAG, "Soft reset command sent");
}

bool BNO085::has_reset() {
    if (receive_packet()) {
        if (_shtp_header[2] == SHTP_CHANNEL_EXECUTABLE && _shtp_data[0] == 1) {
            return true;
        }
    }
    return false;
}

bool BNO085::set_accelerometer_range(uint8_t range_g) {
    //BNO085 handles this internally
    ESP_LOGI(TAG, "Accelerometer range configuration requested: %dG", range_g);
    return true;
}

bool BNO085::enable_rotation_vector(uint32_t interval_us) {
    return set_feature_command(SENSOR_REPORTID_ROTATION_VECTOR, interval_us);
}

bool BNO085::enable_linear_acceleration(uint32_t interval_us) {
    return set_feature_command(SENSOR_REPORTID_LINEAR_ACCELERATION, interval_us);
}

bool BNO085::enable_gravity(uint32_t interval_us) {
    return set_feature_command(SENSOR_REPORTID_GRAVITY, interval_us);
}


bool BNO085::data_available() {
    if (_int_pin != GPIO_NUM_NC) {
        if (gpio_get_level(_int_pin) == 1) {
            return false;
        }
    }

    if (receive_packet()) {
        //Parse sensor data reports from BNO085
        if (_shtp_header[2] == SHTP_CHANNEL_REPORTS || _shtp_header[2] == SHTP_CHANNEL_WAKE_REPORTS) {
            uint16_t data_len = ((uint16_t)_shtp_header[1] << 8 | _shtp_header[0]) & 0x7FFF;
            if (data_len > 0) {
                parse_input_report(_shtp_data, data_len);
                return true;
            }
        }
    }
    return false;
}

bool BNO085::tare_now(uint8_t axis_mask, bool use_persistent_tare) {
    uint8_t p1 = use_persistent_tare ? 0b01 : 0b00;
    return send_command(SHTP_CMD_TARE_NOW, p1, axis_mask);
}

bool BNO085::save_tare() {
    return send_command(SHTP_CMD_SAVE_DCD);
}

BNO085_State_t BNO085::get_state() { return _state; }
BNO085_Error_t BNO085::get_last_error() { return _last_error; }

float BNO085::get_quat_i() { return _q_i; }
float BNO085::get_quat_j() { return _q_j; }
float BNO085::get_quat_k() { return _q_k; }
float BNO085::get_quat_real() { return _q_real; }
float BNO085::get_quat_accuracy() { return _quat_accuracy; }

float BNO085::get_linear_accel_x() { return _lax; }
float BNO085::get_linear_accel_y() { return _lay; }
float BNO085::get_linear_accel_z() { return _laz; }
uint8_t BNO085::get_linear_accel_accuracy() { return _linear_accel_accuracy; }

float BNO085::get_gravity_x() { return _gravx; }
float BNO085::get_gravity_y() { return _gravy; }
float BNO085::get_gravity_z() { return _gravz; }
uint8_t BNO085::get_gravity_accuracy() { return _gravity_accuracy; }

esp_err_t BNO085::i2c_read_shtp_header() {
    return i2c_master_write_read_device(_i2c_port, BNO085_DEFAULT_ADDRESS, 
                                       nullptr, 0, _shtp_header, 4, 
                                       pdMS_TO_TICKS(100));
}

esp_err_t BNO085::i2c_read_shtp_payload(uint16_t length) {
    if (length > sizeof(_shtp_data)) {
        _last_error = BNO085_ERROR_PACKET_TOO_LARGE;
        return ESP_ERR_INVALID_SIZE;
    }
    return i2c_master_write_read_device(_i2c_port, BNO085_DEFAULT_ADDRESS, 
                                       nullptr, 0, _shtp_data, length, 
                                       pdMS_TO_TICKS(100));
}

bool BNO085::receive_packet() {
    //Read 4-byte SHTP header first
    if (i2c_read_shtp_header() != ESP_OK) {
        _last_error = BNO085_ERROR_I2C_COMM;
        return false;
    }

    //Extract packet length from header
    uint16_t packet_length = ((uint16_t)_shtp_header[1] << 8 | _shtp_header[0]);
    packet_length &= 0x7FFF;
    
    if (packet_length == 0) {
        return false;
    }

    uint16_t data_length = packet_length - 4;

    if (i2c_read_shtp_payload(data_length) != ESP_OK) {
        _last_error = BNO085_ERROR_READ_FAILURE;
        return false;
    }

    return true;
}

bool BNO085::send_packet(uint8_t channel, uint8_t length, const uint8_t* data) {
    uint8_t packet[length + 4];
    
    //SHTP Header
    uint16_t total_length = length + 4;
    packet[0] = total_length & 0xFF;          //LSB of length
    packet[1] = total_length >> 8;            //MSB of length
    packet[2] = channel;                      //Channel number
    packet[3] = _sequence_number[channel]++;  //Sequence number
    
    //Data payload
    for (int i = 0; i < length; i++) {
        packet[i + 4] = data[i];
    }

    esp_err_t ret = i2c_master_write_to_device(_i2c_port, BNO085_DEFAULT_ADDRESS, 
                                              packet, length + 4, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        _last_error = BNO085_ERROR_I2C_COMM;
        ESP_LOGE(TAG, "Failed to send packet on channel %d", channel);
        return false;
    }
    return true;
}

bool BNO085::set_feature_command(uint8_t report_id, uint32_t time_between_reports, uint32_t specific_config) {
    uint8_t command_packet[17];
    uint8_t cmd_seq_num = _sequence_number[SHTP_CHANNEL_CONTROL];
    
    command_packet[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Report ID
    command_packet[1] = report_id;                       //Feature Report ID
    command_packet[2] = 0;                               //Feature flags
    command_packet[3] = 0;                               //Change sensitivity (LSB)
    command_packet[4] = 0;                               //Change sensitivity (MSB)
    
    //Report interval (LSB first)
    command_packet[5] = time_between_reports & 0xFF;
    command_packet[6] = (time_between_reports >> 8) & 0xFF;
    command_packet[7] = (time_between_reports >> 16) & 0xFF;
    command_packet[8] = (time_between_reports >> 24) & 0xFF;

    for (int i = 9; i < 13; i++) command_packet[i] = 0;

    //Sensor-specific configuration word
    command_packet[13] = specific_config & 0xFF;
    command_packet[14] = (specific_config >> 8) & 0xFF;
    command_packet[15] = (specific_config >> 16) & 0xFF;
    command_packet[16] = (specific_config >> 24) & 0xFF;

    if (!send_packet(SHTP_CHANNEL_CONTROL, 17, command_packet)) {
        return false;
    }

    //Check for command response
    if (receive_packet()) {
        if (_shtp_data[0] == SHTP_REPORT_COMMAND_RESPONSE) {
            if (_shtp_data[1] == cmd_seq_num && _shtp_data[3] == 0) {
                ESP_LOGI(TAG, "Feature %02X enabled successfully", report_id);
                return true;
            } else {
                ESP_LOGW(TAG, "Feature command failed - seq: %d vs %d, status: %d", 
                        _shtp_data[1], cmd_seq_num, _shtp_data[3]);
            }
        }
    }
    _last_error = BNO085_ERROR_FEATURE_RESPONSE;
    return false;
}

bool BNO085::send_command(uint8_t command, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
    uint8_t command_packet[12];
    uint8_t cmd_seq_num = _sequence_number[SHTP_CHANNEL_CONTROL];
    
    command_packet[0] = SHTP_REPORT_COMMAND_REQUEST;
    command_packet[1] = cmd_seq_num;
    command_packet[2] = command;
    command_packet[3] = p1;
    command_packet[4] = p2;
    command_packet[5] = p3;
    command_packet[6] = p4;
    for (int i = 7; i < 12; i++) command_packet[i] = 0;

    if (!send_packet(SHTP_CHANNEL_CONTROL, 12, command_packet)) {
        return false;
    }

    //Check for command response
    if (receive_packet()) {
        if (_shtp_data[0] == SHTP_REPORT_COMMAND_RESPONSE) {
            //Check sequence number match and status
            if (_shtp_data[1] == cmd_seq_num && _shtp_data[3] == 0) {
                ESP_LOGI(TAG, "Command %02X executed successfully", command);
                return true;
            } else {
                ESP_LOGW(TAG, "Command failed - seq: %d vs %d, status: %d", 
                        _shtp_data[1], cmd_seq_num, _shtp_data[3]);
            }
        }
    }
    _last_error = BNO085_ERROR_CMD_RESPONSE;
    return false;
}


void BNO085::parse_input_report(uint8_t* report, uint16_t report_length) {
    //Extract report ID and status from BNO085 data packet
    uint8_t report_id = report[0];
    uint8_t status = report[2] & 0x03;
    
    //Helper to parse 16-bit signed values from little-endian format
    auto parse_s16 = [&](int offset) {
        return (int16_t)((uint16_t)report[offset+1] << 8 | report[offset]);
    };

    if (report_id == SENSOR_REPORTID_ROTATION_VECTOR) {
        int16_t q_i_raw = parse_s16(5);
        int16_t q_j_raw = parse_s16(7);
        int16_t q_k_raw = parse_s16(9);
        int16_t q_real_raw = parse_s16(11);
        int16_t accuracy_raw = parse_s16(13);

        float q_point_factor = 1.0f / (1 << 14);
        _q_i = q_i_raw * q_point_factor;
        _q_j = q_j_raw * q_point_factor;
        _q_k = q_k_raw * q_point_factor;
        _q_real = q_real_raw * q_point_factor;
        
        float accuracy_q_point_factor = 1.0f / (1 << 12); //Q12 format for accuracy
        _quat_accuracy = accuracy_raw * accuracy_q_point_factor;
        
        ESP_LOGD(TAG, "Rotation Vector: i=%.4f j=%.4f k=%.4f real=%.4f", 
                _q_i, _q_j, _q_k, _q_real);
    } else if (report_id == SENSOR_REPORTID_LINEAR_ACCELERATION) {
        //Linear acceleration in m/s² (gravity removed by sensor fusion)
        _lax = parse_s16(5) * (1.0f / (1 << 8)); //Q8 format
        _lay = parse_s16(7) * (1.0f / (1 << 8));
        _laz = parse_s16(9) * (1.0f / (1 << 8));
        _linear_accel_accuracy = status;
        
        ESP_LOGD(TAG, "Linear Accel: x=%.3f y=%.3f z=%.3f", _lax, _lay, _laz);
    } else if (report_id == SENSOR_REPORTID_GRAVITY) {
        //Gravity vector in m/s² (isolated by sensor fusion)
        _gravx = parse_s16(5) * (1.0f / (1 << 8)); //Q8 format
        _gravy = parse_s16(7) * (1.0f / (1 << 8));
        _gravz = parse_s16(9) * (1.0f / (1 << 8));
        _gravity_accuracy = status;
        
        ESP_LOGD(TAG, "Gravity: x=%.3f y=%.3f z=%.3f", _gravx, _gravy, _gravz);
    }
}
