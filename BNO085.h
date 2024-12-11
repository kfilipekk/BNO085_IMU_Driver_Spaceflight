#ifndef BNO085_H
#define BNO085_H

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Default I2C Address
#define BNO085_DEFAULT_ADDRESS 0x4A

//SHTP Channel Numbers
#define SHTP_CHANNEL_COMMAND 0
#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_CONTROL 2
#define SHTP_CHANNEL_REPORTS 3
#define SHTP_CHANNEL_WAKE_REPORTS 4

//SHTP Report IDs
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_COMMAND_REQUEST 0xF2

//Commands from BNO085 datasheet figure 3-2
enum shtp_command_t {
    SHTP_CMD_TARE_NOW = 3,
    SHTP_CMD_SAVE_DCD = 6,
};

//Sensor Report IDs optimized for lander applications
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ACCELEROMETER 0x01

//State and Error management
enum BNO085_State_t {
    BNO085_STATE_UNINITIALIZED,
    BNO085_STATE_INITIALIZED,
    BNO085_STATE_ERROR
};

enum BNO085_Error_t {
    BNO085_SUCCESS = 0,
    BNO085_ERROR_I2C_COMM,
    BNO085_ERROR_INVALID_ID,
    BNO085_ERROR_CMD_RESPONSE,
    BNO085_ERROR_FEATURE_RESPONSE,
    BNO085_ERROR_PACKET_TOO_LARGE,
    BNO085_ERROR_READ_FAILURE,
    BNO085_ERROR_BOOT_TIMEOUT
};


class BNO085 {
public:
    BNO085();

    bool begin(i2c_port_t i2c_port, gpio_num_t int_pin = GPIO_NUM_NC);
    bool set_accelerometer_range(uint8_t range_g = 16);
    
    bool enable_rotation_vector(uint32_t interval_us = 10000);
    bool enable_gravity(uint32_t interval_us = 20000);
    bool enable_linear_acceleration(uint32_t interval_us = 10000);

    bool data_available();
    void soft_reset();
    bool has_reset();

    bool tare_now(uint8_t axis_mask = 0b111, bool use_persistent_tare = false);
    bool save_tare();

    BNO085_State_t get_state();
    BNO085_Error_t get_last_error();

    //Get quaternion values
    float get_quat_i();
    float get_quat_j();
    float get_quat_k();
    float get_quat_real();
    float get_quat_accuracy();

    //Get vector values (m/s^2)
    float get_linear_accel_x();
    float get_linear_accel_y();
    float get_linear_accel_z();
    uint8_t get_linear_accel_accuracy();

    //Get vector values (m/s^2)
    float get_gravity_x();
    float get_gravity_y();
    float get_gravity_z();
    uint8_t get_gravity_accuracy();

private:
    i2c_port_t _i2c_port;
    gpio_num_t _int_pin;
    BNO085_State_t _state;
    BNO085_Error_t _last_error;
    
    uint8_t _shtp_header[4];
    uint8_t _shtp_data[300]; //Packet buffer
    uint8_t _sequence_number[6]; //Sequence number for each channel

    //Parsed sensor data
    float _q_i, _q_j, _q_k, _q_real;
    float _quat_accuracy;
    float _lax, _lay, _laz;
    uint8_t _linear_accel_accuracy;
    float _gravx, _gravy, _gravz;
    uint8_t _gravity_accuracy;

    esp_err_t i2c_read_shtp_header();
    esp_err_t i2c_read_shtp_payload(uint16_t length);
    bool receive_packet();
    bool send_packet(uint8_t channel, uint8_t length, const uint8_t* data);
    void parse_input_report(uint8_t* report, uint16_t report_length);
    bool set_feature_command(uint8_t report_id, uint32_t time_between_reports, uint32_t specific_config = 0);
    bool send_command(uint8_t command, uint8_t p1 = 0, uint8_t p2 = 0, uint8_t p3 = 0, uint8_t p4 = 0);
    bool wait_for_initialization(uint32_t timeout_ms = 5000);
};

#endif
