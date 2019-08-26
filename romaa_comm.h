#ifndef _ROMAA_COMM
#define _ROMAA_COMM

#include <iostream>
#include "commands.h"

#include <unistd.h>   // Unix standar function definitions.
#include <fcntl.h>    // File control definitions.
#include <string.h>   // For memset.
#include <termios.h>

#define BUFFER_SIZE 1024

#define   PACKET_SIZE_CMD         1
#define   PACKET_SIZE_1BYTE       1
#define   PACKET_SIZE_2INT        8
#define   PACKET_SIZE_2FLOAT      8
#define   PACKET_SIZE_3FLOAT      12

// RoMMA communication 4bytes-uint16.
typedef union {
  uint8_t data_ui8[4];
  int32_t data_i32;
} romaa_int32_data_t;

// RoMAA communication 4byte-float.
typedef union {
  uint8_t data_ui8[4];
  float data_f;
} romaa_float_data_t;

class romaa_comm
{
  public:
    // Constructor
    romaa_comm(const std::string& port, const uint32_t baudrate);
    ~romaa_comm();

    bool is_connected();
    void reset();

    // Motor methods.
    void enable_motor();
    void disable_motor();

    // Linear speed PID.
    void set_v_pid(const float kp, const float ki, const float kd);
    int8_t get_v_pid(float& kp, float& ki, float& kd);

    // Angular speed PID.
    void set_w_pid(const float kp, const float ki, const float kd);
    int8_t get_w_pid(float& kp, float& ki, float& kd);

    // Speed methods.
    void set_speed(const float v, const float w);
    int8_t get_speed(float& v, float& w);

    // PWM methods.
    void set_pwm(const int32_t pwm1, const int32_t pwm2);
    int8_t get_pwm(int32_t& pwm1, int32_t& pwm2);

    // Odometry methods.
    void reset_odometry();
    void set_odometry(const float x, const float y, const float a);
    int8_t get_odometry(float& x, float& y, float& a);

    // Kinematic parameters.
    void set_kinematic_params(const float wheelbase, const float wheel_radious);
    int8_t get_kinematic_params(float& wheelbase, float& wheel_radious);
    int8_t get_body_geometry(float& width, float& lenght, float& height);

  private:
    bool connected;
    int fd;
    struct termios ttyold, ttynew;
    uint8_t buffer[BUFFER_SIZE];

    // Append data to serial buffer.
    void append_buffer(uint8_t * buff, const uint8_t * data, uint32_t n)
    {
      for(uint32_t i = 0; i < n; i++)
        buff[i] = data[i];
    }
    
    // Send empty package (only command).
    void send_package(const uint8_t& cmd);

    // Send 1 uint8_t package.
    void send_package(const uint8_t& cmd, const uint8_t ui8_param);

    // Send 2 int32_t package.
    void send_package(const uint8_t& cmd,
        const int32_t i32_param1, const int32_t i32_param2);

    // Send 2 float package.
    void send_package(const uint8_t& cmd,
        const float f_param1, const float f_param2);

    // Send 3 float package.
    void send_package(const uint8_t& cmd, 
        const float f_param1, const float f_param2, const float f_param3);

    // Bytes to uint32.
    int32_t buffer_to_int32(uint8_t * buff)
    {
      romaa_int32_data.data_ui8[0] = buff[0];
      romaa_int32_data.data_ui8[1] = buff[1];
      romaa_int32_data.data_ui8[2] = buff[2];
      romaa_int32_data.data_ui8[3] = buff[3];
      return romaa_int32_data.data_i32;
    }

    // Bytes to float.
    float buffer_to_float(uint8_t * buff)
    {
      romaa_float_data.data_ui8[0] = buff[0];
      romaa_float_data.data_ui8[1] = buff[1];
      romaa_float_data.data_ui8[2] = buff[2];
      romaa_float_data.data_ui8[3] = buff[3];
      return romaa_float_data.data_f;
    }

    romaa_int32_data_t romaa_int32_data;
    romaa_float_data_t romaa_float_data;
};

#endif

