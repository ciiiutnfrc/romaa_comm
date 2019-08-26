#include "romaa_comm.h"

// Constructor.
romaa_comm::romaa_comm(const std::string& port, const uint32_t baudrate)
{
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if(fd < 0)
  {
    std::cout << "[RoMAA comm ERROR]: opening port " << port << std::endl;
    connected = false;
    return;
  }

  speed_t speed;            // termios type
  switch(baudrate)
  {
    case 115200: speed = B115200;
                 break;
    case 57600: speed = B57600;
                break;
    case 38400: speed = B38400;
                break;
    case 19200: speed = B19200;
                break;
    case 9600: speed = B9600;
               break;
  }

  if(tcgetattr(fd, &ttyold) < 0)
  {
    std::cout << "[RoMAA comm ERROR]: unable to get terminal attributes." << std::endl;
    connected = false;
    return;
  }
  ttynew = ttyold;

  cfsetospeed(&ttynew, speed);
  cfsetispeed(&ttynew, speed);

  ttynew.c_cflag = (ttynew.c_cflag & ~CSIZE) | CS8;   // 8 data bits (8)
  ttynew.c_cflag &= ~(PARENB | PARODD);               // no parity   (N)
  ttynew.c_cflag &= ~CSTOPB;                          // 1 stop bit  (1)

  ttynew.c_cflag |= (CLOCAL | CREAD);                 // ignore modem status lines, and
                                                      // enable reading
  ttynew.c_cflag &= ~CRTSCTS;                         // no flow control
  
  ttynew.c_iflag &= ~IGNBRK;                          // disable break processing
  ttynew.c_iflag &= ~(IXON | IXOFF | IXANY);          // shut off xon/xoff ctrl

  ttynew.c_lflag = 0;                                 // no signaling chars, no echo,
  ttynew.c_oflag = 0;                                 // no remapping, no delays
  // Read "Beginning Linux Programming (4th Edition) – Ch05".
  ttynew.c_cc[VMIN]  = 10;
  ttynew.c_cc[VTIME] = 10;

  /*
   * TCSANOW: Make the change immediately.
   * TCSADRAIN: Make the change after waiting until all queued output has been written. 
   * TCSAFLUSH: This is like TCSADRAIN, but also discards any queued input.
   */
  if(tcsetattr(fd, TCSAFLUSH, &ttynew) != 0)
  {
    std::cout << "[RoMAA comm ERROR]: unable to set terminal attributes." << std::endl;
    connected = false;
    return;
  }

  this->set_speed(0.0, 0.0);
  connected = true;
}

// Destructor.
romaa_comm::~romaa_comm()
{
  this->disable_motor();
  this->set_speed(0.0, 0.0);
  connected = false;
  close(fd);
}

// Is connected?.
bool romaa_comm::is_connected()
{
  return connected;
}

// Reset command.
void romaa_comm::reset()
{
  send_package(CMD_RESET);
}

// Enable motor command.
void romaa_comm::enable_motor()
{
  send_package(CMD_ENABLE_MOTOR);
}

// Disable motor command.
void romaa_comm::disable_motor()
{
  send_package(CMD_DISABLE_MOTOR);
}

// Set v PID.
void romaa_comm::set_v_pid(const float kp, const float ki, const float kd)
{
  send_package(CMD_SET_MOTOR_PID, kp, ki, kd);
}

// Get v PID.
int8_t romaa_comm::get_v_pid(float& kp, float& ki, float& kd)
{
  ssize_t n;
  send_package(CMD_GET_MOTOR_PID);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_3FLOAT+2);
  if(n != PACKET_SIZE_3FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_v_pid)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_3FLOAT) )
  {
    kp = buffer_to_float(&(buffer[2]));
    ki = buffer_to_float(&(buffer[6]));
    kd = buffer_to_float(&(buffer[10]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_v_pid)." << std::endl;
    return -1;
  }
}

// Set w PID.
void romaa_comm::set_w_pid(const float kp, const float ki, const float kd)
{
  send_package(CMD_SET_VW_PID, kp, ki, kd);
}

// Get w PID.
int8_t romaa_comm::get_w_pid(float& kp, float& ki, float& kd)
{
  ssize_t n;
  send_package(CMD_GET_VW_PID);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_3FLOAT+2);
  if(n != PACKET_SIZE_3FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_w_pid)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_3FLOAT) )
  {
    kp = buffer_to_float(&(buffer[2]));
    ki = buffer_to_float(&(buffer[6]));
    kd = buffer_to_float(&(buffer[10]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_w_pid)." << std::endl;
    return -1;
  }
}

// Set speed.
void romaa_comm::set_speed(const float v, const float w)
{
  send_package(CMD_SET_SPEED, v, w);
}

// Get speed.
int8_t romaa_comm::get_speed(float& v, float& w)
{
  ssize_t n;
  send_package(CMD_GET_SPEED);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_2FLOAT+2);
  if(n != PACKET_SIZE_2FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_speed)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_2FLOAT) )
  {
    v = buffer_to_float(&(buffer[2]));
    w = buffer_to_float(&(buffer[6]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_speed)." << std::endl;
    return -1;
  }
}

// Reset odometry.
void romaa_comm::reset_odometry()
{
  send_package(CMD_RESET_ODOMETRY);
}

// Set odometry.
void romaa_comm::set_odometry(const float x, const float y, const float a)
{
  send_package(CMD_SET_ODOMETRY, x, y, a);
}

// Get odometry.
int8_t romaa_comm::get_odometry(float& x, float& y, float& a)
{
  ssize_t n;
  send_package(CMD_GET_ODOMETRY);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_3FLOAT+2);
  if(n != PACKET_SIZE_3FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_odometry)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_3FLOAT) )
  {
    x = buffer_to_float(&(buffer[2]));
    y = buffer_to_float(&(buffer[6]));
    a = buffer_to_float(&(buffer[10]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_odometry)." << std::endl;
    return -1;
  }
}

// Set PWM
void romaa_comm::set_pwm(const int32_t pwm1, const int32_t pwm2)
{
  send_package(CMD_SET_PWM, pwm1, pwm2);
}

// Get PWM
int8_t romaa_comm::get_pwm(int32_t& pwm1, int32_t& pwm2)
{
  ssize_t n;
  send_package(CMD_GET_PWM);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_2INT+2);
  if(n != PACKET_SIZE_2INT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_pwm)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_2INT) )
  {
    pwm1 = buffer_to_int32(&(buffer[2]));
    pwm2 = buffer_to_int32(&(buffer[6]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_pwm)." << std::endl;
    return -1;
  }
}

// Set kinematic parameters.
void romaa_comm::set_kinematic_params(const float wheel_radious, const float wheelbase)
{
  std::cout << "[RoMAA comm WAR]: not implemented." << std::endl;
}

// Get kinematic parameters.
int8_t romaa_comm::get_kinematic_params(float& wheel_radious, float& wheelbase)
{
  ssize_t n;
  send_package(CMD_GET_KINEMATIC);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_2FLOAT+2);
  if(n != PACKET_SIZE_2FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_kinematic_params)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_2FLOAT) )
  {
    wheel_radious = buffer_to_float(&(buffer[2]));
    wheelbase = buffer_to_float(&(buffer[6]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_kinematic_params)." << std::endl;
    return -1;
  }
}

// Get body geometry.
int8_t romaa_comm::get_body_geometry(float& width, float& lenght, float& height)
{
  ssize_t n;
  send_package(CMD_GET_BODY_GEOM);

  memset(buffer, 0, BUFFER_SIZE);
  tcflush(fd, TCIFLUSH);
  n = read(fd, buffer, PACKET_SIZE_3FLOAT+2);
  if(n != PACKET_SIZE_3FLOAT + 2)
  {
    std::cout << "[RoMAA comm ERROR]: Read failed (get_body_geometry)." << std::endl;
    return -1;
  }
  if( (buffer[0] == CMD_INIT_FRAME) &&
      (buffer[1] == PACKET_SIZE_3FLOAT) )
  {
    width = buffer_to_float(&(buffer[2]));
    lenght = buffer_to_float(&(buffer[6]));
    height = buffer_to_float(&(buffer[10]));
    return 0;
  }
  else
  {
    std::cout <<"[RoMAA comm ERROR]: bad frame (get_body_geometry)." << std::endl;
    return -1;
  }
}

// Send package (Only command).
void romaa_comm::send_package(const uint8_t& cmd)
{
  buffer[0] = CMD_INIT_FRAME;
  buffer[1] = PACKET_SIZE_CMD;
  buffer[2] = cmd;

  if( write(fd, buffer, PACKET_SIZE_CMD+2) != static_cast<int>(PACKET_SIZE_CMD+2) )
  {
    std::cout << "[RoMAA comm ERROR]: Did not write enough bytes." << std::endl;
  }

  tcflush(fd, TCOFLUSH);
}

// Send package (1 uint8 parameter).
void romaa_comm::send_package(const uint8_t& cmd, const uint8_t ui8_param)
{
  std::cout << "[RoMAA comm WAR]: not implemented." << std::endl;
}

// Send package (2 uint32 parameters).
void romaa_comm::send_package(const uint8_t& cmd,
    const int32_t i32_param1, const int32_t i32_param2)
{
  buffer[0] = CMD_INIT_FRAME;
  buffer[1] = PACKET_SIZE_CMD + PACKET_SIZE_2INT;
  buffer[2] = cmd;
  append_buffer(&(buffer[3]), (uint8_t*)(&i32_param1), 4);
  append_buffer(&(buffer[7]), (uint8_t*)(&i32_param2), 4);

  if( write(fd, buffer, PACKET_SIZE_CMD + PACKET_SIZE_2INT + 2) !=
      static_cast<int>(PACKET_SIZE_CMD + PACKET_SIZE_2INT + 2) )
  {
    std::cout << "[RoMAA comm ERROR]: Did not write enough bytes." << std::endl;
  }
}

// Send package (2 float parameters).
void romaa_comm::send_package(const uint8_t& cmd,
    const float f_param1, const float f_param2)
{
  buffer[0] = CMD_INIT_FRAME;
  buffer[1] = PACKET_SIZE_CMD + PACKET_SIZE_2FLOAT;
  buffer[2] = cmd;
  append_buffer(&(buffer[3]), (uint8_t*)(&f_param1), 4);
  append_buffer(&(buffer[7]), (uint8_t*)(&f_param2), 4);

  if( write(fd, buffer, PACKET_SIZE_CMD + PACKET_SIZE_2FLOAT + 2) != 
      static_cast<int>(PACKET_SIZE_CMD + PACKET_SIZE_2FLOAT + 2) )
  {
    std::cout << "[RoMAA comm ERROR]: Did not write enough bytes." << std::endl;
  }

  tcflush(fd, TCOFLUSH);
}

// Send package (3 float parameters).
void romaa_comm::send_package(const uint8_t& cmd, 
    const float f_param1, const float f_param2, const float f_param3)
{
  buffer[0] = CMD_INIT_FRAME;
  buffer[1] = PACKET_SIZE_CMD + PACKET_SIZE_3FLOAT;
  buffer[2] = cmd;
  append_buffer(&(buffer[3]), (uint8_t*)(&f_param1), 4);
  append_buffer(&(buffer[7]), (uint8_t*)(&f_param2), 4);
  append_buffer(&(buffer[11]), (uint8_t*)(&f_param3), 4);

  if( write(fd, buffer, PACKET_SIZE_CMD + PACKET_SIZE_3FLOAT + 2) != 
      static_cast<int>(PACKET_SIZE_CMD + PACKET_SIZE_3FLOAT + 2) )
    std::cout << "[RoMAA comm ERROR]: Did not write enough bytes." << std::endl;

  tcflush(fd, TCOFLUSH);
}

