#ifndef COMMAND_H
#define COMMAND_H

// defines of comunication frames keys
#define CMD_INIT_FRAME		        '*'       // init frame
#define CMD_RESET   		          'r'       // reset all

#define CMD_SET_OPEN_LOOP_MODE    'o'       // set open loop mode
#define CMD_SET_CLOSE_LOOP_MODE   'c'       // set close loop mode
#define CMD_SET_MOTOR_LOOP_MODE   'f'       // set loop motor mode
#define CMD_GET_LOOP_MODE         'e'       // get loop mode

#define CMD_ENABLE_MOTOR          'h'       // 
#define CMD_DISABLE_MOTOR         'H'       //

#define CMD_SET_MOTOR_PID				  'm'       // set motor PID controller constants
#define CMD_GET_MOTOR_PID         'M'       // get motor PID controller constants
#define CMD_SET_VW_PID            'v'       // set vw PID controller constants
#define CMD_GET_VW_PID            'V'       // get vw PID controller constants

#define CMD_SET_SPEED			        's'       // set speed (v,w)
#define CMD_GET_SPEED             'S'       // get speed (v,w)
#define CMD_SET_WHEEL_SPEED       'w'       // set wheel speed (v1,v2)

#define CMD_SET_PWM               'p'       // set motor PWM (in open loop)
#define CMD_GET_PWM               'P'       // get motor PWM

#define CMD_SET_T_LOOP            't'       // set close loop time
#define CMD_GET_T_LOOP            'T'       // get close loop time
#define CMD_SET_T_SAMPLING        'l'       // set sampling time of data logging
#define CMD_GET_T_SAMPLING        'L'       // get sampling time of data logging
#define CMD_SET_T_ODOMETRY        'x'       // set update odometry time
#define CMD_GET_T_ODOMETRY        'X'       // get update odometry time

#define CMD_SET_ODOMETRY          'y'       // set odometry (x,y,theta)
#define CMD_GET_ODOMETRY          'Y'       // get odometry (x,y,theta)
#define CMD_RESET_ODOMETRY        'z'       // reset odometry (x,y,theta)=(0,0,0)

#define CMD_GET_ENC_COUNTER       'A'       // get encoder counters (in number of pulses)
#define CMD_GET_WHEEL_V           'B'       // get wheel linear speed
#define CMD_GET_WHEEL_W           'C'       // get wheel angular speed
#define CMD_GET_WHEEL_D           'D'       // get wheel linear displacement

#define CMD_LOG_STOP              '0'       // stop data logging
#define CMD_LOG_WHEEL_V           '1'       // start wheel linear speed data logging
#define CMD_LOG_WHEEL_W           '2'       // start wheel angular speed data logging
#define CMD_LOG_WHEEL_D           '3'       // start wheel linear displacement data logging
#define CMD_LOG_ENC_COUNTER       '4'       // start encoder counters data logging
#define CMD_LOG_PWM               '5'       // start PWM data loggint
#define CMD_LOG_VW                '6'       // start speed (v,w) data logging
#define CMD_LOG_ODOM              '7'       // start odometry data logging

#define CMD_SET_KINEMATIC         'k'       // set kinematic parameters (r,b)
#define CMD_GET_KINEMATIC         'K'       // get kinematic parameters (r,b)

#define CMD_GET_BODY_GEOM         'G'       // get body geometry (width,lenght,high)
#define CMD_GET_ODOMETRY_OFFSET   'F'       // get odometry offset (x,y)

#endif

