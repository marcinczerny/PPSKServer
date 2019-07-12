#!/usr/bin/env python

#ARDUINO
CONST_SERIAL_RPI_INITIALIZED = 48
CONST_SERIAL_RPI_STOP = 49
CONST_SERIAL_RPI_START = 50
CONST_SERIAL_RPI_SPEED = 51
CONST_SERIAL_RPI_DIRECTION = 52
CONST_STATE_MOVEMENT = 70
CONST_STATE_STOP = 71
CONST_STATE_STAIRS = 72
CONST_STATE_OBSTACLE = 73
CONST_OBSTACLE_SWITCHES = 74
CONST_OBSTACLE_SONAR_FRONT = 75
CONST_OBSTACLE_SONAR_BACK = 76
CONST_FLOOR = 77
CONST_LIMIT = 78
CONST_MPU = 79
CONST_STOP_PACKET = 255

#CONTROL
CONST_STOP = 1 
CONST_START = 2
CONST_INITIALIZE = 3
CONST_SPEED = 4
CONST_DIRECTION = 5
CONST_READ_STATE = 6
CONST_READ_DISTANCE_FRONT = 7
CONST_READ_DISTANCE_REAR = 8
CONST_READ_FLOOR = 9
CONST_READ_LIMIT_SWITCHES = 0
#STATE

#RESPONSE
CONST_RESPONSE_STATE="S"
CONST_RESPONSE_DISTANCE_FRONT = "F"
CONST_RESPONSE_LIMIT_SWITCHES = "L"
CONST_RESPONSE_DISTANCE_READ = "R"
CONST_RESPONSE_FLOOR = "P"
CONST_RESPONSE_POSITION = "O"

#TIMER
CONST_TIMER = 2