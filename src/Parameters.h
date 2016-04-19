#pragma once
#ifndef PARAMETERS_H
#define PARAMETERS_H

#define NUM_DRIVES 12 // The number of motors we are controlling.
#define ARDUINO_PORT_NUMBER 1
#define ARDUINO_BAUD 256000 // 256000bps = 25600 bytes/sec (10 bits/byte) -> 1969 fps (12 bytes/frame)
//#define ARDUINO_READ_PERIOD 800 

#define DEFAULT_LOOP_PERIOD_MICROSECONDS 30000//1000 // 1000 times per second (500Hz)
#define MIN_LOOP_PERIOD 2000
#define MAX_LOOP_PERIOD 100000

// Drive position limits for ~120mm draw
#define POSITION_MAX 120000//90000
#define POSITION_MIN 0

// Pressure Limits
// TODO: Recalibrate these
#define PIN_ACTIVATION_PRESSURE 1
#define PIN_MIN_CONTROLLABLE_PRESSURE 50
#define PIN_MAX_CONTROLLABLE_PRESSURE 240 // maximum force a drive can actually exhert (about 1lb in practice) NOTE: this is the (scaled) pressure value streamed by arduino when the pin is exherting the max force

// PID Gains
#define DEFAULT_PROPORTIONAL_GAIN 50
#define PROPORTAIONAL_TUNING_MAX 150.0

#define DEFAULT_DERIVATIVE_GAIN   500000.0 //100000
#define DERIVATIVE_TUNING_MAX   10000000.0
#define DERIVATIVE_TUNING_MIN  -10000000.0

#define DEFAULT_INTEGRAL_GAIN 0.000002//0.00001
#define INTEGRAL_TUNING_MAX   0.00005

// Debouncing and other trickery
#define MIN_DEBOUNCE_TIME_ACTIVATION 0
#define MAX_DEBOUNCE_TIME_ACTIVATION 1000000
#define DEFAULT_PIN_ACTIVATION_TOUCH_DEBOUNCE_WINDOW_US 200000

#define MIN_DEBOUNCE_TIME_DEACTIVATION 0
#define MAX_DEBOUNCE_TIME_DEACTIVATION 1000000
#define DEFAULT_PIN_DEACTIVATION_TOUCH_DEBOUNCE_WINDOW_US 400000

//#define USE_PID_LINEARIZATION
#ifdef USE_PID_LINEARIZATION
	#define DEFAULT_PID_SETPOINT_MULTIPLIER 0.04
	#define MIN_PID_SETPOINT_MULTIPLIER 0
	#define MAX_PID_SETPOINT_MULTIPLIER 2.0
#endif

//Debug and benchmarking
//#define DO_BENCHMARKING
//#define PRINT_PRESSURES
//#define PRINT_POSITIONS
//#define PRINT_PID_OUTPUT
//#define PRINT_PRESSURE_UPDATE_HIT_RATIO
//#define PRINT_TIME
//#define PRINT_PERIOD_MICROSECONDS 20000 // 50 times per second (50hz)

//#define USE_MUTEXES
#endif