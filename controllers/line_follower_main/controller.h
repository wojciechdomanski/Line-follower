/*
 * File:          controller.h
 * Date:          2024
 * Description:   Header file for controller functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#ifndef CONTROLLER_DOT_H
#define CONTROLLER_DOT_H

#include "webots_interface.h"
#include "video_processing.h"

// Controller macros
#define TARGET_ANGLE 0.0               // [degrees]
#define SET_VELOCITY 40                // [rad/s]
#define INTEGRAL_LIMIT 100.0
#define VEL_P_COMP 0.1
#define VEL_I_COMP 16.0
#define VEL_D_COMP 0.0
#define POS_P_COMP 55.0
#define POS_I_COMP 0.0
#define POS_D_COMP 6.5

/* Controller typedef */
typedef enum {
  LOG_LVL_NONE = 0,
  LOG_LVL_BASIC = 1,
  LOG_LVL_FULL = 2
}loggerTypeDef;

typedef enum {
  READY = 0,
  CLOCKWISE_MOTION = 1,
  FIRST_TARGET_REACHED = 2,
  COUNTER_CLOCKWISE_MOTION = 3,
  SECOND_TARGET_REACHED = 4,
  FINISH = 5
} stateMachineTypeDef;

typedef struct {
  double set_value;
  double current_value;
  double pid_comp[3];
  double p_prev;
  double i_prev;
  double output;
}PidStructTypeDef;

typedef struct {
  PidStructTypeDef pos;
  PidStructTypeDef vel;
}CascadePidStructTypeDef;

/* Controller functions */

/*
  Defines values for controller specific struct variables
*/
void controller_init();

/*
  Updates all crucial controller variables
*/
void update_controller_values();

/*
  Prints status or debug information depending on set LOGGING level
  * Parameter [int*] waypoint Current waypoint values
  * Parameter [int] binarized_array Array, which contains skeletonized camera frame
*/
void state_logger(int* waypoint, int binarized_array[IMG_HEIGHT][IMG_WIDTH]);

/*
  Calculates wheel angular speed value based on delta of position sensor
  values and delta of time values
  * Parameter [double] curr_pos Current angular position value
  * Parameter [double] prev_pos Previous itteration angular position value
  * Returns [double] Angular speed value
*/
double calculate_speed(double curr_pos, double prev_pos);

/* 
  Limits input value, so that it fits in specific range
  * Parameter [double] value Input value to be limited
  * Parameter [double] min Lower limit value
  * Parameter [double] max Higher limit value
  * Returns [double] Limited value
*/
double limit(double value, double min, double max);

/* 
  Calculates PID controller output value
  * Parameter [PidStructTypeDef*] pid_parameters Struct containing PID parameters
*/
void pid_controller(PidStructTypeDef* pid_parameters);

/* 
  Calculates target velocity of left wheel with a use of cascade PID controller
  * Parameter [float] set_velocity Target velocity
  * Parameter [CascadePidStructTypeDef*] pid_struct Struct containing parameters for wheel cascade PID controller
*/
void control_loop_left(double set_velocity, CascadePidStructTypeDef* pid_struct);

/* 
  Calculates target velocity of right wheel with a use of cascade PID controller
  * Parameter [float] set_velocity Target velocity
  * Parameter [CascadePidStructTypeDef*] pid_struct Struct containing parameters for wheel cascade PID controller
*/
void control_loop_right(float set_velocity, CascadePidStructTypeDef * pid_struct);

/*
  Performs control loops for both wheels and sets up velocity value to each wheel accoringly
*/
void robot_controller();

/*
  Calculates distance from target based on supervision functions
  * Parameter [int] target_index Target selection parameter
  * Returns [double] distance from selected target
*/
double target_distance(int target_index);

/*
  Updates state machine based on specified transfer conditions
*/
void state_machine_update();

/*
  Specifies robot action based on current state
*/
void robot_control();

/*
  Returns one if machine state reaches 'FINISH', else it returns zero
*/
int run_finished();

#endif /* CONTROLLER_DOT_H */
