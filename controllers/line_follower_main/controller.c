/*
 * File:          controller.c
 * Date:          2024
 * Description:   Implementation file for controller functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#include <stdio.h>
#include <math.h>

#include "controller.h"

// Logger enum
loggerTypeDef logger = LOGGING;

stateMachineTypeDef stateMachine;

// Controller structs
PidStructTypeDef default_pos_struct_left;
PidStructTypeDef default_vel_struct_left;
CascadePidStructTypeDef default_struct_left;
CascadePidStructTypeDef* pid_struct_left;

PidStructTypeDef default_pos_struct_right;
PidStructTypeDef default_vel_struct_right;
CascadePidStructTypeDef default_struct_right;
CascadePidStructTypeDef* pid_struct_right;

/*
  Defines values for controller specific struct variables
*/
void controller_init()
{
  // Control struct definitions for left wheel
  default_pos_struct_left.set_value = TARGET_ANGLE;
  default_pos_struct_left.pid_comp[0] = POS_P_COMP;
  default_pos_struct_left.pid_comp[1] = POS_I_COMP;
  default_pos_struct_left.pid_comp[2] = POS_D_COMP;
  
  default_vel_struct_left.current_value = INITIAL_VELOCITY;
  default_vel_struct_left.pid_comp[0] = VEL_P_COMP;
  default_vel_struct_left.pid_comp[1] = VEL_I_COMP;
  default_vel_struct_left.pid_comp[2] = VEL_D_COMP;

  default_struct_left.pos = default_pos_struct_left;
  default_struct_left.vel = default_vel_struct_left;
  pid_struct_left = &default_struct_left;
  
  // Control struct definitions for right wheel
  default_pos_struct_right.set_value = TARGET_ANGLE;
  default_pos_struct_right.pid_comp[0] = POS_P_COMP;
  default_pos_struct_right.pid_comp[1] = POS_I_COMP;
  default_pos_struct_right.pid_comp[2] = POS_D_COMP;
  
  default_vel_struct_right.current_value = INITIAL_VELOCITY;
  default_vel_struct_right.pid_comp[0] = VEL_P_COMP;
  default_vel_struct_right.pid_comp[1] = VEL_I_COMP;
  default_vel_struct_right.pid_comp[2] = VEL_D_COMP;

  default_struct_right.pos = default_pos_struct_right;
  default_struct_right.vel = default_vel_struct_right;
  pid_struct_right = &default_struct_right;
  
  stateMachine = READY;
}

/*
  Updates all crucial controller variables
*/
void update_controller_values()
{
  static double prev_pos_left = 0, prev_pos_right = 0;

  static int bin_arr[IMG_HEIGHT][IMG_WIDTH]; 
  binarize_frame(bin_arr);

  // Set waypoint
  int* waypoint = find_waypoint(bin_arr);
  float normalized_waypoint = normalize_waipoint_value(waypoint[0]);
  pid_struct_left->pos.current_value = normalized_waypoint;
  pid_struct_right->pos.current_value = normalized_waypoint;

  // Current positions
  double curr_pos_left = get_pos_left();
  double curr_pos_right = get_pos_right();

  // Set current velocity
  pid_struct_left->vel.current_value = calculate_speed(curr_pos_left, prev_pos_left);
  pid_struct_right->vel.current_value = calculate_speed(curr_pos_right, prev_pos_right);
  prev_pos_left = curr_pos_left;
  prev_pos_right = curr_pos_right;

  // Call logger function
  state_logger(waypoint, bin_arr);
}

/*
  Prints status or debug information depending on set LOGGING level
  * Parameter [int*] waypoint Current waypoint values
  * Parameter [int] binarized_array Array, which contains skeletonized camera frame
*/
void state_logger(int* waypoint, int binarized_array[IMG_HEIGHT][IMG_WIDTH])
{
  if(logger >= LOG_LVL_BASIC) 
  {
    printf("========================================================================================\n");
    printf("Timestamp:\t\t|\t%.2f\t\t\t|\t[s]\n", get_sim_time());
    printf("Current state:\t|\t%d\n", stateMachine);
    printf("----------------------------------------------------------------------------------------\n");
    printf("\t\t|\tLeft wheel\t|\tRight wheel\n");
    printf("----------------------------------------------------------------------------------------\n");
    printf("Set velocity:\t|\t%.2f\t|\t%.2f\t|\t[m/s]\n",  pid_struct_left->vel.output, pid_struct_right->vel.output); 
    printf("Current velocity:\t|\t%.2f\t|\t%.2f\t|\t[m/s]\n",  pid_struct_left->vel.current_value, pid_struct_right->vel.current_value);
    printf("----------------------------------------------------------------------------------------\n");
    printf("\t\t|\tX\t|\tY\t\n");
    printf("----------------------------------------------------------------------------------------\n");
    printf("Waypoint:\t\t|\t%d\t|\t%d\t|\t[pixel]\n\n", waypoint[0], waypoint[1]);
  }

  if(logger == LOG_LVL_FULL) 
  { 
    for(int i = 0; i < IMG_HEIGHT; i++)
    {
        for(int j = 0; j < IMG_WIDTH; j++)
        {
            (waypoint[0] == j && waypoint[1] == i)? printf("%d", 5):printf("%d", binarized_array[i][j]);
        }
    
        printf("\n");
    }
    printf("\n");
  }
}

/*
  Calculates wheel angular speed value based on delta of position sensor
  values and delta of time values
  * Parameter [double] curr_pos Current angular position value
  * Parameter [double] prev_pos Previous itteration angular position value
  * Returns [double] Angular speed value
*/
double calculate_speed(double curr_pos, double prev_pos)
{
  return (curr_pos - prev_pos)/DT;
}

/* 
  Limits input value, so that it fits in specific range
  * Parameter [double] value Input value to be limited
  * Parameter [double] min Lower limit value
  * Parameter [double] max Higher limit value
  * Returns [double] Limited value
*/
double limit(double value, double min, double max)
{
  if(value > max) { return max; }
  else if(value < min) { return min; }
  else { return value; }
}

/*
  Calculates PID controller output value
  * Parameter [PidStructTypeDef*] pid_parameters Struct containing PID parameters
*/
void pid_controller(PidStructTypeDef* pid_parameters)
{
  double error, proportional, integral, derivative;
  error = pid_parameters->set_value - pid_parameters->current_value;
  proportional = pid_parameters->pid_comp[0] * error;
  // Calling 'limit' to prevent integral from reaching 'infinite' values
  integral = limit(pid_parameters->i_prev + pid_parameters->pid_comp[1] * error * DT, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  derivative = pid_parameters->pid_comp[2] * (error - pid_parameters->p_prev) / DT;

  pid_parameters->output = proportional + integral + derivative;
  pid_parameters->p_prev = error;
  pid_parameters->i_prev = integral;
}

/* 
  Calculates target velocity of left wheel with a use of cascade PID controller
  * Parameter [float] set_velocity Target velocity
  * Parameter [CascadePidStructTypeDef*] pid_struct Struct containing parameters for wheel cascade PID controller
*/
void control_loop_left(double set_velocity, CascadePidStructTypeDef* pid_struct)
{
    pid_controller(&pid_struct->pos);
    
    if(pid_struct->pos.output >= 0) 
    { pid_struct->vel.set_value = limit(set_velocity - pid_struct->pos.output, ZERO_VELOCITY, MAX_VELOCITY); }
    else if(pid_struct->pos.output < 0) { pid_struct->vel.set_value = set_velocity; }
    pid_controller(&pid_struct->vel);
    pid_struct->vel.output = limit(pid_struct->vel.output, ZERO_VELOCITY, MAX_VELOCITY);
}

/* 
  Calculates target velocity of right wheel with a use of cascade PID controller
  * Parameter [float] set_velocity Target velocity
  * Parameter [CascadePidStructTypeDef*] pid_struct Struct containing parameters for wheel cascade PID controller
*/
void control_loop_right(float set_velocity, CascadePidStructTypeDef * pid_struct)
{
    pid_controller(&pid_struct->pos);

    if(pid_struct->pos.output > 0) { pid_struct->vel.set_value = set_velocity; }
    else if(pid_struct->pos.output <= 0) 
    { pid_struct->vel.set_value = limit(set_velocity + pid_struct->pos.output, ZERO_VELOCITY, MAX_VELOCITY); }
    pid_controller(&pid_struct->vel);
    pid_struct->vel.output = limit(pid_struct->vel.output, ZERO_VELOCITY, MAX_VELOCITY);
}

/*
  Performs control loops for both wheels and sets up velocity value to each wheel accoringly
*/
void robot_controller()
{
  control_loop_left(SET_VELOCITY, pid_struct_left);
  control_loop_right(SET_VELOCITY, pid_struct_right);

  set_left_velocity(pid_struct_left->vel.output);
  set_right_velocity(pid_struct_right->vel.output);  
}

/*
  Calculates distance from target based on supervision functions
  * Parameter [int] target_index Target selection parameter
  * Returns [double] distance from selected target
*/
double target_distance(int target_index)
{
  double target[3];
  enum rotation {
  FIRST_TARGET = 0,
  SECOND_TARGET = 1
  };

  const double* current_pos = get_supervisor_pos();
  if(target_index == FIRST_TARGET) 
  {
    target[0] = FIRST_POSITION_X;
    target[2] = FIRST_POSITION_Z;
  }
  else
  {
    target[0] = SECOND_POSITION_X;
    target[2] = SECOND_POSITION_Z;
  }
  
  return(sqrt(pow(current_pos[0] - target[0], 2) + pow(current_pos[2] - target[2], 2)));
}

/*
  Updates state machine based on specified transfer conditions
*/
void state_machine_update()
{
  switch(stateMachine)
  {
  case READY:
  {
    stateMachine = CLOCKWISE_MOTION;
    break;
  }
  case CLOCKWISE_MOTION:
  {
    if(target_distance(0) < 0.1) stateMachine = FIRST_TARGET_REACHED;
    break;
  }
  case FIRST_TARGET_REACHED:
  {
    if(target_distance(0) < 0.01) stateMachine = COUNTER_CLOCKWISE_MOTION;
    break;
  }
  case COUNTER_CLOCKWISE_MOTION:
  {
    if(target_distance(1) < 0.1) stateMachine = SECOND_TARGET_REACHED;
    break;
  }
  case SECOND_TARGET_REACHED:
  {
    stateMachine = FINISH;
    break;
  }
  case FINISH:
  {
    break;
  }
  default:
    break;
  }
}

/*
  Specifies robot action based on current state
*/
void robot_control()
{
  static double start_time1 = 0;
  static double run_time1 = 0;
  static double start_time2 = 0;
  if(stateMachine == READY)
  {
    set_supervision_position(1);
    set_supervision_rotation(1);
    start_time1 = get_sim_time();
  }
  else if(stateMachine == CLOCKWISE_MOTION)
  {
    robot_controller();
  }
  else if(stateMachine == FIRST_TARGET_REACHED)
  {
    run_time1 = get_sim_time() - start_time1;
    printf("\nRobot has reached first destination. Clockwise run took: %.2f\n", run_time1);
    set_init_velocity();
    set_supervision_velocity_zero();
    set_supervision_position(0);
    set_supervision_rotation(0);
    supervisor_reset_physics();
    start_time2 = get_sim_time();
  }
  else if(stateMachine == COUNTER_CLOCKWISE_MOTION)
  {
    robot_controller();
  }
  else if(stateMachine == SECOND_TARGET_REACHED)
  {
    set_init_velocity();
    printf("\nRobot has reached final destination.\nClockwise run took: %.2f\nCounter clockwise run took: %.2f\n",
           run_time1, get_sim_time() - start_time2);
  }
  else if(stateMachine == FINISH)
  {
    // Idle
  }
  state_machine_update();
}

/*
  Returns one if machine state reaches 'FINISH', else it returns zero
*/
int run_finished()
{
  if(stateMachine == FINISH) return(1);
  else return(0);
}
