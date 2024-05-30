/*
 * File:          webots_interface.c
 * Date:          2024
 * Description:   Integration file for webots interface functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

#include "webots_interface.h"

/* Devices */

// Motors
WbDeviceTag left_motor;
WbDeviceTag right_motor;

//Camera
WbDeviceTag camera;

//Position Sensors
WbDeviceTag left_pos_sensor;
WbDeviceTag right_pos_sensor;

/* Supervision refs */
WbNodeRef robot_node;
WbFieldRef trans_field;
WbFieldRef rotation_field;

/*
  Initializes and enables Webots devices, sets initial positions and velocity for motors
*/
void webots_init()
{
  webots_device_init();
  set_init_positions();
  set_init_velocity();
  supervision_init();
}

/*
  Initializes and enables Webots devices
*/
void webots_device_init()
{
  /* Get devices */
  // Motors
  left_motor = wb_robot_get_device("left_wheel_hinge");
  right_motor = wb_robot_get_device("right_wheel_hinge");

  // Position sensors
  left_pos_sensor = wb_robot_get_device("left_position_sensor");
  right_pos_sensor = wb_robot_get_device("right_position_sensor");
  
  // Camera
  camera = wb_robot_get_device("camera");

  /* Enable devices */
  // Position sensors
  wb_position_sensor_enable(left_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(right_pos_sensor, TIME_STEP);

  // Camera
  wb_camera_enable(camera, TIME_STEP);
}

/*
  Sets initial positions for motors
*/
void set_init_positions()
{
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
}

/*
  Sets initial velocity for motors
*/
void set_init_velocity()
{
  wb_motor_set_velocity(left_motor, ZERO_VELOCITY);
  wb_motor_set_velocity(right_motor, ZERO_VELOCITY);
}

/*
  Sets velocity for left wheel motor
  * Parameter [double] velocity Value to be set
*/
void set_left_velocity(double velocity)
{
  wb_motor_set_velocity(left_motor, velocity);
}

/*
  Sets velocity for right wheel motor
  * Parameter [double] velocity Value to be set
*/
void set_right_velocity(double velocity)
{
  wb_motor_set_velocity(right_motor, velocity);
}

/* Read current values */
/*
  Reads current simulation time
  * Returns [double] simulation time
*/
double get_sim_time()
{
  return wb_robot_get_time();
}
 
/*
  Reads left wheel position
  * Returns [double] wheel position
*/
double get_pos_left()
{
  return wb_position_sensor_get_value(left_pos_sensor);
}

/*
  Reads right wheel position
  * Returns [double] wheel position
*/
double get_pos_right()
{
  return wb_position_sensor_get_value(right_pos_sensor);
}

/*
  Reads current camera frame
  * Returns [const unsigned char*] camera frame
*/
const unsigned char* get_image()
{
  return wb_camera_get_image(camera);
}

/*
  Coverts current frame's specific pixel value into shades of gray
  * Parameter [const unsigned char*] image Base image
  * Parameter [int] image_width Width of the image
  * Parameter [int] x X axis index of pixel to be modified
  * Parameter [int] y Y axis index of pixel to be modified
  * Returns [int] selected pixel in shades of gray
*/
int get_gray_image(const unsigned char* image, int image_width, int x, int y)
{
  return wb_camera_image_get_gray(image, IMG_WIDTH, y, x);
}

/* Supervision functions */

/*
  Initializes supervision definitions
*/
void supervision_init()
{
  robot_node = wb_supervisor_node_get_from_def("robot1");
  trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
}

/*
  Sets robot's acctual velocity to zero
*/
void set_supervision_velocity_zero()
{
  const double INITIAL_VEL[6] = {0, 0, 0, 0, 0, 0};
  wb_supervisor_node_set_velocity(robot_node, INITIAL_VEL);
}

/*
  Sets robot's position based on two specified presets
  * Parameter [int] rotation_index Specifies which preset will be used
*/
void set_supervision_position(int position_index)
{
  enum position {
  FIRST_POSITION = 0,
  SECOND_POSITION = 1
  };

  if(position_index == FIRST_POSITION)
  {
    const double POSITION[3] = {FIRST_POSITION_X, FIRST_POSITION_Y, FIRST_POSITION_Z};
    wb_supervisor_field_set_sf_vec3f(trans_field, POSITION);
  }
  else
  {
    const double POSITION[3] = {SECOND_POSITION_X, SECOND_POSITION_Y, SECOND_POSITION_Z};
    wb_supervisor_field_set_sf_vec3f(trans_field, POSITION);
  }
}

/*
  Sets robot's rotation based on two specified presets
  * Parameter [int] rotation_index Specifies which preset will be used
*/
void set_supervision_rotation(int rotation_index)
{
  enum rotation {
  FIRST_ROTATION = 0,
  SECOND_ROTATION = 1
  };

  if(rotation_index == FIRST_ROTATION)
  {
    const double ROTATION[4] = {FIRST_ROTATION_RX, FIRST_ROTATION_RY, FIRST_ROTATION_RZ, FIRST_ROTATION_A};
    wb_supervisor_field_set_sf_rotation(rotation_field, ROTATION);
  }
  else
  {
    const double ROTATION[4] = {SECOND_ROTATION_RX, SECOND_ROTATION_RY, SECOND_ROTATION_RZ, SECOND_ROTATION_A};
    wb_supervisor_field_set_sf_rotation(rotation_field, ROTATION);
  }
}

/*
  Reads robot's position based on supervision function
  * Returns [const double*] pointer to position array
*/
const double* get_supervisor_pos()
{
  return wb_supervisor_field_get_sf_vec3f(trans_field);
}

/*
  Resets Webots node physics
*/
void supervisor_reset_physics()
{
  wb_supervisor_node_reset_physics(robot_node);
}
