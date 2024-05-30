/*
 * File:          webots_interface.h
 * Date:          2024
 * Description:   Header file for webots interface functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#ifndef WEBOTS_INTERFACE_DOT_H
#define WEBOTS_INTERFACE_DOT_H

// Simulation macros
#define TIME_STEP 32                     // [ms]
#define DT 0.032                         // [s]
#define LOGGING 1

// Position macros
#define FIRST_POSITION_X 5.23            // [m]
#define FIRST_POSITION_Y 1.42447e-5      // [m]
#define FIRST_POSITION_Z 0.56339         // [m]
#define SECOND_POSITION_X 5.30           // [m]
#define SECOND_POSITION_Y 1.72436e-5     // [m]
#define SECOND_POSITION_Z 1.62339        // [m]

// Rotation macros
#define FIRST_ROTATION_RX -6.07279e-07
#define FIRST_ROTATION_RY -0.707108
#define FIRST_ROTATION_RZ -0.707106
#define FIRST_ROTATION_A 3.14159         // [deg]
#define SECOND_ROTATION_RX -6.07279e-07
#define SECOND_ROTATION_RY -0.707108
#define SECOND_ROTATION_RZ -0.707106
#define SECOND_ROTATION_A 3.14159        // [deg]

// Robot macros
#define ROBOT_NAME robot1
#define INITIAL_VELOCITY 0.001           // [rad/s]
#define ZERO_VELOCITY 0.0                // [rad/s]
#define MAX_VELOCITY 50                  // [rad/s]

// Camera macros
#define IMG_HEIGHT 90                    // [pixel]
#define IMG_WIDTH 160                    // [pixel]
#define HALF_IMG_HEIGHT 45               // [pixel]
#define HALF_IMG_WIDTH 80                // [pixel]

/*
  Initializes and enables Webots devices, sets initial positions and velocity for motors
*/
void webots_init();

/*
  Initializes and enables Webots devices
*/
void webots_device_init();

/*
  Sets initial positions for motors
*/
void set_init_positions();

/*
  Sets initial velocity for motors
*/
void set_init_velocity();

/*
  Sets velocity for left wheel motor
  * Parameter [double] velocity Value to be set
*/
void set_left_velocity(double velocity);

/*
  Sets velocity for right wheel motor
  * Parameter [double] velocity Value to be set
*/
void set_right_velocity(double velocity);

/* Read current values */
/*
  Reads current simulation time
  * Returns [double] simulation time
*/
double get_sim_time();
 
/*
  Reads left wheel position
  * Returns [double] wheel position
*/
double get_pos_left();

/*
  Reads right wheel position
  * Returns [double] wheel position
*/
double get_pos_right();

/*
  Reads current camera frame
  * Returns [const unsigned char*] camera frame
*/
const unsigned char* get_image();

/*
  Coverts current frame's specific pixel value into shades of gray
  * Parameter [const unsigned char*] image Base image
  * Parameter [int] image_width Width of the image
  * Parameter [int] x X axis index of pixel to be modified
  * Parameter [int] y Y axis index of pixel to be modified
  * Returns [int] selected pixel in shades of gray
*/
int get_gray_image(const unsigned char* image, int image_width, int x, int y);

/* Supervision functions */

/*
  Initializes supervision definitions
*/
void supervision_init();

/*
  Sets robot's acctual velocity to zero
*/
void set_supervision_velocity_zero();

/*
  Sets robot's position based on two specified presets
  * Parameter [int] rotation_index Specifies which preset will be used
*/
void set_supervision_position(int position_index);

/*
  Sets robot's rotation based on two specified presets
  * Parameter [int] rotation_index Specifies which preset will be used
*/
void set_supervision_rotation(int rotation_index);

/*
  Reads robot's position based on supervision function
  * Returns [const double*] pointer to position array
*/
const double* get_supervisor_pos();

/*
  Resets Webots node physics
*/
void supervisor_reset_physics();

#endif /* WEBOTS_INTERFACE_DOT_H */
