/*
 * File:          line_follower_main.c
 * Date:          2024
 * Description:   Simulation of line follower robot, which is controlled based on machine vision
 * Author:        Wojciech Domański
 * Modifications:
 */

/* Libraries */
#include <stdio.h>
#include <stdint.h>
#include <webots/robot.h>

#include "controller.h"

int main(int argc, char **argv)
{
  /* Initialization */
  wb_robot_init(); // It is necessary to initialize webots resources
  webots_init();
  controller_init();

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1 && run_finished() == 0)
  {
    /* Read current values */
    update_controller_values();

    /* Controller */
    robot_control();
  };

  wb_robot_cleanup(); // This is necessary to cleanup webots resources
  return 0;
}
