/*
 * File:          video_processing.h
 * Date:          2024
 * Description:   Header file for video processing functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#ifndef VIDEO_PROCESSING_DOT_H
#define VIDEO_PROCESSING_DOT_H

#include "webots_interface.h"

// Math macros
#define PI 3.141892654

// Video processing macros
#define BINARIZE_THRESHOLD 90
#define NEIGHBOURS 8

/* Video processing functions */

/* 
  Sums every element in passed array
  * Parameter [int*] array Input array to be summed up
  * Parameter [int] array_size Size of an array
  * Returns [int] Summed up array values
*/
int sum(int* array, int array_size);

/*
  Inverts binarized pixel value
  * Parameter [int] input Input pixel
  * Returns [int] Inverted pixel value
*/
int invert(int input);

/*
  Binarizes pixel depending on treshold value
  * Parameter [int] gray_value Pixel value described as shades of gray
  * Parameter [int] treshold Treshold value used to classify pixel
  * Returns [int] Binarized pixel value
*/
int binarize(int gray_value, int treshold);

/*
  Binarizes current camera frame
  * Parameter [int] binarized_array Array which will hold all binarized pixel values
*/
void binarize_frame(int binarized_array[IMG_HEIGHT][IMG_WIDTH]);

/*
  Maps negihbouring pixels in array
  * Parameter [int[8]] neighbours Array, which will contain neighbour pixel values
  * Parameter [int] x X axis pixel value
  * Parameter [int] y Y axis pixel value
  * Parameter [int[height][width]] image Array containing image pixel data
*/
void neighbours_array(int neighbours[8], int x, int y, int image[IMG_HEIGHT][IMG_WIDTH]);

/*
  Calculates the number of transitions from white to 
  black in the sequence P2,P3,P4,P5,P6,P7,P8,P9,P2.
  * Parameter [int] neighbours Array, which contains neighbour pixel values
  * Returns [int] Number of transitions
*/
int transitions(int neighbours[NEIGHBOURS]);

/*
  Function processes image based on Zhang Suen algorithm,
  which results in receiving morphological skeleton.
  * Parameter [int[height][width]] image Array containing image pixel data  
*/
void zhang_suen(int image[IMG_HEIGHT][IMG_WIDTH]);

/*
  Calculates waypoint based on provided skeletonized camera frame.
  It maps single arch on frame in order to compare against detected path.
  In case multiple paths are detected it chooses the closest one to arch's middle point
  * Parameter [int] binarized_array Array, which contains binarized camera frame
  * Returns [int] X and Y values of waypoint
*/
int* find_waypoint(int binarized_array[IMG_HEIGHT][IMG_WIDTH]);

/*
  Calculates normalized waypoint value
  * Parameter [int] waypoint Path waypoint
  * Returns [float] Normalized value ranging from 0.0 to 1.0
*/
float normalize_waipoint_value(int waypoint);

#endif /* VIDEO_PROCESSING_DOT_H */
