/*
 * File:          video_processing.c
 * Date:          2024
 * Description:   Implementation file for video processing functions
 * Author:        Wojciech Domański
 * Modifications:
 */
#include <math.h>

#include "video_processing.h"

/* Video processing functions */

/* 
  Sums every element in passed array
  * Parameter [int*] array Input array to be summed up
  * Parameter [int] array_size Size of an array
  * Returns [int] Summed up array values
*/
int sum(int* array, int array_size)
{
  int result = 0;
  for(int i = 0; i < array_size; i++) result += array[i];
  return result;
}

/*
  Inverts binarized pixel value
  * Parameter [int] input Input pixel
  * Returns [int] Inverted pixel value
*/
int invert(int input)
{
  return(input == 0)? 1:0;
}

/*
  Binarizes pixel depending on treshold value
  * Parameter [int] gray_value Pixel value described as shades of gray
  * Parameter [int] treshold Treshold value used to classify pixel
  * Returns [int] Binarized pixel value
*/
int binarize(int gray_value, int treshold)
{
  return(gray_value < treshold)? 0:1;
}

/*
  Binarizes current camera frame
  * Parameter [int] binarized_array Array which will hold all binarized pixel values
*/
void binarize_frame(int binarized_array[IMG_HEIGHT][IMG_WIDTH])
{
  // Binarized current camera frame
  const unsigned char *image = get_image();
  for (int x = 0; x < IMG_HEIGHT; x++)
  {
    for (int y = 0; y < IMG_WIDTH; y++) 
    {
      int gray = get_gray_image(image, IMG_WIDTH, x, y);
      binarized_array[x][y] = invert(binarize(gray, BINARIZE_THRESHOLD));
    }
  }
}

/*
  Maps negihbouring pixels in array
  * Parameter [int[8]] neighbours Array, which will contain neighbour pixel values
  * Parameter [int] x X axis pixel value
  * Parameter [int] y Y axis pixel value
  * Parameter [int[height][width]] image Array containing image pixel data
*/
void neighbours_array(int neighbours[8], int x, int y, int image[IMG_HEIGHT][IMG_WIDTH])
{
  neighbours[0] = image[y - 1][x];
  neighbours[1] = image[y - 1][x + 1];
  neighbours[2] = image[y][x + 1];
  neighbours[3] = image[y + 1][x + 1];
  neighbours[4] = image[y + 1][x];
  neighbours[5] = image[y + 1][x - 1];
  neighbours[6] = image[y][x - 1];
  neighbours[7] = image[y - 1][x - 1];
}

/*
  Calculates the number of transitions from white to 
  black in the sequence P2,P3,P4,P5,P6,P7,P8,P9,P2.
  * Parameter [int] neighbours Array, which contains neighbour pixel values
  * Returns [int] Number of transitions
*/
int transitions(int neighbours[NEIGHBOURS])
{
  int n[9];
  int transitions_sum = 0;

  for(int i = 0; i < 8; i++) n[i] = neighbours[i];
  n[8] = neighbours[0];
  
  for(int i = 0; i < 8; i++) transitions_sum += (n[i] == 0)&(n[i + 1] == 1);
  return transitions_sum;
}

/*
  Function processes image based on Zhang Suen algorithm,
  which results in receiving morphological skeleton.
  * Parameter [int[height][width]] image Array containing image pixel data  
*/
void zhang_suen(int image[IMG_HEIGHT][IMG_WIDTH])
{
  int changing1[500][2] = { {-1, -1} };
  int changing2[500][2] = { {-1, -1} };
  int index1 = -1, index2 = -1;
  int neighbours[8];

  while(index1 != 0 || index2 != 0)
  {
      //Step 1
      changing1[0][0] = -1;
      changing1[0][1] = -1;
      index1 = 0;

      for(int y = 1; y < IMG_HEIGHT - 1; y++)
        for(int x = 1; x < IMG_WIDTH - 1; x++)
        {
         neighbours_array(neighbours, x, y, image);

         if(image[y][x] == 1 &&
            neighbours[2] * neighbours[4] * neighbours[6] == 0 &&
            neighbours[0] * neighbours[2] * neighbours[4] == 0 &&
            transitions(neighbours) == 1 &&
            2 <= sum(neighbours, 8) &&
            sum(neighbours, 8) <= 6)
            {
              changing1[index1][0] = x;
              changing1[index1][1] = y;
              index1++;
            }
          }

      for(int i = 0; i < index1; i++) image[changing1[i][1]][changing1[i][0]] = 0;

      //Step 2
      changing2[0][0] = -1;
      changing2[0][1] = -1;
      index2 = 0;

      for(int y = 1; y < IMG_HEIGHT - 1; y++)
        for(int x = 1; x < IMG_WIDTH - 1; x++)
        {
         neighbours_array(neighbours, x, y, image);
         
         if(image[y][x] == 1 &&
            neighbours[0] * neighbours[4] * neighbours[6] == 0 &&
            neighbours[0] * neighbours[2] * neighbours[6] == 0 &&
            transitions(neighbours) == 1 &&
            2 <= sum(neighbours, 8) &&
            sum(neighbours, 8) <= 6)
            {
              changing2[index2][0] = x;
              changing2[index2][1] = y;
              index2++; 
            }
          }
         
      for(int i = 0; i < index2; i++) image[changing2[i][1]][changing2[i][0]] = 0;
  }
}

/*
  Calculates waypoint based on provided skeletonized camera frame.
  It maps single arch on frame in order to compare against detected path.
  In case multiple paths are detected it chooses the closest one to arch's middle point
  * Parameter [int] binarized_array Array, which contains binarized camera frame
  * Returns [int] X and Y values of waypoint
*/
int* find_waypoint(int binarized_array[IMG_HEIGHT][IMG_WIDTH])
{
  // Morphological skeletonization
  zhang_suen(binarized_array);

  double angle_offset = 179.0 * (PI / 180.0);
  double angle;
  int arch_x[2 * IMG_WIDTH], arch_y[2 * IMG_WIDTH];
  int array_index = 0;
  int points[100][2], index = 0, minimal, current;

  for(int i = 0; i < 2 * IMG_WIDTH; i++)
  {
    // Generate arch
    angle = i * (181.0 / (2.0 * IMG_WIDTH)) * (PI / 180.0) + angle_offset;
    arch_x[array_index] = round(HALF_IMG_WIDTH * cos(angle) + HALF_IMG_WIDTH);
    arch_y[array_index] = round(HALF_IMG_HEIGHT * sin(angle) + IMG_HEIGHT);
    array_index++;
    
    // Check arches against skeletonized path and save all cross points
    if(binarized_array[arch_y[i]][arch_x[i]] == 1)
    {
      if(index == 0)
      {
        points[index][0] = arch_x[i];
        points[index][1] = arch_y[i];
        index++;
      }
      else if(points[index - 1][0] != arch_x[i] && points[index - 1][1] != arch_y[i])
      {
        points[index][0] = arch_x[i];
        points[index][1] = arch_y[i];
        index++;
      }
    }
  }

  // Select point closest to the middle
  static int final_point[2];
  final_point[0] = HALF_IMG_WIDTH;
  final_point[1] = HALF_IMG_HEIGHT;
  if(index > 1)
  {
    minimal = 9999;
    for(int i = 0; i < index; i++)
    {
      current = abs(points[i][0] - HALF_IMG_WIDTH);
      if(current < minimal)
      {
        minimal = current;
        final_point[0] = points[i][0];
        final_point[1] = points[i][1];
      }
    }
  }
  else if(index == 1)
  {
    final_point[0] = points[0][0];
    final_point[1] = points[0][1];
  }
  return final_point;
}

/*
  Calculates normalized waypoint value
  * Parameter [int] waypoint Path waypoint
  * Returns [float] Normalized value ranging from 0.0 to 1.0
*/
float normalize_waipoint_value(int waypoint)
{
  return ((double)waypoint - HALF_IMG_WIDTH) / HALF_IMG_WIDTH;
}
