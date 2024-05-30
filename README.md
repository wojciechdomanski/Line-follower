# Line follower project
The intention of this project was to learn C coding. It's a simulated mobile robot running on Webots[^1]. As mentioned robot's controller has been written in C. The robot makes use of machine vision (camera), which serves as an input for controller. Cascade PID controller[^2] has been chosen for control system.

## Robot
At the begining of the project it was decided to use only existing robots. Due to its relatively small size, acceptable motor capabilities and efficient processing unit NVIDIA JetBot[^3] has been chosen for the project.

The robot makes use of NVIDIA Jetson Nano board, which is perfect for video processing and definately is more than enough to handle designed code.
The only deviations from its original form are: 
* slightly changed mounting angle for camera,
* wider wheel axels,
* added position sensors for electric motors.

Camera had to be adjusted, so that control horizon wouldn't be so big. Wider wheel axels on the other hand helped a lot with handling sharp corners during high speeds. Position sensors were necessary to calculate wheel speeds for controller feedback.

All of sensors/actuators interfaces are provided by Webots environment, thus integration of mentioned elements is not in scope of this project.

## Machine vision
Originaly camera used by default in NVIDIA Jetbot used HD 1280x720 resolution, but it was decided, that there is no need for such a big array of data to be processed every itteration. In the end 160x90 resolution was considered to be good enough consensus.

Video processing starts by aquiring current camera frame. Then the image is converted to shades of gray scale and binarized based on preconfigured treshold. After that, frame udergoes morphological skeletonization based on Zhang Suen algorithm[^4]. The workspace is then narrowed down to arch, which starts from bottom corners of the image and its middle point being at the center of an array. Algorithm then checks wether path found by skeletonization intersects the arch. If multiple intersections are detected, then the most center one is chosen. Selected array element then serves as a waypoint for controller.

## Control system
A cascade PID controller has been chosen for control system. Using this kind of controller is a conviniet way of making control problem easier, because of decomposition. It consists of two feedback control loops, where the inner one takes care of velocity and the outter one handles the position. 

Position control loop takes normalized waypoint calculated from vision system as an input and compares it with target position. Said waypoint is expressed as deviation from center of the image, thus the target is set to be a zero value. Based on that wheels velocity is computed. The inner loop then takes that value as an input and compares it with measured velocity. This results in valid velocity value, evaluated with proper inertia, which is then passed to the Webots motor interface. 

## Video presentation
[![Watch the video](https://img.youtube.com/vi/26f_vONlVig/0.jpg)](https://www.youtube.com/watch?v=26f_vONlVig)


Scenario seen in the video assumes, that robot should complete track bothways - first going clockwise and next counter clockwise. The track selected for the project was taken from *Deep Learning Approach for the Mobile-Robot Motion Control System* article[^5].

## References
[^1]: Webots https://cyberbotics.com/
[^2]: Cascade PID controller schematics https://doc.synapticon.com/circulo/tutorials/tuning_guides/position_controller_with_cascaded_structure.html
[^3]: NVIDIA JetBot https://cyberbotics.com/doc/guide/jetbot?version=R2021b
[^4]: Zhang Suen https://rosettacode.org/wiki/Zhang-Suen_thinning_algorithm
[^5]: Line follower map https://www.techscience.com/iasc/v29n2/42936/html