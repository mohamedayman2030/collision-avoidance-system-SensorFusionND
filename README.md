# SFND 3D Object Tracking



<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, , I completed four major tasks: 
1. First, I matched 3D objects in different frames using Features extractions and matching.
2. Second, I compute TTC based on Lidar by calculating the mean distance to the ego car and using the constant velocity equations
3. then i calculated TTC using camera for all matched 3D objects using keypoint and bounding boxes in the current
4. the performance evaluation :
     1. TTC using Lidar isn't always correct because there are some of outliers
     2. also in camera TTC could be infected according to error in keypoint matching
     3. for the perforamce and speed of matching please check my project to get the best detectors & descriptors : https://github.com/mohamedayman2030/Camera-Based-2D-Feature-tracking
     in the following example you will see the change on TTC in Lidar & camera estimation on the frames
     ![first frame](https://i.ibb.co/xFrSrV4/1.png)
     ![second frame](https://i.ibb.co/DKDGbsC/2.png)
     ![third frame](https://i.ibb.co/HxvVsVy/3.png)
## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
hint : you need to download yolo weights from yolo website
