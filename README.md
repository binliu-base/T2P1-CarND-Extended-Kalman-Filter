# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

# Project Overview

The goals / steps of this project are the following:

* Complete the Extended Kalman Filter algorithm in C++.
* Apply it to pedestrian position/speed estimation problem 
* Evaluate it again two sampled data, the metrics used is RMSE

[//]: # (Image References)
[pedestrian_tracking_1]: (https://cloud.githubusercontent.com/assets/24623272/24600423/3659ca86-1887-11e7-939b-1a56ba1d1729.JPG)
[pedestrian_tracking_2]: (https://cloud.githubusercontent.com/assets/24623272/24600428/3b37d926-1887-11e7-9ad1-1e741acde492.JPG)


## Final Result

#### 1. sample data 1

![Pedestrian Tracking 1](https://cloud.githubusercontent.com/assets/24623272/24600423/3659ca86-1887-11e7-939b-1a56ba1d1729.JPG)

Accuracy - RMSE:  
0.026922   
0.0243627   
0.335336   
0.370813 

#### 2. sample data 2
![Pedestrian Tracking 2](https://cloud.githubusercontent.com/assets/24623272/24600428/3b37d926-1887-11e7-9ad1-1e741acde492.JPG)

Accuracy - RMSE:  
0.170537   
0.167369    
0.38511   
0.644944   

Note please check the notebook (ekf-visualization.ipynb) for details.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`
