# Extended Kalman Filters for Vehicle Detection

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# Description

This repository contains the files for a project from the [Udacity Self Driving Car Nanodegree Program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013). The goal of this project was to track and predict vehicle movement using a fusion of LIDAR and RADAR data processed with an Extended Kalman Filter.

<img src="https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/ekf_flow.png" width="500px">

The project code was compiled and used in tandem with a simulator from Udacity that mapped out the vehicle's movement along with the EKF's predictions. RMSE (error) values were computed simultaneously to assist in understanding the accuracy of the sensor data and EKF performance.

## Extended Kalman Filter Performance

### First Dataset

<img src="https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/dataset_1.png" width="500px">

| X-position RMSE | Y-position RMSE | X-velocity RMSE | Y-velocity RMSE |
| --------------- | --------------- | --------------- | --------------- |
| 0.0964          | 0.0853          | 0.4154          | 0.4316          |

### Second Dataset

<img src="https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/dataset_2.png" width="500px">

| X-position RMSE | Y-position RMSE | X-velocity RMSE | Y-velocity RMSE |
| --------------- | --------------- | --------------- | --------------- |
| 0.0727          | 0.0968          | 0.4893          | 0.5078          |

**Project code:**

* [Main C++ file (main.cpp)](https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/src/main.cpp) containing the script to ingest the sensor data and call the EKF
* [EKF object class (kalman_filter.cpp)](https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/src/kalman_filter.cpp) with predict and update functions
* [EKF tools (tools.cpp)](https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/src/tools.cpp) containing functions for RMSE and Jacobian Matrix calculations
* [EKF fusion (FusionEKF.cpp)](https://github.com/stephenvfg/ekf-vehicle-detection/blob/master/src/FusionEKF.cpp) for intepreting the RADAR and LIDAR data and fusing/processing it with the appropriate EKF functions
