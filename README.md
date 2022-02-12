# IMU-ROS Simulation Package

> This package is meant to be used alongside ROS and a simulation env (e.g. Unity...). The noise model algorithm adds noise to a "true" IMU reading (linear acceleration, angular velocity) to produce a "real" IMU reading. The noise added should be unpredictable. The ultimate goal of this package is to create a more realistic simulation.

<!-- > Live demo [_here_](https://www.example.com). If you have the project hosted somewhere, include the link here. -->

Disclaimer: The algorithm is based on https://github.com/Aceinna/gnss-ins-sim. Modifications were made to suit real-time use case. 1 true input reading(from simulator) -> 1 real output reading , instead of taking in a bunch of readings and perform the algorithm.

## Table of Contents

- [General Info](#general-information)
- [Features](#features)
<!-- - [Imu Models](#imu-models) -->
- [Setup](#setup)
- [Usage](#usage)
- [Project Status](#project-status)
<!-- - [Room for Improvement](#room-for-improvement) -->
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)
<!-- * [License](#license) -->

## General Information

- The motivation of developing this package is to integrate it into a Unity-ROS simulation to test navigation systmes of underwater robots with more accurate sensor models. However, this package could virtually be integrated with any robotics simulation that uses IMU.

<!-- You don't have to answer all the questions - just the ones relevant to your project. -->

## Features

-Three pre-defined IMU models are described in imu_model_data.py, with different accuracies -> "low","mid","high". Custom IMU models are possible. Refer to https://github.com/Aceinna/gnss-ins-sim.

- "imu_noise" node subscribes to /imu_true and publishes to /imu_real. Both of which uses sensor_msgs/Imu message type , only linear_acceleration and angular_velocity data are defined.

- Noises are due to bias constant, bias drift and white noise(random walk). Bias drift is the only component that accumulates over time. The bias parameters (a,b) which can be found in imu_gen_model.py / bias_drift() are fixed for each axis (x,y,z) and generated upon receiving first true reading.Do note that if bias instability correlation is not defined, bias drift will simply be drift constant multiply a random number from a Gaussian distribution. More details can be found -> https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/gnss-ins-sim-doc.md

<!-- ## Imu Models

Demo of pre-defined IMU models:

AHRS380 (low accuracy) -->


## Setup

Requirement:

- ROS Noetic (other ROS distros might require modifications to scripts)
- python3

Dependencies:

- rospy
- sensor_msgs
- numpy
- std_msgs

Go to ros workspace and clone repo <br>
`cd /<catkin_ws>/src/`<br>
`git clone https://github.com/zejiekong/imu_ros.git`

Build workspace <br>
`cd ..`<br>
`catkin_make`

Source setup.bash
`source devel/setup.bash`

## Usage

To initialize the noise model <br>
`roslaunch imu_noise imu_noise.launch`

Visualisation of output in rqt_plot <br>
`rqt_plot /imu_real/<angular_velocity/linear_acceleration>`<br>

Tick autoscroll for real-time input.

Parameters:

- Accuracy of IMU ("low"/"mid"/"high"). Default -> "high"
- Sampling frequency of IMU. Default -> 100.

Can be set in "imu_noise/config/params.yaml".

## Project Status

<!--
Project is: _in progress_ / _complete_ / _no longer being worked on_. If you are no longer working on it, provide reasons why. -->

Project is : _in progress_

<!-- ## Room for Improvement

Include areas you believe need improvement / could be improved. Also add TODOs for future development.

Room for improvement:

- Improvement to be done 1
- Improvement to be done 2

To do:

- Feature to be added 1
- Feature to be added 2 -->

## Acknowledgements

<!-- Give credit here. -->

<!-- - This project was inspired by...
- This project was based on [this tutorial](https://www.example.com). -->

- Many thanks to https://github.com/Aceinna/gnss-ins-sim

## Contact

Created by [@zejiekong](https://zejiekong.github.io/Portfolio/) - feel free to contact me!

<!-- Optional -->
<!-- ## License -->
<!-- This project is open source and available under the [... License](). -->

<!-- You don't have to include all sections - just the one's relevant to your project -->
