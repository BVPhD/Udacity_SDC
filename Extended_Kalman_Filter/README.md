# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project an Extended Kalman Filter (EKF) is implemented in C++ code, to estimate the 2D position (p_x,p_y) and 2D velocity (v_x,v_y), state of a car of where noisy LIDAR and RADAR measurements are sequentially observed.  The EKF sequentially takes measurements, integrates the measurements with a previous state likelihood to give maximum likelihood estimates of the state of the object, as well as related uncertainties, under the assumption of Gaussian noise in each measurement, and each forward step in the evolution of the car's true position. 

The Project is organized into, `data, Docs, src` and some files to install WebSocketsIO with `install-mac.sh` or whatever OS you are running.  The `CMakeLists.txt` is used to generate the make files for compiling the source code.  All of the main code segments are found in `src`, and specifically in `FusionEKF.cpp` and `kalman_filter.cpp` for the code that is related to the Kalman filter.  The other code segments are used for connecting the program to the simulator provided by Udacity, and for processing the data points that are sequentially provided to the program.  

The main discussion of the code and theory behind it is found in `EKF_theory.ipynb` and suggest starting there.
