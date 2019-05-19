# Behavioral Cloning Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
In this project the goal is to use a Unity car driving simulator (provided to me from Udacity) to take samples of images taken from a simulated, forward facing set of cameras, while traveling around a track under my control. From that data, I will train a neural network to map camera images to steering angles. If this is done with high enough accuracy, then the simulation should be able to steer itself successfully around the track, nearly cloning the steering behavior that I controlled the simulation with.

I used a convolutional nueral network inspired from [M. Bojarski *et al*](https://arxiv.org/pdf/1604.07316v1.pdf) with an architecture provided in the image below:

[covnet architecture](./covnet_diagram.jpg)

The model was able to use center of the car forward facing images to interpret the scenery and make correct steering inputs to travel around the track both in the clockwise and anti-clockwise directions.  Detailed discussion of methods and how this network was trained can be found in the Behavioral_Cloning.ipynb notebook.

The files provided in this repo: 
* model.py (script used to create and train the model)
* drive.py (script to drive the car provided by Udacity)
* model.h5 (a trained Keras model)
* Behavioral_Cloning.ipynb (Discussion of project methods and code, a pdf version is also provided)
* track1_lap.mp4 (a video recording of vehicle driving autonomously around the track for one full lap)
