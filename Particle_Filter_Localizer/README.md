# Markov Localization - Particle Filter

In this example, I am given a simulation environment where a robot (represented as a car) is placed within a known 2D map system, which contains 37 landmark locations, see figure 1.

![figure 1](https://github.com/BVPhD/Udacity_SDC/blob/master/Particle_Filter_Localizer/Markov_Localizer_Map.jpg)

within this simulation environment, at each step, the robot moves with a scripted movement pattern that is unknown to me.  However, the robot does receive noisy ranging measurements to an unknown number of objects within the car's sensor range. The goal of the project is to implement a particle filter localization algorithm in C++ that accurately estimates the state of the robot within 0.01 meter accuracy and to do so in a timely manner.  The C++ code I implemented completes this task and I examen the effect of error on the number of particles at the end.  

The image below shows a screenshot from the simulation environment where the car image is the ground-truth pose of the robot, the blue circle with an arrow is the estimated location of the robot and the arrow points along it's estimated orientation.  The green lines are the ground-truth landmarks that are returning ranging signals while the blue lines are the estimated location of the landmark that is being ranged.

![figure 2](https://github.com/BVPhD/Udacity_SDC/blob/master/Particle_Filter_Localizer/sim_screenshot_particleFilter.jpg)

More details of the project implementation is found in the writeup juypter notebook in this repo.
