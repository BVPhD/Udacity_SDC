# Markov Localization - Particle Filter

In this example, I am given a simulation environment where a robot (represented as a car) is placed within a known 2D map system, which contains 37 landmark locations, see figure 1.

![figure 1](./Markov_Localizer_Map.jpeg)

within this simulation environment, at each step, the robot moves with a scripted movement pattern that is unknown to me.  However, the robot does receive noisy ranging measurements to an unknown number of objects within the car's sensor range. The goal of the project is to implement a particle filter localization algorithm in C++ that accurately estimates the state of the robot within 0.01 meter accuracy and to do so in a timely manner.  The C++ code I implemented completes this task and I examen the effect of error on the number of particles at the end.  

The image below shows a screenshot from the simulation environment where the car image is the ground-truth pose of the robot, the blue circle with an arrow is the estimated location of the robot and the arrow points along it's estimated orientation.  The green lines are the ground-truth landmarks that are returning ranging signals while the blue lines are the estimated location of the landmark that is being ranged.

![figure 2](./sim_screenshot_particleFilter.jpeg)


## Quick Particle Filter Details
In localization tasks we are attempting to estimate the state variable of a robot, in this case it's 2D position in the map coordinate system and it's orientation with respect to the map coordinate x-axis. Under the Markov Model assumption, the current state of a robot $x_t$ at step $t$, only depends on the previous time step state $x_{t-1}$, control variables $u_t$, and environmental observations $z_t$, we express this state belief as a probability distribution function (pdf) given by,

${\rm bel}(x_t) = P(x_t|x_{t-1},u_t,z_t)$.

(Note I am using the same notation as "Probabilistic Robotics" - Sebastian Thrun, Wolfram Burgard, Dieter Fox) The estimation process proceeds in a two step fashion, first evolve your previous state under a noisy motion model given the control variables described by the function below,

$x_t = f(x_{t-1},u_t) + \epsilon_t$.

Where $\epsilon_t$ is a random variable capturing the imprecise actuation of the control command $u_t$, and has zero mean.  When we convolve this motion model in our belief we end up with a state pdf given by,

$P(\hat{x}_t|u_t) = \int P_{\rm motion}\left[\hat{x}_t-f(x_{t-1},u_t)\right]{\rm bel}(x_{t-1}) dx_{t-1}$.

Above, $P_{\rm motion}\left[\hat{x}_t-f(x_{t-1},u_t)\right]$ is the probabilistic model for the actuation noise.  

### Sampled vs Parametric Models
We can go one further approximation level and rather than storing the distribution information in a function ${\rm bel}(x_t)$ which might be approximated by a Gaussian for a Kalman filter, where the task is to estimate the Gaussian parameters (the mean and covariance), we can approximate the function by a kernel density estimator, 

${\rm bel}(x) \approx \frac{1}{N}\sum_{n=1}^N \phi(x|x^n)$,

where $x_t^n \sim {\rm bel}(x_t)$, and $\phi(x|x_t)$ is a normalized kernel function which is centered about $x_t$.  What we're doing in this case is trading a parametric estimation of ${\rm bel}(x_t)$ for an approximation that is contained in the evolution of the samples $x^n$ known as the particles of the filter.

Maintaining this approximation method (sampling), the predict step requires us to sample from $P(\hat{x}_t|u_t)=\overline{{\rm bel}}(\hat{x}_t)$. This can be done by evolving each of the particles $x^n_t$ under $f(x_{t-1},u_t)$ and sampling the actuation noise and summing.


The second step is rectifying environment measurements $z_t$ with the state estimate from the prediction step, ie we we need to sample the pdf

${\rm bel}(x_t)=P(x_t|z_t)= P(z_t|x_t)P(x_t)/P(z_t) \propto P(z_t|x_t)P(x_t)$.

We take $P(x_t) = \overline{{\rm bel}}(x_t)$ in the above.  Sampling this distribution can be done via importance sampling against the particle states which come from $\overline{{\rm bel}}(x_t)$. This means each particle receives a weight proportional to $P(z_t|x_t)$. In this example problem we model the sensor uncertainty as Gaussian noise where each observation is independent of the others thus,

$P(z_t|x_t) = \prod_{m=1}^{M_t} N(z^m_t|x_t,\sigma)$,

for $M_t$ observations at step $t$.  Once the weights $w^m$ are computed for each particle $m$, we resample the particles with replacement with probability of selection proportional to the $w^m$.  This results in a sampling of ${\rm bel}(x_t)$ and the process repeats.


## Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
|   helper_functions.h
|   main.cpp
|   map.h
|   particle_filter.cpp
|   particle_filter.h
```
The `main.cpp` code sets up communication with the Term 2 Simulator and when it begins to receive telemetry data from the simulator, initializes the particle filter class, implemented in `particle_filter.cpp`, and `particle_filter.h`, after which the functions:

`pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);`

implements the evolution of the state of each particle under a constant turn rate bicycle motion model (so long as the turn rate is not zero), the equations are given below for a state variable $x_t = (x_{x,t},x_{y,t},\theta_y)$ where $x_{x,t},x_{y,t}$ are the map coordinate system coordinates and $\theta$ the orientation of the robot.

$x_{x,t} = x_{x,{t-1}} + \int_0^{dt} v(\tau) cos(\theta(\tau)) d\tau$

$x_{y,t} = x_{y,{t-1}} + \int_0^{dt} v(\tau) sin(\theta(\tau)) d\tau$

$\theta_t = \theta_{t-1} + \int_0^{dt} \theta'(\tau) d\tau$

Under the $v(\tau) = v$ and $\theta'(\tau) = \theta'$ constant during $dt$ these equations become, 


$x_{x,t} = x_{x,{t-1}} + \int_{\theta_{t-1}}^{\theta_t} \frac{v_{t-1}}{\theta'_{t-1}} cos(\theta) d\theta$

$x_{y,t} = x_{y,{t-1}} + \int_{\theta_{t-1}}^{\theta_t} \frac{v_{t-1}}{\theta'_{t-1}} sin(\theta) d\theta$

$\theta_t = \theta_{t-1} +  \theta'_{t-1} dt$

finally

$x_{x,t} = x_{x,{t-1}} + \frac{v_{t-1}}{\theta'_{t-1}}(sin(\theta_{t-1})-sin(\theta_t))$

$x_{y,t} = x_{y,{t-1}} + \frac{v_{t-1}}{\theta'_{t-1}}(cos(\theta_t)-cos(\theta_{t-1}))$

$\theta_t = \theta_{t-1} + \theta'_{t-1} dt$

The yaw rate and velocities are taken as control variables in the above ie the $u_t$ variable. 

The observations are then integrated into the particles via the function:

`pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);`

which updates the importance of each particle dependent on it's compatibility with the observations given its internal state estimate.  The noise of the sensor readouts in robot local x-y coordinates is modeled by a Gaussian noise profile where the variances are given in the variable `sigma_landmark[2] = {0.3, 0.3};` found in `main.cpp`. Finally the particles that make up the filter are resampled via:

`pf.resample();`

and the loop repeats with some status print outs to console.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

running the binary particle_filter will wait for communication with the Udacity Term 2 Simulation where you will select project 3 and pushing run will display ground truth observations in gray rays, ground truth state of the car as a car image, and the best estimated observations and state estimate in blue.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.

### Examination of the Accuracy

We expect to see the accuracy of the estimation act as a function of $N^{-1/2}$ where $N$ is the number of particles, below we show some empirical results from running the C code for various particle sizes and somewhat confirm this behavior for a small sample size.

![figure 3](./empircal_accuracy_results.jpeg)
