/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>


#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   * first position (based on estimates of x, y, theta and their uncertainties
   * from GPS) and all weights to 1. 
   */
  num_particles = 100;  // Set the number of particles
  
  std::default_random_engine generator;
  std::normal_distribution<double> x_normal_dist(x,std[0]);
  std::normal_distribution<double> y_normal_dist(y,std[1]);
  std::normal_distribution<double> theta_normal_dist(theta,std[2]);
  Particle particle;
  
  // make a bunch of particles that are distributed
  // as a Guassian about the initial GPS measurement
  // push each new particle into the Particles vector
  // for the class
  for(unsigned int idx = 0; idx < num_particles; ++idx){
    particle.x = x_normal_dist(generator);
    particle.y = y_normal_dist(generator);
    particle.theta = theta_normal_dist(generator);
    particle.weight = 1.0;
    
    particles.push_back(particle);
    weights.push_back(1.0);
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * make the prediction step for each particle with random Gaussian noise.
   * for the control variables.
   **/
  std::default_random_engine generator;
  std::normal_distribution<double> x_normal_dist(0.0,std_pos[0]);
  std::normal_distribution<double> y_normal_dist(0.0,std_pos[1]);
  std::normal_distribution<double> theta_normal_dist(0.0,std_pos[2]);
  
   // evolve each particle under the constant turn rate motion model
  for (unsigned int idx = 0; idx < particles.size(); ++idx){
    // use the constant turn rate model if yaw_rate is not zero
    if(yaw_rate != 0.0){
  	 particles[idx].x += velocity/yaw_rate*(sin(particles[idx].theta+delta_t*yaw_rate)-sin(particles[idx].theta)) + x_normal_dist(generator)*delta_t;
     particles[idx].y += velocity/yaw_rate*(cos(particles[idx].theta)-cos(particles[idx].theta+delta_t*yaw_rate)) + y_normal_dist(generator)*delta_t;
    }else{
    // else use the velocity propagation model
        particles[idx].x += delta_t*(velocity*cos(particles[idx].theta) + x_normal_dist(generator));
        particles[idx].y += delta_t*(velocity*sin(particles[idx].theta) + y_normal_dist(generator));
    }
    particles[idx].theta += delta_t*(yaw_rate + theta_normal_dist(generator));
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> landmarks,
                                     vector<LandmarkObs>& observations) {
  /**
   *  Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   **/
    double d, best_d, best_x, best_y;
    
    for(unsigned int i = 0; i < observations.size(); ++i){
        best_d = -1.0;
        for(unsigned int j = 0; j < landmarks.size(); ++j){
            d = dist(landmarks[j].x,landmarks[j].y,
                     observations[i].x,observations[i].y);
            if(d <= best_d || best_d < 0.0){
                best_d = d;
                best_x = (landmarks[j].x-observations[i].x);
                best_y = (landmarks[j].y-observations[i].y);
            }
        }
        observations[i].x = best_x;
        observations[i].y = best_y;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   *  Update the weights of each particle using a multi-variate Gaussian
   *   distribution. You can read more about this distribution here
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems.
   **/
  double orientation;
  double obs_x, obs_y;
  double x_t, y_t, x_m, y_m;
  double test_lm_x, test_lm_y, dist, best_dist;
  double d_x,d_y;
  double gauss_norm,exponent;
  long double new_weight;
  unsigned int best_idx;
  
  // For each particle in the simulation...
  for(unsigned int idx = 0; idx < particles.size(); ++idx){
    // get the particle coordinates and rotation (in map frame)
    orientation = particles[idx].theta;
    x_t = particles[idx].x;
    y_t = particles[idx].y;
    
    // for each of the observations, convert them from
    // the particle frame to the map frame coordinates
    // and store the new best landmark associations
    new_weight = 1.0;
    particles[idx].associations.clear();
    particles[idx].sense_x.clear();
    particles[idx].sense_y.clear();
    
    for(unsigned int jdx = 0; jdx < observations.size(); ++jdx){
        obs_x = observations[jdx].x;
        obs_y = observations[jdx].y;
        
        // converting the observations to the map coordinate system
      	x_m = x_t + std::cos(orientation)*obs_x - std::sin(orientation)*obs_y;
      	y_m = y_t + std::sin(orientation)*obs_x + std::cos(orientation)*obs_y;
        
        // for the particle, store the map coordinates of
        // the current observation
        particles[idx].sense_x.push_back(x_m);
        particles[idx].sense_y.push_back(y_m);
        
        // compare each obs with the landmark map and
        // find the nearest landmark for the current observation
        best_dist = -1.0;
        for(unsigned int kdx = 0; kdx < map_landmarks.landmark_list.size(); ++kdx){
            test_lm_x = map_landmarks.landmark_list[kdx].x_f;
            test_lm_y = map_landmarks.landmark_list[kdx].y_f;
            
            dist = sqrt(pow(test_lm_x-x_m,2) + pow(test_lm_y-y_m,2));
            if(dist <= best_dist || best_dist < 0.0){
                best_dist = dist;
                d_x = test_lm_x-x_m;
                d_y = test_lm_y-y_m;
                best_idx = map_landmarks.landmark_list[kdx].id_i;
            }
        }
        // push back the best landmark association for the current observation
        particles[idx].associations.push_back(best_idx);
        
        // compute probability of the observation
        // for nearest-neighbor landmarking using
        // a Guassian noise model
        gauss_norm = 1./(2.*M_PI*std_landmark[0]*std_landmark[1]);
        exponent = (d_x*d_x)/pow(std_landmark[0],2)+(d_y*d_y)/pow(std_landmark[1],2);
        new_weight *= gauss_norm * exp(-exponent/2.);
    }
    particles[idx].weight = new_weight; //update the particle weights for the averaging in main.cpp
    weights[idx] = new_weight; //update the weights vector for the resampling part
  }

}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   std::default_random_engine generator;
   
   std::discrete_distribution<int> distro (weights.begin(),weights.end());
   unsigned int index;
    
   vector<Particle> new_particles;
   for(unsigned int i = 0; i < particles.size(); ++i){
       index = distro(generator);
       new_particles.push_back(particles[index]);
   }
   particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //  and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

// These functions are used to integrate with the simulation
// visualization and not really used for any algorithimic propose
string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
