/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // Too few won't fully represent distribution, too many will slow down performance
  particle_number = 40;
  
  // Init particle number and particle vectors
  weights = std::vector<double>(particle_number);
  particles = std::vector<Particle>(particle_number);
  
  // Note these distribution and gen can't be extracted into a method for default_random_engine is Pseudo-random number generation
  // If extrat this into a method all particles calling this method will end up being the same and sampling does not make sense
  normal_distribution<double> x_norm_dist(x, std[0]);
  normal_distribution<double> y_norm_dist(y, std[1]);
  normal_distribution<double> theta_norm_dist(theta, std[2]);
  default_random_engine gen;
  
  // Generate particles according to initial GPS obs plus Gaussian noise
  for(int i = 0; i < particle_number; i++) {
    particles[i].x = x_norm_dist(gen);
    particles[i].y = y_norm_dist(gen);
    particles[i].theta = theta_norm_dist(gen);
    particles[i].id = i;
    weights[i] = 1.0 / particle_number;  // Weights are initialized assuming uniformed distribution
    particles[i].weight = weights[i];
  }
  is_initialized = true;
  eps_ = 1e-4;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  // initialise associations vector
  normal_distribution<double> x_norm_dist(0, std_pos[0]);
  normal_distribution<double> y_norm_dist(0, std_pos[1]);
  normal_distribution<double> theta_norm_dist(0, std_pos[2]);
  default_random_engine gen;
  if(fabs(yaw_rate) < eps_) {
    yaw_rate = eps_ < 0 ? 0 - eps_ : eps_;  // not divided by zero
  }
  for(int i = 0; i < particle_number; i++) {
    double theta_0 = particles[i].theta;
    // Following equations in Calculate Prediction Step Quiz of Lesson 14
    // The equations for updating x, y and the yaw angle when the yaw rate is not equal to zero
    particles[i].x  += ((velocity / yaw_rate) * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0)) + x_norm_dist(gen));
    particles[i].y += ((velocity / yaw_rate) * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t)) + y_norm_dist(gen));
    particles[i].theta += (yaw_rate * delta_t + theta_norm_dist(gen));
    cout << "predicted particle " << i << endl << "x: " << particles[i].x << endl << "y: " << particles[i].y << endl << "theta: " << particles[i].theta << endl;
  }
}

// Return distance between a landmark and obs
static double dist(const Map::single_landmark_s landmark, const LandmarkObs obs) {
  return sqrt(pow(landmark.x_f - obs.x, 2) + pow(landmark.y_f - obs.y, 2));
}

// Note: Changed original signature
std::vector<Map::single_landmark_s> ParticleFilter::dataAssociation(const std::vector<Map::single_landmark_s> predicted, const std::vector<LandmarkObs> observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  std::vector<Map::single_landmark_s> nearest_landmarks;
  // For each of the observation, find nearest landmark
  for (int i = 0; i < observations.size(); i++) {
    int min_index = 0;
    
    cout << "obs: " << i << endl << "x: " << observations[i].x << endl << "y: " << observations[i].y << endl << "id: " << observations[i].id << endl;
    double min_distance = dist(predicted[0], observations[i]);
    for (int j = 1; j < predicted.size(); j++) {
      double distance = dist(predicted[j], observations[i]);
      if (distance < min_distance) {
        min_index = j;
        min_distance = distance;
      }
    }
    cout << "nearest landmark: " << i << endl << "x: " << predicted[min_index].x_f << endl << "y: " << predicted[min_index].y_f  << endl;
    nearest_landmarks.push_back(predicted[min_index]);
  }
  return nearest_landmarks;
}

static std::vector<LandmarkObs> transformObservation(const std::vector<LandmarkObs> observations, const Particle particle) {
  std::vector<LandmarkObs> transformed_observations;
  for (int i = 0; i < observations.size(); i++) {
    LandmarkObs transformed_landmark;
    cout << "Orig obs " << i << endl << "x: " << observations[i].x << endl << "y: " << observations[i].y << endl << "id: " << observations[i].id << endl;
    transformed_landmark.id = observations[i].id;
    // Homogeneous transform to convert car observation to global coordinate (used by landmarks in map)
    transformed_landmark.x = observations[i].x * cos(particle.theta) - observations[i].y * sin(particle.theta) + particle.x;
    transformed_landmark.y = observations[i].x * sin(particle.theta) + observations[i].y * cos(particle.theta) + particle.y;
    transformed_observations.push_back(transformed_landmark);
    cout << "Transformed obs " << i << endl << "x: " << transformed_observations[i].x << endl << "y: " << transformed_observations[i].y << endl << "id: " << transformed_observations[i].id << endl;
  }
  return transformed_observations;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  // `observations` is an array of LandmarkObs, each el of which
  // has id, x and y properties (see helper_functions.h)
  
  
  // Defined in Update Step lecture
  double sigma_x_square = pow(std_landmark[0], 2);
  double sigma_y_square = pow(std_landmark[1], 2);
  double sigma_x_by_sigma_y = std_landmark[0] * std_landmark[1];
  
  for (int i = 0; i < particle_number; i++) {
    // Note this O(3MN) solution makes code clean but could be done in O(MN). So clean code here impacts performance
    
    // Homogeneous transform to convert car observation to global coordinate (used by landmarks in map)
    std::vector<LandmarkObs> transformed_observations = transformObservation(observations, particles[i]);
    
    // First half is original observation. Second half is corresponding predicted measurement
    std::vector<Map::single_landmark_s> nearest_landmarks = dataAssociation(map_landmarks.landmark_list, transformed_observations);
    cout << "Original weight: " << i << endl << particles[i].weight << endl;
    particles[i].weight = 1;
    for (int j = 0; j < transformed_observations.size(); j++) {
      double x_diff = transformed_observations[j].x - nearest_landmarks[j].x_f;
      double y_diff = transformed_observations[j].y - nearest_landmarks[j].y_f;
      
      // Following equations derived from Update Step lecture of Lesson 14, final version in Particle Weights Solution lecture
      particles[i].weight *= (exp(-0.5 * ((x_diff * x_diff)/sigma_x_square + (y_diff * y_diff)/sigma_y_square)) / (2 * M_PI * sigma_x_by_sigma_y));
      weights[i] = particles[i].weight;
    }
    cout << "Updated weight: " << i << endl << particles[i].weight << endl;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  default_random_engine gen;
  
  // Take a discrete distribution with pmf equal to weights
  discrete_distribution<> weights_pmf(weights.begin(), weights.end());
  // initialise new particle array
  vector<Particle> updated_particles;
  // resample particles
  for (int i = 0; i < particle_number; i++) {
    updated_particles.push_back(particles[weights_pmf(gen)]);
  }
  particles = updated_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
