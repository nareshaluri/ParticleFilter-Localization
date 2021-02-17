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
  // Set the number of particles. 
  // Initialize all particles to first position (based on estimates of 
  // x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  num_particles = 20;
  particles.resize(num_particles);
  weights.resize(num_particles);

  default_random_engine gen;
  double sample_x, sample_y, sample_psi;
  //create a normal (Gaussian) distribution for x, y, psi.
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_psi(theta, std[2]);

  //Add random Gaussian noise to each particle
  for(int i=0; i<num_particles; ++i)
  {
    // Sample  and from these normal distrubtions 
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_psi = dist_psi(gen);


    Particle& p = particles.at(i);
    p.id = i;
    p.x = sample_x;
    p.y = sample_y;
    p.theta = sample_psi;
    p.weight = 1;
    weights[i] = 1;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Add measurements to each particle and add random Gaussian noise.
  // using std::normal_distribution and std::default_random_engine

  default_random_engine gen;
  double sample_x, sample_y, sample_psi;
  //create a normal (Gaussian) distribution for x, y, psi.
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_psi(0, std_pos[2]);

  for(int i=0; i<num_particles; i++)
  {
    Particle &p = particles.at(i);
    if(fabs(yaw_rate) < 0.00001)
    {
      p.x = p.x + velocity * delta_t * cos(p.theta);
      p.y = p.y + velocity * delta_t * sin(p.theta);
    }
    else
    {
      p.x = (p.x + (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta)));
      p.y = (p.y + (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t)));
      p.theta = p.theta + yaw_rate * delta_t;
    }

    //Add random Gaussian noise to each particle
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_psi(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& predicted, 
    std::vector<LandmarkObs>& observations) {
  // helper function to find the predicted measurement that is closest to each observed measurement and assign the 
  // observed measurement to this particular landmark.
  for(int i=0, observations_size = observations.size(); i<observations_size; i++)
  {
    int lm_id;
    double min_dist = std::numeric_limits<double>::max();
    for(int j=0, predicted_size = predicted.size(); j<predicted_size; j++)
    {
      double cur_dist = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if(cur_dist <= min_dist)
      {
        min_dist = cur_dist;
        lm_id = predicted[j].id;
      }
    }
    observations.at(i).id = lm_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    std::vector<LandmarkObs>& observations, Map& map_landmarks) {
  //   Update the weights of each particle using a mult-variate Gaussian distribution.
  //   The observations are given in the VEHICLE'S coordinate system. Our particles are located
  //   according to the MAP'S coordinate system. We will need to transform between the two systems.
  //   i.e both rotation AND translation.

  //Update particle weights
  for(int i=0; i<num_particles; i++)
  {
    Particle &p = particles[i];

    //Create valid map landmarks vector which are in sensor range of a given particle
    std::vector<LandmarkObs> map_land_marks_near_observed_land_marks;
    for(int j=0, map_lm_lst_size = map_landmarks.landmark_list.size(); j<map_lm_lst_size; j++)
    {
      double distance = dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f, p.x, p.y);
      if(distance <= sensor_range)
      {
        //Consider land marks which are in sensor range
        map_land_marks_near_observed_land_marks.push_back(LandmarkObs());
        LandmarkObs& map_lm_ob = map_land_marks_near_observed_land_marks.back();
        map_lm_ob.x = map_landmarks.landmark_list[j].x_f;
        map_lm_ob.y = map_landmarks.landmark_list[j].y_f;
        map_lm_ob.id = map_landmarks.landmark_list[j].id_i;
      }
    }

    //Translate observed landmarks from car co-ordinate system to Map coordinate system
    //store it in global_observations vector
    int observations_size = observations.size();
    std::vector<LandmarkObs> global_observations(observations_size);
    for(int j=0; j<observations_size; j++)
    {
      global_observations.at(j).x = p.x + (observations[j].x * cos(p.theta)) - (observations[j].y * sin(p.theta));
      global_observations.at(j).y = p.y + (observations[j].x * sin(p.theta)) + (observations[j].y * cos(p.theta)); 
    }

    //associate each obseravation with a landmark
    dataAssociation(map_land_marks_near_observed_land_marks, global_observations);

    //Calculate multi-variate gaussian probability & final weight of a particle
    double final_weight = 1;
    for(int j=0; j<global_observations.size(); j++)
    {
      LandmarkObs& global_ob = global_observations.at(j);
      for(int k =0; k<map_land_marks_near_observed_land_marks.size(); k++)
      {
        LandmarkObs& global_pred = map_land_marks_near_observed_land_marks.at(k);
        if(global_pred.id == global_ob.id)
        {
          double mvg = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1])) * 
            exp(-1 * ((((global_ob.x - global_pred.x)*(global_ob.x - global_pred.x))/(2 * std_landmark[0] * std_landmark[0]))+
              (((global_ob.y - global_pred.y)*(global_ob.y - global_pred.y))/(2 * std_landmark[1] * std_landmark[1]))));
          //Multiply mvgs for all observations
          final_weight *= mvg;
        }
      }
    }

    //Update weights of particle
    p.weight = final_weight;
    weights[i] = final_weight;

    //Set assications, sense_x, sense_y for this particle
    int global_ob_size = global_observations.size();
    std::vector<double> sense_x(global_ob_size); //the associations x mapping already converted to world coordinates
    std::vector<double> sense_y(global_ob_size); //the associations y mapping already converted to world coordinates
    std::vector<int> associations(global_ob_size); // The landmark id that goes along with each listed association
    for(int j=0; j<global_ob_size; j++)
    {
      LandmarkObs& tmp = global_observations.at(j);
      sense_x[j] = tmp.x;
      sense_y[j] = tmp.y;
      associations[j] = tmp.id;
    }

    p.associations = std::move(associations);
    p.sense_x = std::move(sense_x);
    p.sense_y = std::move(sense_y);
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight. 
  // using std::discrete_distribution

  std::vector<Particle> new_particles;
  new_particles.resize(num_particles);
  // Use discrete_distribution to obtain set of particles 
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  for(int i=0; i<num_particles; ++i) 
  {
    new_particles[i] = particles[d(gen)];
  }
  particles = std::move(new_particles);
}

void ParticleFilter::eval_best_particle(Particle& best)
{
  // Calculate and output the average weighted error of the particle 
  // filter over all time steps so far.
  double highest_weight = -1.0;
  Particle best_particle;
  double weight_sum = 0.0;
  for (int i = 0; i < num_particles; ++i) 
  {
    if (particles[i].weight > highest_weight) 
    {
      highest_weight = particles[i].weight;
      best_particle = particles[i];
    }
    weight_sum += particles[i].weight;
  }
  cout << "highest w " << highest_weight << endl;
  cout << "average w " << weight_sum/num_particles << endl;
  best = best_particle;
}

string ParticleFilter::getAssociations(Particle& best)
{
  vector<int> v = best.associations;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle& best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle& best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}