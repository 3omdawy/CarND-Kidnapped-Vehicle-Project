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
using namespace std;

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	default_random_engine gen;
	num_particles = 100; // Emperical value to balance between execution time and accuracy
	// Create a normal (Gaussian) distribution for x, y, and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_psi(theta, std[2]);
	for (int i = 0; i < num_particles; ++i) {
		Particle p;
		p.id = i;
		// Sample  and from these normal distrubtions: 
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_psi(gen);
		while (p,theta > M_PI)		p.theta-=2*M_PI;
		while (p.theta <-M_PI)		p.theta+=2*M_PI;

		p.weight = 1.0;
		
		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	double vOverYawRate = velocity / yaw_rate;
	double thetaDotDeltaT = yaw_rate * delta_t;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_psi(0, std_pos[2]);
	
	for (int i = 0; i < num_particles; ++i) {
		Particle * p = &particles[i];

		double thetaAndDeltat = p->theta + thetaDotDeltaT;
		p->x = p->x + (vOverYawRate * ( sin(thetaAndDeltat) - sin(p->theta) ) );
		p->y = p->y + (vOverYawRate * ( cos(p->theta) - cos(thetaAndDeltat) ) );
		p->theta = thetaAndDeltat;
		while (p->theta > M_PI)		p->theta-=2*M_PI;
		while (p->theta <-M_PI)		p->theta+=2*M_PI;
		p->x += dist_x(gen);
		p->y += dist_y(gen);
		p->theta += dist_psi(gen);
		while (p->theta > M_PI)		p->theta-=2*M_PI;
		while (p->theta <-M_PI)		p->theta+=2*M_PI;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// Poor declaration that also does not contain a passed map --> skip and implement inside updateWeights
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	
	int nbObservations = observations.size();
	int nbLandmarks = map_landmarks.landmark_list.size();
	double xObservation, yObservation, xObservationOriginal, yObservationOriginal;
	double sigmaX = std_landmark[0];
	double sigmaY = std_landmark[1];
	const int kMinDist = 2 * sensor_range * sensor_range ;
	double twoSigX2 = 2 * sigmaX * sigmaX;
	double twoSigY2 = 2 * sigmaY * sigmaY;
	double twoPiSigXY = 2 * M_PI * sigmaX * sigmaY;
	
	for (int i = 0; i < num_particles; i++)
	{
		//cout << "Particle number = " << i << endl;
		double cosTheta = cos(particles[i].theta);
		double sinTheta = sin(particles[i].theta);
		double xParticle = particles[i].x;
		double yParticle = particles[i].y;
		double newWeight = 1.0;
		// Step 1: Convert observations to map coordinates
		for (int j = 0; j < nbObservations; j++)
		{
			double predictedX ;
			double predictedY ;
			xObservationOriginal = observations[j].x;
			yObservationOriginal = observations[j].y;
			xObservation =  xParticle + (xObservationOriginal * cosTheta) - (yObservationOriginal * sinTheta);
			yObservation = yParticle + (xObservationOriginal * sinTheta) + (yObservationOriginal * cosTheta);
			//cout << "xObservation = " << xObservation << " and yObservation = " << yObservation << endl;
			double minDistance = kMinDist;
			// Step 2: Associate observations with landmarks
			for (int k = 0; k < nbLandmarks; k++)
			{
				double currentDistance ;
				double mapX = map_landmarks.landmark_list[k].x_f;
				double mapY = map_landmarks.landmark_list[k].y_f;
				double absDiffX = abs(mapX - xObservation);
				double absDiffY = abs(mapY - yObservation);
					currentDistance = dist(xObservation, yObservation, mapX, mapY);
					//cout << "Current distance = " << currentDistance << " at k = " << k << endl;
					if (currentDistance < minDistance)
					{
						//cout << "Minimum distance = " << currentDistance << "at k = " << k << endl;
						minDistance = currentDistance;
						predictedX = mapX;
						predictedY = mapY;
						if ( (absDiffX < sigmaX) && (absDiffY < sigmaY)  )
							break;
					}
			}
			if (minDistance < sqrt(2)*(sensor_range+sigmaX) )
			{
					// Step 3: Update particle weights if landmark was found
					double diffX = xObservation - predictedX;			
					double diffY = yObservation - predictedY;			
					double currentWeight = exp(- ( ( diffX * diffX / twoSigX2 ) + ( diffY * diffY / twoSigY2 ) ) ) / twoPiSigXY;
					//cout << "Current Weight = " << currentWeight << "at j = " << j << endl;
					newWeight *= currentWeight;
			}
			else
			{
				newWeight = 0;
			}
		}
		
		particles[i].weight = newWeight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<double> currentWeights;
	vector<Particle> newParticles;
	
	for (int i = 0; i < num_particles; i++)
		currentWeights.push_back(particles[i].weight);
	
    random_device rd;
    mt19937 gen(rd());
    std::discrete_distribution<> d(begin(currentWeights), end(currentWeights));
    for(int n=0; n<num_particles; ++n) {
        newParticles.push_back(particles[d(gen)]);
    }
   particles = newParticles;
   /*int index = rand() % num_particles;
   double beta = 0;
   double maxW = 0;
   vector<Particle> originalParticles = particles;
   for (int i = 0; i < num_particles; i++)
   {
		if (particles[i].weight > maxW)
			maxW = originalParticles[i].weight;
   }
   for (int i = 0; i < num_particles; i++)
   {
	   beta += (rand() / RAND_MAX) * 2 * maxW;
	   while (beta > originalParticles[index].weight)
	   {
		   beta -= originalParticles[index].weight;
		   index = (index + 1) % num_particles;
	   }
	   particles[i] = originalParticles[index];
   }*/
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
