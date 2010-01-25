/**
* @file Predictor.h
* @author Thomas Mörwald
* @date January 2009
* @version 0.1
* @brief Prediction for motion of object to track
* @namespace Tracker
*/

#ifndef __PREDICTOR_H__
#define __PREDICTOR_H__

#include "headers.h"
#include "Distribution.h"
#include "Timer.h"
#include "TM_Vector3.h"

#define GAUSS  0
#define NORMAL 1

/** @brief class Predictor */
class Predictor
{
private:

	Timer m_timer;
	float m_fTime;
	Tracking::TM_Vector3 m_cam_view;
	
	float noise(float sigma, unsigned int type=GAUSS);
	Tracking::Particle genNoise(float sigma, Tracking::Particle pConstraint, unsigned int type=GAUSS);

	void addsamples(Tracking::Distribution& d, int num_particles, Tracking::Particle p_initial, Tracking::Particle p_constraints, float sigma=1.0);
	
public:
	Predictor();
	
	/** @brief Set vector pointing from camera to object mean, to enable zooming*/
	void setCamViewVector(Tracking::TM_Vector3 v){ m_cam_view = v; }
	
	/**	@brief Resample particles accourding to current likelihood distribution (move particles)
	*		@param d pointer to distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle)
	*/
	virtual void resample(Tracking::Distribution& d, int num_particles, Tracking::Particle variance);
	
	/** @brief Sample new distribution 
	*		@param d pointer to distribution
	*		@param num_particles number of particles to represent likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle)
	*/
	virtual void sample(Tracking::Distribution& d, int num_particles, Tracking::Particle mean, Tracking::Particle variance);
	
};
 
 #endif