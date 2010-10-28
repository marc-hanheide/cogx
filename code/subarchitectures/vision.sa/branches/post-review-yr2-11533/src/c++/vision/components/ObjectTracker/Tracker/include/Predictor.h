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
#include "TM_Vector3.h"
#include "Timer.h"


#define GAUSS  0
#define NORMAL 1

/** @brief class Predictor */
class Predictor
{
protected:
	float m_dTime;
	float m_powTime;
	float m_powTimeSteps;
	
	Tracking::TM_Vector3 m_cam_view;
	
	float noise(float sigma, unsigned int type=GAUSS);
	Tracking::Particle genNoise(float sigma, Tracking::Particle pConstraint, unsigned int type=GAUSS);

	/** @brief Adds samples/particles to a distribution d by adding gaussian noise
	*		@param d particle distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle)
	*		@param sigma standard variation of sampling (dependent on confidence value -> sigma(c)) */
	virtual void addsamples(Tracking::Distribution& d, int num_particles, Tracking::Particle mean, Tracking::Particle variance, float sigma=1.0);
	
public:
	Predictor();
	
	/** @brief Set vector pointing from camera to object mean, to enable zooming*/
	void setCamViewVector(Tracking::TM_Vector3 v){ m_cam_view = v; m_cam_view.normalize(); }
	
	/**	@brief Resample particles accourding to current likelihood distribution (move particles)
	*		@param d pointer to distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void resample(Tracking::Distribution& d, int num_particles, Tracking::Particle variance);
	
	/** @brief Sample new distribution 
	*		@param d pointer to distribution
	*		@param num_particles number of particles to represent likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void sample(Tracking::Distribution& d, int num_particles, Tracking::Particle mean, Tracking::Particle variance);
	
	/** @brief Updates time of prediction system for higher order motion model
	*		@param dTime Time since last frame */
	virtual void updateTime(double dTime);
	
};
 
 #endif