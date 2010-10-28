/**
* @file Predictor.h
* @author Thomas Mörwald
* @date January 2009
* @version 0.1
* @brief Prediction for motion of object to track
* @namespace Tracker
*/

#ifndef _MY_PREDICTOR_H_
#define _MY_PREDICTOR_H_

#include "headers.h"
#include "Predictor.h"

/** @brief class Predictor */
class myPredictor : public Predictor
{
protected:
	virtual void addsamples(Tracking::Distribution& d, int num_particles, Tracking::Particle p_initial, Tracking::Particle p_constraints, float sigma=1.0);
	
};
 
 #endif