 /**
 * @file ModelEntry.h
 * @author Thomas Mörwald
 * @date January 2009
 * @version 0.1
 * @brief Main file of Tracker.
 * @namespace Tracking
 */
#ifndef _MODEL_ENTRY_H_
#define _MODEL_ENTRY_H_

#include "mathlib.h"
#include "TrackerModel.h"
#include "Distribution.h"
#include "Predictor.h"

namespace Tracking{

/** @brief class ModelEntry */
struct ModelEntry
{
	ModelEntry(){
		del_predictor = predictor = new Predictor();
		bfc = false;
		lock = false;
		num_convergence = 0;
	}
	
	~ModelEntry(){
		delete(del_predictor);
	}
	
	std::string label;
	
	TrackerModel		model;					///< The model to track
	Distribution 		distribution;		///< Likelihood distribution
	Predictor*			predictor;			///< Movement prediction
	Particle				pose;						///< Current pose of the model
	Particle				initial_pose;		///< Initial pose, pose where to reset model to
	
	int 		id;											///< ID of model (for removing)
	int			num_convergence;				///< number of steps until convergence (static object);
	int			hypothesis_id;					///< ID of model to compare with (if this ModelEntry is a hypothesis)
	std::vector<float> past_confidences;				///< vector of the past confidences
	int			num_particles;					///< Number of particles used for representing likelihood distribution
	int			num_recursions;					///< Number of recursions per image
	
	bool		bfc;				///< enable/disable backface culling
	bool		lock;
	
	mat4 modelviewprojection;			///< Tranformation matrix from model to image coordinates
	TM_Vector3 vCam2Model;				///< Vector from camera to model (for zooming DOF and texturing)
	
private:
	Predictor* del_predictor;

};

} // namespace Tracking

#endif