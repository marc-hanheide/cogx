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

#include <v4r/TomGine/tgMathlib.h>
#include "TrackerModel.h"
#include "Distribution.h"
#include "Predictor.h"
#include "Filter.h"

namespace Tracking{

enum TrackingState{
	ST_OK,
	ST_OCCLUDED,
	ST_LOST,
	ST_LOCKED,
	ST_DISABLED,
};

/** @brief class ModelEntry */
class ModelEntry
{
//private:
//	void poseDiff(float &t, float &a);
//	void speed();
//
//	float max(float a, float b){ return (((a) > (b)) ? (a) : (b)); }
//	float abs(float a){ return ((a>=0.0f) ? a : (-a)); }
	
	
	
public:
	ModelEntry(float lpf);
	ModelEntry(const TomGine::tgModel& m, float lpf);
	~ModelEntry();
	
	void resetPose();
	void setInitialPose(const TomGine::tgPose &p);
	void filter_pose();
	void evaluate_signals();
	void evaluate_tsd();
	
	std::string label;
	
//	float t, a, t_max, a_max, c_edge, c_th, c_lost, abs_a, abs_t;
	
	TrackingState ts;						///< tracking state
	
	TrackerModel		model;				///< The model to track
	Distribution 		distribution;		///< Likelihood distribution
	Predictor*			predictor;			///< Movement prediction
	TomGine::tgTexture2D*			mask;				///< Mask for model edges not to be considered (image space)
	Particle			pose;				///< Current pose of the model
	Particle			pose_prev;			///< Previouse pose of the model
	Particle			lpf_pose;		///< Low pass filtered pose;
	Particle			initial_pose;		///< Initial pose, pose where to reset model to
	
	float	speed_angular;					///< Current angular speed
	float	speed_translational;			///< Current translational speed

	unsigned 	id;							///< ID of model (for removing)
	unsigned	num_convergence;			///< number of steps until convergence (static object);
	unsigned	hypothesis_id;				///< ID of model to compare with (if this ModelEntry is a hypothesis)
	std::vector<float> past_confidences;	///< vector of the past confidences
	unsigned	num_particles;				///< Number of particles used for representing likelihood distribution
	unsigned	num_recursions;				///< Number of recursions per image
	
	bool		bfc;						///< enable/disable backface culling
	bool		lock;						///< enable/disable lock
	bool		mask_geometry_edges;		///< enable/disable masking of geometry edges (use texture only)
	
	TomGine::mat4 modelviewprojection;				///< Tranformation matrix from model to image coordinates
	TomGine::vec3 vCam2Model;			///< Vector from camera to model (for zooming DOF and texturing)
	
public:
//	Predictor* del_predictor;
	FloatFilter m_lpf_pose_tx;
	FloatFilter m_lpf_pose_ty;
	FloatFilter m_lpf_pose_tz;
	FloatFilter m_lpf_pose_qx;
	FloatFilter m_lpf_pose_qy;
	FloatFilter m_lpf_pose_qz;
	FloatFilter m_lpf_pose_qw;
	float c_mkp;	///< mean confidence of current kept particles (!= static particles)
	FloatFilter w_msp;	///< weighted mean of static particles (in ratio to sum of weights of kept particles)
	float c_occ;
	float accuracy;
	float c_max_runtime;

	float c_normalized;

	FloatFilterFall c_comp;
	FloatFilterFall c_occ_comp;

public:
//	FloatFilter m_lpf_a;
//	FloatFilter m_lpf_t;
//	FloatFilter m_lpf_cs;
//	FloatFilter m_lpf_cl;

};

} // namespace Tracking

#endif
