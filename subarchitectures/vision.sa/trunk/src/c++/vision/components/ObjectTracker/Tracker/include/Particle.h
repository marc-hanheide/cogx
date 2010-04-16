 /**
 * @file Particle.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Defining a point/particle in the likelihood distribution
 * @namespace Tracker
 */
 
#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "Pose.h"
#include "Quaternion.h"
#include "mathlib.h"
#include "headers.h"		// stdio.h

namespace Tracking{

/**	@brief class Particle */
class Particle : public Pose
{
public:
	vec3 r;
	vec3 rp;			///< angular speed
	vec3 tp;			///< translational speed
	float z;			///< scaling (zoom)
	float zp;			///< scaling speed (zoom)
	
	double w;			///< weighted likelihood (sum of w of distribution = 1)
	double c;			///< confidence level (matching pixels divided by overall pixels)

	Particle(float val=0.0);
	Particle(const Particle& p2);
	Particle(const Pose& p2);
	
	Particle& operator=(const Particle& p2);
	Particle& operator=(const Pose& p);
	Particle& operator*(const float& f);
	
	/**	@brief Comparing weighted likelihood of two particles	*/
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }

};

} // namespace Tracking

#endif
