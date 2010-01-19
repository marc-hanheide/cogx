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

#include "Pose3.h"
#include "Quaternion.h"
#include "mathlib.h"
#include "headers.h"		// stdio.h

/**	@brief class Particle */
class Particle : public Pose3
{
public:
	vec3 rp;			///< angular speed
	vec3 tp;			///< translational speed
	float z;			///< scaling (zoom)
	float zp;			///< scaling speed (zoom)
	
	float w;			///< weighted likelihood (sum of w of distribution = 1)
	float c;			///< confidence level (matching pixels divided by overall pixels)

	Particle();
	Particle(float val);
	Particle(const Particle& p2);
	
	Particle& operator=(const Particle& p2);
	
	/**	@brief Comparing weighted likelihood of two particles	*/
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }

};


#endif
