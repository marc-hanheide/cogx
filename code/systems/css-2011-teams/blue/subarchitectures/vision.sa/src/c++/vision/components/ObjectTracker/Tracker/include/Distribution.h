 /**
 * @file Distribution.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Likelihood distribution with GPU likelihood evaluation using comparisson shaders for object tracking.
 * @namespace Tracker
 */
#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include "Particle.h"
#include "Quaternion.h"
#include "mathlib.h"
#include "Resources.h"
#include "Timer.h"
#include "headers.h"  // stdio.h, stdlib.h, algorithm, gl.h, vector

namespace Tracking{

/** @brief typedef ParticleList */
typedef std::vector<Particle> ParticleList;

/**	@brief class Distribution */
class Distribution
{
public:
	
	ParticleList m_particlelist;				///< List of particles forming the likelihood distribution
	std::vector<unsigned int> queryMatches;	///< OpenGL Occlussion Query for counting matching pixels
	std::vector<unsigned int> queryEdges;		///< OpenGL Occlussion Query for counting overall edge pixels
	Particle m_meanParticle;						///< Particle representing weighted mean of particle distribution
	
	int v_max;								///< Maximum number of  overall edge pixels of current view
	double w_sum;							///< Sum of all weights, before normalisation (afterwards its 1)
	double w_max;							///< Maximum weighted likelihood in one likelihood measure
	double c_max;							///< Maximum confidence level in one likelihood measure
	double c_mean;
	TM_Vector3 m_cam_view;
	
	// Functions
	/** @brief Resize Occlusion Queries on GPU */
	void updateQueries();
	
	/** @brief Calculates weighted mean and stores it in m_meanParticle */
	void calcMean();
	
	void drawParticlesEdges(TrackerModel& model, Shader* shadeCompare, bool showparticles=false);
	void drawParticlesTextured(TrackerModel& model, Shader* shadeCompare, bool showparticles=false);
	void calcLikelihood(int convergence=5);	
	
public:
	Distribution();
	~Distribution();
	
	/**	@brief Gets weighted mean of likelihood distribution */
	Particle 		getMean(){ return m_meanParticle; }
	
	/** @brief Gets particle from distribution by id */
	Particle		getParticle(int id){ return (m_particlelist[id]); }
	
	/** @brief Calculates variance confidence level of likelihood distribution */
	double 			getVariance();
	
	/**	@brief Recalculates weights of particlelist to sum up to 1 */
	void				normalizeW();
	
	/** @brief Gets maximum weighted likelihood of last likelihood measure */
	double 			getMaxW(){ return w_max; }
	
	/** @brief Gets maximum confidence level of last likelihood measure */
	double 			getMaxC(){ return c_max; }
	
	/** @brief Gets weighted likelihood of a particle by id */
	double				getW(int id){ return m_particlelist[id].w; }
	
	/** @brief gets confidence level of a particle by id */
	double				getC(int id){ return m_particlelist[id].c; }
	
	/** @brief Number of particles representing the distribution*/
	int 				size(){ return m_particlelist.size(); }
	
	/** @brief Copy ParticleList representing distribution std::vector<Particle> */
	void				copy(ParticleList &list){ list = m_particlelist; }
	
	/** @brief Copy a particle from the distribution */
	void				copyParticle(Particle& p, int id){ if(id<=m_particlelist.size()) p = m_particlelist[id]; }
	
	/** @brief Drops all particles of the ParticleList */
	void				clear(){ m_particlelist.clear(); }
	
	/** @brief Adds a particle to the ParticleList of the distribution */
	void				push_back(Particle p){ m_particlelist.push_back(p); }
	
	/** @brief Update confidence levels and weighted likelihoods of distribution 
	*		@param TrackerModel 3D model to be rendered and compared to camera image
	*		@param Shader comparisson shader which draws only pixels matching with the corresponding pixel of the camera image
	*		@param textured flag wether to use textured models or edge models (TextureTracker / EdgeTracker)
	*		@param convergence convergence of weight distribution
	*		@param showparticles makes particle distribution visible on screen
	*/
	void		updateLikelihood(TrackerModel& model, Shader* shadeCompare, bool textured, int convergence=5, bool showparticles=false);

};

} // namespace Tracking


#endif
