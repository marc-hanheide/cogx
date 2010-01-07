
#ifndef __PARTICLES_H__
#define __PARTICLES_H__

class Particle;
class Particles;
#include "Quaternion.h"
#include "mathlib.h"
#include "Resources.h"
#include "Timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <GL/gl.h>
#include <vector>

const unsigned int GAUSS  = 0;
const unsigned int NORMAL = 1;

class Particle
{
public:
	vec3 r;				// rotation
	vec3 s;				// position
	vec3 rp;			// angular speed
	vec3 sp;			// speed
	float z;			// scale (zoom)
	float zp;
	
	float w;			// weight
	float c;			// confidence level
	Quaternion q;		// representing rotation with quaternions
	
	Particle();
	Particle(float val);
	Particle(const Particle& p2);
	
	Particle& operator=(const Particle& p2);
	
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }
	
	void activate();
	void deactivate();
	
	void print();
	//void getModelView(float* matrix4x4);
	void setPose(mat3 rot, vec3 pos);
	void getPose(mat3 &rot, vec3 &pos);
	void rotate(float x, float y, float z);
	void rotate(vec3 rot);
	void translate(float x, float y, float z);
	void translate(vec3 trans);

};

class Particles
{
private:
	
	vector<Particle> m_particlelist;
	vector<unsigned int> queryMatches;
	vector<unsigned int> queryEdges;
	Particle m_maxParticle;
	
	Timer m_timer;
	float m_fTime;

	int v_max;
	float w_sum;
	float w_max;
	float c_max;
	TM_Vector3 m_cam_view;
	
	// Functions
	Particle* calcMax();
	float noise(float sigma, unsigned int distribution=GAUSS);
	Particle genNoise(float sigma, Particle pConstraint, unsigned int distribution=GAUSS);
	void resizeQueries(int new_size);	
	
public:
	Particles(int num, Particle p);
	~Particles();
	
	void setCamViewVector(TM_Vector3 v){ m_cam_view = v; }
	void resetTimer(){ m_timer.Update(); }

	Particle* get(int id){ return &m_particlelist[id]; }
	Particle* getMax(){ return &m_maxParticle; }
	float getMaxW(){ return w_max; }
	float getMaxC(){ return c_max; }
	int getNumParticles(){ return m_particlelist.size(); }
	
	void addsamples(int num_particles, Particle p_initial, Particle p_constraints, float sigma=1.0);
	void sample(int num_particles, Particle p_initial, Particle p_constraints);
	void resample(int num_particles, Particle p_constraints);
	
	void activate(int id);
	void deactivate(int id);
	
	void startCountD(int id);
	void startCountV(int id);
	void endCountD();
	void endCountV();
	
	void calcLikelihood(int power=5);
	
	float getVariance();
	
	void setAll(Particle p);
};


#endif
