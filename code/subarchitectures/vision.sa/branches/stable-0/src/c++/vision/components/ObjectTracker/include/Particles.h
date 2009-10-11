
#ifndef __PARTICLES_H__
#define __PARTICLES_H__

class Particle;
class Particles;
#include "Quaternion.h"
#include "mathlib.h"
#include "Resources.h"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <GL/gl.h>
#include <vector>

#define GAUSS 0
#define NORMAL 1

class Particle
{
public:
	float rX, rY, rZ;	// rotation
	float tX, tY, tZ;	// translation
	float w;			// Likelihood
	float v, d;
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
	void translate(float x, float y, float z);

};

class Particles
{
private:
	
	Particle* m_particlelist;
	int m_num_particles;
	int id_max;
	int v_max;
	int d_max;
	float w_max;
	float m_frustum_offset;
	
	unsigned int* queryMatches; // query for all edge pixel count with NV_Occlusion_Query
    unsigned int* queryEdges; // query for coinciding edge pixel count with NV_Occlusion_Query
    
    float noise(float rMin, float rMax, unsigned int precision, unsigned int distribution=GAUSS);

public:
	Particles(int num, Particle p);
	~Particles();
	
	void set(int id, Particle p){ m_particlelist[id] = p; }
	
	Particle* get(int id){ return &m_particlelist[id]; }
	Particle* getMax(){ return &m_particlelist[id_max]; }
	int getMaxID(){ return id_max; }
	int getVMax(){ return v_max; }
	void setVMax(int val){ v_max = val; }
	int getNumParticles(){ return m_num_particles; }
	void printMax(){ m_particlelist[id_max].print(); }
	
	void perturb(Particle noise_particle, int num_particles, Particle* p_ref=NULL, unsigned int distribution=GAUSS);
	void activate(int id);
	void deactivate(int id);
	void activateMax(){ activate(id_max); }
	void deactivateMax(){ deactivate(id_max); }
	
	void startCountD(int id);
	void startCountV(int id);
	void endCountD();
	void endCountV();
	
	void calcLikelihood(int num_particles, unsigned int num_avaraged_particles=1);
	
	void setAll(Particle p);


};


#endif