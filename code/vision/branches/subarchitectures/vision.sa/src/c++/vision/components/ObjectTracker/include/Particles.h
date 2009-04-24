
#ifndef __PARTICLES_H__
#define __PARTICLES_H__

class Particle;
class Particles;
#include "Resources.h"

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <GL/gl.h>

#define GAUSS 0
#define NORMAL 1

class Particle
{
public:
	float rX, rY, rZ;	// rotation
	float tX, tY, tZ;	// translation
	float w;			// Likelihood

	Particle();
	Particle(float val);
	Particle(float* mv);
	Particle(mat3 rot, vec3 pos);
	
	Particle& operator=(const Particle& p2);
	bool operator==(const Particle& p2);
	
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }
	
	void print();

};
	
class Particles
{
private:
	
	Particle* m_particlelist;
	int m_num_particles;
	int id_max;
	int v_max;
	float m_frustum_offset;
	
	unsigned int* queryD; // query for all edge pixel count with NV_Occlusion_Query
    unsigned int* queryV; // query for coinciding edge pixel count with NV_Occlusion_Query
    
    float noise(float rMin, float rMax, unsigned int precision, unsigned int distribution=GAUSS);

public:
	Particles(int num, Particle p);
	~Particles();
	
	void set(int id, Particle p){ m_particlelist[id] = p; }
	
	Particle* get(int id){ return &m_particlelist[id]; }
	Particle* getMax(){ return &m_particlelist[id_max]; }
	int getMaxID(){ return id_max; }
	int getVMax(){ return v_max; }
	unsigned int* getQueryD(int id){ return &queryD[id]; }
	unsigned int* getQueryV(int id){ return &queryV[id]; }
	int getNumParticles(){ return m_num_particles; }
	void printMax(){ m_particlelist[id_max].print(); }
	
	void perturb(Particle noise_particle, Particle* p_ref=NULL, unsigned int distribution=GAUSS);
	void activate(int id);
	void activateMax(){ activate(id_max); }
	void deactivate();
	
	void startCountD(int id);
	void startCountV(int id);
	void endCountD();
	void endCountV();
	
	void calcLikelihood(int num_particles, unsigned int num_avaraged_particles=1);
	
	void setAll(Particle p);
	

};


#endif