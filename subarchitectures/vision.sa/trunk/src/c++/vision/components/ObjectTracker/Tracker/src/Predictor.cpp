
#include "Predictor.h"

using namespace Tracking;

// *** private ***
float Predictor::noise(float sigma, unsigned int type){
    float random = 0.0;
        
    // Gaussian noise
    if(type == GAUSS){
		for(int i=0; i<10; i++){
			random += float(rand())/RAND_MAX;
		}
		random = 2.0 * (random / 10.0 - 0.5);
    }
    
    // Uniform distributed noise
    if(type == NORMAL){
    	random = 2.0 * (float(rand())/RAND_MAX - 0.5);
    }
    
    // Adjusting to range
    random = random * sigma;
    
    return random;
}

Particle Predictor::genNoise(float sigma, Particle pConstraint, unsigned int type){
	if(sigma == 0.0f)
		printf("[Predictor::genNoise] Warning standard deviation sigma is 0.0\n");
	
	Particle epsilon;
	
	epsilon.t.x = noise(sigma, type) * pConstraint.t.x;
	epsilon.t.y = noise(sigma, type) * pConstraint.t.y;
	epsilon.t.z = noise(sigma, type) * pConstraint.t.z;
	epsilon.tp.x = noise(sigma, type) * pConstraint.tp.x;
	epsilon.tp.y = noise(sigma, type) * pConstraint.tp.y;
	epsilon.tp.z = noise(sigma, type) * pConstraint.tp.z;
	
	epsilon.r.x = noise(sigma, type) * pConstraint.r.x;
	epsilon.r.y = noise(sigma, type) * pConstraint.r.y;
	epsilon.r.z = noise(sigma, type) * pConstraint.r.z;
	epsilon.rp.x = noise(sigma, type) * pConstraint.rp.x;
	epsilon.rp.y = noise(sigma, type) * pConstraint.rp.y;
	epsilon.rp.z = noise(sigma, type) * pConstraint.rp.z;
	
	epsilon.z   = noise(sigma, type) * pConstraint.z;
	epsilon.zp  = noise(sigma, type) * pConstraint.zp;
	
	return epsilon;
}

void Predictor::addsamples(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
	for(int i=0; i<num_particles; i++){
		
		p = mean;
		
		// Distribution function (gaussian noise)
		epsilon = genNoise(sigma, variance);
		
		// TODO Prediction model, Better motion model (motion estimator, physical correct)
		
		p.tp = p.tp + epsilon.tp;
		p.rp = p.rp + epsilon.rp;
		p.zp = p.zp + epsilon.zp;
		epsilon.z = epsilon.z + epsilon.zp*m_dTime;
		p.translate(epsilon.t + p.tp*m_dTime);
		p.rotate(epsilon.r + p.rp*m_dTime);
		p.translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);
		
		d.push_back(p);
		
// 		printf("m_dTime: %f\n", m_dTime);
// 		usleep(500000);
	}	
}

// *** public ***
Predictor::Predictor(){
	m_dTime = 0.0;
}

void Predictor::resample(Distribution& d, int num_particles, Particle variance){
	
	if(d.size() <= 0)
		return;
	
	Particle epsilon, p;
	int n, id, i, nid=0;
	float sigma = 0.01;
	float c=0.0;
	
	float partition = 0.9;
	
	if(num_particles<=0){
		printf("[Distribution::resample] Warning number of particles to low (0)\n");
		num_particles = d.size();
	}
	
	ParticleList particlelist_tmp;
	d.copy(particlelist_tmp);
	d.clear();
	
	// Particles with motion
	for(id=0; id<particlelist_tmp.size() && d.size()<(int)num_particles*partition; id++){
		
		// resampling according to weight
		n = round(particlelist_tmp[id].w * num_particles);
		c = particlelist_tmp[id].c;
		
		// Tukey estimator
		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
		// ensure range of sigma
		if(sigma==0.0) sigma = 0.001;
		if(sigma>1.0) sigma = 1.0;
		
		addsamples(d, n, particlelist_tmp[id], variance, sigma);
	}
	
	// Particles voting for no motion
	int s=d.size();
	for(id=0; id<(num_particles-s); id++){
		d.copyParticle(p, id);
		p.tp = vec3(0.0,0.0,0.0);
		p.rp = vec3(0.0,0.0,0.0);
		p.zp = 0.0;
		d.push_back(p);
	}
}

void Predictor::sample(Distribution& d, int num_particles, Particle mean, Particle variance){
	updateTime(0.0);
	d.clear(); 
	addsamples(d, num_particles, mean, variance);
}

void Predictor::updateTime(double dTime){
	m_dTime = dTime;
}

