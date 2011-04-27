
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
	
	epsilon.r.x = noise(sigma, type) * pConstraint.r.x;
	epsilon.r.y = noise(sigma, type) * pConstraint.r.y;
	epsilon.r.z = noise(sigma, type) * pConstraint.r.z;
	epsilon.t.x = noise(sigma, type) * pConstraint.t.x;
	epsilon.t.y = noise(sigma, type) * pConstraint.t.y;
	epsilon.t.z = noise(sigma, type) * pConstraint.t.z;
	epsilon.tp.x = noise(sigma, type) * pConstraint.tp.x;
	epsilon.tp.y = noise(sigma, type) * pConstraint.tp.y;
	epsilon.tp.z = noise(sigma, type) * pConstraint.tp.z;
	
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
		
// 		if(m_dTime>0.0){
// 			// TODO Prediction model, Better motion model (motion estimator, physical correct)
// 			// WARNING motion model not possible while recursive tracking
// 			p.tp = p.tp + epsilon.tp;
// 			p.rp = p.rp + epsilon.rp;
// 			p.zp = p.zp + epsilon.zp;
// 				
// 			epsilon.z = epsilon.z + epsilon.zp*m_dTime;
// 			
// 			p.translate(epsilon.t + p.tp*m_dTime);
// 			p.rotate(epsilon.r + p.rp*m_dTime);
// 			p.translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);
// 		
// 		}else{
		
			p.translate(epsilon.t);
			p.rotateAxis(epsilon.r);
			p.translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);
			
// 		}
		
		d.push_back(p);
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
	double sigma = 0.01;
	double c=0.0;
	double cn=0.0;
	
	float pNormal = 0.7;
	float pNoMotion = 0.1;
	float pNoConvergence = 0.2;
	
	if(num_particles<=0){
		printf("[Distribution::resample] Warning number of particles to low (0)\n");
		num_particles = d.size();
	}
	
	ParticleList particlelist_tmp;
	d.copy(particlelist_tmp);
	d.clear();
	
	// Particles with motion
	for(id=0; id<particlelist_tmp.size() && d.size()<(int)num_particles*pNormal; id++){
		
		// resampling according to weight
		n = round(particlelist_tmp[id].w * num_particles);
		c = particlelist_tmp[id].c;
		
		// TODO evaluate if this makes sense (higher accuracy/convergence)
		// Remark: Defenitely removes jittering, maybe makes it less robust against fast movements
		if(d.getMaxC()>0.0){
			cn = c / d.getMaxC();
			c = c*(1.0-c) + cn*c;
// 			c = 0.8*cn + c*(1.0-cn);
		}
		
		// Tukey estimator
		// TODO evaluate optimal power of estimator (accuracy / robustness)
// 		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
		sigma = 1.0 - c;
		
		// ensure range of sigma
		if(sigma==0.0) sigma = 0.001;
		if(sigma>1.0) sigma = 1.0;
		
		addsamples(d, n, particlelist_tmp[id], variance, sigma);
	}
	
	// Particles voting for no motion
	for(id=0; id<((int)num_particles*pNoMotion); id++){
		d.copyParticle(p, id);
		p.tp = vec3(0.0,0.0,0.0);
		p.rp = vec3(0.0,0.0,0.0);
		p.zp = 0.0;
		d.push_back(p);
	}
	
	// Particle voting for no convergence
	n = num_particles - d.size();
	p = particlelist_tmp[0];
	addsamples(d, n, p, variance, 1.0);
}

void Predictor::sample(Distribution& d, int num_particles, Particle mean, Particle variance){
	d.clear(); 
	addsamples(d, num_particles, mean, variance);
}

void Predictor::updateTime(double dTime){
	m_dTime = dTime;
}
