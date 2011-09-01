
#include "Predictor.h"
#include "Noise.h"

using namespace Tracking;
using namespace std;

Predictor::Predictor(){
	m_dTime = 0.0;
	c_pred = 0.0;
	m_noConvergence = 0.0f;
	m_keepParticles=0.0f;
}

// *** private ***
float Predictor::noise(float sigma, unsigned int type){
    float random = 0.0f;
        
    // Gaussian noise
    if(type == GAUSS){
		for(unsigned i=0; i<10; i++){
			random += float(rand())/RAND_MAX;
		}
		random = 2.0f * (random / 10.0f - 0.5f);
    }
    
    // Uniform distributed noise
    if(type == NORMAL){
    	random = 2.0f * (float(rand())/RAND_MAX - 0.5f);
    }
    
    // Adjusting to range
    random = random * sigma;
    
    return random;
}

Particle Predictor::genNoise(float sigma, Particle pConstraint, unsigned int type){
	if(sigma == 0.0f)
		printf("[Predictor::genNoise] Warning standard deviation sigma is 0.0\n");
	
	Particle epsilon;
	Noise N;

	epsilon.r.x = N.randn_notrig(0.0f, sigma * pConstraint.r.x);
	epsilon.r.y = N.randn_notrig(0.0f, sigma * pConstraint.r.y);
	epsilon.r.z = N.randn_notrig(0.0f, sigma * pConstraint.r.z);
	epsilon.t.x = N.randn_notrig(0.0f, sigma * pConstraint.t.x);
	epsilon.t.y = N.randn_notrig(0.0f, sigma * pConstraint.t.y);
	epsilon.t.z = N.randn_notrig(0.0f, sigma * pConstraint.t.z);
	
	epsilon.z = N.randn_notrig(0.0f, sigma * pConstraint.z);
		
	return epsilon;
}

void Predictor::sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
//	d.push_back(mean);
	for(int i=0; i<num_particles; i++){
		
		// particle to sample from
		p = mean;
		
		// move function (gaussian noise)
		epsilon = genNoise(sigma, variance);
		
		// apply movement
		p.Translate(epsilon.t);
		p.RotateAxis(epsilon.r);
		p.Translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);

		// add particle to distribution
		d.push_back(p);
	}	
}

void Predictor::movePredicted(const TomGine::tgPose& poseIn, TomGine::tgPose& poseOut, float &c){
	poseOut = poseIn;
	c = 0.0;
}

// *** public ***

void Predictor::resample(Distribution& d, int num_particles, Particle variance, const int &iter, const int &max_iter){
	
	if(d.size() <= 0){
		printf("[Predictor::resample] Warning distribution d is empty.\n");
		return;
	}
	
	unsigned n, id, nid=0;
	float sigma = 0.01f;
	float c=0.0f;
	float cn=0.0f;
	float c_pred_av = 0.0f;
	float w_sum=0.0f;
	
	if(num_particles<=0){
		printf("[Distribution::resample] Warning number of particles to low (0)\n");
		num_particles = d.size();
	}
	
	ParticleList pl;
	d.copy(pl);
	d.clear();

	if(iter==max_iter-1){
		d.m_numStaticParticles = 0;
		d.m_meanWeightOfStaticParticles = 0.0f;
	}

	std::vector<Particle> keptParticles;

	// Particles with motion
	for(	id=0; 
			id<pl.size() &&
			d.size()<(int)(num_particles*(1.0f - m_noConvergence)); 
			id++)
	{
		
		// resampling according to weight
		n = (unsigned)round(pl[id].w * num_particles);
		c = pl[id].c;
		
		// Keep best particles
		vector<Particle>::iterator it;
		if(id<ceil(float(pl.size())*m_keepParticles)){
			d.push_back(pl[id]);
			n--;
			// Evaluate how many particles remain static since last frame
			if(iter==max_iter-1){
				it = find(d.m_keptParticles.begin(), d.m_keptParticles.end(), pl[id]);
				if(it<d.m_keptParticles.end()){
					d.m_meanWeightOfStaticParticles += pl[id].w;
					d.m_numStaticParticles++;
				}
				keptParticles.push_back(pl[id]);
				w_sum += pl[id].w;
			}
		}
		
		// Standard deviation for normal distribution
		// TODO evaluate optimal power of estimator (accuracy / robustness)
// 		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
		sigma = 1.0f - c;
		
		// crop range of sigma
		if(sigma==0.0) sigma = 0.001f;
		if(sigma>1.0) sigma = 1.0f;

		// Sample particles from Gaussian distribution
		sampleFromGaussian(d, n, pl[id], variance, sigma);
	}

	//// Particle voting for no convergence
	if(m_noConvergence > 0.0f){
		n = num_particles - d.size();
		Particle p = pl[0];
		sampleFromGaussian(d, n, p, variance, 1.0f);
	}

	// Mean and weighted mean of kept particles
	if(iter==max_iter-1){
		if(keptParticles.empty())
			printf("[Predictor::resample] Warning number of particles to keep to low (KeepBestParticles in ini file).\n");
		d.m_keptParticles = keptParticles;
		d.m_meanWeightOfStaticParticles = d.m_meanWeightOfStaticParticles / w_sum;
		d.m_meanKeptParticle = d.calcWeightedMean(d.m_keptParticles);
	}
}

void Predictor::sample(Distribution& d, int num_particles, Particle mean, Particle variance){
	d.clear(); 
	sampleFromGaussian(d, num_particles, mean, variance);
}

void Predictor::updateTime(double dTime){
	m_dTime = dTime;
}
