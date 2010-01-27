
#include "myPredictor.h"

using namespace Tracking;


void myPredictor::addsamples(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
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
	}	
}