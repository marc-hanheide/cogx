
#include "myPredictor.h"

using namespace Tracking;


void myPredictor::addsamples(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
	for(int i=0; i<num_particles; i++){
		
		p = mean;
		
		epsilon = genNoise(sigma, variance);
		
		p.tp = p.tp + epsilon.tp;
		p.rp = p.rp + epsilon.rp;
		p.zp = p.zp + epsilon.zp;
		p.z  = p.z + epsilon.zp*m_dTime;
		
		p.translate(epsilon.t + p.tp*m_dTime);
		p.rotateAxis(epsilon.r + p.rp*m_dTime);
		p.translate( m_cam_view.x * p.z, m_cam_view.y * p.z, m_cam_view.z * p.z);
		
		d.push_back(p);
		
		
	}	
}