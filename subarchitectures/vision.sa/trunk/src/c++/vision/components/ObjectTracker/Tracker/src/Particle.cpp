
#include "Particle.h"

using namespace Tracking;

Particle::Particle(){
	Particle(0.0);
}

Particle::Particle(float val){
	t.x = val;
	t.y = val;
	t.z = val;
	rp.x = val;
	rp.y = val;
	rp.z = val;
	tp.x = val;
	tp.y = val;
	tp.z = val;
	
	z = val;
	zp = val;	
	
	w = val;
	c = val;
	
	q = Quaternion();
}

Particle::Particle(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	rp.x = p2.rp.x;
	rp.y = p2.rp.y;
	rp.z = p2.rp.z;
	tp.x = p2.tp.x;
	tp.y = p2.tp.y;
	tp.z = p2.tp.z;
	z = p2.z;
	zp = p2.zp;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
}

Particle::Particle(const Pose& p2){
	t = p2.t;
	rp.x = 0.0;
	rp.y = 0.0;
	rp.z = 0.0;
	tp.x = 0.0;
	tp.y = 0.0;
	tp.z = 0.0;
	
	z = 0.0;
	zp = 0.0;	
	
	w = 0.0;
	c = 0.0;
	
	q = p2.q;
}

Particle& Particle::operator=(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	rp.x = p2.rp.x;
	rp.y = p2.rp.y;
	rp.z = p2.rp.z;
	tp.x = p2.tp.x;
	tp.y = p2.tp.y;
	tp.z = p2.tp.z;
	z = p2.z;
	zp = p2.zp;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
	
	return *this;
}

Particle& Particle::operator=(const Pose& p2){
	t = p2.t;
	rp.x = 0.0;
	rp.y = 0.0;
	rp.z = 0.0;
	tp.x = 0.0;
	tp.y = 0.0;
	tp.z = 0.0;
	
	z = 0.0;
	zp = 0.0;	
	
	w = 0.0;
	c = 0.0;
	
	q = p2.q;
}

