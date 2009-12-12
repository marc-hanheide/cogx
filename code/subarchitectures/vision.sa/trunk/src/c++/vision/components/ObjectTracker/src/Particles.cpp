
#include "Particles.h"

Particle::Particle(){
	Particle(0.0);
}

Particle::Particle(float val){
	r.x = val;
	r.y = val;
	r.z = val;
	s.x = val;
	s.y = val;
	s.z = val;
	rp.x = val;
	rp.y = val;
	rp.z = val;
	sp.x = val;
	sp.y = val;
	sp.z = val;
	
	z = val;
	zp = val;	
	
	w = val;
	c = val;
	
	q = Quaternion();
}

Particle::Particle(const Particle& p2){
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;
	s.x = p2.s.x;
	s.y = p2.s.y;
	s.z = p2.s.z;
	rp.x = p2.rp.x;
	rp.y = p2.rp.y;
	rp.z = p2.rp.z;
	sp.x = p2.sp.x;
	sp.y = p2.sp.y;
	sp.z = p2.sp.z;
	z = p2.z;
	zp = p2.zp;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
}

Particle& Particle::operator=(const Particle& p2){
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;
	s.x = p2.s.x;
	s.y = p2.s.y;
	s.z = p2.s.z;
	rp.x = p2.rp.x;
	rp.y = p2.rp.y;
	rp.z = p2.rp.z;
	sp.x = p2.sp.x;
	sp.y = p2.sp.y;
	sp.z = p2.sp.z;
	z = p2.z;
	zp = p2.zp;
	
	w = p2.w;
	c = p2.c;
	
	q = p2.q;
	
	return *this;
}

void Particle::activate(){
	glPushMatrix();
		glTranslatef(s.x, s.y, s.z);
		glMultMatrixf(q.getMatrix4());
}

void Particle::deactivate(){
	glPopMatrix();
}

void Particle::print(){
	printf("%f %f %f %f %f %f %f\n", r.x, r.y, r.z, s.x, s.y, s.z, w);
}

void Particle::setPose(mat3 rot, vec3 pos){
	// Set Pose doest not work correctly now
	q.fromMatrix(rot);
		
	s.x = pos.x;
	s.y = pos.y;
	s.z = pos.z;
	
	w = 0.0;
	c = 0.0;
	
	//printf("[Particle::setPose] Warning conversion from rotation matrix to quaternion not correct!\n");
}

void Particle::getPose(mat3 &rot, vec3 &pos){
	
	rot = q.getMatrix3();
	
	pos[0] = s.x;
	pos[1] = s.y;
	pos[2] = s.z;
}

void Particle::rotate(float x, float y, float z){
	Quaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	//q.normalise();
}

void Particle::rotate(vec3 rot){
	Quaternion q2;
	
	q2.fromEuler(rot.x,rot.y,rot.z);
	q = q2 * q;
	//q.normalise();
}

void Particle::translate(float x, float y, float z){
	s.x = s.x + x;
	s.y = s.y + y;
	s.z = s.z + z;
}

void Particle::translate(vec3 trans){
	s.x = s.x + trans.x;
	s.y = s.y + trans.y;
	s.z = s.z + trans.z;
}

// PARTICLES

// *** private ***

float Particles::noise(float sigma, unsigned int distribution){
    float random = 0.0;
        
    // Gaussian noise
    if(distribution == GAUSS){
		for(int i=0; i<10; i++){
			random += float(rand())/RAND_MAX;
		}
		random = 2.0 * (random / 10.0 - 0.5);
    }
    
    // Uniform distributed noise
    if(distribution == NORMAL){
    	random = 2.0 * (float(rand())/RAND_MAX - 0.5);
    }
    
    // Adjusting to range
    random = random * sigma;
    
    return random;
}

Particle Particles::genNoise(float sigma, Particle pConstraint, unsigned int distribution){
	if(sigma == 0.0f)
		printf("[Particles::genNoise] Warning standard deviation sigma is 0.0\n");
	
	Particle epsilon;
	
	epsilon.s.x = noise(sigma, distribution) * pConstraint.s.x;
	epsilon.s.y = noise(sigma, distribution) * pConstraint.s.y;
	epsilon.s.z = noise(sigma, distribution) * pConstraint.s.z;
	epsilon.sp.x = noise(sigma, distribution) * pConstraint.sp.x;
	epsilon.sp.y = noise(sigma, distribution) * pConstraint.sp.y;
	epsilon.sp.z = noise(sigma, distribution) * pConstraint.sp.z;
	
	epsilon.r.x = noise(sigma, distribution) * pConstraint.r.x;
	epsilon.r.y = noise(sigma, distribution) * pConstraint.r.y;
	epsilon.r.z = noise(sigma, distribution) * pConstraint.r.z;
	epsilon.rp.x = noise(sigma, distribution) * pConstraint.rp.x;
	epsilon.rp.y = noise(sigma, distribution) * pConstraint.rp.y;
	epsilon.rp.z = noise(sigma, distribution) * pConstraint.rp.z;
	
	epsilon.z   = noise(sigma, distribution) * pConstraint.z;
	epsilon.zp  = noise(sigma, distribution) * pConstraint.zp;
	
	return epsilon;
}

void Particles::resizeQueries(int new_size){
	glDeleteQueriesARB(queryMatches.size(), &queryMatches[0]);
	glDeleteQueriesARB(queryEdges.size(), &queryEdges[0]);
	
	queryMatches.resize(new_size, 0);
	queryEdges.resize(new_size, 0);
	
	glGenQueriesARB(new_size, &queryMatches[0]);
	glGenQueriesARB(new_size, &queryEdges[0]);
}

// *** public ***
Particles::Particles(int num_particles, Particle p){
	v_max = 1.0;
	w_sum = 0.0;
	
	m_particlelist.assign(num_particles, p);
	
	queryMatches.assign(num_particles,0);
	queryEdges.assign(num_particles,0);
	
	glGenQueriesARB(num_particles, &queryMatches[0]);
	glGenQueriesARB(num_particles, &queryEdges[0]);	
}

Particles::~Particles(){
	glDeleteQueriesARB(m_particlelist.size(), &queryMatches[0]);
	glDeleteQueriesARB(m_particlelist.size(), &queryEdges[0]);
}

Particle* Particles::calcMax(){
	m_maxParticle = Particle(0.0);
	vec3 maxAxis = vec3(0.0, 0.0, 0.0);
	double maxAngle = 0.0;
	vec3 axis; double angle;
	w_sum = 0.0;
	int id;
	
	// Normalise particles
	for(id=0; id<m_particlelist.size(); id++)
		w_sum += m_particlelist[id].w;
	
	for(id=0; id<m_particlelist.size() && w_sum > 0.0; id++)
		m_particlelist[id].w = m_particlelist[id].w / w_sum;
	
	// Weighted sum over all particles
	for(id=0; id<m_particlelist.size(); id++){
		m_maxParticle.s += m_particlelist[id].s * m_particlelist[id].w;
		m_maxParticle.sp += m_particlelist[id].sp * m_particlelist[id].w;
		m_maxParticle.rp += m_particlelist[id].rp * m_particlelist[id].w;
		
		m_particlelist[id].q.getAxisAngle(&axis, &angle);
		maxAxis += axis * m_particlelist[id].w;
		maxAngle += angle * m_particlelist[id].w;
	}
	m_maxParticle.q.fromAxis(maxAxis, maxAngle);
	
	m_maxParticle.c = c_max;
	m_maxParticle.w = w_max;
	
	return &m_maxParticle;
}

void Particles::addsamples(int num_particles, Particle p_initial, Particle p_constraints, float sigma){
	Particle p;
	Particle epsilon;
	
	for(int i=0; i<num_particles; i++){
		
		p = p_initial;
		
		// Distribution function (gaussian noise)
		epsilon = genNoise(sigma, p_constraints);
		
		// TODO Prediction model, Better motion model (motion estimator, physical correct)
		
		p.sp = p.sp + epsilon.sp;
		p.rp = p.rp + epsilon.rp;
		p.zp = p.zp + epsilon.zp;
		epsilon.z = epsilon.z + epsilon.zp*m_fTime;
		p.translate(epsilon.s + p.sp*m_fTime);
		p.rotate(epsilon.r + p.rp*m_fTime);
		p.translate( m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);
		
		m_particlelist.push_back(p);
	}	
}

void Particles::sample(int num_particles, Particle p_initial, Particle p_constraints){
	m_particlelist.clear(); 
	addsamples(num_particles, p_initial, p_constraints);
	resizeQueries(num_particles);
}

void Particles::resample(int num_particles, Particle p_constraints){
	Particle epsilon, p;
	int n, id, i, nid=0;
	float sigma = 0.01;
	float c=0.0;
	
	float partition = 0.9;
	
	m_fTime = m_timer.Update();
	
	if(num_particles<=0){
		printf("[Particles::resample] Warning number of particles to low (0)\n");
		num_particles = m_particlelist.size();
	}
	
	vector<Particle> particlelist_tmp;
	particlelist_tmp = m_particlelist;
	m_particlelist.clear();
	
	// Particles with motion
	for(id=0; id<particlelist_tmp.size() && m_particlelist.size()<(int)num_particles*partition; id++){
		
		// TODO wrong: sum(n) != num_particles
		n = round(particlelist_tmp[id].w * num_particles);
		c = m_particlelist[id].c;
		
		// Tukey estimator
		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
		// ensure range of sigma
		if(sigma==0.0) sigma = 0.001;
		if(sigma>1.0) sigma = 1.0;
		
		addsamples(n, particlelist_tmp[id], p_constraints, sigma);
	}
	
	// Particles voting for no motion
	int size=m_particlelist.size();
	for(id=0; id<(num_particles-size); id++){
		p = m_particlelist[id];
		p.sp = vec3(0.0,0.0,0.0);
		p.rp = vec3(0.0,0.0,0.0);
		p.zp = 0.0;
		m_particlelist.push_back(p);
	}
		
	resizeQueries(m_particlelist.size());
}

void Particles::activate(int id){
	m_particlelist[id].activate();
}

void Particles::deactivate(int id){
	m_particlelist[id].deactivate();
}

void Particles::startCountD(int id){
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches[id]);
}

void Particles::startCountV(int id){
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges[id]);
}

void Particles::endCountD(){
	glEndQueryARB(GL_SAMPLES_PASSED_ARB);
}

void Particles::endCountV(){
	glEndQueryARB(GL_SAMPLES_PASSED_ARB);
}

bool sortfunction(Particle p1, Particle p2){ return (p1.w>p2.w); }

void Particles::calcLikelihood(int power){
	int v, d, id;
	int v_max_tmp = 0;
	w_sum = 0.0;
	c_max = 0.0;
	w_max = 0.0;
		
	for(id=0; id<m_particlelist.size(); id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[id], GL_QUERY_RESULT_ARB, &d);
		
		// Get maximum edge pixels (view dependent)
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// avoids  (d/v_max) > 1.0
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		if(v != 0){
			m_particlelist[id].c = (float(d)/float(v) + float(d)/float(v_max)) * 0.5;
			m_particlelist[id].w = pow(m_particlelist[id].c, power*(1.0-m_particlelist[id].c));
		}else{
			m_particlelist[id].c = 0.0;
			m_particlelist[id].w = 0.0;
		}
		
		if(m_particlelist[id].c > c_max)
			c_max = m_particlelist[id].c;
		
		// sum of likelihood over all particles
		w_sum += m_particlelist[id].w;
	}	
	
	// Update v_max (in case of decreasing v_max_tmp)
	v_max = v_max_tmp;
	
	// normalize weights
	float dw_sum = 0.0;
	if(w_sum>0.0){
		dw_sum = 1.0/w_sum;
		for(id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = m_particlelist[id].w * dw_sum;
			if(m_particlelist[id].w > w_max)
				w_max = m_particlelist[id].w;
		}
	}else{
		dw_sum = 1.0/m_particlelist.size();
		for(id=0; id<m_particlelist.size(); id++){
			m_particlelist[id].w = dw_sum;
		}
		w_max = dw_sum;
	}
	
	// calculate maximum particle (=result)
	calcMax();

	// sort particles by likelihood and average most likely particles
	std::sort(m_particlelist.begin(), m_particlelist.end(), sortfunction);
}

// Evaluate Confidence Level by using variance of weight w
float Particles::getVariance(){
	int id=0;
	float mean = 0.0;
	float var = 0.0;
	
	// Evaluate mean
	for(id=0; id<m_particlelist.size(); id++)
		mean += m_particlelist[id].c;
	mean = mean / m_particlelist.size();
	
	// Evaluate standard variation
	for(id=0; id<m_particlelist.size(); id++)
		var += pow(m_particlelist[id].c - mean, 2);
	
	return var;
}

/* Evaluate Confidence Level by using variance of particles, weighted by w
float Particles::getVariance(int num_particles){
	int id=0;
	
	// Evaluate mean
	Particle* mean = getMax(num_particles);
	
	
	// Evaluate standard variation
	Particle var;
	for(id=0; id<num_particles; id++){
		var.r.x += pow((m_particlelist[id].r.x - mean->r.x) * m_particlelist[id].w, 2);
		var.r.y += pow((m_particlelist[id].r.y - mean->r.y) * m_particlelist[id].w, 2);
		var.r.z += pow((m_particlelist[id].r.z - mean->r.z) * m_particlelist[id].w, 2);
		var.s.x += pow((m_particlelist[id].s.x - mean->s.x) * m_particlelist[id].w, 2);
		var.s.y += pow((m_particlelist[id].s.y - mean->s.y) * m_particlelist[id].w, 2);
		var.s.z += pow((m_particlelist[id].s.z - mean->s.z) * m_particlelist[id].w, 2);
	}
	
	return (var.r.x+var.r.y+var.r.z+var.s.x+var.s.y+var.s.z);
}
*/

void Particles::setAll(Particle p){
	for(int i=0; i<m_particlelist.size(); i++)
		m_particlelist[i] = p;
}

