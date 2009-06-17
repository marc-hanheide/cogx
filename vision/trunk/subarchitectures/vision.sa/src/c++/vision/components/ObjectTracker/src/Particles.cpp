
#include "Particles.h"

bool compare_particles(Particle first, Particle second){

	return true;
}

Particle::Particle(){
	Particle(0.0);
}

Particle::Particle(float val){
	rX = val;
	rY = val;
	rZ = val;
	
	tX = val;
	tY = val;
	tZ = val;
	
	q = Quaternion();
	
	w = val;
}

Particle::Particle(mat3 rot, vec3 pos){

	rX = atan2(-rot[7], rot[8]) * 180.0 / PI;
	rY = asin(rot[6]) * 180.0 / PI;
	rZ = atan2(-rot[3], rot[0]) * 180.0 / PI;

	tX = pos.x;
	tY = pos.y;
	tZ = pos.z;
	
	q.FromEuler(rX, rY, rZ);
	
	w = 0.0;
}

Particle::Particle(const Particle& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	q = p2.q;
	
	w = p2.w;
}

Particle& Particle::operator=(const Particle& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	q = p2.q;
	
	w = p2.w;
}

/* Particle::operator==(const Particle& p2)
bool Particle::operator==(const Particle& p2){
	float fTol = 0.01;
	if( (rX - p2.rX) > fTol ||
		(rY - p2.rY) > fTol ||
		(rZ - p2.rZ) > fTol ||
		(rX - p2.tX) > fTol ||
		(rY - p2.tY) > fTol ||
		(rZ - p2.tZ) > fTol ||
		(w - p2.w) > fTol 		){
		return false;
	}
	
	return true;		
}
*/

void Particle::activate(){
	glPushMatrix();
		/*
		glRotatef(rX, 1.0, 0.0, 0.0);
		glRotatef(rY, 0.0, 1.0, 0.0);
		glRotatef(rZ, 0.0, 0.0, 1.0);
		*/
		glMultMatrixf(q.getMatrix());
		glTranslatef(tX, tY, tZ);
}

void Particle::deactivate(){
	glPopMatrix();
}

void Particle::print(){
	printf("r: %f %f %f\n", rX, rY, rZ);
	printf("t: %f %f %f\n", tX, tY, tZ);
	printf("w: %f\n", w);
}

void Particle::getPose(float* matrix3x3, float* pos3){
	
	mat4 m = q.getMatrix();
	matrix3x3[0]=m[0]; matrix3x3[1]=m[1]; matrix3x3[2]=m[2];
	matrix3x3[3]=m[4]; matrix3x3[4]=m[5]; matrix3x3[5]=m[6];
	matrix3x3[6]=m[8]; matrix3x3[7]=m[9]; matrix3x3[8]=m[10];
	
	pos3[0] = tX;
	pos3[1] = tY;
	pos3[2] = tZ;
	
}


// PARTICLES

// *** private ***

float Particles::noise(float rMin, float rMax, unsigned int precision, unsigned int distribution){
    float random = 0.0;
    
    if(rMax <= rMin){
    	float tmp = rMin;
    	rMin = rMax;
    	rMax = tmp;    	
    }
    
    // Gaussian noise
    if(distribution == GAUSS){
		for(int i=0; i<10; i++){
			random += float(rand() % precision)/precision;
		}
		random = random / 10.0;
    }
    
    // Uniform distributed noise
    if(distribution == NORMAL){
    	random = float(rand() % precision)/precision;
    }
    
    // Adjusting to range
    random = (random*(rMax-rMin) + rMin);
    
    return random;
}

// *** public ***
Particles::Particles(int num, Particle p){
	m_num_particles = num;
	id_max = 0;
	v_max = 0;
	d_max = 0;
	w_max = 1.0;
	m_frustum_offset = 0.0;
	
	m_particlelist = (Particle*)malloc(sizeof(Particle) * m_num_particles);
	
	queryD = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	queryV = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	
	glGenQueriesARB(m_num_particles, queryD);
    glGenQueriesARB(m_num_particles, queryV);

	setAll(p);
}

Particles::~Particles(){
	glDeleteQueriesARB(m_num_particles, queryD);
	glDeleteQueriesARB(m_num_particles, queryV);

	free(m_particlelist);
	free(queryD);
	free(queryV);
}

void Particles::perturb(Particle noise_particle, int num_particles, Particle* p_ref, unsigned int distribution){
	Particle* pMax;
	Particle* pIt;
	
	float noiseRotX=0.0, noiseRotY=0.0, noiseRotZ=0.0;
    float noiseTransX=0.0, noiseTransY=0.0, noiseTransZ=0.0;
        
    if(!p_ref)
    	pMax = &m_particlelist[id_max];
    else
    	pMax = p_ref;
    	
   	// keep pMax at id 0
    m_particlelist[0] = *pMax;
   
    // for all other particles add noise
    for(int i=1; i<num_particles; i++){
    	pIt = &m_particlelist[i];
        
        // Generate noise
        noiseRotX   = noise(-noise_particle.rX, noise_particle.rX, 100, distribution);
        noiseRotY   = noise(-noise_particle.rY, noise_particle.rY, 100, distribution);
        noiseRotZ   = noise(-noise_particle.rZ, noise_particle.rZ, 100, distribution);
        noiseTransX = noise(-noise_particle.tX, noise_particle.tX, 100, distribution);
        noiseTransY = noise(-noise_particle.tY, noise_particle.tY, 100, distribution);
        noiseTransZ = noise(-noise_particle.tZ, noise_particle.tZ, 100, distribution);
                
        // Apply noise to particles
        pIt->rX = noiseRotX;
        pIt->rY = noiseRotY;
        pIt->rZ = noiseRotZ;

        Quaternion q2;
        q2.FromEuler(noiseRotX, noiseRotY, noiseRotZ);
        pIt->q = q2 * pMax->q;
        //printf("%f %f %f %f\n", pMax->q.x, pMax->q.y, pMax->q.z, pMax->q.w);
        //printf("%f %f %f %f\n\n", q2.x, q2.y, q2.z, q2.w);
        //pIt->q.normalise();
        
       	pIt->tX = pMax->tX + noiseTransX;
        pIt->tY = pMax->tY + noiseTransY;
        pIt->tZ = pMax->tZ + noiseTransZ;
        
    }
}

void Particles::activate(int id){
	m_particlelist[id].activate();
}

void Particles::deactivate(int id){
	m_particlelist[id].deactivate();
}

void Particles::startCountD(int id){
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryD[id]);
}

void Particles::startCountV(int id){
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryV[id]);
}

void Particles::endCountD(){
	glEndQueryARB(GL_SAMPLES_PASSED_ARB);
}

void Particles::endCountV(){
	glEndQueryARB(GL_SAMPLES_PASSED_ARB);
}

void Particles::calcLikelihood(int num_particles, unsigned int num_avaraged_particles){
	int v, d;
	int id;
	id_max = 0;
	int v_max_tmp = 0;
	
	if(m_num_particles<num_particles)
		printf("[Particles::calcLikelihood] Warning to less storage for 'num_particles'. Please increase number of particles\n");
	
	for(id=0; (id<num_particles && id<m_num_particles); id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryV[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryD[id], GL_QUERY_RESULT_ARB, &d);
		
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// get maximum visible pixles of edge representation
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		if(v != 0)
			m_particlelist[id].w = (float(d)/float(v) + float(d)/float(v_max)) / w_max;
		
		// store id of maximum likely particle
		if(m_particlelist[id].w > m_particlelist[id_max].w)
			id_max = id;
			
		// w_max scales likelihood to range [0 ... 1]
		if(m_particlelist[id].w>1.0)
			w_max = m_particlelist[id].w * w_max;
	}
	
	// Adjust v_max to visible area of object
	if(v_max_tmp < v_max)
		v_max -= (v_max - v_max_tmp) / 100;
	
	// sort particles by likelihood and average most likely particles
	
	if(num_avaraged_particles > 1){
		std::sort(m_particlelist, m_particlelist+m_num_particles);
		std::reverse(m_particlelist, m_particlelist+m_num_particles);
		int mean_range = 5;
		float divider = 1 / float(mean_range);
		Particle p(0.0);
		for(id=0; id<mean_range; id++){
			p.rX += (m_particlelist[id].rX * m_particlelist[id].w);
			p.rY += (m_particlelist[id].rY * m_particlelist[id].w);
			p.rZ += (m_particlelist[id].rZ * m_particlelist[id].w);
			p.tX += (m_particlelist[id].tX * m_particlelist[id].w);
			p.tY += (m_particlelist[id].tY * m_particlelist[id].w);
			p.tZ += (m_particlelist[id].tZ * m_particlelist[id].w);
			p.w  += m_particlelist[id].w;
		}
		divider = 1.0 / p.w;
		p.rX = p.rX * divider;
		p.rY = p.rY * divider;
		p.rZ = p.rZ * divider;
		p.tX = p.tX * divider;
		p.tY = p.tY * divider;
		p.tZ = p.tZ * divider;
		p.w  = p.w / float(mean_range);
		
		Quaternion q2;
		q2.FromEuler(p.rX, p.rY, p.rZ);
		p.q = m_particlelist[0].q * q2;
		
		m_particlelist[0] = p;
		id_max = 0;
	}
}

void Particles::setAll(Particle p){
	
	for(int i=0; i<m_num_particles; i++){
		m_particlelist[i] = p;		
	}

}


