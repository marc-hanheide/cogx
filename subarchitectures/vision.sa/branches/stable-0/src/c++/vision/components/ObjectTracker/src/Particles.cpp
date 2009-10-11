
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

void Particle::activate(){
	glPushMatrix();
		glTranslatef(tX, tY, tZ);
		glMultMatrixf(q.getMatrix4());
		
}

void Particle::deactivate(){
	glPopMatrix();
}

void Particle::print(){
	printf("r: %f %f %f\n", rX, rY, rZ);
	printf("t: %f %f %f\n", tX, tY, tZ);
	printf("w: %f\n", w);
}

void Particle::setPose(mat3 rot, vec3 pos){
	// Set Pose doest not work correctly now
	q.fromMatrix(rot);
		
	tX = pos.x;
	tY = pos.y;
	tZ = pos.z;
	
	w = 1.0;
	
	//printf("[Particle::setPose] Warning conversion from rotation matrix to quaternion not correct!\n");
}

void Particle::getPose(mat3 &rot, vec3 &pos){
	
	rot = q.getMatrix3();
	
	pos[0] = tX;
	pos[1] = tY;
	pos[2] = tZ;
}

void Particle::rotate(float x, float y, float z){
	Quaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	//q.normalise();
}

void Particle::translate(float x, float y, float z){
	tX = tX + x;
	tY = tY + y;
	tZ = tZ + z;
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
	
	queryMatches = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	queryEdges = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	
	glGenQueriesARB(m_num_particles, queryMatches);
    glGenQueriesARB(m_num_particles, queryEdges);

	setAll(p);
}

Particles::~Particles(){
	glDeleteQueriesARB(m_num_particles, queryMatches);
	glDeleteQueriesARB(m_num_particles, queryEdges);

	free(m_particlelist);
	free(queryMatches);
	free(queryEdges);
}

void Particles::perturb(Particle noise_particle, int num_particles, Particle* p_ref, unsigned int distribution){
	Particle* pMax;
	Particle* pIt;
	Quaternion q2;
	
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
        
        pIt->q = pMax->q;
		pIt->rotate(noiseRotX, noiseRotY, noiseRotZ);
		
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

void Particles::calcLikelihood(int num_particles, unsigned int num_avaraged_particles){
	int v, d;
	int id;
	id_max = 0;
	int v_max_tmp = 0;
	
	if(m_num_particles<num_particles){
		printf("[Particles::calcLikelihood] Warning to less storage for 'num_particles'. Please increase number of particles\n");
		num_particles = m_num_particles;	
	}
	
	for(id=0; id<num_particles; id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[id], GL_QUERY_RESULT_ARB, &d);
		
		m_particlelist[id].v = v;
		m_particlelist[id].d = d;
		
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// get maximum visible pixles of edge representation
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		if(v != 0)
			m_particlelist[id].w = (float(d)/float(v) + float(d)/float(v_max)) / w_max;
			//m_particlelist[id].w = (float(d)/float(v));			
		
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
		
		//p.rotate(p.rX, p.rY, p.rZ);
		
		Quaternion q2;
		q2.fromEuler(p.rX, p.rY, p.rZ);
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


