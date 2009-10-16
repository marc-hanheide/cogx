
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
	
	s = 1.0;
	
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
	return *this;
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
	printf("%f %f %f %f %f %f %f\n", rX, rY, rZ, tX, tY, tZ, w);
}

void Particle::setPose(mat3 rot, vec3 pos){
	// Set Pose doest not work correctly now
	q.fromMatrix(rot);
		
	tX = pos.x;
	tY = pos.y;
	tZ = pos.z;
	
	w = 0.0;
	
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

float Particles::noise(float rMin, float rMax, unsigned int distribution){
    float random = 0.0;
    
    if(rMax <= rMin){
    	float tmp = rMin;
    	rMin = rMax;
    	rMax = tmp;    	
    }
    
    // Gaussian noise
    if(distribution == GAUSS){
		for(int i=0; i<10; i++){
			random += float(rand())/RAND_MAX;
		}
		random = random / 10.0;
    }
    
    // Uniform distributed noise
    if(distribution == NORMAL){
    	random = float(rand())/RAND_MAX;
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
	w_normalizer = 1.0;
	m_frustum_offset = 0.0;
	
	m_particlelist = (Particle*)malloc(sizeof(Particle) * m_num_particles);
	setAll(p);
	
	queryMatches = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	queryEdges = (unsigned int*)malloc(sizeof(unsigned int) * m_num_particles);
	
	glGenQueriesARB(m_num_particles, queryMatches);
    glGenQueriesARB(m_num_particles, queryEdges);	
}

Particles::~Particles(){
	glDeleteQueriesARB(m_num_particles, queryMatches);
	glDeleteQueriesARB(m_num_particles, queryEdges);

	free(m_particlelist);
	free(queryMatches);
	free(queryEdges);
}

Particle* Particles::getMax(int num_particles){
	m_maxParticle = Particle(0.0);
	vec3 maxAxis = vec3(0.0, 0.0, 0.0);
	float maxAngle = 0.0;
	vec3 axis; float angle;
	w_sum = 0.0;
	int id;
	
	// Normalise particles
	for(id=0; id<num_particles; id++)
		w_sum += m_particlelist[id].w;
	
	for(id=0; id<num_particles && w_sum > 0.0; id++)
		m_particlelist[id].w = m_particlelist[id].w / w_sum;
	
	// Weighted sum over all particles
	for(id=0; id<num_particles; id++){
		m_maxParticle.tX += m_particlelist[id].tX * m_particlelist[id].w;
		m_maxParticle.tY += m_particlelist[id].tY * m_particlelist[id].w;
		m_maxParticle.tZ += m_particlelist[id].tZ * m_particlelist[id].w;
		
		m_particlelist[id].q.getAxisAngle(&axis, &angle);
		maxAxis += axis * m_particlelist[id].w;
		maxAngle += angle * m_particlelist[id].w;
	}
	m_maxParticle.q.fromAxis(maxAxis, maxAngle);
	return &m_maxParticle;
}

float Particles::getMaxW(int num_particles){
	float w=0.0;
	
	for(int id=0; id<num_particles; id++){
		if(m_particlelist[id].w > w)
			w = m_particlelist[id].w;
	}
	
	return w;
}

void Particles::resample(int num_particles, float noise_rot_max, float noise_trans_max, float noise_zoom_max){
	Particle pNoise;
	unsigned int distribution = GAUSS;
	int n, id, i, nid=0;
	float noiseRotX=0.0, noiseRotY=0.0, noiseRotZ=0.0;
    float noiseTransX=0.0, noiseTransY=0.0, noiseTransZ=0.0, noiseScale=0.0;
    
    for(id=0; id<num_particles && id<m_num_particles; id++){
		
		// TODO wrong: sum(n) != num_particles
		n = round(m_particlelist[id].w * num_particles);
		
		// Prediction model
		//float sigma = pow(1.0 - m_particlelist[id].w * w_sum, 3);
		float sigma = pow(1.0 - m_particlelist[id].w * w_sum, 3);
		
		// factor for standard deviation of noise
		// TODO try to avoid constants (MCMC)
		// mapping
		pNoise.rX = noise_rot_max * sigma;
		pNoise.rY = noise_rot_max * sigma;
		pNoise.rZ = noise_rot_max * sigma;
		pNoise.tX = noise_trans_max * sigma;
		pNoise.tY = noise_trans_max * sigma;
		pNoise.tZ = noise_trans_max * sigma;
		noiseScale = noise_zoom_max * sigma;
		
		for(i=0; i<n && nid<m_num_particles; i++){
			// Generate noise
			noiseRotX   = noise(-pNoise.rX, pNoise.rX, distribution);
			noiseRotY   = noise(-pNoise.rY, pNoise.rY, distribution);
			noiseRotZ   = noise(-pNoise.rZ, pNoise.rZ, distribution);
			noiseTransX = noise(-pNoise.tX, pNoise.tX, distribution);
			noiseTransY = noise(-pNoise.tY, pNoise.tY, distribution);
			noiseTransZ = noise(-pNoise.tZ, pNoise.tZ, distribution);
			noiseScale  = noise(-noiseScale,noiseScale, distribution);
			
			// Apply noise to particles
			m_particlelist[nid] = m_particlelist[id];
			
			m_particlelist[nid].rX = m_particlelist[nid].rX + noiseRotX;
			m_particlelist[nid].rY = m_particlelist[nid].rY + noiseRotY;
			m_particlelist[nid].rZ = m_particlelist[nid].rZ + noiseRotZ;
			m_particlelist[nid].tX = m_particlelist[nid].tX + noiseTransX;
			m_particlelist[nid].tY = m_particlelist[nid].tY + noiseTransY;
			m_particlelist[nid].tZ = m_particlelist[nid].tZ + noiseTransZ;
			m_particlelist[nid].s  = m_particlelist[nid].s  + noiseScale;
			
			m_particlelist[nid].rotate(noiseRotX, noiseRotY, noiseRotZ);
			m_particlelist[nid].translate( m_cam_view.x * noiseScale, m_cam_view.y * noiseScale, m_cam_view.z * noiseScale);
			
			// index of new particle list
			nid++;
		}
	}
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
        noiseRotX   = noise(-noise_particle.rX, noise_particle.rX, distribution);
        noiseRotY   = noise(-noise_particle.rY, noise_particle.rY, distribution);
        noiseRotZ   = noise(-noise_particle.rZ, noise_particle.rZ, distribution);
        noiseTransX = noise(-noise_particle.tX, noise_particle.tX, distribution);
        noiseTransY = noise(-noise_particle.tY, noise_particle.tY, distribution);
        noiseTransZ = noise(-noise_particle.tZ, noise_particle.tZ, distribution);
                
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
	w_sum = 0.0;
	float w_max = 0.0;
	
	if(m_num_particles<num_particles){
		printf("[Particles::calcLikelihood] Warning to less storage for 'num_particles'. Please increase number of particles\n");
		num_particles = m_num_particles;	
	}
	
	for(id=0; id<num_particles; id++){
		// Get number of pixels from Occlusion Query
		glGetQueryObjectivARB(queryEdges[id], GL_QUERY_RESULT_ARB, &v);
		glGetQueryObjectivARB(queryMatches[id], GL_QUERY_RESULT_ARB, &d);
		
		if(v>v_max_tmp)
			v_max_tmp = v;
		
		// get maximum visible pixles of edge representation
		if(v>v_max)
			v_max = v;
		
		// Likelihood calculation formula
		if(v != 0){
			//m_particlelist[id].w = pow((float(d)/float(v) + float(d)/float(v_max)) / w_max , 5);
			//m_particlelist[id].w = pow((float(d)/float(v) + float(d)/float(v_max)) / w_normalizer , 5);
			m_particlelist[id].w = pow((float(d)/float(v)),5);
		}else
			m_particlelist[id].w = 0.0;
			
		// sum of likelihood over all particles
		w_sum += m_particlelist[id].w;
		
		// w_normalizer normalizes likelihood to range [0 ... 1]
		if(m_particlelist[id].w>1.0)
			w_normalizer = m_particlelist[id].w * w_normalizer;
	}
	
	//if(w_max<1.0)
	//	w_normalizer = w_max * w_normalizer;
	
	// Adjust v_max to visible area of object
	if(v_max_tmp < v_max)
		v_max -= (v_max - v_max_tmp) / 100;
	
	// normalize weights
	for(id=0; id<num_particles && w_sum > 0.0; id++){
		m_particlelist[id].w = m_particlelist[id].w / w_sum;
	}

	// sort particles by likelihood and average most likely particles
	std::sort(m_particlelist, m_particlelist+num_particles);
	id_max = num_particles-1;
}

// Evaluate Confidence Level by using variance of weight w
float Particles::getVariance(int num_particles){
	int id=0;
	float mean = 0.0;
	float var = 0.0;
	
	// Evaluate mean
	for(id=0; id<num_particles; id++)
		mean += m_particlelist[id].w;
	mean = mean / num_particles;
	
	// Evaluate standard variation
	for(id=0; id<num_particles; id++)
		var += pow(m_particlelist[id].w - mean, 2);
	
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
		var.rX += pow((m_particlelist[id].rX - mean->rX) * m_particlelist[id].w, 2);
		var.rY += pow((m_particlelist[id].rY - mean->rY) * m_particlelist[id].w, 2);
		var.rZ += pow((m_particlelist[id].rZ - mean->rZ) * m_particlelist[id].w, 2);
		var.tX += pow((m_particlelist[id].tX - mean->tX) * m_particlelist[id].w, 2);
		var.tY += pow((m_particlelist[id].tY - mean->tY) * m_particlelist[id].w, 2);
		var.tZ += pow((m_particlelist[id].tZ - mean->tZ) * m_particlelist[id].w, 2);
	}
	
	return (var.rX+var.rY+var.rZ+var.tX+var.tY+var.tZ);
}
*/

void Particles::setAll(Particle p){
	for(int i=0; i<m_num_particles; i++)
		m_particlelist[i] = p;
}


