
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
	
	w = val;
}

Particle::Particle(float* mv){

	rX = atan2(-mv[9], mv[10]) * 180.0 / PI;
	rY = asin(mv[8]) * 180.0 / PI;
	rZ = atan2(-mv[4], mv[0]) * 180.0 / PI;

	tX = mv[12];
	tY = mv[13];
	tZ = mv[14];
	
	w = 0.0;
}

Particle::Particle(mat3 rot, vec3 pos){

	rX = atan2(-rot[7], rot[8]) * 180.0 / PI;
	rY = asin(rot[6]) * 180.0 / PI;
	rZ = atan2(-rot[3], rot[0]) * 180.0 / PI;

	tX = pos.x;
	tY = pos.y;
	tZ = pos.z;
	
	w = 0.0;
}

Particle::Particle(const Particle& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	w = p2.w;
}

Particle& Particle::operator=(const Particle& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	w = p2.w;
}

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

void Particle::activate(){
	glPushMatrix();
		glTranslatef(tX, tY, tZ);
		glRotatef(rX, 1.0, 0.0, 0.0);
		glRotatef(rY, 0.0, 1.0, 0.0);
		glRotatef(rZ, 0.0, 0.0, 1.0);
}

void Particle::deactivate(){
	glPopMatrix();
}

void Particle::print(){
	printf("r: %f %f %f\n", rX, rY, rZ);
	printf("t: %f %f %f\n", tX, tY, tZ);
	printf("w: %f\n", w);
}

void Particle::getModelView(float* matrix4x4){
	mat3 Rx, Ry, Rz, R;
	float d2r = PI/180;
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glLoadIdentity();
		
		glTranslatef(tX, tY, tZ);
		glRotatef(rX, 1.0, 0.0, 0.0);
		glRotatef(rY, 0.0, 1.0, 0.0);
		glRotatef(rZ, 0.0, 0.0, 1.0);
		
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix4x4);
		/*
		printf("Modelview(rZ)\n");
		printf("%f %f %f %f\n", matrix4x4[0], matrix4x4[1], matrix4x4[2], matrix4x4[3]);
		printf("%f %f %f %f\n", matrix4x4[4], matrix4x4[5], matrix4x4[6], matrix4x4[7]);
		printf("%f %f %f %f\n", matrix4x4[8], matrix4x4[9], matrix4x4[10], matrix4x4[11]);
		printf("%f %f %f %f\n", matrix4x4[12], matrix4x4[13], matrix4x4[14], matrix4x4[15]);
		printf("Rz:\n");
		printf("%f %f %f\n", R[0], R[1], R[2]);
		printf("%f %f %f\n", R[3], R[4], R[5]);
		printf("%f %f %f\n", R[6], R[7], R[8]);
		*/
	glPushMatrix();
}

void Particle::getPose(float* matrix3x3, float* pos3){
	mat3 Rx, Ry, Rz, R;
	vec3 t;
	float d2r = PI/180;

	Rx[0] = 1; 			Rx[1] = 0; 				Rx[2] = 0; 
	Rx[3] = 0; 			Rx[4] = cos(rX*d2r); 	Rx[5] = sin(rX*d2r); 
	Rx[6] = 0; 			Rx[7] = -sin(rX*d2r); 	Rx[8] = cos(rX*d2r); 
	
	Ry[0] = cos(rY*d2r);	Ry[1] = 0;			Ry[2] = -sin(rY*d2r);
	Ry[3] = 0;				Ry[4] = 1;			Ry[5] = 0;
	Ry[6] = sin(rY*d2r);	Ry[7] = 0;			Ry[8] = cos(rY*d2r);
	
	Rz[0] = cos(rZ*d2r);	Rz[1] = sin(rZ*d2r);	Rz[2] = 0;
	Rz[3] = -sin(rZ*d2r);	Rz[4] = cos(rZ*d2r);	Rz[5] = 0;
	Rz[6] = 0;				Rz[7] = 0;				Rz[8] = 1;
	
	R = Rx * Ry * Rz;
	
	matrix3x3[0] = R[0]; matrix3x3[1] = R[1]; matrix3x3[2] = R[2];
	matrix3x3[3] = R[3]; matrix3x3[4] = R[4]; matrix3x3[5] = R[5];
	matrix3x3[6] = R[6]; matrix3x3[7] = R[7]; matrix3x3[8] = R[8];
	
	pos3[0] = tX; pos3[1] = tY; pos3[2] = tZ;
}

// PARTICLES

// *** private ***

float Particles::noise(float rMin, float rMax, unsigned int precision, unsigned int distribution){
    float random = 0.0;
    
    if(rMax <= rMin)
        return 0.0f;
    
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
	
	m_particlelist = (Particle*)malloc(sizeof(Particle) * num);
	
	queryD = (unsigned int*)malloc(sizeof(unsigned int) * num);
	queryV = (unsigned int*)malloc(sizeof(unsigned int) * num);
	
	glGenQueriesARB(num, queryD);
    glGenQueriesARB(num, queryV);

	setAll(p);
}

Particles::~Particles(){
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
        pIt->rX = pMax->rX + noiseRotX;
        pIt->rY = pMax->rY + noiseRotY;
        pIt->rZ = pMax->rZ + noiseRotZ;
        
        pIt->tX = pMax->tX + noiseTransX;
        pIt->tY = pMax->tY + noiseTransY;
        pIt->tZ = pMax->tZ + noiseTransZ;
        
        // limit search to view area of camera (frustum culling)
        if(!g_Resources->GetFrustum()->SphereInFrustum( pIt->tX, pIt->tY, pIt->tZ, -m_frustum_offset))
        {
            pIt->tX = pIt->tX - noiseTransX;
            pIt->tY = pIt->tY - noiseTransY;
            pIt->tZ = pIt->tZ - noiseTransZ;
        }
    }
}

void Particles::activate(int id){
/*
	glPushMatrix();
		glTranslatef(m_particlelist[id].tX, m_particlelist[id].tY, m_particlelist[id].tZ);
		glRotatef(m_particlelist[id].rX, 1.0, 0.0, 0.0);
		glRotatef(m_particlelist[id].rY, 0.0, 1.0, 0.0);
		glRotatef(m_particlelist[id].rZ, 0.0, 0.0, 1.0);
*/
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
	unsigned int v, d;
	int id;
	id_max = 0;
	int v_max_tmp = 0;
	
	if(m_num_particles<num_particles)
		printf("[Particles::calcLikelihood] Warning to less storage for 'num_particles'. Please increase number of particles\n");
	
	for(id=0; (id<num_particles && id<m_num_particles); id++){
		// Get number of pixels from Occlusion Query
		glGetOcclusionQueryuivNV(queryV[id], GL_PIXEL_COUNT_NV, &v);
		glGetOcclusionQueryuivNV(queryD[id], GL_PIXEL_COUNT_NV, &d);
		
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
		
		m_particlelist[0] = p;
		id_max = 0;
	}
}

void Particles::setAll(Particle p){
	
	for(int i=0; i<m_num_particles; i++){
		m_particlelist[i] = p;		
	}

}


