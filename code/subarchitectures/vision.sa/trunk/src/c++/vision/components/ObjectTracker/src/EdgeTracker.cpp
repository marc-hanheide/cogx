
#include "EdgeTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	/*
	m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[1]);
	m_ip->spreading(m_tex_frame_ip[1], m_tex_frame_ip[2]);
	m_ip->spreading(m_tex_frame_ip[2], m_tex_frame_ip[2]);
	m_ip->spreading(m_tex_frame_ip[2], m_tex_frame_ip[3]);
	*/
}

// Calculate motion function (noise) and perturb particles
void EdgeTracker::particle_motion(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax(params.number_of_particles)->w;
	Particle noise_particle;
	
	// Standard deviation for gaussian noise
	noise_particle.rX = params.noise_rot_max * pow_scale * (1-w);
	noise_particle.rY = params.noise_rot_max * pow_scale * (1-w);
	noise_particle.rZ = params.noise_rot_max * pow_scale * (1-w);
	noise_particle.tX = params.noise_trans_max * pow_scale * (1-w);
	noise_particle.tY = params.noise_trans_max * pow_scale * (1-w);
	noise_particle.tZ = params.noise_trans_max * pow_scale * (1-w);
	
    if(!m_lock){
   		m_particles->perturb(noise_particle, params.number_of_particles, p_ref, distribution);
   	}
}

void EdgeTracker::particle_filtering(Particle& p_result, int recursions){
	
	// Calculate Zoom Direction and pass it to the particle filter
	TM_Vector3 vCam = m_cam_perspective->GetPos();
	TM_Vector3 vObj = TM_Vector3(p_result.tX, p_result.tY, p_result.tZ);
	TM_Vector3 vCamObj = vObj - vCam;
	vCamObj.normalize();
	m_particles->setCamViewVector(vCamObj);
	m_particles->setCamS( m_cam_perspective->GetS() );
	m_particles->setCamU( m_cam_perspective->GetU() );
	
	
	glLineWidth(2);
	glColor3f(0.0,0.0,0.0);
	m_tex_frame_ip[0]->bind();	// bind camera image
	float ns = 1.0;
	
	for(int i=0; i<recursions; i++){
		ns = (1.0 - float(i)/recursions);
		
		glColor3f(0.0,1.0-ns,1.0-ns);
		
		particle_processing(params.number_of_particles, 1);
		
		m_particles->resample(	params.number_of_particles,
								params.noise_rot_max * ns,
								params.noise_trans_max * ns,
								params.noise_scale_max * ns);
	}
	p_result = *m_particles->getMax(params.number_of_particles);
}

// Draw each particle, count matching pixels, calculate likelihood for edge tracking
void EdgeTracker::particle_processing(int num_particles, unsigned int num_avaraged_particles){
	m_cam_perspective->Activate();
	
	m_model->setTexture(0);
	
	for(int i=0; i<num_particles; i++){
	
		m_particles->activate(i);
		
		glColorMask(0,0,0,0); glDepthMask(1);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_model->drawFaces();
		
		glDepthMask(0);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		
		m_particles->startCountV(i);
		m_model->drawEdges();
		m_particles->endCountV();
		
		glColorMask(0,0,0,0);
		m_shadeEdgeCompare->bind();
		m_shadeEdgeCompare->setUniform("analyze", false);
		m_particles->startCountD(i);
		m_model->drawEdges();
		m_particles->endCountD();
		m_shadeEdgeCompare->unbind();
		
		m_particles->deactivate(i);
	}
	// Reset render masks
	glColorMask(1,1,1,1); glDepthMask(1);
	
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
}


// *** PUBLIC ***

EdgeTracker::EdgeTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_zero_particles = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_testflag = false;
	m_bfc = false;
	
	int id;
	// Shader
	if((id = g_Resources->AddShader("edgetest", "edgetest.vert", "edgetest.frag")) == -1)
		exit(1);
	m_shadeEdgeCompare = g_Resources->GetShader(id);
}

// Initialise function (must be called before tracking)
bool EdgeTracker::initInternal()
{	
	m_shadeEdgeCompare->bind();
	m_shadeEdgeCompare->setUniform("texFrame", 0);
	m_shadeEdgeCompare->setUniform("width", params.width);
	m_shadeEdgeCompare->setUniform("height", params.height);
	m_shadeEdgeCompare->setUniform("fTol", params.edge_tolerance);
	m_shadeEdgeCompare->unbind();
	
	return (m_tracker_initialized = true);
}

bool EdgeTracker::track(Model* model,
						Camera* camera,
						Particle& p_result)		// storage to write tracked position
{
	m_timer.Update();
	time_tracking = 0.0;
		
	m_model = model;
	m_cam_perspective = camera;
	
	// Recursive particle filtering
	if(!m_lock)
		particle_filtering(p_result, params.recursions);		
	
	// Copy result to output
	if(m_zero_particles){
		m_particles->setAll(params.zP);
		p_result = params.zP;
		m_zero_particles = false;
	}
	
	time_tracking += m_timer.Update();
	
	return true;
}

bool EdgeTracker::track(	unsigned char* image,
							Model* model,
							Camera* camera,
							Particle& p_result)		// storage to write tracked position
{	
	// Check if input is valid
	isReady(image, model, camera);	
	
	// Process image from camera (edge detection)
	image_processing(image);
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[0]);
	else
		m_ip->render(m_tex_frame);
	
	// Recursive particle filtering
	if(!m_lock)
		particle_filtering(p_result, params.recursions);		

	
	// Copy result to output
	if(m_zero_particles){
		m_particles->setAll(params.zP);
		p_result = params.zP;
		m_zero_particles = false;
	}
	
	time_tracking += m_timer.Update();
	
	return true;
}

// Draw result of edge tracking (particle with maximum likelihood)
void EdgeTracker::drawResult(Particle* p, Model* m){
	m_cam_perspective->Activate();
	
	glColor3f(1.0,0.0,0.0);
	glLineWidth(5);
	
	p->activate();
	glColorMask(0,0,0,0); glDepthMask(1);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	m_model->drawFaces();
	glColorMask(1,1,1,1);
	//m_opengl.RenderSettings(true, true);

	//if(!m_showmodel)m_shadeEdgeCompare->bind();
	//if(!m_showmodel)m_shadeEdgeCompare->setUniform("analyze", true);
	m_model->drawEdges();
	//if(!m_showmodel)m_shadeEdgeCompare->unbind();
	
	glColor3f(1.0,1.0,1.0);	
	p->deactivate();
	
}


