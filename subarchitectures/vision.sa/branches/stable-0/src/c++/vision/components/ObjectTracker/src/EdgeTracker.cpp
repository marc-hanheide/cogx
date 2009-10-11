
#include "EdgeTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image){

	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	m_timer.Update();
	
	// Preprocessing for camera image
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	m_ip->spreading(m_tex_frame_ip[0], m_tex_frame_ip[1]);
	m_ip->spreading(m_tex_frame_ip[1], m_tex_frame_ip[2]);
	m_ip->spreading(m_tex_frame_ip[2], m_tex_frame_ip[3]);
}

// Calculate motion function (noise) and perturb particles
void EdgeTracker::particle_motion(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax()->w;
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

void EdgeTracker::particle_filtering(Particle* p_estimate){
	glLineWidth(10);
	glColor3f(0.0,0.0,0.0);
	m_tex_frame_ip[2]->bind();	// bind camera image
	particle_motion(1.0, p_estimate, GAUSS);
	particle_processing(params.number_of_particles*0.75, 1);
	
	glLineWidth(4);
	glColor3f(1.0,0.0,0.0);
	//m_tex_frame_ip[2]->bind();	// bind camera image
	particle_motion(0.3, NULL, GAUSS);
	particle_processing(params.number_of_particles*0.25, 1);
}

// Draw each particle, count matching pixels, calculate likelihood for edge tracking
void EdgeTracker::particle_processing(int num_particles, unsigned int num_avaraged_particles){

	m_cam_perspective->Activate();
	m_model->setTexture(0);
	
	for(int i=0; i<num_particles; i++){
	
		m_particles->activate(i);
		
		m_opengl.RenderSettings(false, true);
		m_opengl.ClearBuffers(false, true);
		m_model->drawFaces();
		
		
		m_particles->startCountV(i);
		if(m_showparticles)
			m_opengl.RenderSettings(true, false);
		else
			m_opengl.RenderSettings(false, false);
		m_model->drawEdges();
		m_opengl.RenderSettings(false, true);
		m_particles->endCountV();
		
		m_shadeEdgeCompare->bind();
		m_shadeEdgeCompare->setUniform("analyze", false);
		m_particles->startCountD(i);
		m_opengl.RenderSettings(false, false);
		m_model->drawEdges();
		m_opengl.RenderSettings(false, true);
		m_particles->endCountD();
		m_shadeEdgeCompare->unbind();
		
		m_particles->deactivate(i);
	}
	
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
	glPopAttrib();
}


// *** PUBLIC ***

EdgeTracker::EdgeTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_kalman_enabled = false;
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

bool EdgeTracker::track(	unsigned char* image,
							Model* model,
							Camera* camera,
							Particle p_estimate,
							Particle& p_result)		// storage to write tracked position
{
	//glReadBuffer(GL_AUX1);
	//glDrawBuffer(GL_AUX1);
	
	// Check if input is valid
	isReady(image, model, camera, &p_estimate);	
	
	// Reduce viewport for higher performance
	m_cam_perspective->SetViewport(128, 128);
	
	// Process image from camera (edge detection)
	image_processing(image);
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[1]);
	else
		m_ip->render(m_tex_frame);
	
	if(!m_lock){
		// Recursive particle filtering
		particle_filtering(&p_estimate);
		
		// Kalman filter
		if(m_kalman_enabled)
			kalman_filtering(m_particles->getMax());
			
		p_result = Particle(*m_particles->getMax());
		
		// adjust number of particles according to tracking speed
		time_tracking = m_timer.Update();
		params.number_of_particles += 10;
		if(time_tracking > params.track_time && params.number_of_particles > 100)
			params.number_of_particles += 1000 * (params.track_time - time_tracking);
		else if(time_tracking < params.track_time)
			params.number_of_particles += 1000 * (params.track_time - time_tracking);
		if(params.number_of_particles > m_particles->getNumParticles())
			params.number_of_particles = m_particles->getNumParticles();
	}else{
		time_tracking = m_timer.Update();
		p_result = p_estimate;
	}
	
	// Copy result to output
	if(m_zero_particles){
		p_result = params.zP;
		m_zero_particles = false;
	//}else if(m_particles->getMax()->w < 0.1){
	//	p_result = p_estimate;
	}
	
	//glReadBuffer(GL_BACK);
	//glDrawBuffer(GL_BACK);
	
	return true;
}

// Draw result of edge tracking (particle with maximum likelihood)
void EdgeTracker::drawResult(Particle* p){
	glLineWidth(1);
	m_cam_perspective->SetViewport(params.width, params.height);
	m_cam_perspective->Activate();
	p->activate();
		
	m_opengl.RenderSettings(false, true);
	m_opengl.ClearBuffers(false, true);
	glColor3f(0.0,0.0,1.0);
	m_model->drawFaces();
	m_opengl.RenderSettings(true, true);

	glColor3f(1.0,0.0,0.0);
	if(!m_showmodel)m_shadeEdgeCompare->bind();
	if(!m_showmodel)m_shadeEdgeCompare->setUniform("analyze", true);
	glLineWidth(5);
	m_model->drawEdges();
	if(!m_showmodel)m_shadeEdgeCompare->unbind();
	p->deactivate();
}


