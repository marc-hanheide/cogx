
#include "EdgeTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image){
	
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	glDepthMask(1);
}

// particle filtering
void EdgeTracker::particle_filtering(Particle& p_result, Particle p_constraints, int recursions, float fTime){
	
	// Calculate Zoom Direction and pass it to the particle filter
	TM_Vector3 vCam = m_cam_perspective->GetPos();
	TM_Vector3 vObj = TM_Vector3(p_result.s.x, p_result.s.y, p_result.s.z);
	TM_Vector3 vCamObj = vObj - vCam;
	vCamObj.normalize();
	m_particles->setCamViewVector(vCamObj);
	
	
	glLineWidth(2);
	glColor3f(0.0,0.0,0.0);
	m_tex_frame_ip[0]->bind();	// bind camera image
	float ns = 0.5;
	
	for(int i=0; i<recursions; i++){
		if(recursions > 1)
			ns = 1.0 - float(i)/(recursions-1);
		
		glColor3f(0.0,1.0-ns,1.0-ns);
		
		particle_processing();
		
		m_particles->resample(	params.number_of_particles, p_constraints);
	}
	m_pose = *m_particles->getMax();
}

// Draw each particle, count matching pixels, calculate likelihood for edge tracking
void EdgeTracker::particle_processing(){
	m_cam_perspective->Activate();
	
	m_model->setTexture(0);
	
	for(int i=0; i<params.number_of_particles; i++){
	
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
	m_particles->calcLikelihood(5);
}


// *** PUBLIC ***

EdgeTracker::EdgeTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_zero_particles = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
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

bool EdgeTracker::track(TrackerModel* model,
												Camera* camera,
												int num_recursions,
												int num_particles,
												Particle p_constraints, 
												Particle& p_result,
												float fTime)
{
	m_timer.Update();
	params.time_tracking = 0.0;
		
	m_model = model;
	m_cam_perspective = camera;
	params.number_of_particles = num_particles;
	params.recursions = num_recursions;
	
	// Recursive particle filtering
	if(!m_lock){
		particle_filtering(p_result, p_constraints, params.recursions, fTime);		
		p_result = m_pose;
	}
	
	// Copy result to output
	if(m_zero_particles){
		m_particles->sample(params.number_of_particles, params.zP, p_constraints);
		p_result = params.zP;
		m_zero_particles = false;
	}
	
	params.time_tracking += m_timer.Update();
	
	return true;
}

// Draw result of edge tracking (particle with maximum likelihood)
void EdgeTracker::drawResult(Particle* p, TrackerModel* m){
// 	float var = 0.0;
// 	var = m_particles->getVariance(params.number_of_particles);
// 	glColor3f(1000*var,1-1000*var,0.0);
	m_cam_perspective->Activate();
	
	glColor3f(1.0,0.0,0.0);
	glLineWidth(2);
	
	p->activate();
	glColorMask(0,0,0,0); glDepthMask(1);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	m_model->drawFaces();
	glColorMask(1,1,1,1);
	//m_opengl.RenderSettings(true, true);
	if(!m_showmodel)m_tex_frame_ip[0]->bind();
	if(!m_showmodel)m_shadeEdgeCompare->bind();
	if(!m_showmodel)m_shadeEdgeCompare->setUniform("analyze", true);
	m_model->drawEdges();
	if(!m_showmodel)m_shadeEdgeCompare->unbind();
	
	glColor3f(1.0,1.0,1.0);	
	p->deactivate();
	
}


