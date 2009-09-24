
#include "TextureTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image){
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
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
void TextureTracker::particle_motion(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax()->w;
	Particle noise_particle;
	
	// Standard deviation for gaussian noise
	noise_particle.rX = params.noise_rot_max * pow_scale * (1.5-w);
	noise_particle.rY = params.noise_rot_max * pow_scale * (1.5-w);
	noise_particle.rZ = params.noise_rot_max * pow_scale * (1.5-w);
	noise_particle.tX = params.noise_trans_max * pow_scale * (1.2-w);
	noise_particle.tY = params.noise_trans_max * pow_scale * (1.2-w);
	noise_particle.tZ = params.noise_trans_max * pow_scale * (1.2-w);
	
    if(!m_lock){
   		m_particles->perturb(noise_particle, params.number_of_particles, p_ref, distribution);
   	}
}

// Particle filtering
void TextureTracker::particle_filtering(Particle* p_estimate){
	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(0.0,0.0,1.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[3]->bind(1);
	m_model->setTexture(m_tex_model_ip[3]);
	particle_motion(1.0, p_estimate, GAUSS);
	particle_processing(params.number_of_particles*0.75, 1);
	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(1.0,0.0,0.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[2]->bind(1);
	m_model->setTexture(m_tex_model_ip[2]);
	particle_motion(0.5, NULL, GAUSS);
	particle_processing(params.number_of_particles*0.125, 1);
	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(0.0,1.0,0.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[1]->bind(1);
	m_model->setTexture(m_tex_model_ip[1]);
	particle_motion(0.1, NULL, GAUSS);
	particle_processing(params.number_of_particles*0.125, 1);
	
}

// Draw Model to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(){
	
	// Render camera image as background
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	//m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	//m_ip->render(m_tex_frame);
	
	// Render model to screen (=projection)
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_cam_perspective->Activate();			// activate perspective view
	m_lighting.Activate();
	m_particles->activateMax();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, m_projection);
	m_modelviewprojection = m_projection * m_modelview;	
	
	m_model->restoreTexture();
	
	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("modelviewprojection", m_model->getModelviewProjection(), GL_FALSE);		
		m_model->drawPass(m_shadeTexturing);
	m_shadeTexturing->unbind();
	
	m_particles->deactivate(0);
	m_lighting.Deactivate();
	
	// calculate modelview_projection_matrix from maximum-likelihood-particle
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("modelviewprojection", m_modelviewprojection, GL_FALSE); // send matrix to shader
	m_shadeTextureCompare->unbind();
	
	// Copy screen to texture (=reprojection to model)
	m_tex_model->copyTexImage2D(params.width, params.height);
		
	// perform image processing with reprojected image
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	if(m_model->isTextured()){
		m_ip->gauss(m_tex_model, m_tex_model_ip[0]);
	}else{
		m_ip->copy(m_tex_model, m_tex_model_ip[0]);
	}
	m_ip->sobel(m_tex_model_ip[0], m_tex_model_ip[0]);
	m_ip->thinning(m_tex_model_ip[0], m_tex_model_ip[0]);
	m_ip->spreading(m_tex_model_ip[0], m_tex_model_ip[1]);
	m_ip->spreading(m_tex_model_ip[1], m_tex_model_ip[2]);
	m_ip->spreading(m_tex_model_ip[2], m_tex_model_ip[3]);
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	
}

// Draw each particle, count matching pixels, calculate likelihood for texture tracking
void TextureTracker::particle_processing(int num_particles, unsigned int num_avaraged_particles){
	
	// Set perspective mode and smaller viewport
	m_opengl.RenderSettings(false, false);	// (color-enabled, depth-enabled)
	
	m_cam_perspective->Activate();
	//glPushAttrib(GL_VIEWPORT_BIT);
	//glViewport(0,0,params.viewport_width,params.viewport_height);
	
	// Draw particles and count pixels
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("analyze", false);
	for(int i=0; i<num_particles; i++){
		m_particles->activate(i);

		m_particles->startCountV(i);
		m_shadeTextureCompare->setUniform("compare", false);
		if(m_showparticles)
			m_opengl.RenderSettings(true, false);
		m_model->drawFaces();
		m_particles->endCountV();
		
		m_opengl.RenderSettings(false, false);
		
		m_particles->startCountD(i);
		m_shadeTextureCompare->setUniform("compare", true);
		m_model->drawFaces();
		m_particles->endCountD();
		
		m_particles->deactivate(i);
	}
	
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
	m_shadeTextureCompare->unbind();

	m_opengl.RenderSettings(true, true);
}


// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_kalman_enabled = true;
	m_zero_particles = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_testflag = false;
	m_bfc = false;
		
	int id;
	// Shader
	if((id = g_Resources->AddShader("texturetest", "texturetest.vert", "texturetest.frag")) == -1)
		exit(1);
	m_shadeTextureCompare = g_Resources->GetShader(id);
	
	if((id = g_Resources->AddShader("texturing", "texturing.vert", "texturing.frag")) == -1)
		exit(1);
	m_shadeTexturing = g_Resources->GetShader(id);

}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal()
{	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("tex_frame", 1);
	m_shadeTextureCompare->setUniform("tex_model", 0);
	m_shadeTextureCompare->setUniform("fTol", params.edge_tolerance);
	m_shadeTextureCompare->unbind();
		
	return (m_tracker_initialized = true);
}

// Tracking function for texture tracking
bool TextureTracker::track(	unsigned char* image,		// camera image (3 channel, unsigned byte each)
							Model* model,				// tracking model (textured, vertexlist, facelist) 
							Camera* camera,				// extrinsic camera (defining pose and projection)
							Particle p_estimate,		// position estimate of model (R,t)
							Particle& p_result)			// storage to write tracked position
{
	// Start time measurement
	m_timer.Update();
	
	// Check if input is valid
	isReady(image, model, camera, &p_estimate);	
	
	// Process image from camera (edge detection)
	image_processing(m_image);
	
	// Process model (texture reprojection, edge detection)
	model_processing();
	
	// Clear framebuffer and render image from camera
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[1]);
	else
		m_ip->render(m_tex_frame);
	
	if(!m_lock){
	
		particle_filtering(&p_estimate);
		
		// Kalman filter
		if(m_kalman_enabled)
			kalman_filtering(m_particles->getMax());
			
		// Copy result to output
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
	//}else if(m_particles->getMax()->w < 0.2){
		//p_result = p_estimate;
	}
	
	
	
	/*
		m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
		m_lighting.getLightDirection(m_tex_frame, m_particles->getMax(), m_model, m_cam_perspective);
	*/	
	
	return true;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage()
{
	Particle p_estimate = *m_particles->getMax();
	Particle p_result, p_old, p_new;	
	
	m_model->setOriginalTexture(0);
	m_model->setTexture(0);
	m_model->enableTexture(false);
	//params.noise_rot_max = params.noise_rot_max * 0.5;
	//params.noise_trans_max = params.noise_trans_max * 0.5;
	m_kalman_enabled=false;
	
	track(m_image, m_model, m_cam_perspective, p_estimate, p_result);
	drawResult(&p_result);
	swap();
	SDL_Delay(100);	
	
	m_model->textureFromImage(m_image, params.width, params.height, &p_result);
	m_shadeTexturing->bind();
	m_shadeTexturing->setUniform("useTexCoords", false);
	m_shadeTexturing->unbind();
	
	m_model->enableTexture(true);
	
	//params.noise_rot_max = params.noise_rot_max * 2;
	//params.noise_trans_max = params.noise_trans_max * 2;
	m_kalman_enabled=true;	
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(Particle* p){
	bool texmodel = false;
	
	m_cam_perspective->Activate();
	m_lighting.Activate();
	p->activate();
	
	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);

	
	if(m_showmodel){
		m_model->restoreTexture();
		m_shadeTexturing->bind();
		m_model->drawPass(m_shadeTexturing);
	}else if(texmodel){
		m_model->setTexture(m_tex_model_ip[1]);
		m_shadeTextureCompare->bind();
		m_shadeTextureCompare->setUniform("analyze", true);
		m_shadeTextureCompare->setUniform("compare", true);
		m_model->drawFaces();
	}else{
	
		glEnable(GL_DEPTH_TEST);
		m_lighting.Deactivate();
		m_opengl.RenderSettings(false, true);
		m_opengl.ClearBuffers(false, true);
		m_model->drawFaces();
		m_opengl.RenderSettings(true, true);
		glLineWidth(3);
		glColor3f(1.0,0.0,0.0);
		m_model->drawEdges();
		glColor3f(1.0,1.0,1.0);
	}
	 
	if(m_showmodel){
	 	m_shadeTexturing->unbind();
	}else if(texmodel){
		m_shadeTextureCompare->unbind();
	}
	
	glEnable(GL_DEPTH_TEST);
	m_opengl.RenderSettings(true, true);
	
	p->deactivate();
	m_lighting.Deactivate();
}

