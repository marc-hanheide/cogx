
#include "TextureTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image){
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	m_timer.Update();
	
	// Preprocessing for camera image
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	m_cam_ortho->Activate();
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->render(m_tex_frame_ip[0]);
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

// Draw Model to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(){
	
	// Render camera image as background
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	//m_cam_ortho->Activate();
	//m_ip->render(m_tex_frame);
	
	// Render model to screen (=projection)
	m_cam_perspective->Activate();			// activate perspective view
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_particles->activateMax();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, m_projection);
	m_model->restoreTexture();
	m_model->drawFaces();
	m_particles->deactivate(0);
	// calculate modelview_projection_matrix from maximum-likelihood-particle
	m_modelviewprojection = m_projection * m_modelview;	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("modelviewprojection", m_modelviewprojection, GL_FALSE); // send matrix to shader
	m_shadeTextureCompare->unbind();
	
	// Copy screen to texture (=reprojection to model)
	m_tex_model_ip->copyTexImage2D(params.width, params.height);
	m_model->setTexture(m_tex_model_ip);
		
	// perform image processing with reprojected image
	m_cam_ortho->Activate();
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	//ip->render(m_tex_model_ip);
	m_ip->gauss(m_tex_model_ip, m_tex_model_ip);
	m_ip->sobel(m_tex_model_ip, m_tex_model_ip);
	m_ip->thinning(m_tex_model_ip, m_tex_model_ip);
	m_ip->spreading(m_tex_model_ip, m_tex_model_ip);
	m_ip->spreading(m_tex_model_ip, m_tex_model_ip);
	m_ip->spreading(m_tex_model_ip, m_tex_model_ip);
	
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
}

// Draw each particle, count matching pixels, calculate likelihood for texture tracking
void TextureTracker::particle_processing(int num_particles, unsigned int num_avaraged_particles){
	
	// Set perspective mode and smaller viewport
	m_opengl.RenderSettings(false, false);	// (color-enabled, depth-enabled)
	
	m_cam_perspective->Activate();
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0,0,params.viewport_width,params.viewport_height);
	
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
		m_opengl.RenderSettings(false, false);
		m_particles->endCountV();
		
		m_particles->startCountD(i);
		m_shadeTextureCompare->setUniform("compare", true);
		m_model->drawFaces();
		m_particles->endCountD();
		
		m_particles->deactivate(i);
	}
	
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
	m_shadeTextureCompare->unbind();
	glPopAttrib();
}


// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_kalman_enabled = true;
	m_cascaded = true;
	m_draw_coordinates = false;
	m_draw_edges = false;
	m_result_textured = true;
	m_tracker_initialized = false;

	int id;
	// Shader
	if((id = g_Resources->AddShader("texturetest", "texturetest.vert", "texturetest.frag")) == -1)
		exit(1);
	m_shadeTextureCompare = g_Resources->GetShader(id);
	
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
	if(!m_tracker_initialized){
		printf("[TextureTracker::trackTexture] Error tracker not initialized\n");
		return false;
	}
	
	if(!image || !model){
		printf("[TextureTracker::trackTexture] Error model not valid\n");
		return false;
	}
	m_model = model;
	
	if(!camera){
		printf("[TextureTracker::trackTexture] Warning camera not set, using default values\n");
		m_cam_perspective = m_cam_default;
	}else{
		m_cam_perspective = camera;
	}
	
	
	// Process image from camera (edge detection)
	image_processing(image);
	
	// Process model (texture reprojection, edge detection)
	model_processing();
	
	// Clear framebuffer and render image from camera
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[3]);
	else
		m_ip->render(m_tex_frame);
	
	
	// Recursive particle filtering
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(0.0,0.0,1.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[3]->bind(1);
	particle_motion(1.0, &p_estimate, GAUSS);
	particle_processing(params.number_of_particles*0.75, 1);
	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(1.0,0.0,0.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[3]->bind(1);
	particle_motion(0.5, NULL, GAUSS);
	particle_processing(params.number_of_particles*0.125, 1);
	
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("drawcolor", vec4(0.0,1.0,0.0,0.0));
	m_shadeTextureCompare->unbind();
	m_tex_frame_ip[3]->bind(1);
	particle_motion(0.1, NULL, GAUSS);
	particle_processing(params.number_of_particles*0.125, 1);
	
	// Kalman filter
	if(m_kalman_enabled)
		kalman_filtering(m_particles->getMax());
		
	// Copy result to output
	if(m_zero_particles){
		p_result = Particle(0.0);
		m_zero_particles = false;
	}else if(m_particles->getMax()->w < 0.3){
		p_result = p_estimate;
	}else{
		p_result = Particle(*m_particles->getMax());
	}
	
	// Draw result
	drawResult(&p_result);
	
	// Draw coordinates
	if(m_draw_coordinates)
		renderCoordinates();
	
	// Swap GL-buffers (let CPU wait for GPU)
	SDL_GL_SwapBuffers();
	
	// adjust number of particles according to tracking speed
	time_tracking = m_timer.Update();
	params.number_of_particles += 10;
	if(time_tracking > params.track_time && params.number_of_particles > 100)
		params.number_of_particles += 1000 * (params.track_time - time_tracking);
	else if(time_tracking < params.track_time)
		params.number_of_particles += 1000 * (params.track_time - time_tracking);
	if(params.number_of_particles > m_particles->getNumParticles())
		params.number_of_particles = m_particles->getNumParticles();
	
	SDL_Delay(10);
	return inputs();
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(Particle* p){
	
	p->activate();

	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);
	
	if(!m_showmodel) m_shadeTextureCompare->bind();
	if(!m_showmodel) m_shadeTextureCompare->setUniform("analyze", true);
	if(!m_showmodel) m_shadeTextureCompare->setUniform("compare", true);
	if(m_showmodel) m_model->restoreTexture();
	m_model->drawFaces();
	if(!m_showmodel) m_shadeTextureCompare->unbind();
	
	glEnable(GL_DEPTH_TEST);
	m_opengl.RenderSettings(true, true);
	
	p->deactivate();

}


