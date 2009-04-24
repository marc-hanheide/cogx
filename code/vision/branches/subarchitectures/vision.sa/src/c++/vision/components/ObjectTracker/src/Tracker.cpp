
#include "Tracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void Tracker::image_processing_texture(unsigned char* image){
	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	m_cam_ortho->Activate();
	m_ip->gauss(m_tex_frame, m_tex_frame_ip);
	m_ip->sobel(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->thinning(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_thinn);
	m_ip->spreading(m_tex_frame_thinn, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	//m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->render(m_tex_frame);
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void Tracker::image_processing_edge(unsigned char* image){

	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	m_cam_ortho->Activate();
	m_ip->gauss(m_tex_frame, m_tex_frame_ip);
	m_ip->sobel(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->thinning(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->render(m_tex_frame);
}

// Calculate motion function (noise) and perturb particles
void Tracker::particle_motion_texture(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax()->w;
	Particle noise_particle;
	
	if(m_particles->getMax()->w > 0.6)
		pow_scale = 0.1;
		
	noise_particle.rX = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale * 2.0;
	noise_particle.rY = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale * 2.0;
	noise_particle.rZ = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale;
	noise_particle.tX = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale;
	noise_particle.tY = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale;
	noise_particle.tZ = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale * 5.0;
	
    if(!m_lockparticles){
   		m_particles->perturb(noise_particle, p_ref, distribution);
   	}
}

// Calculate motion function (noise) and perturb particles
void Tracker::particle_motion_edge(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax()->w;
	Particle noise_particle;
	
	noise_particle.rX = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale;
	noise_particle.rY = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale;
	noise_particle.rZ = params.noise_rot_min + (params.noise_rot_max - params.noise_rot_min) * (1-w) * pow_scale;
	noise_particle.tX = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale;
	noise_particle.tY = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale;
	noise_particle.tZ = params.noise_trans_min + (params.noise_trans_max - params.noise_trans_min) * (1-w) * pow_scale;
	
    if(!m_lockparticles){
   		m_particles->perturb(noise_particle, p_ref, distribution);
   	}
}

// Draw Model to screen, extract modelview matrix, perform imageprocessing for model
void Tracker::model_processing(){

	// Render model to screen (=projection)
	m_cam_perspective->Activate();			// activate perspective view
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_particles->activateMax();
	glGetFloatv(GL_MODELVIEW_MATRIX, m_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, m_projection);
	m_model->restoreTexture();
	m_model->drawFaces();
	m_particles->deactivate();
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
	//m_ip->spreading(m_tex_model_ip, m_tex_model_ip);
	
	// Tests ...
	
	
	
}

// Draw each particle, count matching pixels, calculate likelihood for texture tracking
void Tracker::particle_processing_texture(int num_particles, unsigned int num_avaraged_particles){

	// Viewport adjustment
	int v = m_particles->getVMax();
	if( v>300 ){
		params.viewport_width = params.viewport_width * 0.5;
		params.viewport_height = params.viewport_height * 0.5;
	}else if( v<100){
		params.viewport_width = params.viewport_width * 2;
		params.viewport_height = params.viewport_height * 2;
	}
	
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
		
		m_particles->deactivate();
	}
	
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
	m_shadeTextureCompare->unbind();
	glPopAttrib();
}

// Draw each particle, count matching pixels, calculate likelihood for edge tracking
void Tracker::particle_processing_edge(int num_particles, unsigned int num_avaraged_particles){

	// Viewport adjustment
	int v = m_particles->getVMax();
	if( v>300 ){
		params.viewport_width = params.viewport_width * 0.5;
		params.viewport_height = params.viewport_height * 0.5;
	}else if( v<100){
		params.viewport_width = params.viewport_width * 2;
		params.viewport_height = params.viewport_height * 2;
	}
	
	m_cam_perspective->Activate();
	m_tex_frame_ip->bind();	// bind camera image
	m_model->setTexture(0);
	
	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0,0,params.viewport_width,params.viewport_height);
	
	for(int i=0; i<m_particles->getNumParticles(); i++){
	
		m_particles->activate(i);
		
		m_opengl.ClearBuffers(false, true);
		m_opengl.RenderSettings(false, true);
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
		
		m_particles->deactivate();
	}
	// Calculate likelihood of particles
	m_particles->calcLikelihood(num_particles, num_avaraged_particles);
	glPopAttrib();
}

// Draw result of texture tracking (particle with maximum likelihood)
void Tracker::draw_result_texture(){
	
	if(m_showmodel){
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(m_modelview);
	
		m_opengl.ClearBuffers(true, true);
		m_opengl.RenderSettings(true, false);
		glDisable(GL_DEPTH_TEST);
		
		//m_shadeTextureCompare->bind();
		//m_shadeTextureCompare->setUniform("analyze", true);
		//m_shadeTextureCompare->setUniform("compare", true);
		m_model->restoreTexture();
		m_model->drawFaces();
		//m_shadeTextureCompare->unbind();
		
		glEnable(GL_DEPTH_TEST);
		m_opengl.RenderSettings(true, true);
	}
}

// Draw result of edge tracking (particle with maximum likelihood)
void Tracker::draw_result_edge(){
	if(m_showmodel){
		m_particles->activateMax();
		m_opengl.ClearBuffers(false, true);
		m_opengl.RenderSettings(false, true);
		m_model->drawFaces();
		m_opengl.RenderSettings(true, true);
		m_shadeEdgeCompare->bind();
		m_shadeEdgeCompare->setUniform("analyze", true);
		m_model->drawEdges();
		m_shadeEdgeCompare->unbind();
		m_particles->deactivate();
	}
}

// Ask for mouse or keyboard inputs
bool Tracker::inputs(){
	vec4 v1,v2;
	TM_Vector3 c;
	float mv[16];
	int id_max = m_particles->getMaxID();
	float alpha, beta, gamma;
	Particle p;
	mat3 rot;
	vec3 pos;
    
	SDL_Event event;
	while(SDL_PollEvent(&event)){
		switch(event.type){
		case SDL_KEYDOWN:
            switch(event.key.keysym.sym){
				case SDLK_ESCAPE:
					return false;
					break;
				case SDLK_l:
					m_lockparticles = !m_lockparticles;
					if(m_lockparticles){
						printf("Tracker locked\n");
						m_particles->setAll(*m_particles->getMax());
					}else{
						printf("Tracker unlocked\n");
					}
					break;
				case SDLK_m:
					m_showmodel = !m_showmodel;
					break;
				case SDLK_p:
					m_showparticles = !m_showparticles;
					break;
				case SDLK_s:
					showStatistics();
					break;
				case SDLK_v:
					glMatrixMode(GL_MODELVIEW);
					glLoadIdentity();
					
					m_particles->activate(id_max);
					glGetFloatv(GL_MODELVIEW_MATRIX, mv);
					m_particles->deactivate();
					
					rot[0] = mv[0]; rot[1] = mv[1]; rot[2] = mv[2];  // mv[3]
					rot[3] = mv[4]; rot[4] = mv[5]; rot[5] = mv[6];  // mv[7]
					rot[6] = mv[8]; rot[7] = mv[9]; rot[8] = mv[10]; // mv[11]
					pos.x = mv[12]; pos.y = mv[13]; pos.z = mv[14];  // mv[15]
					
					p = Particle(rot, pos);
					p.print();
					m_particles->getMax()->print();
					
					break;
				case SDLK_z:
					m_particles->setAll(Particle(0.0));
					break;
                default:
					break;
			}
			break;
		case SDL_QUIT:
			return false;
			break;
		default:
			break;
		}
	}
	return true;
}


// *** PUBLIC ***

Tracker::Tracker(){
	m_lockparticles = true;
	m_showparticles = false;
	m_showmodel = true;
}

Tracker::~Tracker(){
}

// Initialise function (must be called before tracking)
bool Tracker::init(	int width, int height,
					int nop,
					float fovy,
					float cipX, float cipY, float cipZ,
					float n_r_min, float n_r_max,
					float n_t_min, float n_t_max,
					float et,
					int vw, int vh)
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);
	params.number_of_particles = nop;
	params.camera_fovy = fovy;
	params.camera_initial_position_x = cipX;
	params.camera_initial_position_y = cipY;
	params.camera_initial_position_z = cipZ;
	params.noise_rot_min = n_r_min;
	params.noise_rot_max = n_r_max;
	params.noise_trans_min = n_t_min;
	params.noise_trans_max = n_t_max;
	params.edge_tolerance = et;
	params.viewport_width = vw;
	params.viewport_height = vh;
	
	m_opengl.Init();
	
	// Singleton resources
	g_Resources->InitImageProcessor(width, height);
	g_Resources->InitParticles(params.number_of_particles, Particle(0.0));
	g_Resources->InitFrustum();
	
	m_ip = g_Resources->GetImageProcessor();
	m_particles = g_Resources->GetParticles();
	
	// Textures
	int id;
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame")) == -1)
		return false;
	m_tex_frame = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_ip")) == -1)
		return false;
	m_tex_frame_ip = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model_ip")) == -1)
		return false;
	m_tex_model_ip = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_thinn")) == -1)
		return false;
	m_tex_frame_thinn = g_Resources->GetTexture(id);
	
	// Cameras
	if((id = g_Resources->AddCamera("cam_ortho")) == -1)
		return false;
	m_cam_ortho = g_Resources->GetCamera(id);
	m_cam_ortho->Set(	0.0, 0.0, 1.0,
						0.0, 0.0, 0.0,
						0.0, 1.0, 0.0,
						params.camera_fovy, width, height,
						0.1, 10.0,
						GL_ORTHO);
							
	if((id = g_Resources->AddCamera("cam_perspective")) == -1)
		return false;
	m_cam_perspective = g_Resources->GetCamera(id);
	m_cam_perspective->Set(	params.camera_initial_position_x, params.camera_initial_position_y, params.camera_initial_position_z,
							0.0, 0.0, 0.0,
							0.0, 1.0, 0.0,
							params.camera_fovy, width, height,
							0.1, 10.0,
							GL_PERSPECTIVE);
							
	// Shader
	if((id = g_Resources->AddShader("texturetest", "texturetest.vert", "texturetest.frag")) == -1)
		return false;
	m_shadeTextureCompare = g_Resources->GetShader(id);
	m_shadeTextureCompare->bind();
	m_shadeTextureCompare->setUniform("tex_frame", 1);
	m_shadeTextureCompare->setUniform("tex_model", 0);
	m_shadeTextureCompare->setUniform("fTol", params.edge_tolerance);
	m_shadeTextureCompare->unbind();
	
	// Shader
	if((id = g_Resources->AddShader("edgetest", "edgetest.vert", "edgetest.frag")) == -1)
		return false;
	m_shadeEdgeCompare = g_Resources->GetShader(id);
	m_shadeEdgeCompare->bind();
	m_shadeEdgeCompare->setUniform("texFrame", 0);
	m_shadeEdgeCompare->setUniform("width", params.width);
	m_shadeEdgeCompare->setUniform("height", params.height);
	m_shadeEdgeCompare->setUniform("fTol", params.edge_tolerance);
	m_shadeEdgeCompare->unbind();
	
	return true;
}

// Tracking function for texture tracking
bool Tracker::trackTexture(	unsigned char* image,		// camera image (3 channel, unsigned byte each)
							Model* model,				// tracking model (textured, vertexlist, facelist) 
							Particle* p_estimate,		// position estimate of model (R,t)
							Particle* p_result)		// storage to write tracked position
{
	if(!image || !model){
		printf("[Tracker::trackTexture] Error model not valid\n");
		return false;
	}
	m_model = model;
	
	// Process image from camera (edge detection)
	image_processing_texture(image);
	
	// Process model (texture reprojection, edge detection)
	model_processing();
	
	// Clear framebuffer and render image from camera
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	m_ip->render(m_tex_frame);
	
	// Overwrite maximum particle (reference particle) with given estimate
	Particle* p_max = m_particles->getMax();
	if(p_estimate)
		p_max = p_estimate;
		
	// Draw particles and evaluate match likelihood
	m_tex_frame_ip->bind(1);	// bind camera image
	particle_processing_texture(1);		// draw one particle to see if it still matches (object not moved)
	
	if(m_particles->getMax()->w < 0.75){
		particle_motion_texture(1.0, NULL, GAUSS);
		particle_processing_texture(params.number_of_particles * 0.5, 5);
		particle_motion_texture(0.1, NULL, NORMAL);
		particle_processing_texture(params.number_of_particles * 0.25, 5);
		
		particle_motion_texture(0.1, NULL, GAUSS);
		particle_processing_texture(params.number_of_particles * 0.125, 5);
		
		
		m_tex_frame_thinn->bind(1);	// bind image with thinn edges
		particle_motion_texture(0.1, NULL, GAUSS);
		particle_processing_texture(params.number_of_particles * 0.125, 1);
		
	}
	
	// Copy result to output
	if(p_result)
		*p_result = *m_particles->getMax();
	
	// Draw result
	draw_result_texture();
	
	time_tracking = m_timer.Update();
	
	SDL_GL_SwapBuffers();
	SDL_Delay(10);
	return inputs();
}


bool Tracker::trackEdge(	unsigned char* image,
							Model* model,
							Particle* p_estimate,
							Particle* p_result)		// storage to write tracked position
{
	if(!image || !model){
		printf("[Tracker::trackTexture] Error model not valid\n");
		return false;
	}
	m_model = model;
	
	// Process image from camera (edge detection)
	image_processing_edge(image);
	
	// Overwrite maximum particle (reference particle) with given estimate
	Particle* p_max = m_particles->getMax();
	if(p_estimate)
		p_max = p_estimate;
		
	// Draw particles and evaluate match likelihood
	particle_motion_edge(1.0);
	particle_processing_edge(params.number_of_particles * 0.75, 3);
	particle_motion_edge(0.2, NULL, GAUSS);
	particle_processing_edge(params.number_of_particles * 0.25, 50);
	
	// Copy result to output
	if(p_result)
		*p_result = *m_particles->getMax();
	
	// Draw result
	draw_result_edge();
	
	time_tracking = m_timer.Update();
	
	SDL_GL_SwapBuffers();
	
	return inputs();

}


bool Tracker::render(unsigned char* image){
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, false); 	// (color-enabled, depth-enabled)
	m_cam_ortho->Activate();
	m_tex_frame->load(image, params.width, params.height);
	g_Resources->GetImageProcessor()->render(m_tex_frame);
	SDL_GL_SwapBuffers();
	SDL_Delay(10);
	return inputs();
}

bool Tracker::render(Model* model){
	m_opengl.ClearBuffers(true, true);		// clear frame buffers (color, depth)
	m_opengl.RenderSettings(true, true); 	// (color-enabled, depth-enabled)
	m_cam_perspective->Activate();
	model->drawFaces();
	SDL_GL_SwapBuffers();
	SDL_Delay(10);
	return inputs();
}
	
bool Tracker::release(){
	return true;
}

// Show performance and likelihood
void Tracker::showStatistics(){
	printf("\n\nStatistics:\n");
	printf("	Particles: %i\n", m_particles->getNumParticles() );
	printf("	Tracking time: %.0f ms\n", time_tracking * 1000.0 );
	printf("	Pixels of model: %i \n", m_particles->getVMax() );
	printf("	Likelihood p[%i]: %.0f %%\n", m_particles->getMaxID(), m_particles->getMax()->w * 100.0 );
}

