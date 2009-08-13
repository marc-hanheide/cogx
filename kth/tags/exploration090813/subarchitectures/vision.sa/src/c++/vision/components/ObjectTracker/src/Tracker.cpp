
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
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	m_ip->spreading(m_tex_frame_ip, m_tex_frame_ip);
	//m_ip->render(m_tex_frame_thinn);
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
void Tracker::particle_motion(float pow_scale, Particle* p_ref, unsigned int distribution){
	// Perturbing noise function for rotation (powR) and translation (powT)
	float w = m_particles->getMax()->w;
	Particle noise_particle;
		
	noise_particle.rX = params.noise_rot_max * pow_scale * (1.2-w);
	noise_particle.rY = params.noise_rot_max * pow_scale * (1.2-w);
	noise_particle.rZ = params.noise_rot_max * pow_scale * (1.2-w);
	noise_particle.tX = params.noise_trans_max * pow_scale * (1.2-w);
	noise_particle.tY = params.noise_trans_max * pow_scale * (1.2-w);
	noise_particle.tZ = params.noise_trans_max * pow_scale * (1.2-w) * 3.0;
	
    if(!m_lock){
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
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(m_modelview);

	m_opengl.ClearBuffers(true, true);
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

}

// Draw result of edge tracking (particle with maximum likelihood)
void Tracker::draw_result_edge(){
	if(m_showmodel){
		m_cam_perspective->Activate();
		m_particles->activateMax();
		m_opengl.ClearBuffers(false, true);
		m_opengl.RenderSettings(false, true);
		m_model->drawFaces();
		m_opengl.RenderSettings(true, true);
		glColor3f(1.0,0.0,0.0);
		//m_shadeEdgeCompare->bind();
		//m_shadeEdgeCompare->setUniform("analyze", true);
		m_model->drawEdges();
		//m_shadeEdgeCompare->unbind();
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
	Particle* pmax;
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
				case SDLK_c:
					m_cascaded = !m_cascaded;
					printf("Cascaded: %i\n", m_cascaded);
					break;
				case SDLK_f:
					m_draw_coordinates = !m_draw_coordinates;
					printf("Draw Coordinates: %i\n", m_draw_coordinates);
					break;
				case SDLK_k:
					m_kalman_enabled = !m_kalman_enabled;
					if(m_kalman_enabled){
						pmax = m_particles->getMax();
						m_zk[0] = pmax->rX; m_zk[1] = pmax->rY; m_zk[2] = pmax->rZ; 
						m_zk[3] = pmax->tX; m_zk[4] = pmax->tY; m_zk[5] = pmax->tZ;
						m_kalman.init();
						m_kalman.setX(m_zk);
						printf("Kalman filter enabled\n");
					}else{
						printf("Kalman filter disabled\n");
					}
					break;
				case SDLK_l:
					m_lock = !m_lock;
					if(m_lock){
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
				case SDLK_y:
					float m[16];
					pmax = m_particles->getMax();
					pmax->getModelView(m);
					
					break;					
				case SDLK_z:
					printf("Zero particles\n");
					p = Particle(0.0);
					m_zk[0] = p.rX; m_zk[1] = p.rY; m_zk[2] = p.rZ; 
					m_zk[3] = p.tX; m_zk[4] = p.tY; m_zk[5] = p.tZ;
					m_kalman.init();
					m_kalman.setX(m_zk);
					m_particles->setAll(p);
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
	m_lock = true;
	m_showparticles = false;
	m_showmodel = true;
	m_kalman_enabled = true;
	m_cascaded = true;
	m_draw_coordinates = false;
	
	pFile = fopen("../Results/analysis/kalman.txt","w");
}

Tracker::~Tracker(){
}

// Initialise function (must be called before tracking)
bool Tracker::init(	int width, int height,
					int nop,
					float fovy,
					float cipX, float cipY, float cipZ,
					float n_r_max,
					float n_t_max,
					int cs, int cmm,
					float et,
					int vw, int vh,
					bool kal,
					bool dc)
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);
	params.number_of_particles = nop;
	params.camera_fovy = fovy;
	params.camera_initial_position_x = cipX;
	params.camera_initial_position_y = cipY;
	params.camera_initial_position_z = cipZ;
	params.noise_rot_max = n_r_max;
	params.noise_trans_max = n_t_max;
	params.cascade_stages = cs;
	params.cascade_mean_max = cmm;
	params.edge_tolerance = et;
	params.viewport_width = vw;
	params.viewport_height = vh;
	
	m_kalman_enabled = kal;
	m_draw_coordinates = dc;
	
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
							Particle p_estimate,		// position estimate of model (R,t)
							Particle& p_result)		// storage to write tracked position
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
	//Particle* p_max = m_particles->getMax();
	//if(p_estimate)
	//	p_max = p_estimate;
	
	
	// Setup cascading parameters
	int range;
	float distribution;
	int mean;
	range = params.number_of_particles/params.cascade_stages;
	m_tex_frame_ip->bind(1);
	
	// Cascaded particle filter (motion, likelihood, cascading)
	for(int i=0; i<params.number_of_particles; i+=range){
		distribution = (1-float(i)/float(params.number_of_particles));
		mean = floor(float(i+range) * params.cascade_mean_max/float(params.number_of_particles));
		
		particle_motion(distribution, NULL, GAUSS);
		particle_processing_texture(range, params.cascade_mean_max);
	}
	
	// Kalman filter
	if(m_kalman_enabled){
		Particle* pm = m_particles->getMax();
		m_zk[0] = pm->rX; m_zk[1] = pm->rY; m_zk[2] = pm->rZ; 
		m_zk[3] = pm->tX; m_zk[4] = pm->tY; m_zk[5] = pm->tZ; 
		m_kalman.run(m_zk, m_kalmantimer.Update(), m_xk);
		float apptime = m_kalmantimer.GetApplicationTime();
		
		// Put result to file
		//fprintf(	pFile,	"%f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
		//			apptime,
		//			pm->rX, pm->rY, pm->rZ, pm->tX, pm->tY, pm->tZ,
		//			m_xk[0], m_xk[1], m_xk[2], m_xk[3], m_xk[4], m_xk[5]);
			
		pm->rX = m_xk[0]; pm->rY = m_xk[1]; pm->rZ = m_xk[2]; 
		pm->tX = m_xk[3]; pm->tY = m_xk[4]; pm->tZ = m_xk[5]; 
	}
	
	
	// Copy result to output
	//if(p_result)
	//	*p_result = *m_particles->getMax();
	
	// Draw result
	draw_result_texture();
	if(m_draw_coordinates)
		renderCoordinates();
		
	time_tracking = m_timer.Update();
	
	SDL_GL_SwapBuffers();
	SDL_Delay(10);
	return inputs();
}


bool Tracker::trackEdge(	unsigned char* image,
							Model* model,
							Particle p_estimate,
							Particle& p_result)		// storage to write tracked position
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
	*p_max = p_estimate;
	
	// Setup cascading parameters
	int range;
	float distribution;
	int mean;
	range = params.number_of_particles/params.cascade_stages;
	
	// Draw particles and evaluate match likelihood
	for(int i=0; i<params.number_of_particles; i+=range){
		distribution = (1-float(i)/float(params.number_of_particles));
		mean = floor(float(i+range) * params.cascade_mean_max/float(params.number_of_particles));
		
		particle_motion(distribution, NULL, GAUSS);
		particle_processing_edge(range, params.cascade_mean_max);
	}
	
	// Kalman filter
	if(m_kalman_enabled){
		Particle* pm = m_particles->getMax();
		m_zk[0] = pm->rX; m_zk[1] = pm->rY; m_zk[2] = pm->rZ; 
		m_zk[3] = pm->tX; m_zk[4] = pm->tY; m_zk[5] = pm->tZ; 
		m_kalman.run(m_zk, m_kalmantimer.Update(), m_xk);
		float apptime = m_kalmantimer.GetApplicationTime();
		
		// Put result to file
		//fprintf(	pFile,	"%f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
		//			apptime,
		//			pm->rX, pm->rY, pm->rZ, pm->tX, pm->tY, pm->tZ,
		//			m_xk[0], m_xk[1], m_xk[2], m_xk[3], m_xk[4], m_xk[5]);
			
		pm->rX = m_xk[0]; pm->rY = m_xk[1]; pm->rZ = m_xk[2]; 
		pm->tX = m_xk[3]; pm->tY = m_xk[4]; pm->tZ = m_xk[5]; 
	}
	
	// Copy result to output
	p_result = *m_particles->getMax();
	
	// Draw result
	draw_result_edge();
	if(m_draw_coordinates)
		renderCoordinates();
	
	time_tracking = m_timer.Update();
		
	SDL_GL_SwapBuffers();
	
	return inputs();

}


void Tracker::renderCoordinates(){
	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	m_cam_perspective->Activate();
	
	float l1 = 0.06;
	float l2 = 0.02;
	float b1 = 0.001;
	float b2 = 0.003;
	
	// X - Axis
	glPushMatrix();
		glColor3f(1.0,0.0,0.0);
		//glRotatef(m_timer.GetApplicationTime() * PI * 100, 1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
		
	// Y - Axis
	glPushMatrix();
		glColor3f(0.0,1.0,0.0);
		glRotatef(90, 0.0, 0.0, 1.0);
		//glRotatef(m_timer.GetApplicationTime() * PI * 100, 1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
	
	// Z - Axis
	glPushMatrix();
		glColor3f(0.0,0.0,1.0);
		glRotatef(90, 0.0, 1.0, 0.0);
		//glRotatef(m_timer.GetApplicationTime() * PI * 100, 1.0,0.0,0.0);
		glBegin(GL_TRIANGLE_FAN);
			glVertex3f(l1,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	 b1);
			glVertex3f(0.0,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1);
			glVertex3f(l1,		0.0,	-b1-b2);
			glVertex3f(l1+l2,	0.0,	0.0);
			glVertex3f(l1,		0.0,	 b1+b2);
		glEnd();
	glPopMatrix();
	
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	
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
	fclose(pFile);
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

