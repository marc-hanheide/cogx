
#include "Tracker.h"

// *** PRIVATE ***

bool Tracker::isReady(unsigned char* image, Model* model, Camera* camera, Particle* p_estimate){
	if(!m_tracker_initialized){
		printf("[TextureTracker::isReady] Error tracker not initialized\n");
		return false;
	}
	
	if(!image){
		printf("[TextureTracker::isReady] Error image not valid\n");
		return false;
	}else
		m_image = image;
	
	if(!model){
		printf("[TextureTracker::isReady] Error model not valid\n");
		return false;
	}else
		m_model = model;
	
	if(!camera){
		printf("[TextureTracker::isReady] Warning camera not set, using default values\n");
		m_cam_perspective = m_cam_default;
	}else
		m_cam_perspective = camera;
	
	if(!p_estimate){
		printf("[TextureTracker::isReady] Error pose estimation not valid\n");
		return false;
	}
	
	return true;
}

// Applies Kalman filter to maximum particle
void Tracker::kalman_filtering(Particle* pm){
	m_zk[0] = pm->rX; m_zk[1] = pm->rY; m_zk[2] = pm->rZ; 
	m_zk[3] = pm->tX; m_zk[4] = pm->tY; m_zk[5] = pm->tZ;
	
	m_kalman.run(m_zk, m_kalmantimer.Update(), m_xk, m_kalman_gain[3]);
	
	// Put result to file
	//fprintf(	pFile,	"%f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
	//			apptime,
	//			pm->rX, pm->rY, pm->rZ, pm->tX, pm->tY, pm->tZ,
	//			m_xk[0], m_xk[1], m_xk[2], m_xk[3], m_xk[4], m_xk[5]);
	
	m_kalman_gain[3] = m_kalman_gain[2];
	m_kalman_gain[2] = m_kalman_gain[1];
	m_kalman_gain[1] = m_kalman_gain[0];
	m_kalman_gain[0] = pm->w*pm->w*pm->w;
	
	pm->rX = m_xk[0]; pm->rY = m_xk[1]; pm->rZ = m_xk[2]; 
	pm->tX = m_xk[3]; pm->tY = m_xk[4]; pm->tZ = m_xk[5]; 
}


// *** PUBLIC ***

Tracker::Tracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_kalman_enabled = true;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_testflag = false;
	m_bfc = true;
	
	// Textures
	int id;
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame")) == -1)
		exit(1);
	m_tex_frame = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_ip[0]")) == -1)
		exit(1);
	m_tex_frame_ip[0] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_ip[1]")) == -1)
		exit(1);
	m_tex_frame_ip[1] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_ip[2]")) == -1)
		exit(1);
	m_tex_frame_ip[2] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_frame_ip[3]")) == -1)
		exit(1);
	m_tex_frame_ip[3] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model")) == -1)
		exit(1);
	m_tex_model = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model_ip[0]")) == -1)
		exit(1);
	m_tex_model_ip[0] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model_ip[1]")) == -1)
		exit(1);
	m_tex_model_ip[1] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model_ip[2]")) == -1)
		exit(1);
	m_tex_model_ip[2] = g_Resources->GetTexture(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_tex_model_ip[3]")) == -1)
		exit(1);
	m_tex_model_ip[3] = g_Resources->GetTexture(id);
	
	// Cameras
	if((id = g_Resources->AddCamera("cam_ortho")) == -1)
		exit(1);
	m_cam_ortho = g_Resources->GetCamera(id);
	
	if((id = g_Resources->AddCamera("cam_default")) == -1)
		exit(1);
	m_cam_default = g_Resources->GetCamera(id);
	
}



// Initialise function (must be called before tracking)
bool Tracker::init(	int width, int height,							// image size in pixels
					int nop,										// maximum number of particles
					float n_r_max,									// standard deviation of rotational noise in degree
					float n_t_max,									// standard deviation of translational noise in meter
					float et,										// edge matching tolerance in degree
					float tt,										// goal tracking time in seconds
					Particle zp)
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);
	params.number_of_particles = nop;
	params.noise_rot_max = n_r_max;
	params.noise_trans_max = n_t_max;
	params.edge_tolerance = et;
	params.track_time = tt;
	params.zP = zp;
	
	if(!m_opengl.Init())
		return false;
		
	m_kalman_gain[3] = 1;
	m_kalman_gain[2] = 1;
	m_kalman_gain[1] = 1;
	m_kalman_gain[0] = 1;
	
	// Cameras
	m_cam_ortho->Set(	0.0, 0.0, 1.0,
						0.0, 0.0, 0.0,
						0.0, 1.0, 0.0,
						45, width, height,
						0.1, 10.0,
						GL_ORTHO);
	
	m_cam_default->Set(	0.2, 0.2, 0.2,
						0.0, 0.0, 0.0,
						0.0, 1.0, 0.0,
						45, width, height,
						0.1, 10.0,
						GL_PERSPECTIVE);
	
	m_cam_perspective = m_cam_default;
	
	// Singleton resources
	g_Resources->InitImageProcessor(width, height, m_cam_ortho);
	g_Resources->InitParticles(params.number_of_particles, Particle(0.0));
	g_Resources->InitFrustum();
	
	m_ip = g_Resources->GetImageProcessor();
	m_particles = g_Resources->GetParticles();	
	
	return initInternal();
}


// render coordinate frame
void Tracker::drawCoordinates(){
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
		glRotatef(-90, 0.0, 1.0, 0.0);
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
	m_opengl.RenderSettings(true, true);
	
}


void Tracker::drawImage(unsigned char* image){
	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	m_cam_ortho->Activate();
	glColor3f(1.0,1.0,1.0);
	if(image == NULL){
		m_ip->render(m_tex_frame);
	}else{
		m_tex_frame->load(image, params.width, params.height);
		m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
		m_ip->render(m_tex_frame);
	}
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	m_opengl.RenderSettings(true, true);
}

void Tracker::drawPixel(float u, float v, vec3 color, float size){
	m_opengl.RenderSettings(true, false);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	m_cam_ortho->Activate();
	glPointSize(size);
	
	glBegin(GL_POINTS);
		glColor3fv(color);
		glVertex3f(u, v, m_cam_ortho->GetZNear());
	glEnd();
	
	glPointSize(1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	m_opengl.RenderSettings(true, true);
}

void Tracker::drawTest(){
	m_opengl.RenderSettings(true, true);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	
	m_cam_perspective->Activate();
	glLineWidth(1.0);
	
	glBegin(GL_LINE_LOOP);
		glColor3f(1.0,0.0,0.0);
		glVertex3f(0.000, 0.000, 0.000);
		glVertex3f(0.240, 0.000, 0.000);
		glVertex3f(0.240, 0.120, 0.000);
		glVertex3f(0.200, 0.160, 0.000);
		glVertex3f(0.000, 0.160, 0.000);
	glEnd();
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

void Tracker::swap(){
	SDL_GL_SwapBuffers();
}


// Show performance and likelihood
void Tracker::showStatistics(){
	printf("\n\nStatistics:\n");
	printf("	Particles: %i\n", params.number_of_particles );
	printf("	Tracking time: %.0f ms\n", time_tracking * 1000.0 );
	printf("	Pixels of model: %i \n", m_particles->getVMax() );
	printf("	Likelihood p[%i]: %.0f %%\n", m_particles->getMaxID(), m_particles->getMax()->w * 100.0 );
}
	
void Tracker::zeroParticles(){
	Particle p = params.zP;
	m_zk[0] = p.rX; m_zk[1] = p.rY; m_zk[2] = p.rZ; 
	m_zk[3] = p.tX; m_zk[4] = p.tY; m_zk[5] = p.tZ;
	m_kalman.init();
	m_kalman.setX(m_zk);
	m_particles->setAll(p);
	m_particles->setVMax(0);
	m_zero_particles = true;
	printf("Zero particles\n");
}

void Tracker::enableKalman(bool val){
	if(val){
		Particle* pmax;
		pmax = m_particles->getMax();
		m_zk[0] = pmax->rX; m_zk[1] = pmax->rY; m_zk[2] = pmax->rZ; 
		m_zk[3] = pmax->tX; m_zk[4] = pmax->tY; m_zk[5] = pmax->tZ;
		m_kalman.init();
		m_kalman.setX(m_zk);
		m_kalman_enabled = true;
		printf("Kalman filter enabled\n");
	}else{
		m_kalman_enabled = false;
		printf("Kalman filter disabled\n");
	}
}



