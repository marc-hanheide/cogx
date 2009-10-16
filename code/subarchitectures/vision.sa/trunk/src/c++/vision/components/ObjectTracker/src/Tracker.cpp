
#include "Tracker.h"

// *** PRIVATE ***

bool Tracker::isReady(unsigned char* image, Model* model, Camera* camera){
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
	
	return true;
}

// *** PUBLIC ***

Tracker::Tracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = true;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_testflag = false;
	m_bfc = true;
	
	
	m_tracker_id = g_Resources->GetNumTracker();
	g_Resources->AddTracker();
	
	int id;
	char name[FN_LEN];
	
	// Textures
	sprintf(name, "m_tex_frame");
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_frame = g_Resources->GetTexture(id);
	
	sprintf(name, "m_tex_frame_ip[0]");
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_frame_ip[0] = g_Resources->GetTexture(id);
	
	sprintf(name, "m_tex_frame_ip[1]");
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_frame_ip[1] = g_Resources->GetTexture(id);
	
	sprintf(name, "m_tex_frame_ip[2]");
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_frame_ip[2] = g_Resources->GetTexture(id);
	
	sprintf(name, "m_tex_frame_ip[3]");
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_frame_ip[3] = g_Resources->GetTexture(id);
	
	/*
	sprintf(name, "T%d:m_tex_model", m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model = g_Resources->GetTexture(id);
	
	sprintf(name, "T%d:m_tex_model_ip[0]", m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model_ip[0] = g_Resources->GetTexture(id);
	
	sprintf(name, "T%d:m_tex_model_ip[1]", m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model_ip[1] = g_Resources->GetTexture(id);
	
	sprintf(name, "T%d:m_tex_model_ip[2]", m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model_ip[2] = g_Resources->GetTexture(id);
	
	sprintf(name, "T%d:m_tex_model_ip[3]", m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model_ip[3] = g_Resources->GetTexture(id);
	*/
	
	// Cameras
	sprintf(name, "cam_default");
	if((id = g_Resources->AddCamera(name)) == -1)
		exit(1);
	m_cam_default = g_Resources->GetCamera(id);
	
}



// Initialise function (must be called before tracking)
bool Tracker::init(	int width, int height,							// image size in pixels
					int nop,										// maximum number of particles
					int rec,										// recursions per image
					float n_r_max,									// standard deviation of rotational noise in degree
					float n_t_max,									// standard deviation of translational noise in meter
					float et,										// edge matching tolerance in degree
					float tt,										// goal tracking time in seconds
					Particle zp)									// zero particle, initial pose of tracker
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);
	params.number_of_particles = nop;
	params.recursions = rec;
	params.noise_rot_max = n_r_max;
	params.noise_trans_max = n_t_max;
	params.noise_scale_max = n_t_max * 0.0;
	params.edge_tolerance = et;
	params.track_time = tt;
	params.zP = zp;
	
	if(!m_opengl.Init())
		return false;
		
	// Cameras
	m_cam_default->Set(	0.2, 0.2, 0.2,
						0.0, 0.0, 0.0,
						0.0, 1.0, 0.0,
						45, width, height,
						0.1, 10.0,
						GL_PERSPECTIVE);
	
	m_cam_perspective = m_cam_default;
	
	// Singleton resources
	g_Resources->InitImageProcessor(width, height);
	g_Resources->InitFrustum();
	
	m_ip = g_Resources->GetImageProcessor();
	
	// Particles
	int id=0;
	char name[FN_LEN];
	sprintf(name, "T%d:particles", m_tracker_id);
	if((id = g_Resources->AddParticles(params.number_of_particles, params.zP, name)) == -1)
		exit(1);
	m_particles = g_Resources->GetParticles(id);
	
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
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	if(image == NULL){
		if(m_draw_edges)
			m_ip->render(m_tex_frame_ip[0]);
		else
			m_ip->render(m_tex_frame);
	}else{
		m_tex_frame->load(image, params.width, params.height);
		m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
		m_ip->render(m_tex_frame);
	}
	
	glDepthMask(1);
}

/* void Tracker::drawPixel(float u, float v, vec3 color, float size){
// TODO move to Image Processor
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
}*/

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
	printf("\n\nTracker %d:\n", m_tracker_id);
	printf("	Particles: %i\n", params.number_of_particles );
	printf("	Tracking time: %.0f ms\n", time_tracking * 1000.0 );
	printf("	Variance: %f \n", m_particles->getVariance(params.number_of_particles) );
	printf("	Confidence w: %f \n", m_particles->getMaxW(params.number_of_particles));
	//printf("	Likelihood p[%i]: %f %%\n", m_particles->getMaxID(), m_particles->getMax(params.number_of_particles)->w * 100.0 );
}
	
void Tracker::zeroParticles(){
	m_zero_particles = true;
	//printf("Zero particles\n");
}



