
#include "Tracker.h"

// *** PRIVATE ***

bool Tracker::isReady(unsigned char* image, Model* model, Camera* camera, Pose* p_estimate){
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


// *** PUBLIC ***

Tracker::Tracker(){
	m_lock = false;
	m_showmodel = true;
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
					float tt, Pose zp)										// goal tracking time in seconds
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);
	params.track_time = tt;
	params.zP = zp;
	
	if(!m_opengl.Init())
		return false;
		
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
	g_Resources->InitOrthoSearch(params.width, params.height);
	g_Resources->InitFrustum();
	
	m_ip = g_Resources->GetImageProcessor();
	m_ortho_search = g_Resources->GetOrthoSearch();
	
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
	
	m_tex_frame->load(image, params.width, params.height);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->render(m_tex_frame);
	
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
		glVertex3f(0.225, 0.000, 0.000);
		glVertex3f(0.225, 0.113, 0.000);
		glVertex3f(0.188, 0.151, 0.000);
		glVertex3f(0.000, 0.151, 0.000);
	glEnd();
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

void Tracker::swap(){
	SDL_GL_SwapBuffers();
}


// Show performance and likelihood
void Tracker::showStatistics(){
	//printf("\n\nStatistics:\n");
	//printf("	Particles: %i\n", params.number_of_particles );
	//printf("	Tracking time: %.0f ms\n", time_tracking * 1000.0 );
	//printf("	Iterations: %i\n", m_iterations);
	//printf("	Pixels of model: %i \n", m_particles->getVMax() );
	//printf("	Likelihood p[%i]: %.0f %%\n", m_particles->getMaxID(), m_particles->getMax()->w * 100.0 );
}
	
void Tracker::zeroPose(){
	m_zero_pose = true;
	printf("Zero particles\n");
}




