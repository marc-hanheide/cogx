
#include "Tracker.h"

using namespace Tracking;

// *** PRIVATE ***

bool Tracker::isReady(unsigned char* image, TrackerModel* model, Camera* camera){
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
		m_cam_perspective = &m_cam_default;
	}else
		m_cam_perspective = camera;
	
	return true;
}

// *** PUBLIC ***

Tracker::Tracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_bfc = true;
	
	params.m_spreadlvl = 0;
	params.number_of_particles = 10;
	params.recursions = 1;
	params.minTexGrabAngle = 3.0*PI/4.0;
	
	params.m_tracker_id = g_Resources->GetNumTracker();
	g_Resources->AddTracker();
	
	int id;
	char name[FN_LEN];
	
	// Textures
	m_tex_frame = new Texture();

	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		m_tex_frame_ip[i] = new Texture();
	}	
}

Tracker::~Tracker(){
	delete(m_distribution);
	delete(m_tex_frame);
	
	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		delete(m_tex_frame_ip[i]);
	}	
}

bool Tracker::initGL(){
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
		
	const GLubyte *str;
	int glOcclusionQueryAvailable;
	
	// Check for OpenGL and GLSL
	
	// Check for Extension
	str = glGetString(GL_EXTENSIONS);
	glOcclusionQueryAvailable = (strstr((const char *)str, "GL_ARB_occlusion_query") != NULL);
	if(!glOcclusionQueryAvailable){ 
		printf("[OpenGLControl] Error OpenGL extension 'GL_ARB_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!\n");
		return false;
	}
		
	return true;
}

// Initialise function (must be called before tracking)
bool Tracker::init(	int width, int height,			// image size in pixels
										float et,										// edge matching tolerance in degree
										float mtga,
										Particle zp,								// zero particle, initial pose of tracker
										Particle ip)
{	
	// Parameter:
	params.width = float(width);
	params.height = float(height);	
	params.edge_tolerance = et;
	params.minTexGrabAngle = mtga;
	params.zP = zp;	
	
	// OpenGL
	initGL();
		
	// Cameras
	m_cam_default.Set(	0.2, 0.2, 0.2,
											0.0, 0.0, 0.0,
											0.0, 1.0, 0.0,
											45, width, height,
											0.1, 10.0,
											GL_PERSPECTIVE);
	
	m_cam_perspective = &m_cam_default;
	
	// Singleton resources
	g_Resources->InitImageProcessor(width, height);
	m_ip = g_Resources->GetImageProcessor();
	
	// Particles
	m_distribution = new Distribution();
	m_predictor.sample(m_distribution, params.number_of_particles, params.zP, ip);
	printf("particles: %d\n", params.number_of_particles);
	
	return initInternal();
}


// render coordinate frame
void Tracker::drawCoordinates(){
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

void Tracker::drawCalibrationPattern(){
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
	SDL_Delay(1);
}


// Show performance and likelihood
void Tracker::printStatistics(){
	printf("\n\nTracker %d:\n", params.m_tracker_id);
	printf("	Particles: %i\n", m_distribution->size() );
	printf("	Recursions: %i\n", params.recursions);
	printf("	Tracking time: %.0f ms, FPS: %.0f Hz\n", params.time_tracking * 1000.0, 1.0/params.time_tracking);
	printf("	Variance: %f \n", m_distribution->getVariance() );
	printf("	Confidence: %f \n", m_distribution->getMaxC());
	printf("	Weight: %f \n", m_distribution->getMaxW());
	printf("	Spreading Level: %d\n", params.m_spreadlvl);
}

void Tracker::drawSpeedBar(float h){
	Particle* p =  m_distribution->getMean();
	float speed = p->rp.x;
	
	vec2 pos = vec2(params.width*0.5-20,0.0);
	
	float w = 10.0;
		
	// Render settings
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	m_ip->setCamOrtho();
	
	glColor3f(1.0,0.0,0.0);
	glBegin(GL_QUADS);
		glVertex3f(pos.x, pos.y, 0.0);
		glVertex3f(pos.x+w, pos.y, 0.0);
		glVertex3f(pos.x+w, pos.y+h, 0.0);
		glVertex3f(pos.x, pos.y+h, 0.0);	
	glEnd();
	
	glLineWidth(2);
	if(abs(speed) < 0.01)
		glColor3f(0.0,0.0,1.0);
	else
		glColor3f(0.0,1.0,0.0);
		
	glBegin(GL_LINES);
		glVertex3f(pos.x-2, pos.y, 0.0);
		glVertex3f(pos.x+w+2, pos.y, 0.0);
	glEnd();
	
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	
}
	
void Tracker::reset(){
	m_zero_particles = true;
}



