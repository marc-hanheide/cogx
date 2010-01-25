
#include "Tracker.h"

using namespace Tracking;
using namespace std;

// *** PUBLIC ***

Tracker::Tracker(){
	// Default flags
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_draw_edges = false;
	m_tracker_initialized = false;
	
	// Default parameter
	params.width = 640.0;
	params.height = 480.0;
	params.m_spreadlvl = 0;
	params.model_id_count = 0;
	params.num_particles = 100;
	params.num_recursions = 2;
	params.edge_tolerance = 45.0 * PIOVER180;
	params.m_spreadlvl = 1;
	params.variation =Particle(0.1);
	params.minTexGrabAngle = 3.0*PI/4.0;
	params.modelPath = string("resources/model/");
	params.texturePath = string("resources/texture/");
	params.shaderPath = string("resources/shader/");

// 	m_cam_perspective.Set(	0.2, 0.2, 0.2,											// Position of camera relative to Object
// 													0.0, 0.0, 0.0,											// Point where camera looks at (world origin)
// 													0.0, 1.0, 0.0,											// Up vector (y-axis)
// 													45, params.width, params.height,		// field of view angle, image width and height
// 													0.1, 10.0,													// camera z-clipping planes (far, near)
// 													GL_PERSPECTIVE);										// Type of projection (GL_ORTHO, GL_PERSPECTIVE)

}

Tracker::~Tracker(){
	int i;
	delete(m_tex_frame);
	
	for(i=0; i<NUM_SPREAD_LOOPS; i++){
		delete(m_tex_frame_ip[i]);
	}
	
	for(i=0; i<m_modellist.size(); i++){
		delete(m_modellist[i]);
	}
	
	delete g_Resources;
}

bool Tracker::initGL(){
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
		
	const GLubyte *str;
	int glOcclusionQueryAvailable;
	
	// Check for Extension
	str = glGetString(GL_EXTENSIONS);
	glOcclusionQueryAvailable = (strstr((const char *)str, "GL_ARB_occlusion_query") != NULL);
	if(!glOcclusionQueryAvailable){ 
		printf("[OpenGLControl] Error OpenGL extension 'GL_ARB_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!\n");
		return false;
	}
		
	return true;
}

// load INI file of tracker
bool Tracker::loadINI(const char* inifile){
	CDataFile cdfParams;
	cdfParams.Load(inifile);
	
	// Camera Parameters
	params.camPar.zFar = cdfParams.GetFloat("zFar", "CameraParameters");
	params.camPar.zNear = cdfParams.GetFloat("zNear", "CameraParameters");
	params.camPar.fx = cdfParams.GetFloat("fx", "CameraParameters");
	params.camPar.fy = cdfParams.GetFloat("fy", "CameraParameters");
	params.camPar.cx = cdfParams.GetFloat("cx", "CameraParameters");
	params.camPar.cy = cdfParams.GetFloat("cy", "CameraParameters");
	params.camPar.k1 = cdfParams.GetFloat("k1", "CameraParameters");
	params.camPar.k2 = cdfParams.GetFloat("k2", "CameraParameters");
	params.camPar.k3 = cdfParams.GetFloat("k3", "CameraParameters");
	params.camPar.p1 = cdfParams.GetFloat("p1", "CameraParameters");
	params.camPar.p2 = cdfParams.GetFloat("p2", "CameraParameters");
	params.camPar.pos.x = cdfParams.GetFloat("pose.pos.x", "CameraParameters");
	params.camPar.pos.y = cdfParams.GetFloat("pose.pos.y", "CameraParameters");
	params.camPar.pos.z = cdfParams.GetFloat("pose.pos.z", "CameraParameters");
	vec3 vRot;
	vRot.x = cdfParams.GetFloat("pose.rot.x", "CameraParameters");
	vRot.y = cdfParams.GetFloat("pose.rot.y", "CameraParameters");
	vRot.z = cdfParams.GetFloat("pose.rot.z", "CameraParameters");
	params.camPar.rot.fromRotVector(vRot);
	
	// Constraints
	params.variation.rp.x = cdfParams.GetFloat("rp.x", "Constraints");
	params.variation.rp.y = cdfParams.GetFloat("rp.y", "Constraints");
	params.variation.rp.z = cdfParams.GetFloat("rp.z", "Constraints");
	params.variation.t.x 	= cdfParams.GetFloat("t.x", "Constraints");
	params.variation.t.y 	= cdfParams.GetFloat("t.y", "Constraints");
	params.variation.t.z 	= cdfParams.GetFloat("t.z", "Constraints");
	params.variation.tp.x = cdfParams.GetFloat("tp.x", "Constraints");
	params.variation.tp.y = cdfParams.GetFloat("tp.y", "Constraints");
	params.variation.tp.z = cdfParams.GetFloat("tp.z", "Constraints");
	params.variation.z 		= cdfParams.GetFloat("z", "Constraints");	
	params.variation.zp 	= cdfParams.GetFloat("zp", "Constraints");
	
	// Performance
	params.num_recursions = cdfParams.GetInt("recursions", "Performance");
	params.num_particles = cdfParams.GetInt("particles", "Performance");
	
	// Resource Path
	params.modelPath = cdfParams.GetString("ModelPath", "ResourcePath");
	params.texturePath = cdfParams.GetString("TexturePath", "ResourcePath");
	params.shaderPath = cdfParams.GetString("ShaderPath", "ResourcePath");
		
	// Other
	params.edge_tolerance = cdfParams.GetFloat("EdgeMatchingTolerance", "Other") * PIOVER180;
	params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "Other") * PIOVER180;
}

bool Tracker::init(const char* inifile, int width, int height){

	params.width = params.camPar.width = width;
	params.height = params.camPar.height = height;
	
	// Load parameter
	loadINI(inifile);
	
	// OpenGL
	g_Resources->InitScreen(params.width, params.height, "Tracker");
	initGL();
	
	// Set pathes to file resources
	g_Resources->SetModelPath(params.modelPath.c_str());
	g_Resources->SetTexturePath(params.texturePath.c_str());
	g_Resources->SetShaderPath(params.shaderPath.c_str());
	g_Resources->ShowLog(true);
	
	// Load camera parameter
	m_cam_perspective.Load(params.camPar);
	
	// Singleton resources
	g_Resources->InitImageProcessor(params.width, params.height);
	m_ip = g_Resources->GetImageProcessor();
	
	// Textures
	m_tex_frame = new Texture();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		m_tex_frame_ip[i] = new Texture();
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	}
	
	return initInternal();
}

int Tracker::addModel(Model& m, Pose& p, bool bfc){
	
	if(!m_tracker_initialized){
		printf("[Tracker::addModel()] Error tracker not initialised!\n");
		return -1;
	}
	
	ModelEntry* modelEntry = new ModelEntry();
	
	modelEntry->model.setBFC(bfc);
	modelEntry->model = m;
	modelEntry->predictor.sample(modelEntry->distribution, params.num_particles, p, params.variation);
	modelEntry->pose = p;
	modelEntry->initial_pose = p;
	modelEntry->id = params.model_id_count++;
	modelEntry->num_particles = params.num_particles;
	modelEntry->num_recursions = params.num_recursions;
	m_modellist.push_back(modelEntry);
	
	return modelEntry->id;
}

void Tracker::removeModel(int id){
	ModelEntryList::iterator it = m_modellist.begin();
	
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			delete(*it);
			m_modellist.erase(it);
		}
		it++;
	}
}

void Tracker::getModelPose(int id, Pose& p){
	ModelEntryList::iterator it = m_modellist.begin();
	
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			p = (*it)->pose;
			return;
		}
		it++;
	}
}

void Tracker::getModelInitialPose(int id, Pose& p){
	ModelEntryList::iterator it = m_modellist.begin();
	
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			p = (*it)->initial_pose;
			return;
		}
		it++;
	}
}

// render coordinate frame
void Tracker::drawCoordinates(){
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	m_cam_perspective.Activate();
	
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
	
	m_cam_perspective.Activate();
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
	printf("\n\nStatistics:\n");
// 	printf("	Particles: %i\n", m_distribution.size() );
	printf("	Recursions: %i\n", params.num_recursions);
// 	printf("	Tracking time: %.0f ms, FPS: %.0f Hz\n", params.time_tracking * 1000.0, 1.0/params.time_tracking);
// 	printf("	Variance: %f \n", m_distribution.getVariance() );
// 	printf("	Confidence: %f \n", m_distribution.getMaxC());
// 	printf("	Weight: %f \n", m_distribution.getMaxW());
	printf("	Spreading Level: %d\n", params.m_spreadlvl);
}

void Tracker::drawSpeedBar(float h){
// 	Particle p =  m_distribution.getMean();
// 	float speed = p.rp.x;
// 	
// 	vec2 pos = vec2(params.width*0.5-20,0.0);
// 	
// 	float w = 10.0;
// 		
// 	// Render settings
// 	glDisable(GL_DEPTH_TEST);
// 	glDisable(GL_CULL_FACE);
// 	m_ip->setCamOrtho();
// 	
// 	glColor3f(1.0,0.0,0.0);
// 	glBegin(GL_QUADS);
// 		glVertex3f(pos.x, pos.y, 0.0);
// 		glVertex3f(pos.x+w, pos.y, 0.0);
// 		glVertex3f(pos.x+w, pos.y+h, 0.0);
// 		glVertex3f(pos.x, pos.y+h, 0.0);	
// 	glEnd();
// 	
// 	glLineWidth(2);
// 	if(abs(speed) < 0.01)
// 		glColor3f(0.0,0.0,1.0);
// 	else
// 		glColor3f(0.0,1.0,0.0);
// 		
// 	glBegin(GL_LINES);
// 		glVertex3f(pos.x-2, pos.y, 0.0);
// 		glVertex3f(pos.x+w+2, pos.y, 0.0);
// 	glEnd();
// 	
// 	glEnable(GL_CULL_FACE);
// 	glEnable(GL_DEPTH_TEST);
	
}
	
void Tracker::reset(){
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i]->predictor.sample(m_modellist[i]->distribution, params.num_particles, m_modellist[i]->initial_pose, params.variation);
		m_modellist[i]->pose = m_modellist[i]->initial_pose;
	}
}



