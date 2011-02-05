
#include "Tracker.h"

using namespace Tracking;
using namespace std;

// *** PUBLIC ***

Tracker::Tracker(){
	// Default flags
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 1;
	m_draw_edges = false;
	m_tracker_initialized = false;
	
	// Default parameter
	params.width = 640.0;
	params.height = 480.0;
	params.m_spreadlvl = 0;
	params.model_id_count = 0;
	params.hypotheses_id_count = 0;
	params.num_particles = 100;
	params.num_recursions = 2;
	params.convergence = 5;
	params.edge_tolerance = 45.0 * PIOVER180;
	params.m_spreadlvl = 0;
	params.kernel_size = 0;
	params.variation =Particle(0.1);
	params.minTexGrabAngle = 3.0*PI/4.0;
	params.modelPath = string("resources/model/");
	params.texturePath = string("resources/texture/");
	params.shaderPath = string("resources/shader/");
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
	
	if(!cdfParams.Load(inifile))
		return false;
	
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
	params.variation.r.x = cdfParams.GetFloat("r.x", "Constraints") * PIOVER180;
	params.variation.r.y = cdfParams.GetFloat("r.y", "Constraints") * PIOVER180;
	params.variation.r.z = cdfParams.GetFloat("r.z", "Constraints") * PIOVER180;
	params.variation.rp.x = cdfParams.GetFloat("rp.x", "Constraints") * PIOVER180;
	params.variation.rp.y = cdfParams.GetFloat("rp.y", "Constraints") * PIOVER180;
	params.variation.rp.z = cdfParams.GetFloat("rp.z", "Constraints") * PIOVER180;
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
	params.hypotheses_trials = cdfParams.GetInt("hypotheses", "Performance");
	params.convergence = cdfParams.GetInt("convergence", "Performance");
	
	// Resource Path
	params.modelPath = cdfParams.GetString("ModelPath", "ResourcePath");
	params.texturePath = cdfParams.GetString("TexturePath", "ResourcePath");
	params.shaderPath = cdfParams.GetString("ShaderPath", "ResourcePath");
		
	// Other
	params.edge_tolerance = cdfParams.GetFloat("EdgeMatchingTolerance", "Other") * PIOVER180;
	params.minTexGrabAngle = cdfParams.GetFloat("MinTextureGrabAngle", "Other") * PIOVER180;
	
	return true;
}


void Tracker::setFrameTime(double dTime){
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i]->predictor->updateTime(dTime);
	}
}

bool Tracker::setCameraParameters(CameraParameter cam_par){
	
	if(cam_par.zFar <= 0.0){
		printf("[Tracker::setCameraParameters] Error 'Far Clipping Plane' not valid: %f", cam_par.zFar);
		return false;
	}
	if(cam_par.zNear < 0.0){
		printf("[Tracker::setCameraParameters] Error 'Near Clipping Plane' not valid: %f", cam_par.zNear);
		return false;
	}
	
	if(cam_par.zNear >= cam_par.zFar){
		printf("[Tracker::setCameraParameters] Error 'Clipping Panes' not valid: %f > %f", cam_par.zNear, cam_par.zFar);
		return false;
	}
	
	m_cam_perspective.Load(cam_par);
	return true;
}

bool Tracker::init(const char* inifile, int width, int height){

	params.width = params.camPar.width = width;
	params.height = params.camPar.height = height;
	
	// Load parameter
	if(!loadINI(inifile))
		return false;

	// OpenGL
	g_Resources->InitScreen(params.width, params.height, "Tracker");
	initGL();
	
	// Set pathes to file resources
	g_Resources->SetModelPath(params.modelPath.c_str());
	g_Resources->SetTexturePath(params.texturePath.c_str());
	g_Resources->SetShaderPath(params.shaderPath.c_str());
	g_Resources->ShowLog(false);
	
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


int Tracker::addModel(Model& m, Pose& p, std::string label, bool bfc){
	if(!m_tracker_initialized){
		printf("[Tracker::addModel()] Error tracker not initialised!\n");
		return -1;
	}
// 	static int idx = 0;
	
	ModelEntry* modelEntry = new ModelEntry();
	modelEntry->label = label;
	modelEntry->model.setBFC(bfc);
	
	modelEntry->model = m;
	
// 	printf("%d\n", idx);
	
	modelEntry->pose = p;
	modelEntry->initial_pose = p;
	modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
	modelEntry->id = params.model_id_count++;
	modelEntry->num_particles = params.num_particles;
	modelEntry->num_recursions = params.num_recursions;
	m_modellist.push_back(modelEntry);
	
	return modelEntry->id;
}

int Tracker::addModelFromFile(const char* filename, Pose& p, std::string label, bool bfc){
	if(!m_tracker_initialized){
		printf("[Tracker::addModel()] Error tracker not initialised!\n");
		return -1;
	}
	
	ModelEntry* modelEntry = new ModelEntry();
	modelEntry->label = label;
	modelEntry->model.setBFC(bfc);
	
	ModelLoader modelloader;
	modelloader.LoadPly(modelEntry->model, filename);
	
	modelEntry->pose = p;
	modelEntry->initial_pose = p;
	modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
	modelEntry->id = params.model_id_count++;
	modelEntry->num_particles = params.num_particles;
	modelEntry->num_recursions = params.num_recursions;
	m_modellist.push_back(modelEntry);
	
	return modelEntry->id;
}

void Tracker::addPoseHypothesis(int id, Pose &p, std::string label, bool bfc){
	if(!m_tracker_initialized){
		printf("[Tracker::addHypothesis()] Error tracker not initialised!\n");
		return;
	}
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			ModelEntry* modelEntry = new ModelEntry();
			modelEntry->label = label;
			modelEntry->model.setBFC(bfc);
			modelEntry->model = m_modellist[id]->model;  // TODO maybe buggy
			modelEntry->pose = p;
			modelEntry->initial_pose = p;
			modelEntry->predictor->sample(modelEntry->distribution, params.num_particles, p, params.variation);
			modelEntry->id = params.hypotheses_id_count++;
			modelEntry->num_particles = params.num_particles;
			modelEntry->num_recursions = params.num_recursions;
			modelEntry->hypothesis_id = id;
			m_hypotheses.push_back(modelEntry);
			return;
		}
		it++;
	}
	printf("[Tracker::addHypothesis()] Warning model for hypothesis not found: %d - %d \n", id, m_modellist.size());
}

void Tracker::removeModel(int id){
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			delete(*it);
			m_modellist.erase(it);
			return;
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

void Tracker::getModelConfidence(int id, float& c){
	ModelEntryList::iterator it = m_modellist.begin();
	
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			c = (*it)->pose.c;
			return;
		}
		it++;
	}
}

bool Tracker::getModelPoint3D(int id, int x_win, int y_win, float& x3, float& y3, float& z3){
	ModelEntryList::iterator it;
	
	for(it = m_modellist.begin(); it < m_modellist.end(); it++){
		if(id==(*it)->id){
			// Activate Camera
			Camera cam = m_cam_perspective;
			cam.SetZRange(0.0, 1.0);
			cam.Activate();
			
			// Clear Depth Buffer
			glClear(GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			
			// Apply pose
			(*it)->pose.activate();
			
			// Draw Model Faces
			m_lighting.Activate();
			(*it)->model.drawFaces();
			m_lighting.Deactivate();
			
			// ************************
			int viewport[4];
			double modelview[16];
			double projection[16];
			double result[3];
			
			glGetDoublev(GL_MODELVIEW_MATRIX, &modelview[0] );
			glGetDoublev(GL_PROJECTION_MATRIX, &projection[0] );
			glGetIntegerv(GL_VIEWPORT, &viewport[0] );
			y_win = viewport[3] - y_win;	// flip y for OpenGL
		
			// Read value of depth buffer at position (x_win, y_win)
			glReadPixels(x_win, y_win, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z3 );
			
			if(z3 > 0.99)
				return false;
			
			// calculate intersection of camera viewing vector and model surface
			gluUnProject((double)x_win, (double)y_win, (double)z3, modelview, projection, viewport, &result[0], &result[1], &result[2]); 
			
			x3 = result[0];
			y3 = result[1];
			z3 = result[2];
			
			(*it)->pose.deactivate();
			
			return true;			
		}
	}
	return false;
}

void Tracker::setModelInitialPose(int id, Pose& p){
	ModelEntryList::iterator it = m_modellist.begin();
	
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			(*it)->initial_pose = p;
			return;
		}
		it++;
	}
}

void Tracker::setModelPredictor(int id, Predictor* predictor){
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			(*it)->predictor = predictor;			
			return;
		}
		it++;
	}
}

void Tracker::setModelLock(int id, bool lock){
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			(*it)->lock = lock;
			return;
		}
		it++;
	}
}

void Tracker::saveModel(int id, const char* pathname){
	ModelLoader modelloader;
	ModelEntryList::iterator it = m_modellist.begin();
	string name;
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			name = string(pathname);
			name.append((*it)->label);
			modelloader.SavePly((*it)->model, name.c_str());
			return;
		}
		it++;
	}
}

void Tracker::saveModels(const char* pathname){
	ModelLoader modelloader;
	ModelEntryList::iterator it = m_modellist.begin();
	string name;
	while(it != m_modellist.end()){
		name = string(pathname);
		name.append((*it)->label);
		modelloader.SavePly((*it)->model, name.c_str());
		it++;
	}
}

void Tracker::saveScreenshot(const char* filename){
	IplImage* img = cvCreateImage ( cvSize ( params.width, params.height ), IPL_DEPTH_8U, 3 );
	glReadPixels(0,0,params.width,params.height,GL_RGB,GL_UNSIGNED_BYTE, img->imageData);
	cvConvertImage(img, img, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	cvSaveImage(filename, img);
	cvReleaseImage(&img);
}

void Tracker::setLockFlag(bool val){
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		(*it)->lock = val;
		it++;
	}
	m_lock = val;
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
	glColor3f(1.0,0.0,0.0);
	
	glBegin(GL_LINE_LOOP);
		
		glVertex3f(0.000, 0.000, 0.000);
		glVertex3f(0.240, 0.000, 0.000);
		glVertex3f(0.240, 0.120, 0.000);
		glVertex3f(0.200, 0.160, 0.000);
		glVertex3f(0.000, 0.160, 0.000);
	glEnd();
}

void Tracker::drawPixel(float u, float v, vec3 color, float size){
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	m_ip->setCamOrtho();
	
	glPointSize(size);
	glColor3f(color.x, color.y, color.z);
	
	glBegin(GL_POINTS);
		glVertex3f(u,v,0.0);
	glEnd();
}

void Tracker::swap(){
	int err = glGetError();
	switch(err){
		case GL_NO_ERROR:
			break;
		case GL_INVALID_ENUM:
			printf("glGetError: GL_INVALID_ENUM\n");
			break;
		case GL_INVALID_VALUE:
			printf("glGetError: GL_INVALID_VALUE\n");
			break;
		case GL_INVALID_OPERATION:
			printf("glGetError: GL_INVALID_OPERATION\n");
			break;
		case GL_STACK_OVERFLOW:
			printf("glGetError: GL_STACK_OVERFLOW\n");
			break;
		case GL_STACK_UNDERFLOW:
			printf("glGetError: GL_STACK_UNDERFLOW\n");
			break;
		case GL_OUT_OF_MEMORY:
			printf("glGetError: GL_OUT_OF_MEMORY\n");
			break;
		default:
			printf("glGetError: no known error\n");
			break;
	}
	
	SDL_GL_SwapBuffers();
	SDL_Delay(1);
}


// Show performance and likelihood
void Tracker::printStatistics(){
	printf("\n\nStatistics: \n");

	for(int i=0; i<m_modellist.size(); i++){
		Pose pMean =  m_modellist[i]->distribution.getMean();
		printf("	Object %d '%s'\n", i, m_modellist[i]->label.c_str());
		printf("		Textured: %d\n", m_modellist[i]->model.m_textured);
		printf("		Recursions: %i\n", m_modellist[i]->num_recursions );
		printf("		Particles: %i\n", m_modellist[i]->distribution.size() );
		printf("		Mean Pose: %f %f %f\n", pMean.t.x, pMean.t.y, pMean.t.z);
		printf("		Variance: %f \n", m_modellist[i]->distribution.getVariance() );
		printf("		Confidence: %f \n", m_modellist[i]->distribution.getMaxC());
		printf("		Weight: %f \n", m_modellist[i]->distribution.getMaxW());
		printf("		Spread: %d\n", params.m_spreadlvl);
		printf("		Kernel: %d \n", params.kernel_size);
	}
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
		m_modellist[i]->predictor->sample(m_modellist[i]->distribution, params.num_particles, m_modellist[i]->initial_pose, params.variation);
		m_modellist[i]->pose = m_modellist[i]->initial_pose;
	}
}

void Tracker::reset(int id){
	if(id >= 0 && id<m_modellist.size()){
		m_modellist[id]->predictor->sample(m_modellist[id]->distribution, params.num_particles, m_modellist[id]->initial_pose, params.variation);
		m_modellist[id]->pose = m_modellist[id]->initial_pose;
	}
}



