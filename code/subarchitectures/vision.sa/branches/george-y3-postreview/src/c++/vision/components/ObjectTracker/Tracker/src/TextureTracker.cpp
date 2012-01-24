
#include "TextureTracker.h"


using namespace std;
using namespace Tracking;

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image){
	
	// Load camera image to texture
	m_image = image;
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], 0.03, true);
	for(int i=1; i<NUM_SPREAD_LOOPS; i++)
		m_ip->spreading(m_tex_frame_ip[i-1], m_tex_frame_ip[i]);
	
	glDepthMask(1);
}

// Particle filtering
void TextureTracker::particle_filtering(ModelEntry* modelEntry){
	Particle p;
	int num_particles;
	m_cam_perspective.Activate();
	
	// Calculate Zoom Direction and pass it to the particle filter
	TM_Vector3 vCam = m_cam_perspective.GetPos();
	TM_Vector3 vObj = TM_Vector3(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z);
	modelEntry->vCam2Model = vObj - vCam;
	modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
	
	float c_max = modelEntry->distribution.getMaxC();

	for(int i=0; i<modelEntry->num_recursions; i++){
		
		// TODO Evaluate if this makes sense (robustness, accuracy)
		if(modelEntry->model.getTextured()){
			params.m_spreadlvl = (int)floor((NUM_SPREAD_LOOPS-1)*2.0*(1.0-c_max));
			if(params.m_spreadlvl>=NUM_SPREAD_LOOPS)
				params.m_spreadlvl = NUM_SPREAD_LOOPS-1;
			else if(params.m_spreadlvl<0)
				params.m_spreadlvl = 0;
		}else{
			params.m_spreadlvl = 0;
		}
		
		// TODO Evaluate if this makes sense (robustness, accuracy)
// 		params.kernel_size = (int)floor(0.5*(MAX_KERNEL_SIZE)*(1.0-c_max));
// 		m_shadeCompare->bind();
// 		m_shadeCompare->setUniform("kernelsize", params.kernel_size);
// 		m_shadeCompare->unbind();
		
		m_tex_model_ip[params.m_spreadlvl]->bind(0);
		m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
		m_tex_model->bind(2);
		m_tex_frame->bind(3);
				
		// update importance weights and confidence levels
		modelEntry->distribution.updateLikelihood(modelEntry->model, m_shadeCompare, 1, params.convergence, m_showparticles);

// 		printf("%f\n", ( 1.0 - i * 0.5 / (float)(modelEntry->num_recursions-1)));
		// TODO Evaluate if this makes sense (accuracy)
// 		p = (params.variation * ( 1.0 - i * 0.8 / (float)(modelEntry->num_recursions-1)));
		
		// predict movement of object
		num_particles = modelEntry->num_particles * (1.0 - c_max);
// 		printf("%d\n", num_particles);
		modelEntry->predictor->resample(modelEntry->distribution, num_particles, params.variation);
		
		// set timestep to 0.0 for further recursion (within same image)
		modelEntry->predictor->updateTime(0.0);
	}
	
	// weighted mean	
	modelEntry->pose = modelEntry->distribution.getMean();
}

// evaluate single particle
void TextureTracker::evaluateParticle(ModelEntry* modelEntry){
	unsigned int queryMatches;
	unsigned int queryEdges;
	int v, d;
	int id;
	
	glGenQueriesARB(1, &queryMatches);
	glGenQueriesARB(1, &queryEdges);

	m_cam_perspective.Activate();
	//glViewport(0,0,256,256);
	glColorMask(0,0,0,0); glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

// 	m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
// 	modelEntry->model.setTexture(m_tex_model_ip[params.m_spreadlvl]);
// 	
	m_tex_model_ip[params.m_spreadlvl]->bind(0);
	m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
	m_tex_model->bind(2);
	m_tex_frame->bind(3);
	
	// Draw particles and count pixels
	m_shadeCompare->bind();
	m_shadeCompare->setUniform("analyze", false);
    
	modelEntry->pose.activate();
	
		m_shadeCompare->setUniform("compare", false);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		modelEntry->model.drawTexturedFaces();
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		m_shadeCompare->setUniform("compare", true);
		m_shadeCompare->setUniform("textured", true);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches);
		modelEntry->model.drawTexturedFaces();
		m_shadeCompare->setUniform("textured", false);
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
	
	modelEntry->pose.deactivate();

	m_shadeCompare->unbind();

	glGetQueryObjectivARB(queryMatches, GL_QUERY_RESULT_ARB, &d);
	glGetQueryObjectivARB(queryEdges, GL_QUERY_RESULT_ARB, &v);

	if(v != 0){
		modelEntry->pose.c = (float(d)/float(v));
	}else{
		modelEntry->pose.c = 0.0;
	}
	
	glColorMask(1,1,1,1); glDepthMask(1);
	glDeleteQueriesARB(1, &queryMatches);
	glDeleteQueriesARB(1, &queryEdges);
}

// Draw TrackerModel to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(ModelEntry* modelEntry){
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	
	// Render camera image as background
	glDepthMask(0);
	m_ip->render(m_tex_frame);
	glDepthMask(1);
	
	glEnable(GL_DEPTH_TEST);
		
	// Setup camera, lighting and pose and rendering
	m_cam_perspective.Activate();			// activate perspective view
	m_lighting.Activate();
	
	modelEntry->pose.activate();
	
	// Render textured model to screen
	modelEntry->model.restoreTexture();
	modelEntry->model.drawPass();
	
	// Extract modelview-projection matrix
	mat4 modelview, projection;
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
	modelEntry->modelviewprojection = projection * modelview;
	
	// pass new modelview matrix to shader
	m_shadeCompare->bind();
	m_shadeCompare->setUniform("modelviewprojection", modelEntry->modelviewprojection, GL_FALSE); // send matrix to shader
	m_shadeCompare->unbind();
	
	// deactivate particles, lighting
	modelEntry->pose.deactivate();
	m_lighting.Deactivate();
		
	// Copy screen to texture (=reprojection to model)
	m_tex_model->copyTexImage2D(params.width, params.height);
		
	// perform image processing with reprojected image
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	m_ip->gauss(m_tex_model, m_tex_model_ip[0]);
	m_ip->sobel(m_tex_model_ip[0], m_tex_model_ip[0], 0.03, true);
	for(int i=1; i<NUM_SPREAD_LOOPS; i++)
		m_ip->spreading(m_tex_model_ip[i-1], m_tex_model_ip[i]);
	glDepthMask(1);
}


// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_drawimage = false;
}

TextureTracker::~TextureTracker(){
	delete(m_tex_model);
	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		delete(m_tex_model_ip[i]);
	}
}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal(){	
	
	int id;

	// Shader
	if((id = g_Resources->AddShader("texEdgeTest", "texEdgeTest.vert", "texEdgeTest.frag")) == -1)
		exit(1);
	m_shadeTexEdgeTest = g_Resources->GetShader(id);
	m_shadeCompare = m_shadeTexEdgeTest;
	
	if((id = g_Resources->AddShader("texColorTest", "texColorTest.vert", "texColorTest.frag")) == -1)
		exit(1);
	m_shadeTexColorTest = g_Resources->GetShader(id);
	
	// Texture
	m_tex_model = new Texture();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		m_tex_model_ip[i] = new Texture();
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	}
	
	// Load 
	float w = (float)params.width;
	float h = (float)params.height;
	
	GLfloat offX[9] = { -1.0/w, 0.0, 1.0/w,
											-1.0/w, 0.0, 1.0/w,
											-1.0/w, 0.0, 1.0/w };
	GLfloat offY[9] = {  1.0/h, 1.0/h,  1.0/h,
												0.0,   0.0,    0.0,
											-1.0/h,-1.0/h, -1.0/h };
                        
	m_shadeTexEdgeTest->bind();
	m_shadeTexEdgeTest->setUniform("tex_model_edge", 0);
	m_shadeTexEdgeTest->setUniform("tex_frame_edge", 1);
	m_shadeTexEdgeTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexEdgeTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
  m_shadeTexEdgeTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
  m_shadeTexEdgeTest->setUniform("drawcolor", vec4(1.0,0.0,0.0,0.0));
  m_shadeTexEdgeTest->setUniform("kernelsize", (GLint)1);
	m_shadeTexEdgeTest->unbind();
	
	m_shadeTexColorTest->bind();
	m_shadeTexColorTest->setUniform("tex_model_color", 2);
	m_shadeTexColorTest->setUniform("tex_frame_color", 3);
	m_shadeTexColorTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexColorTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
  m_shadeTexColorTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
  m_shadeTexColorTest->setUniform("drawcolor", vec4(0.0,0.0,1.0,0.0));
  m_shadeTexColorTest->setUniform("kernelsize", (GLint)1);
	m_shadeTexColorTest->unbind();
	
	tgMaterial matSilver;
	matSilver.ambient = vec4(0.19,0.19,0.19,1.0);
	matSilver.diffuse = vec4(0.51,0.51,0.51,1.0);
	matSilver.specular = vec4(0.77,0.77,0.77,1.0);
	matSilver.shininess = 51.2;
	m_lighting.ApplyMaterial(matSilver);
	
	tgLight light0;
	light0.ambient = vec4(0.3,0.3,0.3,1.0);
	light0.diffuse = vec4(1.0,1.0,1.0,1.0);
	light0.specular = vec4(0.2,0.2,0.2,1.0);
	light0.position = vec4(1.0,1.0,1.0,0.0);
	m_lighting.ApplyLight(light0,0);
	
	tgLight light1;
	light1.ambient = vec4(0.3,0.3,0.3,1.0);
	light1.diffuse = vec4(0.5,0.5,1.0,1.0);
	light1.specular = vec4(0.1,0.1,0.2,1.0);
	light1.position = vec4(-1.0,0.0,1.0,0.0);
	m_lighting.ApplyLight(light1,1);
	
	return (m_tracker_initialized = true);
}

bool TextureTracker::track(){
	if(!m_tracker_initialized){
		printf("[TextureTracker::track()] Error tracker not initialised!\n");
		return false;
	}
	
	// Track models
	for(int i=0; i<m_modellist.size(); i++){
		track(m_modellist[i]);
	}
	
	// Track hypothesis
	ModelEntryList::iterator m1 = m_hypotheses.begin();
	while(m1 < m_hypotheses.end()){
		track((*m1));
		if((*m1)->num_convergence++>params.hypotheses_trials){
			if((*m1)->past_confidences.size() < params.hypotheses_trials){
				
				(*m1)->past_confidences.push_back((*m1)->distribution.getMaxC());
				m_modellist[(*m1)->hypothesis_id]->past_confidences.push_back(m_modellist[(*m1)->hypothesis_id]->distribution.getMaxC());
			}else{
				
				// Evaluate mean confidence of hypothesis
				float c_hyp = 0.0;
				int s = (*m1)->past_confidences.size();
				for(int j=0; j<s; j++){
					c_hyp += (*m1)->past_confidences[j];
				}
				if(s>0)
					c_hyp = c_hyp / (float)s;
				
				// Evaluate mean confidence of model, the hypothesis belongs to
				float c_model = 0.0;
				s = m_modellist[(*m1)->hypothesis_id]->past_confidences.size();
				for(int j=0; j<s; j++){
					c_model += m_modellist[(*m1)->hypothesis_id]->past_confidences[j];
				}
				if(s>0)
					c_model = c_model / (float)s;
				
				// Compare confidence of model to the hypothesis
				if(c_model >= c_hyp){
					// if model is more confident, delete hypothesis
					delete(*m1);
					m_hypotheses.erase(m1);
				}else{
					// if hypothesis is more confident, delete model and replace modellist-entry with hypothesis
					delete(m_modellist[(*m1)->hypothesis_id]);
					(*m1)->id = (*m1)->hypothesis_id;
					m_modellist[(*m1)->hypothesis_id] = (*m1);
					m_hypotheses.erase(m1);
				}
			}
		}
		m1++;
	}
	
	return true;
}

bool TextureTracker::track(ModelEntry *modelEntry){

	// Process model (texture reprojection, edge detection)
	model_processing(modelEntry);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	if(m_drawimage)
		drawImage(0);
	
	// Apply particle filtering
	if(!modelEntry->lock){
		particle_filtering(modelEntry);
		if(!m_cam_perspective.GetFrustum()->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z))
			reset();
	}else{
		evaluateParticle(modelEntry);
	}
	
	return true;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(bool use_num_pixels){
	std::vector<Vertex> vertices;

	for(int i=0; i<m_modellist.size(); i++){

		m_cam_perspective.Activate();

		vector<int> faceUpdateList = m_modellist[i]->model.getFaceUpdateList(m_modellist[i]->pose, 
					vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
					params.minTexGrabAngle,
					use_num_pixels);

		if(!faceUpdateList.empty()){
			vertices.clear();
			m_modellist[i]->model.textureFromImage(	m_tex_frame,
																							params.width, params.height,
																							m_modellist[i]->pose,
																							vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
																							params.minTexGrabAngle,
																							faceUpdateList,
																							vertices, 
																							&m_cam_perspective);

		}
		faceUpdateList.clear();
	}
}

void TextureTracker::untextureModels(){
	for(int i=0; i<m_modellist.size(); i++){
		m_modellist[i]->model.releasePassList();	
	}
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(){
	bool texmodel = false;
	glColor3f(1.0,1.0,1.0);
	
	m_cam_perspective.Activate();
	m_lighting.Activate();

	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	for(int i=0; i<m_modellist.size(); i++){
		drawModelEntry(m_modellist[i]);
	}
	
// 	for(int i=0; i<m_hypotheses.size(); i++){
// 		drawModelEntry(m_hypotheses[i]);
// 	}
	
	m_lighting.Deactivate();
}

void TextureTracker::drawModelEntry(ModelEntry* modelEntry){

		modelEntry->pose.activate();
		
		switch(m_showmodel){
			case 0:
				modelEntry->model.restoreTexture();
				modelEntry->model.drawPass();
				break;
			case 1:
				m_lighting.Deactivate();
				glColorMask(0,0,0,0);
				modelEntry->model.drawFaces();
				glColorMask(1,1,1,1);
				glLineWidth(3);
				glColor3f(1.0,1.0,1.0);
				modelEntry->model.drawEdges();
				glColor3f(1.0,1.0,1.0);
				break;
			case 2:
				m_tex_model_ip[params.m_spreadlvl]->bind(0);
				m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
				m_tex_model->bind(2);
				m_tex_frame->bind(3);
				m_shadeCompare->bind();
				m_shadeCompare->setUniform("analyze", true);
				m_shadeCompare->setUniform("compare", true);
				m_shadeCompare->setUniform("textured", true);
				modelEntry->model.drawTexturedFaces();
				m_shadeCompare->setUniform("textured", false);
				modelEntry->model.drawUntexturedFaces();
				m_shadeCompare->unbind();
				break;
			case 3:
				break;
			default:
				m_showmodel = 0;
				break;			
		}
				
// 		m_modellist[i]->model.drawCoordinates();
// 		m_modellist[i]->model.drawNormals();
		modelEntry->pose.deactivate();
}


void TextureTracker::evaluatePDF( int id,
																	float x_min, float y_min,
																	float x_max, float y_max,
																	int res,
																	const char* meshfile, const char* xfile)
{
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			vector<float> pdf;
			pdf = getPDFxy((*it), x_min, y_min, x_max, y_max, res);
			savePDF(pdf, x_min, y_min, x_max, y_max, res, meshfile, xfile);			
			return;
		}
		it++;
	}
}

// Plots pdf in x-y plane (with z and rotational DOF locked)
vector<float> TextureTracker::getPDFxy(	ModelEntry* modelEntry,
																				float x_min, float y_min,
																				float x_max, float y_max,
																				int res)
{
	int i = 0;
	printf("Evaluating PDF constrained to x,y movement only\n");
	float x_step = (x_max-x_min) / res;
	float y_step = (y_max-y_min) / res;
	
	float scale = 0.1;
	
	x_min = (modelEntry->pose.t.x += x_min);
	y_min = (modelEntry->pose.t.y += y_min);
	
	vector<float> vPDF;
	vPDF.assign(res*res, 0.0);
	
	Frustum* frustum = m_cam_perspective.GetFrustum();
	
	float p;
	
	i=0;
	modelEntry->pose.t.y = y_min;
	for(int n=0; n<res; n++){
		modelEntry->pose.t.x = x_min;
		for(int m=0; m<res; m++){
			if(frustum->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z)){
				evaluateParticle(modelEntry);
				p = modelEntry->pose.c * scale;		
				printf("%f\n", p);
			}else{
				p = 0.0;
			}
			vPDF[i] = p;
			
			i++;
			modelEntry->pose.t.x += x_step;
		}
		modelEntry->pose.t.y += y_step;
	}
	
//  	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// 	glEnable(GL_BLEND);
// 	glBegin(GL_QUADS);
// 		glColor4f(1.0,0.0,0.0,0.5);
// 		glVertex3f(x_min, y_min, 0.0);
// 		glVertex3f(x_min+x_step*res, y_min, 0.0);
// 		glVertex3f(x_min+x_step*res, y_min+y_step*res, 0.0);
// 		glVertex3f(x_min, y_min+y_step*res, 0.0);
// 	glEnd();
// 	glDisable(GL_BLEND);
// 	
// 	swap();
// 	usleep(3000000);
	
	return vPDF;
}

// draws a rectangular terrain using a heightmap
void TextureTracker::savePDF(	vector<float> vPDFMap,
															float x_min, float y_min,
															float x_max, float y_max,
															int res,
															const char* meshfile, const char* xfile)
{
	int i,d,x,y;
	
	float x_step = (x_max-x_min) / res;
	float y_step = (y_max-y_min) / res;
	
	vector<vec3> vertexlist;
	vector<vec3> normallist;
	vector<vec2> texcoordlist;
	vector<unsigned int> indexlist;
	
	for(y=0; y<res; y++){
		for(x=0; x<res; x++){
			vec3 v[5], n;
			vec2 tc;
			d=0;
			
			v[0].x = x_min + float(x)*x_step;
			v[0].y = y_min + float(y)*y_step;
			v[0].z = vPDFMap[y*res+x];
			
			if(x<res-1){
			v[1].x = x_min + float(x+1)*x_step;
			v[1].y = y_min + float(y)*y_step;
			v[1].z = vPDFMap[y*res+(x+1)];
			n += (v[1]-v[0]);
			d++;
			}
			
			if(y<res-1){
			v[2].x = x_min + float(x)*x_step;
			v[2].y = y_min + float(y+1)*y_step;
			v[2].z = vPDFMap[(y+1)*res+x];
			n += (v[2]-v[0]);
			d++;
			}
			
			if(x>0){
			v[3].x = x_min + float(x-1)*x_step;
			v[3].y = y_min + float(y)*y_step;
			v[3].z = vPDFMap[y*res+(x-1)];
			n += (v[3]-v[0]);
			d++;
			}
			
			if(y>0){
			v[4].x = x_min + float(x)*x_step;
			v[4].y = y_min + float(y-1)*y_step;
			v[4].z = vPDFMap[(y-1)*res+x];
			n += (v[4]-v[0]);
			d++;
			}
			
			n = n * (1.0/d);
			
			tc.x = float(x)/res;
			tc.y = float(y)/res;
			
			vertexlist.push_back(v[0]);
			normallist.push_back(n);
			texcoordlist.push_back(tc);
			
			if(x<res-1 && y<res-1){
				indexlist.push_back(y*res+x);
				indexlist.push_back(y*res+(x+1));
				indexlist.push_back((y+1)*res+(x+1));
				
				indexlist.push_back(y*res+x);
				indexlist.push_back((y+1)*res+(x+1));
				indexlist.push_back((y+1)*res+x);
			}
		}
	}
	/*
	glClear(GL_DEPTH_BUFFER_BIT);
	m_lighting.Activate();
	
	glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(3, GL_FLOAT, 0, &vertexlist[0]);
	glEnableClientState(GL_NORMAL_ARRAY); glNormalPointer(GL_FLOAT, 0, &normallist[0]);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY); glTexCoordPointer(2, GL_FLOAT, 0, &texcoordlist[0]);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, 6 * (xres-1) * (yres-1), GL_UNSIGNED_INT, &indexlist[0]);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	m_lighting.Deactivate();
	*/
	if(meshfile){
		FILE* fd = fopen(meshfile,"w");
		fprintf(fd, "ply\nformat ascii 1.0\n");
		fprintf(fd, "element vertex %d\n", res*res);
		fprintf(fd, "property float x\n");
		fprintf(fd, "property float y\n");
		fprintf(fd, "property float z\n");
		fprintf(fd, "property float nx\n");
		fprintf(fd, "property float ny\n");
		fprintf(fd, "element face %d\n", (res-1)*(res-1)*2);
		fprintf(fd, "property list uchar uint vertex_indices\n");
		fprintf(fd, "end_header\n");
		
		for(i=0; i<vertexlist.size(); i++){
			vec3 v = vertexlist[i];
			vec3 n = normallist[i];
			fprintf(fd, "%f %f %f %f %f\n", v.x, v.y, v.z, n.x, n.y);
		}
		
		for(i=0; i<indexlist.size(); i+=3){
			fprintf(fd, "3 %d %d %d\n", indexlist[i], indexlist[i+1], indexlist[i+2]);
		}
		
		printf("  output written to '%s'\n  done\n", meshfile);
		
		fclose(fd);
	}
	
	if(xfile){
		FILE* fd2 = fopen(xfile,"w");
		y = res/2;
		for(x=0; x<res; x++){
			fprintf(fd2, "%f\n", vPDFMap[y*res+x]);
		}
		printf("  output written to '%s'\n  done\n", xfile);
		fclose(fd2);
	}
}

