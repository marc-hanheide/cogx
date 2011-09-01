
#include "TextureTracker.h"
#include <v4r/TomGine/tgLight.h>
#include <v4r/TomGine/tgMaterial.h>
#include <v4r/TomGine/tgError.h>
#include <v4r/TomGine/tgHistDesc2D.h>


using namespace std;
using namespace Tracking;
using namespace TomGine;

// *** PRIVATE ***

// Draw TrackerModel to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(ModelEntry* modelEntry){
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Render camera image as background
	if(modelEntry->model.getTextured()){
		glDisable(GL_DEPTH_TEST);
		glDepthMask(0);
			m_ip->render(m_tex_frame);
		glDepthMask(1);
		glEnable(GL_DEPTH_TEST);
	}

	// Render textured model to screen
	m_cam_perspective.Activate();
	glEnable(GL_LIGHTING);
	modelEntry->pose.Activate();
		modelEntry->model.restoreTexture();
		modelEntry->model.drawPass();

		// Extract modelview-projection matrix
		mat4 modelview, projection;
		glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
		glGetFloatv(GL_PROJECTION_MATRIX, projection);
		modelEntry->modelviewprojection = projection * modelview;

		// pass new modelview matrix to shader
		m_shadeTexEdgeTest->bind();
		m_shadeTexEdgeTest->setUniform("modelviewprojection", modelEntry->modelviewprojection, GL_FALSE); // send matrix to shader
		m_shadeTexEdgeTest->unbind();
		m_shadeTexColorTest->bind();
		m_shadeTexColorTest->setUniform("modelviewprojection", modelEntry->modelviewprojection, GL_FALSE); // send matrix to shader
		m_shadeTexColorTest->unbind();
	modelEntry->pose.Deactivate();
	glDisable(GL_LIGHTING);

	// Copy model rendered into image to texture (=reprojection to model)
	m_tex_model.CopyTexImage2D(params.camPar.width, params.camPar.height);

	// perform image processing with reprojected image
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);

	// Extract edges in various line widths
	if(modelEntry->model.getTextured()){
//		m_ip->gauss(m_tex_model, *m_tex_model_ip[0]);
		m_ip->sobel(m_tex_model, *m_tex_model_ip[0], params.model_sobel_th, true);
	}else{
		m_ip->sobel(m_tex_model, *m_tex_model_ip[0], params.model_sobel_th, true);
	}

	if(modelEntry->mask_geometry_edges){
		TomGine::tgTexture2D mask;
		this->computeModelEdgeMask(modelEntry, mask);
		m_ip->thinning(*m_tex_model_ip[0], *m_tex_model_ip[0], &mask);
	}else{
		m_ip->thinning(*m_tex_model_ip[0], *m_tex_model_ip[0]);
	}

	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(*m_tex_model_ip[i-1], *m_tex_model_ip[i]);

#ifdef DEBUG
	tgCheckError("[TextureTracker::model_processing]");
#endif
	glDepthMask(1);
}

void TextureTracker::model_processing(ModelEntry* modelEntry, TomGine::tgTexture2D &tex_model_ip)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Render camera image as background
	if(modelEntry->model.getTextured()){
		glDisable(GL_DEPTH_TEST);
		glDepthMask(0);
			m_ip->render(m_tex_frame);
		glDepthMask(1);
		glEnable(GL_DEPTH_TEST);
	}

	// Render textured model to screen
	m_cam_perspective.Activate();
	glEnable(GL_LIGHTING);
	modelEntry->pose.Activate();
		modelEntry->model.restoreTexture();
		modelEntry->model.drawPass();
	modelEntry->pose.Deactivate();
	glDisable(GL_LIGHTING);

	// Copy model rendered into image to texture (=reprojection to model)
	tex_model_ip.CopyTexImage2D(params.camPar.width, params.camPar.height);

	// perform image processing with reprojected image
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);

	// Extract edges in various line widths
	if(modelEntry->model.getTextured()){
//		m_ip->gauss(tex_model_ip, tex_model_ip);
		m_ip->sobel(tex_model_ip, tex_model_ip, params.model_sobel_th, true);
	}else{
		m_ip->sobel(tex_model_ip, tex_model_ip, params.model_sobel_th, true);
	}

	m_ip->thinning(tex_model_ip, tex_model_ip);

	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(tex_model_ip, tex_model_ip);

#ifdef DEBUG
	tgCheckError("[TextureTracker::model_processing]");
#endif
	glDepthMask(1);
}

void TextureTracker::tsd_processing(	ModelEntry* modelEntry,
										unsigned segx, unsigned segy,
										double vis_ratio)
{
	TomGine::tgHistDesc2D desc_g_image(segx,segy,8,1);
	TomGine::tgHistDesc2D desc_g_model(segx,segy,8,1);
	TomGine::tgHistDesc2D desc_c_image(segx,segy,16,1);
	TomGine::tgHistDesc2D desc_c_model(segx,segy,16,1);
	TomGine::tgRect2Di bb;
	float w = m_ip->getWidth();
	float h = m_ip->getHeight();

	// get bounding rectangle of tracked object
	modelEntry->model.getBoundingBox2D(modelEntry->pose, bb, false);
	if(bb.w%segx)
		bb.w = (bb.w/segx+1)*segx;
	if(bb.h%segy)
		bb.h = (bb.h/segy+1)*segy;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	unsigned size = bb.w*bb.h;
	std::vector<vec2> grad_image(size), grad_model(size);
	std::vector<vec2> hue_image(size), hue_model(size);
	std::vector<bool> mask(size);
	std::vector<float> depth_model(size);
	TomGine::tgTexture2D tex_tmp;

	// get gradients
	drawEdgeImage();
	tex_tmp.CopyTexImage2D(bb.x, bb.y, bb.w, bb.h);
	m_ip->param2polar(tex_tmp, tex_tmp);
	glReadPixels(0, 0, bb.w, bb.h, GL_RG, GL_FLOAT, &grad_image[0].x);

	m_ip->param2polar(*m_tex_model_ip[params.m_spreadlvl], tex_tmp);
	glReadPixels(bb.x, bb.y, bb.w, bb.h, GL_RG, GL_FLOAT, &grad_model[0].x);

	// get color
	m_ip->rgb2hsv(m_tex_frame, tex_tmp);
	glReadPixels(bb.x, bb.y, bb.w, bb.h, GL_RG, GL_FLOAT, &hue_image[0].x);

	drawModelEntry(modelEntry, 0);
	glReadPixels(bb.x, bb.y, bb.w, bb.h, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_model[0]);

	tex_tmp.CopyTexImage2D(bb.x, bb.y, bb.w, bb.h);
	m_ip->rgb2hsv(tex_tmp, tex_tmp);
	glReadPixels(0, 0, bb.w, bb.h, GL_RG, GL_FLOAT, &hue_model[0].x);


	// mask pixels that lie outside of the frame
	for(unsigned j=0; j<bb.h; j++){
		for(unsigned i=0; i<bb.w; i++){
			if(	(bb.x+i < 0) || (bb.x+i > w) || (bb.y+j < 0) || (bb.y+j > h) ){
				depth_model[j*bb.w+i] = 1.0f;
			}
		}
	}

	// convert to polar coordinates
	for(unsigned i=0; i<size; i++)
	{
		grad_image[i] = vec2(grad_image[i].x * 2.0 * M_PI, grad_image[i].y);

		grad_model[i] = vec2(grad_model[i].x * 2.0 * M_PI, grad_model[i].y);

		hue_image[i].x = 360.0 * hue_image[i].x;
		hue_image[i].y = 0.5;
		hue_model[i].x = 360.0 * hue_model[i].x;
		hue_model[i].y = 0.5;

		if(depth_model[i] < 0.1f )
			depth_model[i] = 1.0f;
		mask[i] = ( depth_model[i] < 1.0f );
	}

	// evaluate descriptor
	desc_g_image.Process(grad_image, bb, vec2(0.0,0.0), vec2(2.0f*M_PI, 1.0f), mask, vis_ratio);
	desc_g_model.Process(grad_model, bb, vec2(0.0,0.0), vec2(2.0f*M_PI, 1.0f), mask, vis_ratio);
	desc_c_image.Process(hue_image, bb, vec2(0.0,0.0), vec2(360.0, 1.0f), mask, vis_ratio);
	desc_c_model.Process(hue_model, bb, vec2(0.0,0.0), vec2(360.0, 1.0f), mask, vis_ratio);

	// intersect histograms of descriptor
	std::vector<double> isec_g = desc_g_model.Intersect(desc_g_image, 0.0);
	std::vector<double> isec_c = desc_c_model.Intersect(desc_c_image, 0.0);

	// compute variance of intersection match
	std::vector<double> isec;
	double mean_tmp(0.0), var_tmp(0.0);
	for(unsigned i=0; i<isec_g.size(); i++){
		double val = isec_g[i]*isec_c[i];
		if(desc_g_model.m_maskratio[i]>vis_ratio){
			isec.push_back(val);
			mean_tmp += (val);
		}
	}
	mean_tmp /= isec.size();
	for(unsigned i=0; i<isec.size(); i++)
		var_tmp += pow(isec[i]-mean_tmp,2);
	var_tmp /= isec.size();

	modelEntry->c_occ = sqrt(var_tmp);
}

// Particle filtering
void TextureTracker::particle_filtering(ModelEntry* modelEntry){
	Particle p;
	int num_particles;
	m_cam_perspective.Activate();
	
	// Calculate Zoom Direction and pass it to the particle filter
	vec3 vCam = m_cam_perspective.GetPos();
	vec3 vObj = vec3(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z);
	modelEntry->vCam2Model = vObj - vCam;
	modelEntry->predictor->setCamViewVector(modelEntry->vCam2Model);
	
	Particle variation = params.variation;
	
	for(unsigned i=0; i<modelEntry->num_recursions; i++){
// 		unsigned i=0;
		// TODO Evaluate if this makes sense (robustness, accuracy)
// 		float c_max = modelEntry->distribution.getMaxC();
// 		params.m_spreadlvl = (int)floor((params.num_spreadings)*(0.5-c_max));
// 		if(params.m_spreadlvl>=params.num_spreadings)
// 			params.m_spreadlvl = params.num_spreadings-1;
// 		else if(params.m_spreadlvl<0)
// 			params.m_spreadlvl = 0;

		// TODO Evaluate if this makes sense (robustness, accuracy)
		int j = (int)params.num_spreadings - (int)i;
		if(j > 1){
			params.m_spreadlvl = j - 1;
		}else{
			params.m_spreadlvl = 1;
		}
		float varred_t = 1.0f;
		float varred_q = 1.0f;
		float varred_z = 1.0f;
		if(modelEntry->num_recursions > 1){
			varred_t = 1.0f - 0.7f * float(i)/(modelEntry->num_recursions - 1);
			varred_q = 1.0f - 0.8f * float(i)/(modelEntry->num_recursions - 1);
			varred_z = 1.0f - 0.8f * float(i)/(modelEntry->num_recursions - 1);
		}
		variation.t = params.variation.t * varred_t;
		variation.q = params.variation.q * varred_q;
		variation.z = params.variation.z * varred_z;
		
// 		variation.z = params.variation.z * varred;
// 		printf("%d %f %f %f\n", params.m_spreadlvl, variation.t.x, variation.r.x*180/PI, varred_t);
		
// 		if(i == modelEntry->num_recursions - 1)
// 			m_showparticles = true;
// 		else
// 			m_showparticles = false;
		
		m_tex_model_ip[params.m_spreadlvl]->Bind(0);
		m_tex_frame_ip[params.m_spreadlvl]->Bind(1);
		m_tex_model.Bind(2);
		m_tex_frame.Bind(3);

		// TODO Evaluate if this makes sense (robustness, accuracy)
// 		params.kernel_size = (int)floor(0.5*(params.max_kernel_size)*(1.0-c_max));
// 		m_shadeCompare->bind();
// 		m_shadeCompare->setUniform("kernelsize", params.kernel_size);
// 		m_shadeCompare->unbind();
		
		// update importance weights and confidence levels
// 		printf("Recursion[%d] %d c:%f p: ", i, params.m_spreadlvl, modelEntry->distribution.getParticle(0).c); 
// 		modelEntry->distribution.getParticle(0).Print();
		modelEntry->distribution.updateLikelihood(modelEntry->model,
				m_shadeTexEdgeTest, m_shadeTexColorTest, params.method,
				modelEntry->model.getTextured(), modelEntry->model.getFullTextured(),
				params.convergence, m_showparticles);
		
		//modelEntry->distribution = d1;
// 		printf("%f\n", ( 1.0 - i * 0.5 / (float)(modelEntry->num_recursions-1)));
		// TODO Evaluate if this makes sense (accuracy)
// 		p = (params.variation * ( 1.0 - i * 0.8 / (float)(modelEntry->num_recursions-1)));
		
		// "Attention mechanism" modifies number of particles indirect proportional to the overall confidence
		//num_particles = modelEntry->num_particles * (1.0 - c_max);
		num_particles = modelEntry->num_particles;
		
		// predict movement of object

		modelEntry->predictor->resample(modelEntry->distribution, num_particles, variation, i, params.num_recursions);
		
		// set timestep to 0.0 for further recursion (within same image)
		modelEntry->predictor->updateTime(0.0);
	}
#ifdef DEBUG
	tgCheckError("TextureTracker::particle_filtering");
#endif
// 	printf("\n");
	// weighted mean
	modelEntry->pose_prev = modelEntry->pose;
//	modelEntry->pose = modelEntry->distribution.getMean();
	modelEntry->pose = modelEntry->distribution.getMeanOfKeptParticles();
}



// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_drawimage = false;
	m_tsd = false;
	m_pose_filter = true;
}

TextureTracker::~TextureTracker(){
	for(unsigned i=0; i<params.num_spreadings; i++){
		delete(m_tex_model_ip[i]);
	}
}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal(){	
	
	int id;

	// TomGine::tgShader
	if((id = g_Resources->AddShader("texEdgeTest", "texEdgeTest.vert", "texEdgeTest.frag")) == -1)
		exit(1);
	m_shadeTexEdgeTest = g_Resources->GetShader(id);
	
	if((id = g_Resources->AddShader("texColorTest", "texColorTest.vert", "texColorTest.frag")) == -1)
		exit(1);
	m_shadeTexColorTest = g_Resources->GetShader(id);
	
//	if((id = g_Resources->AddShader("mmConfidence", "mmConfidence.vert", "mmConfidence.frag")) == -1)
//		exit(1);
//	m_shadeConfidenceMM = g_Resources->GetShader(id);
	
//	m_shadeCompare = m_shadeTexEdgeTest;
	
	// TomGine::tgTexture2D
	m_tex_model.Bind();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	m_tex_model_ip.push_back( new TomGine::tgTexture2D() );
	for(int i=0; i<(int)params.num_spreadings-1; i++)
		m_tex_model_ip.push_back( new TomGine::tgTexture2D() );
	
	// Load 
	float w = (float)params.camPar.width;
	float h = (float)params.camPar.height;
	
	GLfloat offX[9] = { -1.0f/w, 0.0f, 1.0f/w,
						-1.0f/w, 0.0f, 1.0f/w,
						-1.0f/w, 0.0f, 1.0f/w };
	GLfloat offY[9] = {  1.0f/h, 1.0f/h,  1.0f/h,
						 0.0f,   0.0f,    0.0f,
						-1.0f/h,-1.0f/h, -1.0f/h };
                        
	m_shadeTexEdgeTest->bind();
	m_shadeTexEdgeTest->setUniform("tex_model_edge", 0);
	m_shadeTexEdgeTest->setUniform("tex_frame_edge", 1);
	m_shadeTexEdgeTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexEdgeTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeTexEdgeTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeTexEdgeTest->setUniform("drawcolor", vec4(1.0f,0.0f,0.0f,0.0f));
	m_shadeTexEdgeTest->setUniform("kernelsize", (GLint)params.kernel_size);
	m_shadeTexEdgeTest->unbind();
	
	m_shadeTexColorTest->bind();
	m_shadeTexColorTest->setUniform("tex_model_color", 2);
	m_shadeTexColorTest->setUniform("tex_frame_color", 3);
	m_shadeTexColorTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexColorTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
	m_shadeTexColorTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
	m_shadeTexColorTest->setUniform("drawcolor", vec4(0.0f,0.0f,1.0f,0.0f));
	m_shadeTexColorTest->setUniform("kernelsize", (GLint)params.kernel_size);
	m_shadeTexColorTest->unbind();
	
//	m_shadeConfidenceMM->bind();
//	m_shadeConfidenceMM->setUniform("tex_model_color", 2);
//	m_shadeConfidenceMM->setUniform("tex_frame_color", 3);
//	m_shadeConfidenceMM->setUniform("fTol", params.edge_tolerance);
//	m_shadeConfidenceMM->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
//	m_shadeConfidenceMM->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
//	m_shadeConfidenceMM->setUniform("drawcolor", vec4(0.0f,0.0f,1.0f,0.0f));
//	m_shadeConfidenceMM->setUniform("kernelsize", (GLint)params.kernel_size);
//	m_shadeConfidenceMM->unbind();
	
	vec3 cam_f = m_cam_perspective.GetF();
	m_light.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	m_light.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	m_light.Activate();
	
	return true;
}

float TextureTracker::evaluateParticle(ModelEntry* modelEntry)
{
//	return evaluateParticle(modelEntry, m_shadeCompare);
}

// evaluate single particle
float TextureTracker::evaluateParticle(ModelEntry* modelEntry, TomGine::tgShader* shader){
	unsigned int queryMatches;
	unsigned int queryEdges;
	int v, d;
	float c;
	
	glGenQueriesARB(1, &queryMatches);
	glGenQueriesARB(1, &queryEdges);

	m_cam_perspective.Activate();
	//glViewport(0,0,256,256);
	glColorMask(0,0,0,0); glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

	m_tex_model_ip[params.m_spreadlvl]->Bind(0);
	m_tex_frame_ip[params.m_spreadlvl]->Bind(1);
	m_tex_model.Bind(2);
	m_tex_frame.Bind(3);
	
	// Draw particles and count pixels
	shader->bind();
	shader->setUniform("analyze", false);
    
	modelEntry->pose.Activate();
	
		// Draw all model edge pixels
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges);
		shader->setUniform("compare", false);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		modelEntry->model.drawTexturedFaces();
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches);
		shader->setUniform("compare", true);
		shader->setUniform("textured", true);
		modelEntry->model.drawTexturedFaces();
		shader->setUniform("textured", false);
		modelEntry->model.drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
	
	modelEntry->pose.Deactivate();

	shader->unbind();

	glGetQueryObjectivARB(queryMatches, GL_QUERY_RESULT_ARB, &d);
	glGetQueryObjectivARB(queryEdges, GL_QUERY_RESULT_ARB, &v);
	glColorMask(1,1,1,1); glDepthMask(1);
	glDeleteQueriesARB(1, &queryMatches);
	glDeleteQueriesARB(1, &queryEdges);
	
	c = float(d)/float(v);
	
	return c;
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image, GLenum format){
	
	// Load camera image to texture
	m_tex_frame.Load(image, params.camPar.width, params.camPar.height, GL_RGBA, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);

//	m_ip->gauss(m_tex_frame, *m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame, *m_tex_frame_ip[0], params.image_sobel_th, true);
	m_ip->thinning(*m_tex_frame_ip[0], *m_tex_frame_ip[0]);
	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(*m_tex_frame_ip[i-1], *m_tex_frame_ip[i]);

	glDepthMask(1);
#ifdef DEBUG
	tgCheckError("[TextureTracker::image_processing(unsigned char*, GLenum)]");
#endif
}

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image, const TomGine::tgModel &m, const tgPose &p, GLenum format){
	
	// Load camera image to texture
	m_tex_frame.Load(image, params.camPar.width, params.camPar.height, GL_RGBA, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);

	// Draw model (i.e. a virutal occluder)
	glDepthMask(1);
	glEnable(GL_DEPTH_TEST);
	drawModel(m,p);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	m_tex_frame.CopyTexImage2D(params.camPar.width, params.camPar.height);

	m_ip->gauss(m_tex_frame, *m_tex_frame_ip[0]);
	m_ip->sobel(*m_tex_frame_ip[0], *m_tex_frame_ip[0], params.image_sobel_th, true);
	m_ip->thinning(*m_tex_frame_ip[0], *m_tex_frame_ip[0]);
	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(*m_tex_frame_ip[i-1], *m_tex_frame_ip[i]);
	
	glDepthMask(1);
#ifdef DEBUG
	tgCheckError("[TextureTracker::image_processing(unsigned char*, const TomGine::tgModel&, const tgPose&, GLenum)]");
#endif
}

void TextureTracker::image_processing(unsigned char* image, int model_id, const tgPose &p, GLenum format){
	
	// Load camera image to texture
	m_tex_frame.Load(image, params.camPar.width, params.camPar.height, GL_RGBA, format);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);
	
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	
// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw model (i.e. a virutal occluder)
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(model_id==(*it)->id){
			m_cam_perspective.Activate();
			glClear(GL_DEPTH_BUFFER_BIT);
			glEnable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(1);
			p.Activate();
			
			//(*it)->model.restoreTexture();
			(*it)->model.drawPass();
			
			p.Deactivate();
			glDepthMask(0);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_LIGHTING);
			glClear(GL_DEPTH_BUFFER_BIT);
		}
		it++;
	}
	m_tex_frame.CopyTexImage2D(params.camPar.width, params.camPar.height);

	m_ip->gauss(m_tex_frame, *m_tex_frame_ip[0]);
	m_ip->sobel(*m_tex_frame_ip[0], *m_tex_frame_ip[0], params.image_sobel_th, true);
	m_ip->thinning(*m_tex_frame_ip[0], *m_tex_frame_ip[0]);
	for(unsigned i=1; i<params.num_spreadings; i++)
		m_ip->spreading(*m_tex_frame_ip[i-1], *m_tex_frame_ip[i]);
	
	glDepthMask(1);
#ifdef DEBUG
	tgCheckError("[TextureTracker::image_processing(unsigned char*, int, const tgPose&, GLenum)]");
#endif
}

bool TextureTracker::track(){
	if(!m_tracker_initialized){
		printf("[TextureTracker::track()] Error tracker not initialised!\n");
		return false;
	}
	
	// Track models
	for(unsigned i=0; i<m_modellist.size(); i++){
		track(m_modellist[i]);
	}
	
	// Track hypothesis
//	ModelEntryList::iterator m1 = m_hypotheses.begin();
//	while(m1 < m_hypotheses.end()){
//		track((*m1));
//		if((*m1)->num_convergence++>params.hypotheses_trials){
//			if((*m1)->past_confidences.size() < params.hypotheses_trials){
//
//				(*m1)->past_confidences.push_back((*m1)->distribution.getMax().c);
//				m_modellist[(*m1)->hypothesis_id]->past_confidences.push_back(m_modellist[(*m1)->hypothesis_id]->distribution.getMax().c);
//			}else{
//
//				// Evaluate mean confidence of hypothesis
//				float c_hyp = 0.0;
//				int s = (*m1)->past_confidences.size();
//				for(int j=0; j<s; j++){
//					c_hyp += (*m1)->past_confidences[j];
//				}
//				if(s>0)
//					c_hyp = c_hyp / (float)s;
//
//				// Evaluate mean confidence of model, the hypothesis belongs to
//				float c_model = 0.0;
//				s = m_modellist[(*m1)->hypothesis_id]->past_confidences.size();
//				for(int j=0; j<s; j++){
//					c_model += m_modellist[(*m1)->hypothesis_id]->past_confidences[j];
//				}
//				if(s>0)
//					c_model = c_model / (float)s;
//
//				// Compare confidence of model to the hypothesis
//				if(c_model >= c_hyp){
//					// if model is more confident, delete hypothesis
//					delete(*m1);
//					m_hypotheses.erase(m1);
//				}else{
//					// if hypothesis is more confident, delete model and replace modellist-entry with hypothesis
//					delete(m_modellist[(*m1)->hypothesis_id]);
//					(*m1)->id = (*m1)->hypothesis_id;
//					m_modellist[(*m1)->hypothesis_id] = (*m1);
//					m_hypotheses.erase(m1);
//				}
//			}
//		}
//		m1++;
//	}
#ifdef DEBUG
	tgCheckError("TextureTracker::track()");
#endif
	return true;
}

bool TextureTracker::track(ModelEntry *modelEntry){

	// Process model (texture reprojection, edge detection)
	model_processing(modelEntry);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(m_drawimage)
		drawImage();

	// Apply particle filtering
	if(!modelEntry->lock){
		particle_filtering(modelEntry);
		if(!m_cam_perspective.GetFrustum()->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z))
			reset();
	}else{
//		modelEntry->pose.c = evaluateParticle(modelEntry, m_shadeCompare);
	}
	
	if(m_tsd){
		this->tsd_processing(modelEntry);
		modelEntry->evaluate_signals();
		modelEntry->evaluate_tsd();
	}else{
		modelEntry->evaluate_signals();
		modelEntry->ts = ST_DISABLED;
	}

	if(m_pose_filter)
		modelEntry->filter_pose();
	
	m_ftime = (float)m_timer.Update();
	return true;
}

bool TextureTracker::track(int id){
	if(!m_tracker_initialized){
		printf("[TextureTracker::track(int)] Error tracker not initialised!\n");
		return false;
	}
	
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			track(m_modellist[id]);
#ifdef DEBUG
			tgCheckError("TextureTracker::track(int)");
#endif
			return true;
		}
		it++;
	}
	
	return false;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(bool use_num_pixels){
	std::vector<tgVertex> vertices;
	
	Parameter tmpParams = params;
	
	params.num_particles = 500;
	params.num_recursions = 10;
	params.m_spreadlvl = 0;
	params.variation = params.variation * 0.1;
	setKernelSize(0);

	track();

	params = tmpParams;
	setKernelSize(params.kernel_size);
	
	for(unsigned i=0; i<m_modellist.size(); i++){

		m_cam_perspective.Activate();
		vector<unsigned> faceUpdateList = m_modellist[i]->model.getFaceUpdateList(m_modellist[i]->pose, 
					vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
					params.minTexGrabAngle,
					use_num_pixels);

		if(!faceUpdateList.empty()){
			vertices.clear();

			m_modellist[i]->model.textureFromImage(	m_tex_frame,
													params.camPar.width, params.camPar.height,
													m_modellist[i]->pose,
													vec3(m_modellist[i]->vCam2Model.x, m_modellist[i]->vCam2Model.y, m_modellist[i]->vCam2Model.z),
													params.minTexGrabAngle,
													faceUpdateList,
													vertices, 
													&m_cam_perspective);
			m_modellist[i]->c_max_runtime = 0.0;

		}
		faceUpdateList.clear();
	}
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels){
	std::vector<tgVertex> vertices;
	
	ModelEntry* modelEntry;
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			modelEntry = (*it);
		}
		it++;
	}
	
	modelEntry->pose = pose;
	
	m_cam_perspective.Activate();
	vector<unsigned> faceUpdateList = modelEntry->model.getFaceUpdateList(modelEntry->pose, 
				vec3(modelEntry->vCam2Model.x, modelEntry->vCam2Model.y, modelEntry->vCam2Model.z),
				params.minTexGrabAngle,
				use_num_pixels);

	if(!faceUpdateList.empty()){
		vertices.clear();

		modelEntry->model.textureFromImage(	m_tex_frame,
											params.camPar.width, params.camPar.height,
											modelEntry->pose,
											vec3(modelEntry->vCam2Model.x, modelEntry->vCam2Model.y, modelEntry->vCam2Model.z),
											params.minTexGrabAngle,
											faceUpdateList,
											vertices, 
											&m_cam_perspective);
		modelEntry->c_max_runtime = 0.0;

	}
	faceUpdateList.clear();
}

void TextureTracker::untextureModels(){
	for(unsigned i=0; i<m_modellist.size(); i++){
		m_modellist[i]->model.releasePassList();	
	}
}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(float linewidth){
		
	if(m_showmodel>4||m_showmodel<0)
			m_showmodel=0;
	
	for(unsigned i=0; i<m_modellist.size(); i++)
		drawModelEntry(m_modellist[i], m_showmodel, linewidth);
	
#ifdef DEBUG
	tgCheckError("TextureTracker::drawResult");
#endif
}

void TextureTracker::drawModelEntry(int id, int mode, float linewidth)
{
	ModelEntry* modelEntry;
	ModelEntryList::iterator it = m_modellist.begin();
	while(it != m_modellist.end()){
		if(id==(*it)->id){
			drawModelEntry((*it), mode, linewidth);
			return;
		}
		it++;
	}
}

void TextureTracker::drawModelEdgeImage(){
	glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

	m_ip->render(*m_tex_model_ip[params.m_spreadlvl]);

	glDepthMask(1);
}

void TextureTracker::drawModelEntry(ModelEntry* modelEntry, int mode, float linewidth)
{
	m_cam_perspective.Activate();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);

	modelEntry->pose.Activate();

	switch(mode){
		case 0:
			modelEntry->model.restoreTexture();
			modelEntry->model.drawPass(false);
			break;
		case 1:
			glDisable(GL_LIGHTING);
			//glColorMask(0,0,0,0);
			//modelEntry->model.drawFaces();
			//glColorMask(1,1,1,1);
			if(linewidth<1.0f)
				linewidth = 1.0f;
			glLineWidth(linewidth);
			modelEntry->model.drawEdges();
			glColor3f(1.0,1.0,1.0);
			break;
		case 2:
			m_tex_model_ip[params.m_spreadlvl]->Bind(0);
			m_tex_frame_ip[params.m_spreadlvl]->Bind(1);
			m_tex_model.Bind(2);
			m_tex_frame.Bind(3);
			m_shadeTexEdgeTest->bind();
			m_shadeTexEdgeTest->setUniform("analyze", true);
			m_shadeTexEdgeTest->setUniform("compare", true);
			m_shadeTexEdgeTest->setUniform("textured", true);
			modelEntry->model.drawTexturedFaces();
			m_shadeTexEdgeTest->setUniform("textured", false);
			modelEntry->model.drawUntexturedFaces();
			m_shadeTexEdgeTest->unbind();
			glDisable(GL_TEXTURE_2D);
			break;
		case 3:
			m_tex_model_ip[params.m_spreadlvl]->Bind(0);
			m_tex_frame_ip[params.m_spreadlvl]->Bind(1);
			m_tex_model.Bind(2);
			m_tex_frame.Bind(3);
			m_shadeTexColorTest->bind();
			m_shadeTexColorTest->setUniform("analyze", true);
			m_shadeTexColorTest->setUniform("compare", true);
			modelEntry->model.drawTexturedFaces();
			modelEntry->model.drawUntexturedFaces();
			m_shadeTexColorTest->unbind();
			glDisable(GL_TEXTURE_2D);
			break;
		case 4:
			break;
		default:
			break;
	}

	modelEntry->pose.Deactivate();

	glDisable(GL_LIGHTING);
}

void TextureTracker::evaluatePDF(	int id,
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
	float c = 0.0f;
	float scale = 0.1f;
	
	x_min = (modelEntry->pose.t.x += x_min);
	y_min = (modelEntry->pose.t.y += y_min);
	
	vector<float> vPDF;
	vPDF.assign(res*res, 0.0);
	
	TomGine::tgFrustum* frustum = m_cam_perspective.GetFrustum();
	
	
	float p;
	i=0;
	modelEntry->pose.t.y = y_min;
	for(int n=0; n<res; n++){
		modelEntry->pose.t.x = x_min;
		for(int m=0; m<res; m++){
			if(frustum->PointInFrustum(modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z)){
				p = evaluateParticle(modelEntry, m_shadeTexEdgeTest) * scale;
// 				printf("%f %f %f %f\n", modelEntry->pose.t.x, modelEntry->pose.t.y, modelEntry->pose.t.z, p);
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
								unsigned res,
								const char* meshfile, const char* xfile)
{
	unsigned i,d,x,y;
	
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
			
			n = n * (1.0f/float(d));
			
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
	glEnable(GL_LIGHTING);
	
	glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(3, GL_FLOAT, 0, &vertexlist[0]);
	glEnableClientState(GL_NORMAL_ARRAY); glNormalPointer(GL_FLOAT, 0, &normallist[0]);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY); glTexCoordPointer(2, GL_FLOAT, 0, &texcoordlist[0]);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_TRIANGLES, 6 * (xres-1) * (yres-1), GL_UNSIGNED_INT, &indexlist[0]);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	glDisable(GL_LIGHTING);
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
		y = res>>1;
		for(x=0; x<res; x++){
			fprintf(fd2, "%f\n", vPDFMap[y*res+x]);
		}
		printf("  output written to '%s'\n  done\n", xfile);
		fclose(fd2);
	}
}

