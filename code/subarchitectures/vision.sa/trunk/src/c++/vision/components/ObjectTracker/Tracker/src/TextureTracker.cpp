
#include "TextureTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void TextureTracker::image_processing(unsigned char* image){
	// Load camera image to texture
	m_image = image;
	m_tex_frame->load(image, params.width, params.height);
	
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0], 0.03, true);
	for(int i=1; i<NUM_SPREAD_LOOPS; i++)
		m_ip->spreading(m_tex_frame_ip[i-1], m_tex_frame_ip[i]);
	glDepthMask(1);
}

// Particle filtering
void TextureTracker::particle_filtering(int num_recursions, int num_particles){
	
	// Calculate Zoom Direction and pass it to the particle filter
	TM_Vector3 vCam = m_cam_perspective->GetPos();
	TM_Vector3 vObj = TM_Vector3(m_pose.s.x, m_pose.s.y, m_pose.s.z);
	TM_Vector3 vCamObj = vObj - vCam;
	vCamObj.normalize();
	m_particles->setCamViewVector(vCamObj);
		
	float c_max = m_particles->getMaxC();
	
// 	num_recursions = ceil(params.recursions*(1.0-c_max));
// 	num_particles = (1.0-c_max) * params.number_of_particles;

	for(int i=0; i<num_recursions; i++){
		// adjusting spreading level
		if(m_model->isTextured()){
			params.m_spreadlvl = (int)floor((NUM_SPREAD_LOOPS-1)*2.0*(1.0-c_max));
			if(params.m_spreadlvl>=NUM_SPREAD_LOOPS)
				params.m_spreadlvl = NUM_SPREAD_LOOPS-1;
			else if(params.m_spreadlvl<0)
				params.m_spreadlvl = 0;
		}else{
			params.m_spreadlvl = 0;
		}
		m_tex_model_ip[params.m_spreadlvl]->bind(0);
		m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
		m_tex_model->bind(2);
		m_tex_frame->bind(3);

		// importance weights and confidence levels
		particle_processing(m_particles);
		m_particles->calcLikelihood(9);

		// particle selection
		m_particles->resample(num_particles, m_pConstraints);
	}
	// weighted mean	
	m_pose = *m_particles->getMax();
}

// evaluate single particle
void TextureTracker::evaluateParticle(Particle* p){
	unsigned int queryMatches;
	unsigned int queryEdges;
	int v, d;
	int id;
	
	glGenQueriesARB(1, &queryMatches);
	glGenQueriesARB(1, &queryEdges);

	m_cam_perspective->Activate();
	//glViewport(0,0,256,256);
	glColorMask(0,0,0,0); glDepthMask(0);
	glColor3f(1.0,1.0,1.0);

	m_tex_frame_ip[params.m_spreadlvl]->bind(1);	
	m_model->setTexture(m_tex_model_ip[params.m_spreadlvl]);
		
	// Draw particles and count pixels
	m_shadeCompare->bind();
	m_shadeCompare->setUniform("analyze", false);
    
	p->activate();
	
		m_shadeCompare->setUniform("compare", false);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryEdges);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		m_model->drawTexturedFaces();
		m_model->drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
		
		glColorMask(0,0,0,0);
		
		m_shadeCompare->setUniform("compare", true);
		m_shadeCompare->setUniform("textured", true);
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB, queryMatches);
		m_model->drawTexturedFaces();
		m_shadeCompare->setUniform("textured", false);
		m_model->drawUntexturedFaces();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);
	
	p->deactivate();

	m_shadeCompare->unbind();

	glGetQueryObjectivARB(queryMatches, GL_QUERY_RESULT_ARB, &d);
	glGetQueryObjectivARB(queryEdges, GL_QUERY_RESULT_ARB, &v);

	if(v != 0){
		p->c = (float(d)/float(v));
		p->w = pow(p->c, 5*(1.0-p->c));
	}else{
		p->c = 0.0;
		p->w = 0.0;
	}
	
	glColorMask(1,1,1,1); glDepthMask(1);
	glDeleteQueriesARB(1, &queryMatches);
	glDeleteQueriesARB(1, &queryEdges);
}

// Draw TrackerModel to screen, extract modelview matrix, perform image processing for model
void TextureTracker::model_processing(){
	
	glClear(GL_COLOR_BUFFER_BIT);
	
	// Render camera image as background
	glDepthMask(0);
	m_ip->render(m_tex_frame);
	glDepthMask(1);
		
	// Setup camera, lighting and pose and rendering
	if(m_bfc) glEnable(GL_CULL_FACE);
	else glDisable(GL_CULL_FACE);
	m_cam_perspective->Activate();			// activate perspective view
	m_lighting.Activate();
	m_particles->getMax()->activate();
	
	// Extract modelview-projection matrix
	glGetFloatv(GL_MODELVIEW_MATRIX, m_modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, m_projection);
	m_modelviewprojection = m_projection * m_modelview;	
	
	// Render textured model to screen
	m_model->restoreTexture();
	m_model->drawPass();
	
	// deactivate particles, lighting
	m_particles->getMax()->deactivate();
	m_lighting.Deactivate();
	
	// pass new modelview matrix to shader
	m_shadeCompare->bind();
	m_shadeCompare->setUniform("modelviewprojection", m_modelviewprojection, GL_FALSE); // send matrix to shader
	m_shadeCompare->unbind();
	
	// Copy screen to texture (=reprojection to model)
	m_tex_model->copyTexImage2D(params.width, params.height);
		
	// perform image processing with reprojected image
	glDepthMask(0);
	if(m_model->isTextured()) m_ip->gauss(m_tex_model, m_tex_model_ip[0]);
	else m_ip->copy(m_tex_model, m_tex_model_ip[0]);
	m_ip->sobel(m_tex_model_ip[0], m_tex_model_ip[0], 0.03, true);
	for(int i=1; i<NUM_SPREAD_LOOPS; i++)
		m_ip->spreading(m_tex_model_ip[i-1], m_tex_model_ip[i]);
	glDepthMask(1);
}

// Draw each particle, count matching pixels, calculate likelihood for texture tracking
void TextureTracker::particle_processing(Particles* particles){
	int num_particles = particles->getNumParticles();
	
	// Set perspective mode and smaller viewport
	m_cam_perspective->Activate();
	glColorMask(0,0,0,0); glDepthMask(0);
		
	// Draw particles and count pixels
	m_shadeCompare->bind();
	m_shadeCompare->setUniform("analyze", false);
	for(int i=0; i<num_particles; i++){
		particles->activate(i);
		
		// Draw all model edge pixels
		particles->startCountV(i);
		m_shadeCompare->setUniform("compare", false);
		if(m_showparticles)
			glColorMask(1,1,1,1);
		m_model->drawTexturedFaces();
		m_model->drawUntexturedFaces();
		particles->endCountV();
		
		glColorMask(0,0,0,0);
		
		// Draw matching model edge pixels using a shader
		particles->startCountD(i);
		m_shadeCompare->setUniform("compare", true);
		m_shadeCompare->setUniform("textured", true);
		m_model->drawTexturedFaces();
		m_shadeCompare->setUniform("textured", false);
		m_model->drawUntexturedFaces();
		particles->endCountD();
		
		particles->deactivate(i);
	}
	m_shadeCompare->unbind();
	glColorMask(1,1,1,1); glDepthMask(1);	
}


// *** PUBLIC ***

TextureTracker::TextureTracker(){
	m_lock = false;
	m_showparticles = false;
	m_showmodel = 0;
	m_zero_particles = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_bfc = false;
		
	int id;
	char name[FN_LEN];
	
	// Shader
	if((id = g_Resources->AddShader("texEdgeTest", "texEdgeTest.vert", "texEdgeTest.frag")) == -1)
		exit(1);
	m_shadeTexEdgeTest = g_Resources->GetShader(id);
	m_shadeCompare = m_shadeTexEdgeTest;
	
	if((id = g_Resources->AddShader("texColorTest", "texColorTest.vert", "texColorTest.frag")) == -1)
		exit(1);
	m_shadeTexColorTest = g_Resources->GetShader(id);
	
	sprintf(name, "T%d:m_tex_model", params.m_tracker_id);
	if((id = g_Resources->AddTexture(NULL, name)) == -1)
		exit(1);
	m_tex_model = g_Resources->GetTexture(id);
	
	for(int i=0; i<NUM_SPREAD_LOOPS; i++){
		sprintf(name, "T%d:m_tex_model_ip[%d]",params. m_tracker_id, i);
		if((id = g_Resources->AddTexture(NULL, name)) == -1)
			exit(1);
		m_tex_model_ip[i] = g_Resources->GetTexture(id);
	}
	
	m_tex_frame_cmp = m_tex_frame_ip[2];
	m_tex_model_cmp = m_tex_model_ip[1];
}

// Initialise function (must be called before tracking)
bool TextureTracker::initInternal(){	
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
	m_shadeTexColorTest->setUniform("tex_model", 2);
	m_shadeTexColorTest->setUniform("tex_frame", 3);
	m_shadeTexColorTest->setUniform("fTol", params.edge_tolerance);
	m_shadeTexColorTest->setUniform( "mOffsetX", mat3(offX), GL_FALSE );
  m_shadeTexColorTest->setUniform( "mOffsetY", mat3(offY), GL_FALSE );
  m_shadeTexColorTest->setUniform("drawcolor", vec4(0.0,0.0,1.0,0.0));
  m_shadeTexColorTest->setUniform("kernelsize", 1);
	m_shadeTexColorTest->unbind();
	
	m_ip->setSobelThreshold(0.03);
	
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

bool TextureTracker::track(	TrackerModel* model,				// tracking model (textured, vertexlist, facelist) 
														Camera* camera,				// extrinsic camera (defining pose and projection)
														int num_recursions,
														int num_particles,
														Particle p_constraints, 			
														Particle& p_result,			// storage to write tracked position
														float fTime)
{
	m_timer.Update();
	params.time_tracking = 0.0;
	
	m_model = model;
	m_cam_perspective = camera;
	m_pConstraints = p_constraints;
	params.number_of_particles = num_particles;
	params.recursions = num_recursions;
	
	// Process model (texture reprojection, edge detection)
	model_processing();
	
// 	// Clear framebuffer and render image from camera
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[params.m_spreadlvl]);
	else
		m_ip->render(m_tex_frame);
	glDepthMask(1);
	
	// Apply particle filtering
	if(!m_lock){
		particle_filtering(num_recursions, num_particles);
		p_result=m_pose;
	}else{
		m_particles->resetTimer();
		evaluateParticle(&p_result);
	}
	
	// Copy result to output
	if(m_zero_particles){
		p_result = params.zP;
		m_particles->sample(params.number_of_particles, params.zP, p_constraints);
		m_zero_particles = false;
	}
		
	params.time_tracking += m_timer.Update();
	
	return true;
}

// grabs texture from camera image and attaches it to faces of model
void TextureTracker::textureFromImage(){
		
// 	if(m_model->getUntexturedFaces()>0 && m_pose.c>th)

		// Calculate Zoom Direction and pass it to the particle filter
		TM_Vector3 vCam = m_cam_perspective->GetPos();
		TM_Vector3 vObj = TM_Vector3(m_pose.s.x, m_pose.s.y, m_pose.s.z);
		TM_Vector3 vCamObj = vObj - vCam;
		vCamObj.normalize();
		m_particles->setCamViewVector(vCamObj);
		
		params.m_spreadlvl = 0;
		m_tex_model_ip[params.m_spreadlvl]->bind(0);
		m_tex_frame_ip[params.m_spreadlvl]->bind(1);
			
		for(int i=0; i<4; i++){
			particle_processing(m_particles);
			m_particles->resample(1000, m_pConstraints);
		}								
		m_pose = *m_particles->getMax();
		
		m_model->textureFromImage(m_tex_frame, params.width, params.height, &m_pose, vec3(vCamObj.x, vCamObj.y, vCamObj.z), params.minTexGrabAngle);

}

// Draw result of texture tracking (particle with maximum likelihood)
void TextureTracker::drawResult(Particle* p, TrackerModel* m){
	bool texmodel = false;
	
	m_cam_perspective->Activate();
	m_lighting.Activate();
	p->activate();
	
	glDisable(GL_DEPTH_TEST);
	
	switch(m_showmodel){
		case 0:
			m_model->restoreTexture();
			m_model->drawPass();
			break;
		case 1:
			glEnable(GL_DEPTH_TEST);
			m_lighting.Deactivate();
			glColorMask(0,0,0,0);
			glClear(GL_DEPTH_BUFFER_BIT);
			m_model->drawFaces();
			glColorMask(1,1,1,1);
			glLineWidth(3);
			glColor3f(1.0,1.0,1.0);
			m_model->drawEdges();
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
			m_model->drawTexturedFaces();
			m_shadeCompare->setUniform("textured", false);
			m_model->drawUntexturedFaces();
			m_shadeCompare->unbind();
			break;
		default:
			m_showmodel = 0;
			break;			
	}

	glEnable(GL_DEPTH_TEST);
	m_lighting.Deactivate();
	
// 	m_model->drawNormals();
	p->deactivate();
}

// Plots pdf in x-y plane (with z and rotational DOF locked)
vector<float> TextureTracker::getPDFxy(	Particle pose,
																				float x_min, float y_min,
																				float x_max, float y_max,
																				int res,
																				const char* filename, const char* filename2)
{
	int i = 0;
	printf("Evaluating PDF constrained to x,y movement only\n");
	float x_step = (x_max-x_min) / res;
	float y_step = (y_max-y_min) / res;
	
	float scale = 0.1;
	
	Particle pSearch = pose;
	x_min = (pSearch.s.x += x_min);
	y_min = (pSearch.s.y += y_min);
	
	vector<float> vPDF;
	vPDF.assign(res*res, 0.0);
	
	Frustum* frustum = g_Resources->GetFrustum();
	
	float p;
	
	i=0;
	pSearch.s.y = y_min;
	for(int n=0; n<res; n++){
		pSearch.s.x = x_min;
		for(int m=0; m<res; m++){
			if(frustum->PointInFrustum(pSearch.s.x, pSearch.s.y, pSearch.s.z)){
				evaluateParticle(&pSearch);
				p = pSearch.c * scale;		
			}else{
				p = 0.0;
			}
			vPDF[i] = p;
			
			i++;
			pSearch.s.x += x_step;
		}
		pSearch.s.y += y_step;
	}
	
 	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glBegin(GL_QUADS);
		glColor4f(1.0,0.0,0.0,0.5);
		glVertex3f(x_min, y_min, 0.0);
		glVertex3f(x_min+x_step*res, y_min, 0.0);
		glVertex3f(x_min+x_step*res, y_min+y_step*res, 0.0);
		glVertex3f(x_min, y_min+y_step*res, 0.0);
	glEnd();
	glDisable(GL_BLEND);
	
	swap();
	usleep(3000000);
	
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

