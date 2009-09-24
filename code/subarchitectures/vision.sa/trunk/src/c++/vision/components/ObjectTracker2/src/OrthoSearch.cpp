
#include "OrthoSearch.h"

/*** PRIVATE ***/

// Perturbs pose about reference position
void OrthoSearch::perturb(){
	Quaternion q2;
	
	// Assign perturbing factors
	m_pf[0] = (1-m_p[0].w) * m_w2rad;
	m_pf[1] = (1-m_p[0].w) * m_w2rad;
	m_pf[2] = (1-m_p[0].w) * m_w2rad;
	m_pf[3] = (1-m_p[0].w) * m_w2m;
	m_pf[4] = (1-m_p[0].w) * m_w2m;
	m_pf[5] = (1-m_p[0].w) * m_w2m;
	m_pf[6] = (1-m_p[0].w) * m_w2z;	// zoom
	
	// Set all 6 poses to reference pose
	for(int i=1; i<15; i++)
		m_p[i] = m_p[0];
	
	// Perturb
	m_p[1].rotate( m_pf[0], 0, 0);
	m_p[2].rotate(-m_pf[0], 0, 0);
	m_p[3].rotate( 0, m_pf[1], 0);
	m_p[4].rotate( 0,-m_pf[1], 0);
	m_p[5].rotate( 0, 0, m_pf[2]);
	m_p[6].rotate( 0, 0,-m_pf[2]);
	m_p[7].translate( m_pf[3], 0, 0);
	m_p[8].translate(-m_pf[3], 0, 0);
	m_p[9].translate( 0, m_pf[4], 0);
	m_p[10].translate( 0,-m_pf[4], 0);
	m_p[11].translate( 0, 0, m_pf[5]);
	m_p[12].translate( 0, 0,-m_pf[5]);
	m_p[13].translate( m_pf[6], m_pf[6], m_pf[6]);
	m_p[14].translate(-m_pf[6],-m_pf[6],-m_pf[6]);	
}

// Calculates likelihood using GPU (edges only)
void OrthoSearch::drawPoses(bool useShader){
	unsigned int i;
	unsigned int vp = 128;
	
	// render settings
	glClear(GL_COLOR_BUFFER_BIT);
	glDepthMask(1);	
	glClear(GL_DEPTH_BUFFER_BIT);
	glLineWidth(2);
	m_texFrame->bind();	
	m_cam_perspective->Activate();
	glColor4f(1.0,1.0,1.0,1.0);
	
	// Render edges of reference and the 12 variations to framebuffer
	for(i=0; i<15; i++){
		glViewport( (i-4*(i/4))*vp, vp*(i/4), vp, vp);	// move viewport over framebuffer
		glClear(GL_DEPTH_BUFFER_BIT);
		m_p[i].activateGL();
			
			glColorMask(0,0,0,0);
			m_model->drawFaces();		// render faces to depthbuffer for hidden edge removal
			
			glColorMask(1,1,1,1);
			if(useShader) m_shadeOrthoSearch->bind();
			m_model->drawEdges();
			if(useShader) m_shadeOrthoSearch->unbind();
		m_p[i].deactivateGL();		
	}
}

/*
// Calculates likelihood using GPU (textured)
void OrthoSearch::drawPoses(Shader* shader){
	unsigned int i;
	unsigned int vp = 128;
	
	// render settings
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	glColor4f(1.0,1.0,1.0,1.0);
	m_cam_perspective->Activate();
	m_texFrame->bind(1);
	
	// Render edges of reference and the 12 variations to framebuffer
	shader->bind();
	shader->setUniform("modelviewprojection", m_modelviewprojection, GL_FALSE);
	for(i=0; i<15; i++){
		glViewport( (i-4*(i/4))*vp, vp*(i/4), vp, vp);	// move viewport over framebuffer
		m_p[i].activateGL();
			m_model->drawFaces();		// render faces to depthbuffer for hidden edge removal
		m_p[i].deactivateGL();		
	}
	shader->unbind();
}
*/
// Calculates gradient to solution using likelihood information
void OrthoSearch::getGradient(){	
	float gl=0.0;					// gradient length
	unsigned int ig=0;				// index of gradient
	double p=0.0, n=0.0;			// positive and negative poses	
	vector<double> vfSums, vfMax;	// sums
	
	// calculate sums of pixel (average = total luminance)
	// framebuffer = source
	/*
	drawPoses(m_shadeOrthoSearchTex);
	vfSums = g_Resources->GetImageProcessor()->luminance(m_texResult,9,2);
	drawPoses(m_shadeOrthoTexEdges);
	vfMax = g_Resources->GetImageProcessor()->luminance(m_texResult,9,2);
	*/
	
	drawPoses(true);	// draw poses with shaders = luminance with respect to edge distance and gradients
	vfSums = g_Resources->GetImageProcessor()->luminance(m_texResult,9,2);
	drawPoses(false);	// draw poses without shaders = maximum luminance of each pose
	vfMax = g_Resources->GetImageProcessor()->luminance(m_texResult,9,2);
	
	// calculate confidence level by dividing by sum of maximal luminance
	for(unsigned int i=0; i<15; i++){
		if(vfMax[i] < 0.001)
			m_p[i].w = 0.0;
		else
			m_p[i].w = vfSums[i]/vfMax[i] + 0.1*(vfMax[i] - vfMax[0]);
	}
	
	for(ig=0; ig<7; ig++){
		p = m_p[2*ig+1].w - m_p[0].w;
		n = m_p[2*ig+2].w - m_p[0].w;
		
		m_gradient[ig] = (p-n)*0.5;
	}
		
}

// Moves pose in gradient direction (towards solution)
void OrthoSearch::movePose(){

	m_p[0].rotate( 	m_gradient[0]*m_w2rad,
					m_gradient[1]*m_w2rad,
					m_gradient[2]*m_w2rad);
	
	m_p[0].translate(	m_gradient[3]*m_w2m,
						m_gradient[4]*m_w2m,
						m_gradient[5]*m_w2m);
	
	m_p[0].translate(	m_gradient[6]*m_w2z,
						m_gradient[6]*m_w2z,
						m_gradient[6]*m_w2z);						
}

// Stop condition for iteration loop
bool OrthoSearch::stopSearching(){
	m_fTime += m_timer.Update();
	
	if(m_fTime > m_fTimeThreshold)
		return true;
	
	return false;
}

/*** PUBLIC ***/
OrthoSearch::OrthoSearch(float width, float height){
	int id;
	m_fTimeThreshold = 0.03;
	m_fTime = 0.0;
	m_w2rad = 0.5;
	m_w2m = 0.01;
	m_w2z = 0.01;
	m_width = width;
	m_height = height;
	
	for(int i=0; i<13; i++)
		m_p[i] = 0.0;
	
	for(int i=0; i<6; i++){
		m_gradient[i] = 0.0;
		m_pf[i] = 0.0;
	}
	
	if((id = g_Resources->AddShader("orthosearch", "orthosearch.vert", "orthosearch.frag")) == -1)
		exit(1);
	m_shadeOrthoSearch = g_Resources->GetShader(id);
	m_shadeOrthoSearch->bind();
	m_shadeOrthoSearch->setUniform("dwidth", float(1.0/width));
	m_shadeOrthoSearch->setUniform("dheight", float(1.0/height));
	m_shadeOrthoSearch->setUniform("max_distance", float(15.0));
	m_shadeOrthoSearch->unbind();
	
	if((id = g_Resources->AddShader("orthosearchtex", "orthosearchtex.vert", "orthosearchtex.frag")) == -1)
		exit(1);
	m_shadeOrthoSearchTex = g_Resources->GetShader(id);
	m_shadeOrthoSearchTex->bind();
	m_shadeOrthoSearchTex->setUniform("tex_model", 0);
	m_shadeOrthoSearchTex->setUniform("tex_frame", 1);
	m_shadeOrthoSearchTex->setUniform("dwidth", float(1.0/width));
	m_shadeOrthoSearchTex->setUniform("dheight", float(1.0/height));
	m_shadeOrthoSearchTex->setUniform("max_distance", float(10.0));
	m_shadeOrthoSearchTex->setUniform("alpha_tol", float(30.0 * PI / 180.0));
	m_shadeOrthoSearchTex->unbind();
	
	if((id = g_Resources->AddShader("orthotexedges", "orthotexedges.vert", "orthotexedges.frag")) == -1)
		exit(1);
	m_shadeOrthoTexEdges = g_Resources->GetShader(id);
	
	if((id = g_Resources->AddTexture(NULL, "m_texResult")) == -1)
		exit(1);
	m_texResult = g_Resources->GetTexture(id);
}

// Finds pose of object given an estimate
Pose OrthoSearch::findPose(	Pose p_estimate,
							Texture* texFrame,
							Model* model,
							Texture* texModel,
							Camera* cam_perspective){
	m_fTime = 0.0;
	m_p[0] = p_estimate;	// p[0] = reference pose
	m_texFrame = texFrame;
	m_model = model;
	m_texModel = texModel;
	m_cam_perspective = cam_perspective;
	
	m_timer.Update();	// Start timer
	
	perturb();
	getGradient();
	movePose();
			
	return m_p[0];
}

