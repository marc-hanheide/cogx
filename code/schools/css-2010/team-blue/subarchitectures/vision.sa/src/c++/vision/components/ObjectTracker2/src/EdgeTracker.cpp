
#include "EdgeTracker.h"

// *** PRIVATE ***

// Process camera image (gauss, sobel, thinning, spreading, rendering)
void EdgeTracker::image_processing(unsigned char* image){

	// Load camera image to texture
	m_tex_frame->load(image, params.width, params.height);
		
	// Preprocessing for camera image
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(0);
	glColor3f(1,1,1);
	m_ip->flipUpsideDown(m_tex_frame, m_tex_frame);
	m_ip->gauss(m_tex_frame, m_tex_frame_ip[0]);
	m_ip->sobel(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	m_ip->thinning(m_tex_frame_ip[0], m_tex_frame_ip[0]);
	glDepthMask(1);
}

// *** PUBLIC ***

EdgeTracker::EdgeTracker(){
	m_lock = false;
	m_showmodel = true;
	m_zero_pose = false;
	m_draw_edges = false;
	m_tracker_initialized = false;
	m_bfc = false;
}

// Initialise function (must be called before tracking)
bool EdgeTracker::initInternal()
{	
	
	return (m_tracker_initialized = true);
}

bool EdgeTracker::track(	unsigned char* image,
							Model* model,
							Camera* camera,
							Pose p_estimate,
							Pose& p_result,
							float time)
{
	
	
	// Check if input is valid
	isReady(image, model, camera, &p_estimate);	
	
	// Process image from camera (edge detection)
	image_processing(image);
	
	// Orthogonal edge searching
	p_result = p_estimate;
	time_tracking = 0.0;
	m_iterations = 0;
	m_timer.Update();
	if(!m_lock){
		while(time_tracking < time){
			p_result = m_ortho_search->findPose(p_result, m_tex_frame_ip[0], model, NULL, m_cam_perspective);
			time_tracking += m_timer.Update();
			m_iterations++;
		}
	}
	
	if(m_draw_edges)
		m_ip->render(m_tex_frame_ip[0]);
	else
		m_ip->render(m_tex_frame);
	
	if(m_zero_pose){
		p_result = params.zP;
		m_zero_pose = false;
	}
	
	return true;

}

// Draw result of edge tracking (particle with maximum likelihood)
void EdgeTracker::drawResult(Pose* p){
	m_cam_perspective->SetViewport(params.width, params.height);
	m_cam_perspective->Activate();
	
	p->activateGL();
		glClear(GL_DEPTH_BUFFER_BIT);
		glColorMask(0,0,0,0);
		m_model->drawFaces();
		glColorMask(1,1,1,1);
		
		glColor3f(1.0,0.0,0.0);
		glLineWidth(3);
		m_model->drawEdges();
	p->deactivateGL();
}


