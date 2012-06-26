/** @file TrackerThread.cpp
 * 
 * @author      Sergio Roa (DFKI) 
 * @author	Manuel Noll (DFKI)
 * @author      Thomas Mörwald
 *
 * @version 1.0
 *
 *
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

*/

#include <scenario/TrackerThread.h>
#include <iostream>

using namespace std;
using namespace TomGine;
using namespace blortGLWindow;
using namespace Tracking;

namespace smlearning {

/////////////PUBLIC/////////////
/** \brief std cto initializing the thread
*/
TrackerThread::TrackerThread(const std::string tracker_ini, const std::string cam_ini, const std::string pose_ini, const tgPose initial_object_pose )
{
	tracker_ini_file = tracker_ini;
	cam_ini_file = cam_ini;
	pose_ini_file = pose_ini;
	m_track_pose = initial_object_pose;

}

/////////////PUBLIC/////////////
/** \brief std destructor 
*/
TrackerThread::~TrackerThread()
{

	m_mutex.Lock ();
	// {
	// 	golem::CriticalSectionWrapper csw (cs);
		_quit = true;
 	// }
	m_mutex.Unlock ();
	// m_evData.Set ();
	m_running.Wait ();
// Stop and destroy
	// thread.join ();
	printf("\n... done\n\n");
	// this->Stop();
}

/*void TrackerThread::start ()
{
	thread.start (this);
}*/

/////////////PUBLIC/////////////
/** \brief executes the former standalone.cpp loop as the thread task, updating the GL window and the pose of the object
*/
BOOL TrackerThread::OnTask()
{
// void TrackerThread::run () {

	m_running.Lock ();
	printf("\n Demo Tracker\n\n");

	printf(" Tracking control\n");
	printf(" -------------------------------------------\n");
	printf(" [q,Esc] Quit program\n");
	printf(" [4] Texture edge tracking\n");
	printf(" [5] Texture color tracking\n");
	printf(" [a] High accuracy tracking (less robust)\n");
	printf(" [e] Show edges of image\n");
	printf(" [i] Print tracking information to console\n");
	printf(" [l] Lock/Unlock tracking\n");
	printf(" [m] Switch display mode of model\n");
	printf(" [p] Show particles\n");
	printf(" [r] Reset tracker to initial pose\n");
	printf(" [s] Save model to file (including textures)\n");
	printf(" [t] Capture model texture from image\n");
	printf(" [u] Untexture/remove texture from model\n");
	printf(" \n\n");
	
// *************************************************************************************
// Initialisation

	// PLY Model
	m_plypath = getModelPath(tracker_ini_file.c_str());
	std::string m_plyfile = getModelFile(tracker_ini_file.c_str());
	std::string m_plyname = m_plyfile;
	m_plyname.erase(m_plyname.begin()+m_plyname.find("."), m_plyname.end());
	m_plyfile.insert(0, m_plypath);
	
	int cam_width = getCamWidth(cam_ini_file.c_str());
	int cam_height = getCamHeight(cam_ini_file.c_str());

	GetTrackingParameter(trackParams, tracker_ini_file.c_str());
	// pThreadCamera = new CCameraThread(0, trackParams.camPar.width, trackParams.camPar.height);
	// pThreadCamera = new CCameraThread(0, cam_width, cam_height);
	// pThreadCamera->SetThreadType(ThreadTypeIntervalDriven,0);

	// Initialize camera capture using opencv
	g_Resources->InitCapture(float(cam_width), float(cam_height));
	_img = g_Resources->GetNewImage();
	// _img = cvCreateImage( cvSize(cam_width, cam_height), 8, 3 );

	
	// glWindow.reset(new blortGLWindow::GLWindow(cam_width,cam_height,"Tracking"));
	glWindow = new blortGLWindow::GLWindow(cam_width,cam_height,"Tracking");
	
	// m_tracker.reset(new Tracking::TextureTracker());
	m_tracker = new Tracking::TextureTracker();
	// m_tracker = new Tracking::TextureTracker ();
	if(!m_tracker->init(tracker_ini_file.c_str(), cam_ini_file.c_str(), pose_ini_file.c_str()))
	// if(!m_tracker->init(trackParams))
		cout << "TrackerPredOffline::create(): Failed to initialise tracker" << endl;
	
	// m_initialPose = TomGine::tgPose();
	// m_initialPose.Rotate (0.0f, 0.0f, 0.0f);
	// m_initialPose.Translate (0.26f, 0.2f, 0.05f);
	// m_track_pose = TomGine::tgPose();
	// m_track_pose.Rotate (0.0f, 0.0f, 0.0f);
	// m_track_pose.Translate (0.01f, 0.265f, 0.05f);
	// m_trackpred_id = m_tracker->addModelFromFile(m_plyfile.c_str(), m_initialPose, m_plyname.c_str());
	m_track_id = m_tracker->addModelFromFile(m_plyfile.c_str(), /*m_initialPose*/m_track_pose, m_plyname.c_str());
	// m_ground_id = m_tracker->addModelFromFile(m_plyfile.c_str(), m_initialPose, m_plyname.c_str());
	
	Tracking::ModelLoader m_ply_loader;
	m_ply_loader.LoadPly(m_object, m_plyfile.c_str());

	_quit = false;



	// Main Loop
	blortGLWindow::Event event;
	while(!_quit){
		// golem::CriticalSectionWrapper csw (cs);
		// printf("Entering loop...\n");
		// grab new image from camera
		// m_mutex.Lock ();

		_img = g_Resources->GetNewImage();
		// pThreadCamera->GetImage(_img);
		
		// Image processing
		//m_tracker->image_processing_occluder((unsigned char*)img->imageData, model_occ, p_occ);
		m_tracker->image_processing((unsigned char*)_img->imageData);
		
		// Tracking (particle filtering)
		m_tracker->track(m_track_id);
		
		// store the updated position of the object
		m_tracker->getModelPose(m_track_id,m_track_pose);
		// _new_position = true;

		// Draw result
	        m_tracker->drawImage(0);
		m_tracker->drawResult(2.0f);
		// m_tracker->drawCoordinateSystem(0.2f, 2.0f);
		m_tracker->drawCoordinates ();
		
		m_tracker->getModelMovementState(m_track_id, _movement);
		m_tracker->getModelQualityState(m_track_id, _quality);
		m_tracker->getModelConfidenceState(m_track_id, _confidence);
		
		
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		if(_movement == ST_FAST)
			font.Print("fast", 20, 10, 50);
		else if(_movement == ST_SLOW)
			font.Print("slow", 20, 10, 50);
		else if(_movement == ST_STILL)
			font.Print("still", 20, 10, 50);
			
		if(_confidence == ST_GOOD)
			font.Print("good", 20, 10, 30);
		else if(_confidence == ST_FAIR)
			font.Print("fair", 20, 10, 30);
		else if(_confidence == ST_BAD)
			font.Print("bad", 20, 10, 30);
		
		if(_quality == ST_OK)
			font.Print("ok", 20, 10, 10);
		else if(_quality == ST_OCCLUDED)
			font.Print("occluded", 20, 10, 10);
		else if(_quality == ST_LOST)
			font.Print("lost", 20, 10, 10, 1,0,0);
		else if(_quality == ST_LOCKED)
			font.Print("locked", 20, 10, 10);

		glWindow->Update();

		while( glWindow->GetEvent(event) && !_quit )
			_quit = !InputControl( m_tracker, event );
		
		// m_evData.Set ();
		// m_mutex.Unlock ();
	}
	if (m_tracker)
		delete m_tracker;
	if (glWindow)
		delete glWindow;
	if (_img)
		cvReleaseImage (&_img);
	m_running.Unlock ();
	return TRUE;



}

/////////////PUBLIC/////////////
/** \brief returns the current position of the object as 3x3 rotation matrix and translation vector by reference
*/
TomGine::tgPose TrackerThread::getPose(/*mat3& matrix, vec3& vector*/) const
{
	// if (_new_position)
	// {
		return m_track_pose;
		// _new_position=false;
	// }
}

}; // namespace smlearning 
