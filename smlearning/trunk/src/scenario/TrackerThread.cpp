/** @file TrackerThread.cpp
 * 
 * 
 * @author	Manuel Noll (DFKI)
 * @author      Thomas Mörwald
 *
 * @version 1.0
 *
 * 2011      Manuel Noll
 
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

namespace smlearning {

/////////////PUBLIC/////////////
/** \brief std cto initializing the thread
*/
TrackerThread::TrackerThread(const std::string& ini_file, const std::string& cam_cal_file, const std::string& pose_cal_file )
{
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

	// Read inifile
	
	int width = getCamWidth(cam_cal_file.c_str());
	int height = getCamHeight(cam_cal_file.c_str());
	
	font = new TomGine::tgFont ("../Resources/fonts/comic.ttf");

	// Initialize camera capture using opencv
	g_Resources->InitCapture(float(width), float(height));
	//g_Resources->InitCapture("../Resources/videos/0028.avi");
	_img = g_Resources->GetNewImage();

	// Initialise OpenGL Window

	_m_window = new GLWindow(width, height, "Tracking");

	// Initialize tracker
	//Tracker* m_tracker = new EdgeTracker();
	_m_tracker = new TextureTracker();
	if(! _m_tracker->init(ini_file.c_str(), cam_cal_file.c_str(), pose_cal_file.c_str())){
		printf("Failed to initialise tracker!\n");
		return;
	}
	//Load model
	
	tgPose p;
	p.Rotate(0.0f, PI*0.5f, 0.0f);
	//p.Rotate(0.0f, 0.0f, 0.5f);
	p.Translate(0.0f, 0.0f, 0.0f);
	//p.Translate(0.0f, 0.1f, 0.0f);
	//p.t = vec3(0.037, 0.032, 0.06);
	std::string plypath = getModelPath(ini_file.c_str());
	std::string plyfile = getModelFile(ini_file.c_str());
	std::string plyname = plyfile;
	plyname.erase(plyname.begin()+plyname.find("."), plyname.end());
	plyfile.insert(0,plypath);
	

  	_id_2 = _m_tracker->addModelFromFile(plyfile.c_str(), p, plyname.c_str(), true);
	
// 	ModelLoader m_loader;
// 	TomGine::tgModel m_model;
// 	m_loader.LoadPly(m_model, plyfile.c_str());
// 	_id_2 = m_tracker->addModel(m_model, p, plyname.c_str(), true);

	_quit = false;

	// _new_position = false;


}

/////////////PUBLIC/////////////
/** \brief std destructor 
*/
TrackerThread::~TrackerThread()
{

// Stop and destroy
	delete(_m_tracker);
	delete font;
	printf("\n... done\n\n");
	this->Stop();
}

/////////////PUBLIC/////////////
/** \brief executes the former standalone.cpp loop as the thread task, updating the GL window and the pose of the object
*/
BOOL TrackerThread::OnTask()
{

// Main Loop
	blortGLWindow::Event event;
	while( ! _quit ){
		
		// grab new image from camera
		_img = g_Resources->GetNewImage();
		
		// Image processing
		//m_tracker->image_processing_occluder((unsigned char*)img->imageData, model_occ, p_occ);
		_m_tracker->image_processing((unsigned char*)_img->imageData);
		
		// Tracking (particle filtering)
		_m_tracker->track(_id_2);
		
		// store the updated position of the object
		_m_tracker->getModelPose(_id_2,_object_pose);
		_new_position = true;

		// Draw result
		//m_tracker->drawImage(0);
		_m_tracker->drawResult(2);
		_m_tracker->drawCoordinates();
		
		_m_tracker->getModelMovementState(_id_2, _movement);
		_m_tracker->getModelQualityState(_id_2, _quality);
		_m_tracker->getModelConfidenceState(_id_2, _confidence);
		
		
		glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
		if(_movement == ST_FAST)
			font->Print("fast", 20, 10, 50);
		else if(_movement == ST_SLOW)
			font->Print("slow", 20, 10, 50);
		else if(_movement == ST_STILL)
			font->Print("still", 20, 10, 50);
			
		if(_confidence == ST_GOOD)
			font->Print("good", 20, 10, 30);
		else if(_confidence == ST_FAIR)
			font->Print("fair", 20, 10, 30);
		else if(_confidence == ST_BAD)
			font->Print("bad", 20, 10, 30);
		
		if(_quality == ST_OK)
			font->Print("ok", 20, 10, 10);
		else if(_quality == ST_OCCLUDED)
			font->Print("occluded", 20, 10, 10);
		else if(_quality == ST_LOST)
			font->Print("lost", 20, 10, 10, 1,0,0);
		else if(_quality == ST_LOCKED)
			font->Print("locked", 20, 10, 10);
		
		_m_window->Update();
		
		while( _m_window->GetEvent(event) )
			_quit = !InputControl( _m_tracker, event );
	}
	return TRUE;
}

/////////////PUBLIC/////////////
/** \brief returns the current position of the object as 3x3 rotation matrix and translation vector by reference
*/
void TrackerThread::getPose(mat3& matrix, vec3& vector) const
{
	// if (_new_position)
	// {
		_object_pose.GetPose(matrix,vector);
		// _new_position=false;
	// }
}

}; // namespace smlearning 
