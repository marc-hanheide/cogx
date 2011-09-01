
#include <v4r/Tracker/utils.hpp>
#include <v4r/TomGine/tgFont.h>
#include <iostream>

using namespace V4R;
using namespace TomGine;

int main(int argc, char *argv[]){

	printUsage();
	
// *************************************************************************************
// Initialisation

	// Read inifile
	string ini_file = string(V4R_DIR) + "/v4r/Tracker/example/Resources/tracking.ini";
	string cam_cal_file = string(V4R_DIR) + "/v4r/Tracker/example/Resources/cam.cal";
	string pose_cal_file = string(V4R_DIR) + "/v4r/Tracker/example/Resources/pose.cal";
	string video_file = string(V4R_DIR) + "/v4r/Tracker/example/Resources/0028.avi";

	int width = getCamWidth(cam_cal_file.c_str());
	int height = getCamHeight(cam_cal_file.c_str());

	// Initialize camera capture using opencv
	g_Resources->InitCapture(video_file.c_str());
	IplImage* img = g_Resources->GetNewImage();

	// Initialise OpenGL Window
	GLWindow m_window(width, height, "Tracking");

	// initialize tracker
//	Tracker* m_tracker = new EdgeTracker();
	Tracker* m_tracker = new TextureTracker();
	Tracker::Parameter track_params;
	GetTrackingParameter(track_params, ini_file.c_str());
	GetCameraParameter(track_params.camPar, cam_cal_file.c_str(), pose_cal_file.c_str());

	if(!m_tracker->init(track_params)){
		printf("Failed to initialise tracker!\n");
		return 1;
	}

	// Load model
	int id_2;
	tgPose p, pose;
	p.Rotate(0.0f, -1.57f, 0.0f);
	p.Translate(-0.0f, 0.23f, 0.05f);
	std::string plypath = getModelPath(ini_file.c_str());
	std::string plyfile = getModelFile(ini_file.c_str());
	std::string plyname = plyfile;
	plyname.erase(plyname.begin()+plyname.find("."), plyname.end());
	plyfile.insert(0,plypath);

	id_2 = m_tracker->addModelFromFile(plyfile.c_str(), p, plyname.c_str(), true);

	bool quit = false;
	TrackingState ts;

// *************************************************************************************
// Main Loop
	Event event;
// 	quit = true;
	while( !quit ){

		// grab new image from camera
		img = g_Resources->GetNewImage();

		// image processing
		m_tracker->image_processing((unsigned char*)img->imageData);

		// tracking (particle filtering)
		m_tracker->track(id_2);

		// Draw result
		m_tracker->drawImage();
		m_tracker->drawResult(2.0f);
 		m_tracker->drawCoordinateSystem(0.2f, 2.0f);

		m_tracker->getModelTrackingState(id_2, ts);

		{
			glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
			if(ts == ST_OK)
				g_font->Print("state: ok", 14, 10, 10);
			else if(ts == ST_OCCLUDED)
				g_font->Print("state: occluded", 14, 10, 10);
			else if(ts == ST_LOST)
				g_font->Print("state: lost", 14, 10, 10, 1,0,0);
			else if(ts == ST_LOCKED)
				g_font->Print("state: locked", 14, 10, 10);

			ostringstream os;
			os << "quality: " << unsigned(100*m_tracker->getModelEntry(id_2)->c_normalized) << "%";
			g_font->Print(os.str().c_str(), 14, 10, 25);
			os.str("");
			os << "speed: " << unsigned(100*(1.0-m_tracker->getModelEntry(id_2)->w_msp)) << "%";
			g_font->Print(os.str().c_str(), 14, 10, 40);
		}

		// Update OpenGL Window
		m_window.Update();

		// get keyboard events and process them
		while(m_window.GetEvent(event))
			quit = !InputControl(m_tracker, event);

	}
	
// *************************************************************************************
// Stop and destroy
	delete(m_tracker);
	printf("\n... done\n\n");

	return 0;
}

