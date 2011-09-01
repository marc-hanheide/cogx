

#include <v4r/Tracker/utils.hpp>
#include <iostream>

using namespace V4R;
using namespace TomGine;

int main(int argc, char *argv[]){

	printUsage();
	
// *************************************************************************************
// Initialisation

	// Read inifile
	string ini_file = string(V4R_DIR) + "/apps/testsTracker/Resources/tracking.ini";
	string cam_cal_file = string(V4R_DIR) + "/apps/testsTracker/Resources/cam.cal";
	string pose_cal_file = string(V4R_DIR) + "/apps/testsTracker/Resources/pose.cal";

	int width = getCamWidth(cam_cal_file.c_str());
	int height = getCamHeight(cam_cal_file.c_str());
	int wplot = 800;
	int hplot = 800;
	int whist = 800;
	int hhist = 800;
	int wtomgine = 640;
	int htomgine = 480;

	// Initialize camera capture using opencv
	g_Resources->InitCapture();
	IplImage* img = g_Resources->GetNewImage();
	TomGine::tgTimer m_timer;

	// Initialise OpenGL Window
	TomGine::tgEngine m_plotwindow(wplot,hplot, 1.0f, 0.01f, "Plot");
//	TomGine::tgEngine m_histwindow(whist,hhist, 1.0f, 0.01f, "Histogram");
//	TomGine::tgEngine m_tomgine(wtomgine,htomgine,1.0f, 0.0001f, "TomGine");
	GLWindow m_window(width, height, "Tracking");

	// initialize tracker
//	Tracker* m_tracker = new EdgeTracker();
	Tracker* m_tracker = new TextureTracker();
	Tracker::Parameter track_params;
	GetTrackingParameter(track_params, ini_file.c_str());
	GetCameraParameter(track_params.camPar, cam_cal_file.c_str(), pose_cal_file.c_str());
	track_params.camPar.pos = vec3(0.0,0.0,0.0);
	track_params.camPar.rot = mat3();

	if(!m_tracker->init(track_params)){
		printf("Failed to initialise tracker!\n");
		return 1;
	}

	// initialze plot 2d
	TomGine::tgPlot2D m_plotc(50,50, wplot-100, 400-100);
	m_plotc.Axis(0.0, 10.0, 0.0, 1.0);
	TomGine::tgPlot2D m_plots(50,450, wplot-100, 400-100);
	m_plots.Axis(0.0, 10.0, 0.0, 1.0);

//	TomGine::tgPlot2D m_hist_model(50,50,256,256);
//	m_hist_model.Axis(0.0, 360.0, 0.0, 1.0);
//	TomGine::tgPlot2D m_hist_image(50,(hhist>>1)+50,256,256);
//	m_hist_image.Axis(0.0, 360.0, 0.0, 1.0);

	TomGine::tgModel m_pcl;

	// Load model
	int id_2;
	tgPose p;
//	p.Rotate(0.0f, 0.0f, 0.0f);
//	p.Translate(0.0f, 0.0f, 0.5f);
	vec3 iniax = vec3(-0.722563, -0.664015, -0.192318);
	float iniang = 0.815780;
	p.q.fromAxis(iniax, iniang);
	p.t = vec3(-0.045862, 0.013617, 0.614706);
	std::string plypath = getModelPath(ini_file.c_str());
	std::string plyfile = getModelFile(ini_file.c_str());
	std::string plyname = plyfile;
	plyname.erase(plyname.begin()+plyname.find("."), plyname.end());
	plyfile.insert(0,plypath);

	id_2 = m_tracker->addModelFromFile(plyfile.c_str(), p, plyname.c_str(), true);


	TrackingState ts;
	ModelEntry* modelentry = m_tracker->getModelEntry(id_2);

// *************************************************************************************
// Main Loop
	Event event;
	bool quit = false;
	while( !quit ){

// ******** ******** ********
//			Tracker
// ******** ******** ********
//printf("[main::Tracker] A\n");
		// grab new image from camera
		img = g_Resources->GetNewImage();

		m_window.Activate();

		// image processing
		m_tracker->image_processing((unsigned char*)img->imageData);

		// tracking (particle filtering)
		m_tracker->track(id_2);

		// draw result
		m_tracker->drawImage();
		m_tracker->drawResult(1.0);

		m_tracker->getModelTrackingState(id_2, ts);
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
		os << "quality: " << unsigned(100*modelentry->c_normalized) << "%";
		g_font->Print(os.str().c_str(), 14, 10, 25);
		os.str("");
		os << "static: " << unsigned(100*modelentry->w_msp) << "%";
		g_font->Print(os.str().c_str(), 14, 10, 40);


		// Update OpenGL Window
		// get keyboard events and process them

		while(m_window.GetEvent(event))
			quit = !InputControl(m_tracker, event);
		m_window.Update();
//		continue;

// ******** ******** ********
//			Plots
// ******** ******** ********
//printf("[main::Plots] A\n");
		m_plotwindow.m_window->Activate();
		m_plotwindow.Activate2D();

		float c_mkp = modelentry->c_mkp;
		float c_normalized = modelentry->c_normalized;
		float c_occ = modelentry->c_occ;
		float c_comp = modelentry->c_comp;	///< movement compensated confidence
		float c_occ_comp = modelentry->c_occ_comp;			///< movement and occlusion compensated confidence
		float w_msp = modelentry->w_msp;


		// Plot2D Confidences
		m_plotc.DrawAxis();
		m_plotc.Push(c_mkp, c_normalized, c_comp, c_occ_comp);
		m_plotc.DrawLegend(wplot-150, 400-50, "c_mean", "c_normalized", "c_comp", "c_comp_occ");

		// Plot2D Histogram Variance
		m_plots.DrawAxis();
		m_plots.Push( c_occ, w_msp );
		m_plots.DrawLegend(wplot-150, 800-50, "c_occ", "w_msp");

		m_plotwindow.Update();


// ******** ******** ********
//			HISTOGRAM
// ******** ******** ********
//printf("[main::HISTOGRAM] A\n");
//		m_histwindow.m_window->Activate();
//		m_histwindow.Activate2D();
//		glLineWidth(1.0);
//
//		{
//			std::vector< TomGine::tgPlot2D > m_plothgi, m_plothgm, m_plothci, m_plothcm;
//			unsigned d = 50;
//			unsigned plcnt = 0;
//			g_font->Print("grad image", 20, 20, 250);
//			g_font->Print("grad model", 20, 20, 550);
//
//
//
//			for(unsigned segy=0; segy<desc_g_model.m_nsegy; segy++)
//			{
//				for(unsigned segx=0; segx<desc_g_model.m_nsegx; segx++)
//				{
//					m_plothgi.push_back( TomGine::tgPlot2D( segx*d+20, segy*d+20, 40, 40 ) );
//					m_plothgm.push_back( TomGine::tgPlot2D( segx*d+20, segy*d+320, 40, 40 ) );
//					m_plothci.push_back( TomGine::tgPlot2D( segx*d+20+400, segy*d+20, 40, 40 ) );
//					m_plothcm.push_back( TomGine::tgPlot2D( segx*d+20+400, segy*d+320, 40, 40 ) );
//
//
//					unsigned idx = segy * desc_g_model.m_nsegx + segx;
//					sprintf(cbuffer, "%.3f", isec_g[idx]);
//					g_font->Print(cbuffer, 10, segx*d+20, segy*d+10);
//					sprintf(cbuffer, "%.3f", isec_c[idx]);
//					g_font->Print(cbuffer, 10, segx*d+20+400, segy*d+10);
//					sprintf(cbuffer, "%.3f", isec_print[idx]);
//					g_font->Print(cbuffer, 10, segx*d+20+400, segy*d+310);
//				}
//			}
//			for(unsigned i=0; i<desc_g_model.m_histlist.size(); i++){
//				m_plothgi[i].DrawHistogram(desc_g_image.m_histlist[i]);
//				m_plothgm[i].DrawHistogram(desc_g_model.m_histlist[i]);
//				m_plothci[i].DrawHistogram(desc_c_image.m_histlist[i]);
//				m_plothcm[i].DrawHistogram(desc_c_model.m_histlist[i]);
//			}
//		}
//
//		m_histwindow.Update();


// ******** ******** ********
//			TomGine
// ******** ******** ********
//printf("[main::TomGine] A\n");
//		m_tomgine.m_window->Activate();
//
//		Particle pmean = modelentry->distribution.getMean();
//		pmean.q.fromMatrix(mat4());
//
//		// Draw particle cloud
//		m_pcl.Clear();
//		TomGine::tgColorPoint colpt;
//		TomGine::tgLine lpt;
//		vec3 axis;
// 		float angle;
// 		float axs = 0.001;
//		int n_pts = modelentry->distribution.size();
//		for(int i=0; i<n_pts; i++){
//			Particle pi = modelentry->distribution[i];
//			colpt.pos = pi.t;
//			colpt.color[0] = 255-pi.c*255;  colpt.color[1] = pi.c*255; colpt.color[2] = 0;
//			pi.q.getAxisAngle(axis, angle);
//			lpt.start = pi.t;
//			lpt.end = pi.t + axis*axs*angle;
//			m_pcl.m_colorpoints.push_back(colpt);
//			m_pcl.m_lines.push_back(lpt);
//		}
//		glPointSize(2.0);
//		m_pcl.DrawColorPoints();
//		glLineWidth(0.5);
//		m_pcl.DrawLines(vec3(1.0, 0.0, 0.0));
//
//		// Draw model wireframes and axis of rotation
//		tgPose pose = modelentry->pose;
//		tgPose p_no_rot = pose;
//		p_no_rot.q.fromAxis(iniax, iniang);
//		p_no_rot.Activate();
//		modelentry->model.drawEdges();
//		p_no_rot.Deactivate();
//		pose.Activate();
//		glColor3f(0.0f,1.0f,0.0f);
//		modelentry->model.drawEdges();
//		pose.Deactivate();
//
//		pmean.DrawCoordinates(0.01,1.0);
//		m_tomgine.DrawCoordinates(0.1,2.0);
//		m_tomgine.SetCenterOfRotation(pmean.t.x,pmean.t.y,pmean.t.z);
//		m_tomgine.Update();
//
//		std::vector<V4R::Event> eventlist;
//		m_tomgine.GetEventList(eventlist);
//		for(unsigned i=0; i<eventlist.size(); i++){
//			switch(eventlist[i].type){
//			case TMGL_Press:
//				switch(eventlist[i].input){
//				case TMGL_l:
//					tgCamera cam = m_tomgine.GetCamera0();
//					cam.LookAt(pmean.t);
//					m_tomgine.SetCamera(cam);
//					m_tomgine.UpdateCameraViews(cam);
//					break;
//				}
//				break;
//			}
//		}

	}
	
// *************************************************************************************
// Stop and destroy
	m_window.Activate();	// necessary for shader to be deleted in the right GL context.
	delete(m_tracker);
	printf("\n... done\n\n");

	return 0;
}

