/**
 * @file StereoDetector.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "StereoDetector.h"
#include "StereoBase.h"


using namespace std;
using namespace VisionData;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoDetector();
  }
}

namespace cast
{


StereoDetector::~StereoDetector()
{
	cvDestroyWindow("Stereo left");
	cvDestroyWindow("Stereo right");
	cvDestroyWindow("Stereo results @ left stereo image");
}


void StereoDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
	StereoDetectionCommandPtr detect_cmd = getMemoryEntry<StereoDetectionCommand>(_wmc.address);
	
	log("received detection command ...");
	switch(detect_cmd->cmd){
		case VisionData::SDSTART:
			if(cmd_detect){
				log("stereo detection is allready started");
			}else{
				log("starting stereo detection");
        // start receiving images pushed by the video server
        videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
				cmd_detect = true;
			}
			break;
		case VisionData::SDSTOP:
			if(cmd_detect){
				log("stopping stereo detection");
				cmd_detect = false;
        videoServer->stopReceiveImages(getComponentID().c_str());
			}else{
				log("stereo detection is allready stopped");
			}
			break;
		case VisionData::SDSINGLE:
			if(!cmd_single){
				log("single stereo detection received");
				if(!debug)
				{
					cmd_single = true;
					videoServer->getImage(camIds[0], image_l);
					videoServer->getImage(camIds[1], image_r);
					processImage(image_l, image_r);
					cmd_single = false;
				}
			}else{
				log("single stereo detection already received: too fast triggering!");
			}
			break;
		default:
			log("unknown detection command received, doing nothing");
			break;
	}	
}


void StereoDetector::configure(const map<string,string> & _config)
{
	// create stereo core TODO: Configure and store ini-File?
	score = new Z::StereoCore("subarchitectures/vision.sa/config/tuw_stereo_new.ini");

	cmd_detect = false;
	cmd_single = false;
	showImages = false;
	debug = false;

  map<string,string>::const_iterator it;
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }
	if((it = _config.find("--showImages")) != _config.end())
	{
		showImages = true;
	}
	if((it = _config.find("--debug")) != _config.end())
	{
		log("debug modus is on!");
		debug = true;
	}

}


void StereoDetector::start()
{
	log("StereoDetector::start: Start Component");

  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// add change filter for commands
  addChangeFilter(createLocalTypeFilter<StereoDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveDetectionCommand));

	// define openCV displays
  if(showImages) 
	{
		cvNamedWindow("Stereo left", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvNamedWindow("Stereo right", 1);
// 		cvNamedWindow("Stereo results @ left stereo image", 1);

		cvMoveWindow("Stereo left", 2000, 100);
		cvMoveWindow("Stereo right", 2660, 100);
// 		cvMoveWindow("Stereo results @ left stereo image", 2330, 600);
	}
}


void StereoDetector::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() <= 1)
    throw runtime_error(exceptionMessage(__HERE__, "image list too short: stereo image expected."));
  // process a single image
  if (cmd_detect) processImage(images[0], images[1]);
/*	if (cmd_single)
	{ 
		processImage(images[0], images[1]);
		cmd_single = false;
	}*/
}


void StereoDetector::runComponent()
{
}


void StereoDetector::processImage(const Video::Image &image_l, const Video::Image &image_r)
{
	log("process image");

	int runtime = 1200;											// processing time for detection for left AND right image
	static unsigned frame_counter = 0;
	frame_counter++;
	static unsigned numStereoObjects = 0;

	// Convert images (from Video::Image to iplImage)
	IplImage *iplImage_l = convertImageToIpl(image_l);
	IplImage *iplImage_r = convertImageToIpl(image_r);
// 	IplImage *iplImage_results = convertImageToIpl(image_l);
// 	IplImage *iplImage_org_l = convertImageToIpl(image_l);
// 	IplImage *iplImage_org_r = convertImageToIpl(image_r);

	// Process the stereo images at the stereo core and get visual (stereo matched) objects
	score->ProcessStereoImage(runtime/2, iplImage_l, iplImage_r);
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;

	/// TODO: GET ALL STEREO GESTALTS: Es wird nicht überprüft, ob aktiv
// 	for(int j=0; j<Z::StereoBase::MAX_TYPE; j++)
// 		for(int i=0; i<score->NumStereoMatches((Z::StereoBase::Type) j); i++)
// 		{
// 			score->GetVisualObject((Z::StereoBase::Type) j, i, obj);
// 	
// 			// HACK label object with incremtal raising number
// 			char obj_label[32];
// 			sprintf(obj_label, "Stereo object %d", numStereoObjects);
// 			obj->label = obj_label;
// 			numStereoObjects++;
// 		
// 			// add visual object to working memory
// 			addToWorkingMemory(newDataID(), obj);
// 			log("New stereo object at frame number %u: added to working memory: %s", frame_counter, obj->label.c_str());
// 		}

printf("###################### GET STEREO CLOSURES\n");
	/// TODO: GET STEREO CLOSURES
	for(int i=0; i<score->NumStereoMatches(Z::StereoBase::STEREO_CLOSURE); i++)
	{
		score->GetVisualObject(Z::StereoBase::STEREO_CLOSURE, i, obj);

		char obj_label[32];
		sprintf(obj_label, "Stereo object %d", numStereoObjects);
		obj->label = obj_label;
		numStereoObjects++;
	
		// add visual object to working memory
		addToWorkingMemory(newDataID(), obj);
		log("New closure at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
	}



	// ----------------------------------------------------------------
	// Draw Gestalts to IplImage for the openCv window
	// ----------------------------------------------------------------
	if(showImages) 
	{	
		cvWaitKey(1);		/// TODO wait key to allow openCV to show the images.

// 		bool processDebug = true;
// 		if (debug) {
// 			while (processDebug) {
// // 				iplImage_org_l = cvCloneImage(iplImage_l);
// // 				iplImage_org_r = cvCloneImage(iplImage_r);
// 				processDebug = ProcessDebugOptions(score, iplImage_l, iplImage_r, iplImage_results);
// // 				cvWaitKey(10);
// 			}
// 			debug = false;
// 		}
// 		else


		/// HACK Wo sollte die Draw-Methode aufgerufen werden?
// 		score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE);
// 		score->DrawStereoResults(Z::StereoBase::STEREO_FLAP);
// 		score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE);
// 		score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE);

		cvConvertImage( iplImage_l, iplImage_l, CV_CVTIMG_SWAP_RB);
		cvConvertImage( iplImage_r, iplImage_r, CV_CVTIMG_SWAP_RB);
// 		cvConvertImage( iplImage_results, iplImage_results, CV_CVTIMG_SWAP_RB);
	
		cvShowImage("Stereo left", iplImage_l);
		cvShowImage("Stereo right", iplImage_r);
// 		cvShowImage("Stereo results @ left stereo image", iplImage_results);

		cvWaitKey(10);	/// TODO wait key to allow openCV to show the images.

		cvReleaseImage(&iplImage_l);
		cvReleaseImage(&iplImage_r);
// 		cvReleaseImage(&iplImage_results);
	}

	// clear results after processing
	score->ClearResults();
}


/**
 * @brief Extract camera parameters from video server.
 */
bool StereoDetector::ProcessDebugOptions(Z::StereoCore *score, IplImage *iplImage_l, IplImage *iplImage_r, IplImage *iplImage_results)
{
 	int key = 0;
	key = cvWaitKey(10);

	if (key != -1) printf("Key: %c\n", (char) key);
	if((char) key == 'a') return false;							// return for escape

	switch((char) key)
	{
		case '1':
			log("show flaps.");
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP);
			break;
		case '2':
			log("show rectangles.");
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE);
			break;
// 		case 1048621: if(detail>0) detail--;			// Key '-'
// 			log("Detail switched to %i", detail);
// 			break;
// 		case 1048670: type = 29;									// OBJECT == ^
// 			log("Switched to OBJECT");
// 			break;
// 		case 1048625: type = 20;									// CUBE == 1
// 			log("Switched to CUBE");
// 			break;
// 		case 1048626: type = 12;									// CYLINDER == 2
// 			log("Switched to CYLINDER");
// 			break;
// 		case 1048627: type = 13;									// CONE == 3
// 			log("Switched to CONE");
// 			break;
// 		case 1048628: type = 6;										// BALL == 4
// 			log("Switched to BALL");
// 			break;
// 		case 1048629: type = 5;										// ELLIPSE == 5
// 			log("Switched to ELLIPSE");
// 			break;
// 		case 1048630: type = 0;										// SEGMENT == 6
// 			log("Switched to SEGMENT");
// 			break;
// 		case 1048631: type = 1;										// LINE == 7
// 			log("Switched to LINE");
// 			break;
// 		case 1048632: type = 17;									// RECTANGLE == 8
// 			log("Switched to RECTANGLE");
// 			break;
// 		case 1048633: type = 19;									// FLAP == 9
// 			log("Switched to FLAP");
// 			break;
// 		case 1048676: 														// Enable/Disable display == D
// 			if(showImages) showImages = false;
// 			else showImages = true;
// 			log("Enable/Disable object detector window");
// 			break;
		default: break;
	}

	return true;
}





/**   					TODO TODO TODO TODO TODO TODO TODO TODO  Verschieben auf StereoBase zur Umrechnung???
 * @brief Extract camera parameters from video server.
 * @param image Image from the image server with the camera paramters.
 * @return True for success
 */
bool StereoDetector::GetCameraParameter(const Video::Image & image)
{
	Video::CameraParameters camPars = image.camPars;

	double intrinsic[4];
	intrinsic[0] = camPars.fx;
	intrinsic[1] = camPars.fy;
	intrinsic[2] = camPars.cx;
	intrinsic[3] = camPars.cy;

	double dist[4];
	dist[0] = camPars.k1;
	dist[1] = camPars.k2;
	dist[2] = camPars.p1;
	dist[3] = camPars.p2;

	// get extrinsic parameters from camera image
	double extrinsic[16];
	getRow44(camPars.pose, extrinsic);

	// Recalculation of rotation matrix and transition vector
	// w ... world point ???
	// R ... rotation matrix/home/ari/projects/saa-cogx/tuw/subarchitectures/vision.sa/src/c++/vision/components/StereoFlapDetector/StereoFlapDetector.cpp:322: error: base operand of ‘->’ has non-pointer type ‘Z::Flap3D’

	// p ... image point ???
	// t ... translation vector
	// w = R.p + t
	// w-t = R.p
	// R^T.w - R^T.t = p		==> R(new) = -R^T.t

	// Invert rotation matrix (= transpose)
	double zw = extrinsic[1];
	extrinsic[1] = extrinsic[4];
	extrinsic[4] = zw;
	zw = extrinsic[2];
	extrinsic[2] = extrinsic[8];
	extrinsic[8] = zw;
	zw = extrinsic[6];
	extrinsic[6] = extrinsic[9];
	extrinsic[9] = zw;

	// Translation vector from m to mm
// 	extrinsic[3] *= 1000;
// 	extrinsic[7] *= 1000;
// 	extrinsic[11] *= 1000;

	// Recaclulate translation vector for inverted rotation matrix
	double tneu[3]; 
	tneu[0] = -(extrinsic[0]*extrinsic[3] + extrinsic[1]*extrinsic[7] + extrinsic[2]*extrinsic[11]);
	tneu[1] = -(extrinsic[4]*extrinsic[3] + extrinsic[5]*extrinsic[7] + extrinsic[6]*extrinsic[11]);
	tneu[2] = -(extrinsic[8]*extrinsic[3] + extrinsic[9]*extrinsic[7] + extrinsic[10]*extrinsic[11]);
	extrinsic[3] = tneu[0];
	extrinsic[7] = tneu[1];
	extrinsic[11] = tneu[2];

// printf("Translationsvektor neu: %6.5f   %6.5f   %6.5f\n\n", extrinsic[3], extrinsic[7], extrinsic[11]);

// printf("Camera parameters: extrinsic new:\n	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n", extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]);
// printf("	%6.5f   %6.5f   %6.5f   %6.5f\n\n", extrinsic[12], extrinsic[13], extrinsic[14], extrinsic[15]);

//  	vs3Interface->SetCamParameters(intrinsic, dist, extrinsic);

	return true;
}


}
