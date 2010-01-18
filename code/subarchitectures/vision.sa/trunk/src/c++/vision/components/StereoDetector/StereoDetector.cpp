/**
 * @file StereoDetector.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 * 
 * @TODO Release iplImages: Destructor? destroy() ???
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
// 	cvDestroyWindow("Stereo results @ left stereo image");
}

void StereoDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
	if(debug) return;		// return if debug mode is on

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
				cmd_single = true;
				videoServer->getImage(camIds[0], image_l);
				videoServer->getImage(camIds[1], image_r);
				processImage();
				cmd_single = false;
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
	overlays = 1;

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
		log("debug modus on.");
		debug = true;
	}
}


void StereoDetector::start()
{
	log("start component");

  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// add change filter for commands
  addChangeFilter(createLocalTypeFilter<StereoDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveDetectionCommand));

	// define openCV windows
  if(showImages) 
	{
		cvNamedWindow("Stereo left", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvNamedWindow("Stereo right", 1);

		cvMoveWindow("Stereo left", 2000, 100);
		cvMoveWindow("Stereo right", 2660, 100);
	}
}


void StereoDetector::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() <= 1)
    throw runtime_error(exceptionMessage(__HERE__, "image list too short: stereo image expected."));

	// process a single image
	image_l = images[0];
	image_r = images[1];
  if (cmd_detect) processImage();
}


void StereoDetector::runComponent()
{
	while(debug) Debug();
}


void StereoDetector::processImage()
{
	// clear results before processing new image
	score->ClearResults();

//	ReadSOIs(); 	/// HACK: Read SOIs and write it to the console
	
	log("process new image");

	int runtime = 1200;											// processing time for detection for left AND right image
	static unsigned frame_counter = 0;
	frame_counter++;
	static unsigned numStereoObjects = 0;

	// Convert images (from Video::Image to iplImage)
	iplImage_l = convertImageToIpl(image_l);
	iplImage_r = convertImageToIpl(image_r);

	// Process the stereo images at the stereo core and get visual (stereo matched) objects
	score->ProcessStereoImage(runtime/2, iplImage_l, iplImage_r);

	// Delete all visual objects from the working memory
	DeleteVisualObjects();	

	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;

	/// TODO: GET ALL STEREO GESTALTS: Es wird nicht überprüft, ob aktiv
	for(int j=0; j<Z::StereoBase::MAX_TYPE; j++)
		for(int i=0; i<score->NumStereoMatches((Z::StereoBase::Type) j); i++)
		{
			score->GetVisualObject((Z::StereoBase::Type) j, i, obj);
	
			// HACK label object with incremtal raising number
			char obj_label[32];
			sprintf(obj_label, "Stereo object %d", numStereoObjects);
			obj->label = obj_label;
			numStereoObjects++;
		
		// add visual object to working memory
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
		addToWorkingMemory(objectID, obj);
		log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
		}


	/// TODO: GET STEREO CLOSURES
// 	printf("###################### GET STEREO CLOSURES\n");
// 	for(int i=0; i<score->NumStereoMatches(Z::StereoBase::STEREO_CLOSURE); i++)
// 	{
// 		score->GetVisualObject(Z::StereoBase::STEREO_CLOSURE, i, obj);
// 
// 		char obj_label[32];
// 		sprintf(obj_label, "Stereo object %d", numStereoObjects);
// 		obj->label = obj_label;
// 		numStereoObjects++;
// 	
// 		// add visual object to working memory
// 		std::string objectID = newDataID();
// 		objectIDs.push_back(objectID);
// 		addToWorkingMemory(objectID, obj);
// 		log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
// 	}

	/// TODO: GET STEREO FLAPS
// static int once = 0;
// 	printf("###################### GET STEREO FLAPS\n");
// if(once == 0)
// 	for(int i=0; i<score->NumStereoMatches(Z::StereoBase::STEREO_FLAP); i++)
// 	{
// 		once = 1;
// 		score->GetVisualObject(Z::StereoBase::STEREO_FLAP, i, obj);
// 
// 		char obj_label[32];
// 		sprintf(obj_label, "Stereo object %d", numStereoObjects);
// 		obj->label = obj_label;
// 		numStereoObjects++;
// 	
		// add visual object to working memory
// 		std::string objectID = newDataID();
// 		objectIDs.push_back(objectID);
// 		addToWorkingMemory(objectID, obj);
// 		log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
// 	}

	/// TODO: GET STEREO RECTANGLES
// 	printf("###################### GET STEREO RECTANGLES\n");
// 	for(int i=0; i<score->NumStereoMatches(Z::StereoBase::STEREO_RECTANGLE); i++)
// 	{
// 		score->GetVisualObject(Z::StereoBase::STEREO_RECTANGLE, i, obj);
// 
// 		char obj_label[32];
// 		sprintf(obj_label, "Stereo object %d", numStereoObjects);
// 		obj->label = obj_label;
// 		numStereoObjects++;
// 	
// 		// add visual object to working memory
// 		std::string objectID = newDataID();
// 		objectIDs.push_back(objectID);
// 		addToWorkingMemory(objectID, obj);
// 		log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
// 	}

	/// TODO: GET STEREO ELLIPSES
// 	printf("###################### GET STEREO ELLIPSES\n");
// 	for(int i=0; i<score->NumStereoMatches(Z::StereoBase::STEREO_ELLIPSE); i++)
// 	{
// 		score->GetVisualObject(Z::StereoBase::STEREO_ELLIPSE, i, obj);
// 
// 		char obj_label[32];
// 		sprintf(obj_label, "Stereo object %d", numStereoObjects);
// 		obj->label = obj_label;
// 		numStereoObjects++;
// 	
// 		// add visual object to working memory
// 		std::string objectID = newDataID();
// 		objectIDs.push_back(objectID);
// 		addToWorkingMemory(objectID, obj);
// 		log("New flap at frame number %u: added visual object to working memory: %s", frame_counter, obj->label.c_str());
// 	}


	// Draw Gestalts to IplImage for the openCv window
	if(showImages) ShowImages(false);
}


/**
 * @brief Show images with stereo matched overlays from vs3.
 * @param convertNewIpl set true, to get orginal stereo image reconverted (without overlay drawings).
 */
void StereoDetector::ShowImages(bool convertNewIpl)
{
//		cvWaitKey(1);		/// wait key to allow openCV to show the images.

	// reconvert original stereo image pair
	if(convertNewIpl)
	{
		iplImage_l = convertImageToIpl(image_l);
		iplImage_r = convertImageToIpl(image_r);

		// swap red and blue channel from stereo ipl-images
// 		cvConvertImage( iplImage_l, iplImage_l, CV_CVTIMG_SWAP_RB);
// 		cvConvertImage( iplImage_r, iplImage_r, CV_CVTIMG_SWAP_RB);
	}

	switch(overlays)
	{
		case 1:
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP, iplImage_l, iplImage_r);
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE, iplImage_l, iplImage_r);
			score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE, iplImage_l, iplImage_r);
			score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE, iplImage_l, iplImage_r);
			break;
		case 2:
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP, iplImage_l, iplImage_r);
			break;
		case 3:
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE, iplImage_l, iplImage_r);
			break;
		case 4:
			score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE, iplImage_l, iplImage_r);
			break;
		case 5:
			score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE, iplImage_l, iplImage_r);
			break;
		default:
			printf("StereoDetector::ShowImages: Unknown overlay exception\n"); 	// TODO exception handling
			break;
	}

		cvShowImage("Stereo left", iplImage_l);
		cvShowImage("Stereo right", iplImage_r);

		cvWaitKey(10);	///< TODO wait key to allow openCV to show the images.
}


/**
 * @brief Debug mode of the stereo detector. Catch keyboard events and change displayed results.
 * 		key:	s ... single shot
 * 		key:	1 ... show all
 * 		key:	2 ... show flaps
 * 		key:	3 ... show rectangles
 * 		key:	4 ... show closures
 * 		key:	x ... stop debug modus
 */
void StereoDetector::Debug()
{
	int key = 0;
	key = cvWaitKey(10);

	if (key != -1) printf("StereoDetector::ProcessDebugOptions: Pressed key: %c\n", (char) key);
// 	if((char) key == 'a') return false;							// TODO return for escape

	switch((char) key)
	{
		case 's':
			log("Single shot: process single stereo image.");
			videoServer->getImage(camIds[0], image_l);
			videoServer->getImage(camIds[1], image_r);
			processImage();
			break;
		case '1':
			log("Show all.");
			overlays = 1;
			ShowImages(true);
			break;
		case '2':
			log("Show flaps.");
			overlays = 2;
			ShowImages(true);
			break;
		case '3':
			log("Show rectangles.");
			overlays = 3;
			ShowImages(true);
			break;
		case '4':
			log("Show closures.");
			overlays = 4;
			ShowImages(true);
			break;
		case '5':
			log("Show ellipses.");
			overlays = 5;
			ShowImages(true);
			break;
		case 'x':
			log("Stop debug mode.");
			debug = false;
			break;
		default: break;
	}
}


/**
 * @brief Read the SOIs from the working memory and display it.
 */
void StereoDetector::ReadSOIs()
{
	// read SOIs from working memory
	std::vector <VisionData::SOIPtr> sois;
	getMemoryEntries<VisionData::SOI>(sois);

	printf("Found %u SOIs in working memory:\n", sois.size());

	for(unsigned i=0; i<sois.size(); i++)
	{
		printf("  Sphere: pose: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingSphere.pos.x, sois[i]->boundingSphere.pos.y, sois[i]->boundingSphere.pos.z);
		printf("          radius: %4.3f\n", sois[i]->boundingSphere.rad);
		printf("     Box: pose: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingBox.pos.x, sois[i]->boundingBox.pos.y, sois[i]->boundingBox.pos.z);
		printf("          radius: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingBox.size.x, sois[i]->boundingBox.size.y, sois[i]->boundingBox.size.z);
	}
}

//   class SOI {
//     cogx::Math::Sphere3 boundingSphere;
//     cogx::Math::Box3 boundingBox;
//     // time the SOI was last changed
//     cast::cdl::CASTTime time;
//     // This is a temporary solution only: provide the 3D points that gave rise
//     // to this SOI, iff the SOI was created by plane pop-out.
//     SurfacePointSeq points;   // frontground points
//     SurfacePointSeq BGpoints; //background points
//     SurfacePointSeq EQpoints; //equivocal points which either belongs to fg or bg
//   };



void StereoDetector::DeleteVisualObjects()
{
	for(unsigned i=0; i<objectIDs.size(); i++)
		deleteFromWorkingMemory(objectIDs[i]);
	objectIDs.clear();
}

}








