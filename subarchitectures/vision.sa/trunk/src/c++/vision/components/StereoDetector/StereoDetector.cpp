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
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
	cast::CASTComponentPtr newComponent() {
    return new cast::StereoDetector();
  }
}


namespace cast
{

/**
 *	@brief Destructor of class StereoDetector
 */
StereoDetector::~StereoDetector()
{
	cvDestroyWindow("Stereo left");
	cvDestroyWindow("Stereo right");
}


/**
 *	@brief Configure managed cast component StereoDetector.
 *	@param _config Config
 */
void StereoDetector::configure(const map<string,string> & _config)
{
	gotImage = false;
	cmd_detect = false;
	cmd_single = false;
	showImages = false;
	showDetected = true;
	showMatched = true;
	debug = false;
	single = false;
	overlays = 1;
	VOtoWrite = 1;

  map<string,string>::const_iterator it;
	if((it = _config.find("--videoname")) != _config.end())
	{
		videoServerName = it->second;
	}
	if((it = _config.find("--camconfig")) != _config.end())
  {
    camconfig = it->second;
		try
		{
			score = new Z::StereoCore(camconfig);
		}
		catch(Z::Except &e)
		{
			log("error during initialisation of stereo core.");
			printf("%s\n", e.what());
		}
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
	if((it = _config.find("--singleShot")) != _config.end())
	{
		log("single shot modus on.");
		single = true;
	}
	if((it = _config.find("--debug")) != _config.end())
	{
		log("debug modus on.");
		debug = true;
	}	
}


/**
 *	@brief Start managed cast component StereoDetector.
 */
void StereoDetector::start()
{
	log("start component");

  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// add change filter for vision commands
  addChangeFilter(createLocalTypeFilter<StereoDetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveDetectionCommand));

	// declare openCV windows
  if(showImages) 
	{
		cvNamedWindow("Stereo left", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvNamedWindow("Stereo right", 1);

		cvMoveWindow("Stereo left", 2000, 100);
		cvMoveWindow("Stereo right", 2640, 100);
	}
}


/**
 *	@brief Receive detection command. 
 *	@param wmc Working memory change.
 */
void StereoDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
	if(single) return;		// return if single shot mode is on

	StereoDetectionCommandPtr detect_cmd = getMemoryEntry<StereoDetectionCommand>(_wmc.address);
	
	log("received detection command ...");
	switch(detect_cmd->cmd){
		case VisionData::SDSTART:
			if(cmd_detect){
				log("stereo detection is already started");
			}else{
				log("starting stereo detection");
        // start receiving images pushed by the video server
        videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);		/// TODO Hier wird videoServer->startReceiveImages aufgerufen
				cmd_detect = true;
			}
			break;
		case VisionData::SDSTOP:
			if(cmd_detect){
				log("stopping stereo detection");
				cmd_detect = false;
        videoServer->stopReceiveImages(getComponentID().c_str());
			}else{
				log("stereo detection is already stopped");
			}
			break;
		case VisionData::SDSINGLE:																											/// TODO Hier wird videoServer->getImage aufgerufen
			if(!cmd_single){
				log("single stereo detection received");
				cmd_single = true;
				videoServer->getImage(camIds[0], image_l);
				videoServer->getImage(camIds[1], image_r);
				gotImage = true;
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


/**
 *	@brief Receive images from the video server.
 *	@param images Images from the video server.
 */
void StereoDetector::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() <= 1)
    throw runtime_error(exceptionMessage(__HERE__, "image list too short: stereo image expected."));

	// process a single image
	image_l = images[0];
	image_r = images[1];
	gotImage = true;
	if (cmd_detect) processImage();
}


/**
 *	@brief Run component. Called on start of the component.
 */
void StereoDetector::runComponent()
{
	while(single) SingleShotMode();
// 	while(true) SingleShotMode();
}


/**
 *	@brief Process stereo images at stereo core.
 */
void StereoDetector::processImage()
{
	log("process new image");
	int runtime = 1200;											// processing time for detection for left AND right image (2 vision cores for stereo)

	// clear results before processing new image
	score->ClearResults();

//	ReadSOIs(); 	/// HACK: Read SOIs and write it to the console

	// Convert images (from Video::Image to iplImage)
	iplImage_l = convertImageToIpl(image_l);
	iplImage_r = convertImageToIpl(image_r);

	// Process the stereo images at the stereo core and get visual (stereo matched) objects
	try 
	{
		score->ProcessStereoImage(runtime/2, iplImage_l, iplImage_r);
	}
	catch(Z::Except &e)
	{
		log("errors during processing of stereo images.");
    printf("%s\n", e.what());
	}
  catch (exception &e)
  {
		log("other exception during processing of stereo images");
    cout << e.what() << endl;
  }

	// Write visual objects to working memory
	WriteVisualObjects(VOtoWrite);

	// Draw Gestalts to IplImage for the openCv window
	if(showImages) ShowImages(false);
}


/**
 * @brief Show images with stereo matched overlays from vs3.
 * @param convertNewIpl set true, to get orginal stereo image reconverted (without overlay drawings).
 */
void StereoDetector::ShowImages(bool convertNewIpl)
{
	// return, if never got an image
	if(!gotImage) return;

	// reconvert original stereo image pair from image server to clear images.
	if(convertNewIpl)
	{
// printf("########################## CONVERT NEW IPL #################################\n");
		iplImage_l = convertImageToIpl(image_l);
		iplImage_r = convertImageToIpl(image_r);
	}

	switch(overlays)
	{
		case 1:
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP, iplImage_l, iplImage_r, showDetected, showMatched);
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE, iplImage_l, iplImage_r, showDetected, showMatched);
			score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE, iplImage_l, iplImage_r, showDetected, showMatched);
			score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE, iplImage_l, iplImage_r, showDetected, showMatched);
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP_ARI, iplImage_l, iplImage_r, showDetected, showMatched);
			score->DrawStereoResults(Z::StereoBase::STEREO_CUBE, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 2:
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 3:
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 4:
			score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 5:
			score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 6:
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP_ARI, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		case 7:
			score->DrawStereoResults(Z::StereoBase::STEREO_CUBE, iplImage_l, iplImage_r, showDetected, showMatched);
			break;
		default:
			printf("StereoDetector::ShowImages: Unknown overlay exception\n");
			break;
	}

	// swap red and blue channel from stereo ipl-images							/// TODO TODO Warum muss ich jetzt swapen und wann nicht?
// 	cvConvertImage( iplImage_l, iplImage_l, CV_CVTIMG_SWAP_RB);
// 	cvConvertImage( iplImage_r, iplImage_r, CV_CVTIMG_SWAP_RB);

	cvShowImage("Stereo left", iplImage_l);
	cvShowImage("Stereo right", iplImage_r);

	cvWaitKey(20);	///< TODO wait key to allow openCV to show the images on the window.
}


/**
 * @brief Delete working memory and (re)write different visual objects from the stereo detector.
 * @param VOtoWrite Identifier for visual objects to write
 */
void StereoDetector::WriteVisualObjects(int VOtoWrite)
{
	DeleteVisualObjectsFromWM();	

	switch(VOtoWrite)
	{
		case 1:
			WriteToWM(Z::StereoBase::STEREO_FLAP);
			WriteToWM(Z::StereoBase::STEREO_RECTANGLE);
			WriteToWM(Z::StereoBase::STEREO_CLOSURE);
			WriteToWM(Z::StereoBase::STEREO_ELLIPSE);
			WriteToWM(Z::StereoBase::STEREO_FLAP_ARI);
			WriteToWM(Z::StereoBase::STEREO_CUBE);
			break;
		case 2:
			WriteToWM(Z::StereoBase::STEREO_FLAP);
			break;
		case 3:
			WriteToWM(Z::StereoBase::STEREO_RECTANGLE);
			break;
		case 4:
			WriteToWM(Z::StereoBase::STEREO_CLOSURE);
			break;
		case 5:
			WriteToWM(Z::StereoBase::STEREO_ELLIPSE);
			break;
		case 6:
			WriteToWM(Z::StereoBase::STEREO_FLAP_ARI);
			break;
		case 7:
			WriteToWM(Z::StereoBase::STEREO_CUBE);
			break;
		default:
			log("unknown visual object identifier.");
			break;
	}
}


/**
 * @brief Write visual objects of different type to the working memory
 * @param type Type of stereo-object to write.
 */
void StereoDetector::WriteToWM(Z::StereoBase::Type type)
{
	static unsigned numStereoObjects = 0;
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;

	for(int i=0; i<score->NumStereoMatches((Z::StereoBase::Type) type); i++)
	{
		score->GetVisualObject((Z::StereoBase::Type) type, i, obj);

		// label object with incremtal raising number
		char obj_label[32];
		sprintf(obj_label, "Stereo object %d", numStereoObjects);
		obj->label = obj_label;
		numStereoObjects++;
	
		// add visual object to working memory
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
 		log("Add new visual object to working memory: %s", obj->label.c_str());

// for(unsigned i=0; i<obj->model->vertices.size(); i++)
// 	printf("    Sent object vertices: %4.3f - %4.3f - %4.3f\n", obj->model->vertices[i].pos.x, obj->model->vertices[i].pos.y, obj->model->vertices[i].pos.z);

		addToWorkingMemory(objectID, obj);

cvWaitKey(200);	/// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.

	}
}


/**
 * @brief Single shot mode of the stereo detector. Catch keyboard events and change displayed results.\n
 * 		key:	s ... single shot\n
 * 		key:	m ... draw matched features (on/off)\n
 * 		key:	d ... draw detected features (on/off)\n
 * 		key:	1 ... show all\n
 * 		key:	2 ... show flaps\n
 * 		key:	3 ... show rectangles\n
 * 		key:	4 ... show closures\n
 * 		key:	5 ... show ellipses\n
 * 		key:	6 ... show flaps_ari\n
 * 		key:	7 ... show cubes\n
 * 		key:	x ... stop single shot modus\n
 * 		key:	h ... show help\n
 */
void StereoDetector::SingleShotMode()
{
	int key = 0;
	key = cvWaitKey(10);

	if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %c", (char) key);
	switch((char) key)
	{
		case 's':
			log("process single stereo image.");
			videoServer->getImage(camIds[0], image_l);
			videoServer->getImage(camIds[1], image_r);
			gotImage = true;
			processImage();
			break;
		case 'm':
			if(showMatched) 
			{
				showMatched = false;
				log("draw only matched features: off");
			}
			else
			{ 
				showMatched = true;
				log("draw only matched features: on");
			}
			ShowImages(true);
			break;
		case 'd':
			if(showDetected)
			{
				showDetected = false;
				log("draw all detected features: off");
			}
			else
			{
				 showDetected = true;
				log("draw all detected features: on");
			}
			ShowImages(true);
			break;
		case '1':
			log("show all.");
			overlays = VOtoWrite = 1;
			ShowImages(true);
			WriteVisualObjects(VOtoWrite);
			break;
		case '2':
			log("show flaps.");
			overlays = VOtoWrite = 2;
			ShowImages(true);
			WriteVisualObjects(VOtoWrite);
			break;
		case '3':
			log("show rectangles.");
			overlays = VOtoWrite = 3;
			ShowImages(true);
			WriteVisualObjects(VOtoWrite);
			break;
		case '4':
			log("show closures.");
			overlays = VOtoWrite = 4;
			WriteVisualObjects(VOtoWrite);
			ShowImages(true);
			break;
		case '5':
			log("show ellipses.");
			overlays = VOtoWrite = 5;
			WriteVisualObjects(VOtoWrite);
			ShowImages(true);
			break;
		case '6':
			log("show flaps_ari.");
			overlays = VOtoWrite = 6;
			WriteVisualObjects(VOtoWrite);
			ShowImages(true);
			break;
		case '7':
			log("show cubes.");
			overlays = VOtoWrite = 7;
			WriteVisualObjects(VOtoWrite);
			ShowImages(true);
			break;
		case 'x':
			log("stop debug mode.");
			single = false;
			break;
		case 'h':
			printf( "Keys for single shot mode:\n"
							"    s ... single shot\n"
							"    m ... draw matched features (on/off)\n"
							"    d ... draw detected features (on/off)\n"
							"    1 ... show all\n"
							"    2 ... show flaps\n"
							"    3 ... show rectangles\n"
							"    4 ... show closures\n"
							"    5 ... show ellipses\n"
							"    6 ... show flaps_ari\n"
							"    7 ... show cubes\n"
							"    x ... stop single shot modus\n"
							"    h ... show this help.");
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



/**
 * @brief Delete all visual objects from the working memory. 
 * The IDs are stored in the vector "objectIDs".
 */
void StereoDetector::DeleteVisualObjectsFromWM()
{
	for(unsigned i=0; i<objectIDs.size(); i++)
		deleteFromWorkingMemory(objectIDs[i]);
	objectIDs.clear();
}

}








