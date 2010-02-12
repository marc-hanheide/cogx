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

// Gobal variables: Can't solve this in another way
bool mouseEvent = false;				///< Flag for mouse events
int mouseX = 0, mouseY = 0;			///< Coordinates from mouse event
int mouseSide = 0;							///< Left / right side of stereo

/**
 * @brief Mouse handler for the two stereo images.
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void LeftMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONDOWN:
				printf("Left button down on left image at position: %u / %u\n", x, y);
			break;

		case CV_EVENT_LBUTTONUP:
// 			printf("Left button up!\n");
			mouseEvent = true;
			mouseX = x;
			mouseY = y;
			mouseSide = 0;
			break;
			break;
	}
}


/**
 * @brief Mouse handler for the two stereo images.
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void RightMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONDOWN:
				printf("Left button down on right image at position: %u / %u\n", x, y);
			break;

		case CV_EVENT_LBUTTONUP:
// 			printf("Left button up!\n");
			mouseEvent = true;
			mouseX = x;
			mouseY = y;
			mouseSide = 1;
			break;
	}
}



/**
 *	@brief Destructor of class StereoDetector
 */
StereoDetector::~StereoDetector()
{
	cvDestroyWindow("Stereo left");
	cvDestroyWindow("Stereo right");
}


/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config TODO
 */
void StereoDetector::configure(const map<string,string> & _config)
{
	gotImage = false;
	cmd_detect = false;
	cmd_single = false;

	detail = 0;
	showImages = false;
	showDetected = true;
	showSingleGestalt = false;
	showAllStereo = false;
	showID = 0;
	showMasked = false;
	showMatched = true;
	debug = false;
	single = false;
	showType = Z::Gestalt::SEGMENT;
	overlays = 100;												// TODO durch showType ersetzen

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
 *	@brief Called by the framework after configuration, before run loop.
 *	TODO Change receiver locks the memory until 
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

	// initialize openCV windows
  if(showImages) 
	{
		cvNamedWindow("Stereo left", /*CV_WINDOW_AUTOSIZE*/ 1);
		cvNamedWindow("Stereo right", 1);

		cvMoveWindow("Stereo left", 1920, 10);
		cvMoveWindow("Stereo right", 2560, 10);

	  int mouseParam=0;
		cvSetMouseCallback("Stereo left", LeftMouseHandler, &mouseParam);
		cvSetMouseCallback("Stereo right", RightMouseHandler, &mouseParam);
	}
}


/**
 *	@brief Called by the framework to start component run loop.
 *	@TODO LOCKT DEN SPEICHERBEREICH NICHT, SOLANGE GEARBEITET WIRD
 */
void StereoDetector::runComponent()
{
	while(single) SingleShotMode();
}


/**
 *	@brief Receive a changed detection command, written to the working memory
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
 * @brief The callback function for images pushed by the image server.\n
 * To be overwritten by derived classes.
 * @param images Vector of images from video server.
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
 *	@brief Process stereo image pair at stereo core.
 */
void StereoDetector::processImage()
{
	log("process new image");
	int runtime = 1200;											// processing time for detection for left AND right image (2 vision cores for stereo)

	// clear results before converting and processing new image
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
	WriteVisualObjects();

	// Draw Gestalts to IplImage for the openCv window
	ShowImages(false);
}


/**
 * @brief Show both stereo images in the openCV windows.
 * @param convertNewIpl Convert image again into iplImage to delete overlays.
 */
void StereoDetector::ShowImages(bool convertNewIpl)
{
	// return, if never got an image or not activated in cast-file
	if(!gotImage || !showImages) return;

	// reconvert original stereo image pair from image server to clear images.
	if(convertNewIpl)
	{
		iplImage_l = convertImageToIpl(image_l);
		iplImage_r = convertImageToIpl(image_r);
	}

	if(showDetected || showSingleGestalt) score->DrawMonoResults(showType, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
	
	switch(overlays)
	{
		case 100:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::SEGMENT, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 101:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::LINE, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 102:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::COLLINEARITY, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 103:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::L_JUNCTION, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 104:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::CLOSURE, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_CLOSURE, iplImage_l, iplImage_r, showMatched);
			break;
		case 105:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::RECTANGLE, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_RECTANGLE, iplImage_l, iplImage_r, showMatched);
			break;
		case 106:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::FLAP, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP, iplImage_l, iplImage_r, showMatched);
			break;
		case 107:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::FLAP_ARI, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_FLAP_ARI, iplImage_l, iplImage_r, showMatched);
			break;
		case 108:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::CUBE, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_CUBE, iplImage_l, iplImage_r, showMatched);
			break;

		case 200:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::ARC, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 201:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::CONVEX_ARC_GROUP, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 202:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::A_JUNCTION, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			break;
		case 203:
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::ELLIPSE, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail);
			score->DrawStereoResults(Z::StereoBase::STEREO_ELLIPSE, iplImage_l, iplImage_r, showMatched);
			break;
		case 204:
			printf("StereoDetector::ShowImages: ExtEllipses not yet implemented.\n");
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::EXTELLIPSE, iplImage_l, iplImage_r, showMasked, showAllGestalts, mouseSide, showID, detail);
			break;
		case 205:
			printf("StereoDetector::ShowImages:  Cylinder not yet implemented.\n");
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::CYLINDER, iplImage_l, iplImage_r, showMasked, showAllGestalts, mouseSide, showID, detail);
			break;
		case 206:
			printf("StereoDetector::ShowImages:  Cone not yet implemented.\n");
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::CONE, iplImage_l, iplImage_r, showMasked, showAllGestalts, mouseSide, showID, detail);
			break;
		case 207:
			printf("StereoDetector::ShowImages:  Sphere not yet implemented.\n");
// 			if(showDetected) score->DrawMonoResults(Z::Gestalt::SPHERE, iplImage_l, iplImage_r, showMasked, showAllGestalts, mouseSide, showID, detail);
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
 */
void StereoDetector::WriteVisualObjects()
{
	DeleteVisualObjectsFromWM();	

	if(showAllStereo)
	{
		WriteToWM(Z::StereoBase::STEREO_FLAP);
		WriteToWM(Z::StereoBase::STEREO_RECTANGLE);
		WriteToWM(Z::StereoBase::STEREO_CLOSURE);
		WriteToWM(Z::StereoBase::STEREO_ELLIPSE);
		WriteToWM(Z::StereoBase::STEREO_FLAP_ARI);
		WriteToWM(Z::StereoBase::STEREO_CUBE);
	}
	else
	{
		switch(overlays)
		{
			case 104:
				WriteToWM(Z::StereoBase::STEREO_CLOSURE);
				break;
			case 105:
				WriteToWM(Z::StereoBase::STEREO_RECTANGLE);
				break;
			case 106:
				WriteToWM(Z::StereoBase::STEREO_FLAP);
				break;
			case 107:
				WriteToWM(Z::StereoBase::STEREO_FLAP_ARI);
				break;
			case 108:
				WriteToWM(Z::StereoBase::STEREO_CUBE);
				break;

			case 203:
				WriteToWM(Z::StereoBase::STEREO_ELLIPSE);
				break;

			default:
				log("unknown visual object identifier.");
				break;
		}
	}
}


/**
 * @brief Write visual objects of different type to the working memory.
 * @param type Type of StereoBase feature to write.
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

		addToWorkingMemory(objectID, obj);

cvWaitKey(200);	/// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.
	}
}


  /**
   * @brief Call single shot mode for debugging.
   */
/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   key:	p ... Process single shot \n
 *   key:	^ ... Show this help \n
 *   key:	1 ... Szene: Show all stereo features on virtual szene \n
 *   key:	2 ... Stereo: Show matched Gestalts \n
 *   key:	3 ... Mono: Show detected Gestalts \n
 *   key:	4 ... Mono: Show also masked Gestalts \n
 *   key: 5 ... Mono: Show single Gestalts \n
 *   key:	+ ... Mono: Increase degree of detail \n
 *   key:	- ... Mono: Decrease degree of detail \n
 *   key: . ... Mono: Increase ID of single showed Gestalt \n
 *   key: , ... Mono: Decrease ID of single showed Gestalt \n
 *   key:	x ... Mono: Stop single-shot processing mode.\n\n
 *
 *   key:	q ... Show SEGMENTS \n
 *   key:	w ... Show LINES \n
 *   key:	e ... Show COLLINEARITIES \n
 *   key:	r ... Show L-JUNCTIONS \n
 *   key:	t ... Show CLOSURES \n
 *   key:	z ... Show RECTANGLES \n
 *   key:	u ... Show FLAPS \n
 *   key:	i ... Show FLAPS_ARI \n
 *   key:	o ... Show CUBES \n
 *
 *   key:	a ... Show ARCS \n
 *   key:	s ... Show ARC-GROUPS \n
 *   key:	d ... Show A-JUNCTIONS \n
 *   key:	f ... Show ELLIPSES \n
 *   key:	g ... Show EXT-ELLIPSES: not yet implemented \n
 *   key:	h ... Show CYLINDERS: not yet implemented \n
 *   key:	j ... Show CONES: not yet implemented \n
 *   key:	k ... Show SPHERES: not yet implemented \n
 */
void StereoDetector::SingleShotMode()
{

	if(mouseEvent) MouseEvent();
	mouseEvent = false;



	int key = 0;
	key = cvWaitKey(10);

	if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %c", (char) key);
	switch((char) key)
	{
		case 'p':
			log("process stereo images (single shot).");
			videoServer->getImage(camIds[0], image_l);
			videoServer->getImage(camIds[1], image_r);
			gotImage = true;
			processImage();
			break;
		case '^':
			printf( "Keyboard commands for single shot mode:\n"
							"    key:	p ... Process single shot \n"
							"    key:	^ ... Show this help \n"
							"    key:	1 ... Szene: show all stereo features on virtual szene\n"
							"    key:	2 ... Stereo: show matched Gestalts \n"
							"    key:	3 ... Mono: show detected Gestalts \n"
							"    key:	4 ... Mono: show also masked Gestalts \n"
 							"    key: 5 ... Mono: Show single Gestalts \n"
							"    key:	+ ... Mono: Increase degree of detail \n"
							"    key:	- ... Mono: Decrease degree of detail \n"
							"    key: . ... Mono: Increase ID of single showed Gestalt \n"
							"    key: , ... Mono: Decrease ID of single showed Gestalt \n"
							"    key:	x ... Mono: Stop single-shot processing mode.\n\n"

							"    key:	q ... Show SEGMENTS\n"
							"    key:	w ... Show LINES\n"
							"    key:	e ... Show COLLINEARITIES\n"
							"    key:	r ... Show L-JUNCTIONS\n"
							"    key:	t ... Show CLOSURES\n"
							"    key:	z ... Show RECTANGLES\n"
							"    key:	u ... Show FLAPS\n"
							"    key:	i ... Show FLAPS_ARI\n"
							"    key:	o ... Show CUBES\n\n"

							"    key:	a ... Show ARCS\n"
							"    key:	s ... Show ARC-GROUPS\n"
							"    key:	d ... Show A-JUNCTIONS\n"
							"    key:	f ... Show ELLIPSES\n"
							"    key:	g ... Show EXT-ELLIPSES: not yet implemented\n"
							"    key:	h ... Show CYLINDERS: not yet implemented\n"
							"    key:	j ... Show CONES: not yet implemented\n"
							"    key:	k ... Show SPHERES: not yet implemented\n");
			break;
		case '1':
			if(showAllStereo) 
			{
				showAllStereo = false;
				printf("StereoDetector: Show ALL stereo features at virtual scene: OFF\n");
			}
			else
			{ 
				showAllStereo = true;
				printf("StereoDetector: Show ALL stereo features at virtual scene: ON\n");
			}
			WriteVisualObjects();
			break;
		case '2':
			if(showMatched) 
			{
				showMatched = false;
				printf("StereoDetector: Draw MATCHED features: OFF\n");
			}
			else
			{ 
				showMatched = true;
				printf("StereoDetector: Draw MATCHED features: ON\n");
			}
			if(showImages) ShowImages(true);
			break;
		case '3':
			if(showDetected)
			{
				showDetected = false;
				printf("StereoDetector: Draw all detected features: OFF\n");
			}
			else
			{
				showDetected = true;
				printf("StereoDetector: Draw all detected features: ON\n");
			}
			ShowImages(true);
			break;

		case '4':
			if(showMasked)
			{
				showMasked = false;
				printf("StereoDetector: Draw all MASKED features: OFF\n");
			}
			else
			{
				showMasked = true;
				printf("StereoDetector: Draw all MASKED features: ON\n");
			}
			ShowImages(true);
			break;
		case '5':
			if(showSingleGestalt)
			{
				showSingleGestalt = false;
				printf("StereoDetector: Show single Gestalts: OFF\n");
			}
			else
			{
				showSingleGestalt = true;
				printf("StereoDetector: Show single Gestalts: ON\n");
			}
			ShowImages(true);
			break;
		case '+':
			if(detail < 15)
			{
				detail++;
				printf("StereoDetector: Increased degree of display detail to: %u\n", detail);
				ShowImages(true);
			}
			break;
		case '-':
			if(detail > 0)
			{
				detail--;
				printf("StereoDetector: Decreased degree of display detail to: %u\n", detail);
				ShowImages(true);
			}
			break;
		case '.':
				showID++;
				printf("StereoDetector: Show single Gestalt: %u\n", showID);
				ShowImages(true);
			break;
		case ',':
			if(showID >= 0)
				showID--;
				printf("StereoDetector: Show single Gestalt: %u\n", showID);
				ShowImages(true);
			break;
		case 'x':
			log("stop debug mode.");
			single = false;
			break;


		case 'q':
			printf("StereoDetector: Show SEGMENTS\n");
			showType = Z::Gestalt::SEGMENT;
			overlays = 100;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'w':
			printf("StereoDetector: Show LINES\n");
			showType = Z::Gestalt::LINE;
			overlays = 101;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'e':
			printf("StereoDetector: Show COLLINEARITIES\n");
			showType = Z::Gestalt::COLLINEARITY;
			overlays = 102;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'r':
			printf("StereoDetector: Show L-JUNCTIONS\n");
			showType = Z::Gestalt::L_JUNCTION;
			overlays = 103;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 't':
			printf("StereoDetector: Show CLOSURES\n");
			showType = Z::Gestalt::CLOSURE;
			overlays = 104;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'z':
			printf("StereoDetector: Show RECTANGLES\n");
			showType = Z::Gestalt::RECTANGLE;
			overlays = 105;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'u':
			printf("StereoDetector: Show FLAPS\n");
			showType = Z::Gestalt::FLAP;
			overlays = 106;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'i':
			printf("StereoDetector: Show FLAPS_ARI\n");
			showType = Z::Gestalt::FLAP_ARI;
			overlays = 107;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'o':
			printf("StereoDetector: Show CUBES\n");
			showType = Z::Gestalt::CUBE;
			overlays = 108;
			ShowImages(true);
			WriteVisualObjects();
			break;

		case 'a':
			printf("StereoDetector: Show ARCS\n");
			showType = Z::Gestalt::ARC;
			overlays = 200;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 's':
			printf("StereoDetector: Show ARC-GROUPS\n");
			showType = Z::Gestalt::CONVEX_ARC_GROUP;
			overlays = 201;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'd':
			printf("StereoDetector: Show A-JUNCTIONS\n");
			showType = Z::Gestalt::A_JUNCTION;
			overlays = 202;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'f':
			printf("StereoDetector: Show ELLIPSES\n");
			showType = Z::Gestalt::ELLIPSE;
			overlays = 203;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'g':
			printf("StereoDetector: Show EXT-ELLIPSES: not yet implemented\n");
// 			showType = Z::Gestalt::EXTELLIPSE;
// 			overlays = 204;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'h':
			printf("StereoDetector: Show CYLINDERS: not yet implemented\n");
// 			showType = Z::Gestalt::CYLINDER;
// 			overlays = 205;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'j':
			printf("StereoDetector: Show CONES: not yet implemented\n");
// 			showType = Z::Gestalt::CONE;
// 			overlays = 206;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'k':
			printf("StereoDetector: Show SPHERES: not yet implemented\n");
// 			showType = Z::Gestalt::SPHERE;
// 			overlays = 207;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		default: break;
	}
}


/**
 * @brief Processes the mouse event.
 */
void StereoDetector::MouseEvent()
{
	/// Stereo core (left /right) => ask for id?
// 	unsigned start_after = 0;
// 	bool reject_masked = showMasked;
	showID = score->PickGestaltAt(mouseSide, showType, mouseX, mouseY, 0, false);
	if (showID < 0) return;
	// set id
	printf("StereoDetector::MouseEvent: showID: %u   (%u/%u), type: %u\n", showID, mouseX, mouseY, showType);
	ShowImages(true);
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



//   x,y:   pixel coordinates with respect to the UL corner
// 
//   event: CV_EVENT_LBUTTONDOWN,   CV_EVENT_RBUTTONDOWN,   CV_EVENT_MBUTTONDOWN,
//          CV_EVENT_LBUTTONUP,     CV_EVENT_RBUTTONUP,     CV_EVENT_MBUTTONUP,
//          CV_EVENT_LBUTTONDBLCLK, CV_EVENT_RBUTTONDBLCLK, CV_EVENT_MBUTTONDBLCLK,
//          CV_EVENT_MOUSEMOVE:
// 
//   flags: CV_EVENT_FLAG_CTRLKEY, CV_EVENT_FLAG_SHIFTKEY, CV_EVENT_FLAG_ALTKEY,
//          CV_EVENT_FLAG_LBUTTON, CV_EVENT_FLAG_RBUTTON,  CV_EVENT_FLAG_MBUTTON

}








