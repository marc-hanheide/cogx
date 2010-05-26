/**
 * @file StereoDetector.cpp
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 * 
 * @TODO Release iplImages: Destructor? destroy() ??? Is there any solution for this problem?
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <opencv/highgui.h>
#include "StereoDetector.h"
#include "StereoBase.h"
#include "Draw.hh"


using namespace std;
using namespace VisionData;
using namespace Video;

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
 * @param event Mouse event.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param flags 
 * @param param 
 */
void LeftMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONUP:
			mouseEvent = true;
			mouseX = x;
			mouseY = y;
			mouseSide = 0;
			break;
	}
}


/**
 * @brief Mouse handler for the two stereo images.
 * @param event Mouse event.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param flags 
 * @param param 
 */
void RightMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONUP:
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
StereoDetector::~StereoDetector() {}

/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config TODO
 */
void StereoDetector::configure(const map<string,string> & _config)
{
	// first let the base classes configure themselves (for getRectImage)
	configureStereoCommunication(_config);

	runtime = 800;								// processing time for left AND right image
	cannyAlpha = 0.8;							// Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
	cannyOmega = 0.001;

	receiveImagesStarted = false;
	haveImage = false;
	haveHRImage = false;
	havePrunedImage = false;
	cmd_detect = false;
	cmd_single = false;
	cmd_single_hr = false;

	detail = 0;
	showImages = false;
	showDetected = true;
	showSingleGestalt = false;
	showAllStereo = false;
	showID = 0;
	showMasked = false;
	showStereoMatched = true;
	showAllStereoMatched = false;
	single = false;
	showType = Z::Gestalt::SEGMENT;
	showStereoType = Z::StereoBase::STEREO_CLOSURE;
	showSegments = false;
	showROIs = false;


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
}

/**
 *	@brief Called by the framework after configuration, before run loop.
 */
void StereoDetector::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

	// start stereo communication
  startStereoCommunication(*this);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// add change filter for vision commands
  addChangeFilter(createLocalTypeFilter<StereoDetectionCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveDetectionCommand));

	// TODO add change filter for SOI changes (add / update / delete)
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
//     new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveSOI));
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
// 	  new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::updatedSOI));
//   addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
// 	  new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::deletedSOI));

	// add change filter for ROI changes
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveROI));
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::updatedROI));
  addChangeFilter(createLocalTypeFilter<VisionData::ROI>(cdl::DELETE),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::deletedROI));

	// add change filter for ProtoObject changes
  addChangeFilter(createLocalTypeFilter<ProtoObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveProtoObject));

	// initialize openCV windows
  if(showImages) 
	{
		cvNamedWindow("Stereo left", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Stereo right", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("rectified", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Pruned left", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Pruned right", CV_WINDOW_AUTOSIZE);

		cvMoveWindow("Stereo left", 10, 10);
		cvMoveWindow("Stereo right", 680, 10);
		cvMoveWindow("rectified",  1350, 10);
		cvMoveWindow("Pruned left",  10, 500);
		cvMoveWindow("Pruned right",  680, 500);

	  int mouseParam=0;
		cvSetMouseCallback("Stereo left", LeftMouseHandler, &mouseParam);
		cvSetMouseCallback("Stereo right", RightMouseHandler, &mouseParam);
	}

	isFormat7 = videoServer->inFormat7Mode();
}

/**
 *	@brief Called by the framework to start component run loop.
 *	@TODO LOCKT DEN SPEICHERBEREICH IM WM NICHT, SOLANGE GEARBEITET WIRD
 */
void StereoDetector::runComponent()
{
	while(single && isRunning()) SingleShotMode();
	while(isRunning()) {}

	log("destroy openCV windows.");
	cvDestroyWindow("Stereo left");
	cvDestroyWindow("Stereo right");
	cvDestroyWindow("rectified");
	cvDestroyWindow("Pruned left");
	cvDestroyWindow("Pruned right");
	log("windows destroyed");
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
        videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);		/// TODO Hier wird videoServer->startReceiveImages aufgerufen
				receiveImagesStarted = true;
				cmd_detect = true;
			}
			break;
		case VisionData::SDSTOP:
			if(cmd_detect){
				log("stopping stereo detection");
				cmd_detect = false;
				videoServer->stopReceiveImages(getComponentID().c_str());
				receiveImagesStarted = false;
			}else{
				log("stereo detection is already stopped");
			}
			break;
		case VisionData::SDSINGLE:
			if(!cmd_single){
				log("single stereo detection command received");
				cmd_single = true;
				processImage();
				cmd_single = false;
			}else{
				log("single stereo detection command already received: too fast triggering!");
			}
			break;
		case VisionData::SDSINGLEHR:																										/// TODO Hier wird videoServer->getImage aufgerufen
			if(!cmd_single_hr){
				log("single HR stereo detection command received");
				ProcessPrunedHRImages();
				cmd_single_hr = false;
			}else{
				log("single HR detection already received: too fast triggering!");
			}
			break;
		default:
			log("unknown detection command received, doing nothing");
			break;
	}	
}

/**
 *	@brief Receive a new created SOI.
 *	@param wmc Working memory change.
 */
void StereoDetector::receiveSOI(const cdl::WorkingMemoryChange & _wmc)
{
// 	SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
// 	Video::Image image;
// 	getRectImage(0, image);	// 0 = left image
// 	ROIPtr roiPtr = projectSOI(image.camPars, *soi);
// 	roi.width = roiPtr->rect.width;
// 	roi.height = roiPtr->rect.height;
// 	roi.x = roiPtr->rect.pos.x - roi.width/2;
// 	roi.y = roiPtr->rect.pos.y - roi.height/2;
// 	validROI = true;
// // 	log("calculated roi: x=%i, y=%i, width=%i, height=%i", roi.x, roi.y, roi.width, roi.height);
}

/**
 *	@brief Receive a updated SOI.
 *	@param wmc Working memory change.
 */
void StereoDetector::updatedSOI(const cdl::WorkingMemoryChange & _wmc)
{
// 	SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
// 	Video::Image image;
// 	getRectImage(0, image);	// 0 = left image
// 	ROIPtr roiPtr = projectSOI(image.camPars, *soi);
// 	roi.width = roiPtr->rect.width;
// 	roi.height = roiPtr->rect.height;
// 	roi.x = roiPtr->rect.pos.x - roi.width/2;
// 	roi.y = roiPtr->rect.pos.y - roi.height/2;	
// 	validROI = true;
// // 	log("updated roi: x=%i, y=%i, width=%i, height=%i", roi.x, roi.y, roi.width, roi.height);
}

/**
 *	@brief Receive a deleted SOI.
 *	@param wmc Working memory change.
 */
void StereoDetector::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{
// 	printf("SOI deleted!!!\n");
}

/**
 *	@brief Receive a new created ROI.
 *	@param wmc Working memory change.
 */
void StereoDetector::receiveROI(const cdl::WorkingMemoryChange & _wmc)
{
	try
	{
		ROIPtr roiPtr = getMemoryEntry<ROI>(_wmc.address);
		ROIData roiData;
		roiData.rect.width = roiPtr->rect.width;
		roiData.rect.height = roiPtr->rect.height;
		roiData.rect.x = roiPtr->rect.pos.x - roiData.rect.width/2;
		roiData.rect.y = roiPtr->rect.pos.y - roiData.rect.height/2;	
		PlausibleROI(&roiData);
		rcvROIs[_wmc.address.id] = roiData;
// 		log("received roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect.x, roiData.rect.y, roiData.rect.width, roiData.rect.height, _wmc.address.id.c_str());
	}
	catch (DoesNotExistOnWMException e)
	{
// 		log("ROI with id %s was removed from WM before it could be processed", _wmc.address.id.c_str());
// 		log("  => receiving was not possible.");
	}
	catch (std::exception e)
	{
		log("unknown exception during reading from WM.");
	}
}

/**
 *	@brief Receive a updated ROI.
 *	@param wmc Working memory change.
 */
void StereoDetector::updatedROI(const cdl::WorkingMemoryChange & _wmc)
{
	try
	{
		ROIPtr roiPtr = getMemoryEntry<ROI>(_wmc.address);
		ROIData roiData;
		roiData.rect.width = roiPtr->rect.width;
		roiData.rect.height = roiPtr->rect.height;
		roiData.rect.x = roiPtr->rect.pos.x - roiData.rect.width/2;
		roiData.rect.y = roiPtr->rect.pos.y - roiData.rect.height/2;	
		PlausibleROI(&roiData);
		rcvROIs[_wmc.address.id] = roiData;
// 		log("updated roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect.x, roiData.rect.y, roiData.rect.width, roiData.rect.height, _wmc.address.id.c_str());
	}
	catch (DoesNotExistOnWMException e)
	{
// 		log("ROI with id %s was removed from WM before it could be processed", _wmc.address.id.c_str());
// 		log("  => Update not possible");
	}
	catch (std::exception e)
	{
		log("unknown exception during reading from WM.");
	}
}

/**
 *	@brief Receive a deleted ROI.
 *	@param wmc Working memory change.
 */
void StereoDetector::deletedROI(const cdl::WorkingMemoryChange & _wmc)
{
	try
	{
		rcvROIs.erase(_wmc.address.id);
// 		log("deleted roi @ %s", _wmc.address.id.c_str());
	}
	catch (DoesNotExistOnWMException e)
	{
		log("ROI with id %s was removed from WM before it could be processed", _wmc.address.id.c_str());
	}
	catch (std::exception e)
	{
		log("unknown exception during reading from WM.");
	}
}

/**
 *	@brief Receive a new created ProtoObject.
 *	@param wmc Working memory change.
 */
void StereoDetector::receiveProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
	ProtoObjectPtr poPtr = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);

	Video::Image img;
	img = poPtr->image;
	poImg = convertImageToIpl(img);

// 	Video::ByteSeq data;
// 	data = poPtr->mask.data;

	// write mask to console
	poMask = new int [poPtr->mask.width*poPtr->mask.height];
	for (int i=0; i< poPtr->mask.width*poPtr->mask.height; i++)
	{
		poMask[i] = poPtr->mask.data[i];
		printf("%u  ", poMask[i]);
	}

	// create iplMask image
	IplImage *iplMask;
	iplMask = cvCreateImage(cvSize(poPtr->mask.width, poPtr->mask.height), IPL_DEPTH_8U, 1);
	for(unsigned i=0; i< poPtr->mask.data.size(); i++)
	{
		iplMask->imageData[i] = (char) poPtr->mask.data[i];
	}

	poPatchImage = cvCreateImage(cvSize(poPtr->mask.width*2, poPtr->mask.height*2), IPL_DEPTH_8U, 3);
	cvSetImageROI(poPatchImage, cvRect( 0, 0, poPtr->mask.width, poPtr->mask.height) );
	cvCopyImage(poImg, poPatchImage);
	cvSetImageROI(poPatchImage, cvRect( poPtr->mask.width, 0, poPtr->mask.width, poPtr->mask.height) );
	cvCvtColor(iplMask, poPatchImage, CV_GRAY2RGB);
// 	cvCopyImage(iplMask, poPatchImage);
	cvSetImageROI(poPatchImage, cvRect( 0, poPtr->mask.height, poPtr->mask.width, poPtr->mask.height) );
	cvCopyImage(poImg, poPatchImage);
	cvSetImageROI(poPatchImage, cvRect( poPtr->mask.width, poPtr->mask.height, poPtr->mask.width, poPtr->mask.height) );
	cvCopyImage(poImg, poPatchImage);
	cvResetImageROI(poPatchImage);

//	cvReleaseImage(&poPatchImage);
// 	cvReleaseImage(&iplMask);

// 	cvCvtColor(segPatch, tetraPatch, CV_GRAY2RGB);
// 	cvCvtColor(costPatch, tetraPatch, CV_GRAY2RGB);
// 	cvCvtColor(bgCostPatch, tetraPatch, CV_GRAY2RGB);

// 	delete []poMask;

	printf("  Proto object received:\n    mask w-h: %u - %u\n", poPtr->mask.width, poPtr->mask.height);
// 	printf("    data.size: %u\n", data.size());
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

	lockComponent();
	image_l = images[0];							/// TODO sollte man hier gleich auf iplImage konvertieren?
	image_r = images[1];
	haveImage = true;
	if (cmd_detect) processImage();
	unlockComponent();
}


/**
 *	@brief Process stereo image pair at stereo core.
 *	Use normal resolution of images (640x480).
 */
void StereoDetector::processImage()
{
	log("Process new image with runtime: %ums", runtime);

	// clear results before converting and processing new image
	score->ClearResults();

	// Get the images
	GetImages();

	// Process the stereo images at the stereo core and get visual (stereo matched) objects
	try 
	{
		score->ProcessStereoImage(runtime/2, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);
		log("Calculation of stereo images ended!");
	}
	catch(Z::Except &e)
	{
		log("errors during processing of stereo images.");
    printf("%s\n", e.what());
	}
  catch (exception &e)
  {
		log("unknown exception during processing of stereo images");
    cout << e.what() << endl;
  }

	WriteVisualObjects();
	ShowImages(false);

	log("Processing of stereo images ended.");
}


/**
 * @brief Process HR images with pruned LR-images. \n
 * Get pruned HR images, according to the delivered ROIs and process each stereo pair of them.
 */
void StereoDetector::ProcessHRImages()
{
	log("Process new HR- image with runtime: %ums", runtime);

	// Get HR images
	GetHRImages();
	if(!haveHRImage) log("No HR image available.");
	if(!haveHRImage) return;

printf("HR: width/height: %u/%u\n", iplImage_l_hr->width, iplImage_l_hr->height);

	// clear results before converting and processing new image
	score->ClearResults();

	// resize images
	cvResize(iplImage_l_hr, iplImage_l);
	cvResize(iplImage_r_hr, iplImage_r);

	map<std::string, ROIData>::iterator it;
	for (it=rcvROIs.begin(); it != rcvROIs.end(); it++)
	{
		CvRect roi640 = ((*it).second).rect640;
		bool rect640valid = ((*it).second).rect640valid;
		if(rect640valid)
		{
			log("Process HR images with ROI: %u/%u", roi640.x, roi640.y);
			iplImage_l_pr = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, iplImage_l_hr->nChannels);
			iplImage_r_pr = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, iplImage_r_hr->nChannels);
			PruneImageArea(iplImage_l_hr, *iplImage_l_pr, 640, 480, roi640.x, roi640.y);
			PruneImageArea(iplImage_r_hr, *iplImage_r_pr, 640, 480, roi640.x, roi640.y);
			havePrunedImage = true;

			processPrunedHRImage(roi640.x, roi640.y, 2);		/// TODO Now process the two images
		}
		else log("ROI not valid.");
	}

	WriteVisualObjects();
	ShowImages(false);
	havePrunedImage = false;
}


/**
 * @brief Prune HR images and process for all ROIs. \n
 * Get pruned HR images, according to the delivered ROIs and process each stereo pair of them.
 */
void StereoDetector::ProcessPrunedHRImages()
{
printf("ProcessPrunedHRImages\n");
	// get original image
	GetImages();

printf("ProcessPrunedHRImages 1\n");
	map<std::string, ROIData>::iterator it;
	for (it=rcvROIs.begin(); it != rcvROIs.end(); it++)
	{
		CvRect roi640 = ((*it).second).rect640;
		bool rect640valid = ((*it).second).rect640valid;
		if(rect640valid)
		{
			// get pruned images
			if(!GetPrunedHRImages(roi640.x, roi640.y))
			{
				log("Could not get pruned HR image. Processing stopped.");
				return;
			}
			processPrunedHRImage(roi640.x, roi640.y, 2);						/// TODO Add scale (2:1)
printf("ProcessPrunedHRImages 2\n");
		}
		else log("ROI not valid.");
	}

printf("ProcessPrunedHRImages 3\n");
	WriteVisualObjects();
printf("ProcessPrunedHRImages 4\n");
// 	ShowImages(false);
printf("ProcessPrunedHRImages 5\n");

	log("processing of pruned stereo images ended.");
}

/**
 *	@brief Process a pruned stereo image pair at stereo core.
 *	@param oX Offset of x-coordinate
 *	@param oY Offset of y-coordinate
 *	@param sc Scale between original and pruned image.
 */
void StereoDetector::processPrunedHRImage(int oX, int oY, int sc)
{
	log("Process pruned image with runtime: %ums", runtime);

	
	// TODO Es sollten die VCs gesäubert werden, aber die Stereo-Ergebnisse noch nicht! Stereo-Core umbauen!
	// clear results before converting and processing new image												/// TODO es könnten mehrere geprunte images verarbeitet werden.
	score->ClearResults();	// TODO muss zu ProcessPrunedHRImages wandern!
	
	// Process the stereo images at the stereo core and get visual (stereo matched) objects
	try 
	{
		double ca = 0.4;		/// TODO Canny alpha
		double co = 0.001;	/// TODO Canny omega
		score->ProcessStereoImage(runtime/2, ca, co, iplImage_l_pr, iplImage_r_pr, oX, oY, sc);
		log("Calculation of pruned stereo images ended!");
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
}


/**
 * @brief Show both stereo images in the openCV windows.
 * @param convertNewIpl Convert image again into iplImage to delete overlays.
 */
void StereoDetector::ShowImages(bool convertNewIpl)
{
// printf("ShowImages start\n");
	if(!haveImage || !showImages) return;

	if(convertNewIpl)
	{
		iplImage_l = convertImageToIpl(image_l);
		iplImage_r = convertImageToIpl(image_r);
	}

	if(showSegments)
		score->DrawMonoResults(Z::Gestalt::SEGMENT, iplImage_l, iplImage_r, true, false, mouseSide, showID, detail);

	if(showDetected || showSingleGestalt) 
	{
		while(!(score->DrawMonoResults(showType, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail)) && showID > 0)
			showID--;
		if(showSingleGestalt) log("show single Gestalt: %u", showID);
	}

	if(showStereoMatched || showAllStereoMatched)
	{
		if(showStereoType != Z::StereoBase::UNDEF)
			score->DrawStereoResults(showStereoType, iplImage_l, iplImage_r, showStereoMatched, showAllStereoMatched);
	}

	// get rectified image from stereo server
	Video::Image image;
	getRectImage(0, image);						// 0 = left image
	IplImage *rImg;
	rImg = convertImageToIpl(image);
	
	if(showROIs)
	{
		// draw the ROIs and the rectangle for the pruning
		map<std::string, ROIData>::iterator it;
		for (it=rcvROIs.begin(); it != rcvROIs.end(); it++)
		{
			CvRect roi = ((*it).second).rect;
			CvRect roi640 = ((*it).second).rect640;
			bool roi640valid = ((*it).second).rect640valid;
			int roiScale = ((*it).second).roiScale;
	
			score->DrawROI(0, roi, 1, rImg, iplImage_r);
			if(!havePrunedImage)																									/// TODO ist das noch richtig?
			{
				score->DrawROI(0, roi, roiScale, iplImage_l, iplImage_r);
	
				if(roi640valid)
				{
					score->DrawPrunedROI(0, roi640.x/2, roi640.y/2, iplImage_l, iplImage_r);		// TODO Draw 640x480 ROI for pruned image => /2 kann nicht stimmen
					score->DrawPrunedROI(1, roi640.x/2, roi640.y/2, iplImage_l, iplImage_r);		// TODO Draw 640x480 ROI for pruned image
				}
				else log("ROI for pruning from HR image is not valid!");
			}
		}
	}

	cvShowImage("Stereo left", iplImage_l);
	cvShowImage("Stereo right", iplImage_r);
	cvShowImage("rectified", rImg);
	if (havePrunedImage) 
	{
		cvShowImage("Pruned left", iplImage_l_pr);
		cvShowImage("Pruned right", iplImage_r_pr);
	}

	cvWaitKey(50);	///< TODO TODO wait key to allow openCV to show the images on the window.
// printf("ShowImages end\n");
}


/**
 * @brief Delete working memory and (re)write different visual objects from the stereo detector.
 */
void StereoDetector::WriteVisualObjects()
{
	DeleteVisualObjectsFromWM();	

	if(showAllStereo)
	{
		for(int i=0; i<Z::StereoBase::MAX_TYPE; i++)
		WriteToWM((Z::StereoBase::Type) i);
	}
	else
	{
		if(showStereoType != Z::StereoBase::UNDEF)
			WriteToWM(showStereoType);
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

		addToWorkingMemory(objectID, obj);

		cvWaitKey(150);	/// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.

		log("Add new visual object to working memory: %s - %s", obj->label.c_str(), objectID.c_str());
	}
}


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


/**
 * @brief Single shot mode of the stereo detector for debugging.\n
 * Catch keyboard events and change displayed results:\n
 *   F1 ... Show this help
 *   F2 ... Print number of Gestalts
 *   F3 ... Print information about the visual feature (show single gestalts == on)
 *   F5 ... Refresh display
 *   F9 ... Process single shot \n
 *   F10 .. Process pruned image (Format7 & ROI) \n
 *   F11 .. Process HR image (pruned & ROI)
 * 
 *   key:	p ... Process single shot \n
 *   key:	^ ... Show this help \n
 *   key:	1 ... Szene: Show all stereo features on virtual szene (on/off) \n
 *   key:	2 ... Stereo: Show matched Gestalts (on/off) \n
 *   key:	3 ... Mono: Show detected Gestalts (on/off) \n
 *   key:	4 ... Mono: Show also masked Gestalts (on/off) \n
 *   key:	5 ... Mono: Show single Gestalts (on/off) \n
 *   key:	6 ... Mono: Show edge segments (on/off) \n
 *   key: 7 ... Mono: Show single Gestalts \n
 *   key: 8 ... Mono: Show ROI windows \n
 *   key:	+ ... Mono: Increase degree of detail \n
 *   key:	- ... Mono: Decrease degree of detail \n
 *   key:	. ... Mono: Increase ID of single showed Gestalt \n
 *   key:	, ... Mono: Decrease ID of single showed Gestalt \n
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

// 	if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %c, %i", (char) key, key);
	if (key == 65470 || key == 1114046)	// F1
		printf("Keyboard commands for single shot mode:\n"
						"    F1 ... Show this help \n"
						"    F2 ... Print number of Gestalts\n"
						"    F3 ... Print information (show single gestalts (5) = on)\n\n"
						"    F5 ... Refresh display\n\n"
						"    F9 ... Process single shot\n"
						"    F10 .. Process single shot with region of interest (ROI)\n"
						"    F11 .. Process single shot with HR and ROI\n"

						"    1 ... Szene: Show all stereo features on virtual szene\n"
						"    2 ... Stereo: Show all matched features \n"
						"    3 ... Stereo: Show matched features \n"
						"    4 ... Mono: Show all edge segments \n"
						"    5 ... Mono: Show detected Gestalts \n"
						"    6 ... Mono: Show the masked Gestalts \n"
						"    7 ... Mono: Show single Gestalts \n"
						"    8 ... Mono: Show ROI windows\n"
						"    + ... Mono: Increase degree of detail \n"
						"    - ... Mono: Decrease degree of detail \n"
						"    . ... Mono: Increase ID of single showed Gestalt \n"
						"    , ... Mono: Decrease ID of single showed Gestalt \n"
						"    x ... Mono: Stop single-shot processing mode.\n\n"

						"    q ... Show SEGMENTS\n"
						"    w ... Show LINES\n"
						"    e ... Show COLLINEARITIES\n"
						"    r ... Show L-JUNCTIONS\n"
						"    t ... Show CLOSURES\n"
						"    z ... Show RECTANGLES\n"
						"    u ... Show FLAPS\n"
						"    i ... Show FLAPS_ARI\n"
						"    o ... Show CUBES\n\n"

						"    a ... Show ARCS\n"
						"    s ... Show ARC-GROUPS\n"
						"    d ... Show A-JUNCTIONS\n"
						"    f ... Show ELLIPSES\n"
						"    g ... Show EXT-ELLIPSES: not yet implemented\n"
						"    h ... Show CYLINDERS: not yet implemented\n"
						"    j ... Show CONES: not yet implemented\n"
						"    k ... Show SPHERES: not yet implemented\n");

	if (key == 65471 || key == 1114047)	// F2
	{
		const char* text = (score->GetMonoCore(0))->GetGestaltListInfo();
		log("Gestalt list left:\n%s\n", text);
		text = (score->GetMonoCore(1))->GetGestaltListInfo();
		log("Gestalt list right:\n%s\n", text);
	}

	if (key == 65472 || key == 1114048)	// F3
	{
		if(showSingleGestalt && showID != -1 && (mouseSide == 0 || mouseSide == 1)) 
		{
			const char* text = (score->GetMonoCore(mouseSide))->GetInfo(showType, showID);
			log("Gestalt infos:\n%s\n", text);
		}
	}

	if (key == 65474 || key == 1114050)	// F5
	{
		if(!haveImage) GetImages();
		if(showImages) ShowImages(true);
	}


	if (key == 65478 || key == 1114054)	// F9
	{
		log("process stereo images (single shot).");
		lockComponent();
		processImage();
		unlockComponent();
	}
	if (key == 65479 || key == 1114055)	// F10
	{
		log("process pruned stereo images (single shot with ROI).");
		lockComponent();
		ProcessPrunedHRImages();
		unlockComponent();
	}
	if (key == 65480 || key == 1114056)	// F11
	{
		log("process HR stereo images (single shot with ROI).");
		lockComponent();
		ProcessHRImages();
		unlockComponent();
	}

// 	if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
	switch((char) key)
	{
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
			if(showAllStereoMatched) 
			{
				showAllStereoMatched = false;
				printf("StereoDetector: Draw ALL MATCHED features: OFF\n");
			}
			else
			{ 
				showAllStereoMatched = true;
				printf("StereoDetector: Draw ALL MATCHED features: ON\n");
			}
			if(showImages) ShowImages(true);
			break;
		case '3':
			if(showStereoMatched) 
			{
				showStereoMatched = false;
				printf("StereoDetector: Draw MATCHED stereo features: OFF\n");
			}
			else
			{ 
				showStereoMatched = true;
				printf("StereoDetector: Draw MATCHED stereo features: ON\n");
			}
			if(showImages) ShowImages(true);
			break;
		case '4':
			if(showSegments)
			{
				showSegments = false;
				printf("StereoDetector: Show edge segments: OFF\n");
			}
			else
			{
				showSegments = true;
				printf("StereoDetector: Show edge segments: ON\n");
			}
			ShowImages(true);
			break;
		case '5':
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
		case '6':
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
		case '7':
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
		case '8':
			if(showROIs)
			{
				showROIs = false;
				printf("StereoDetector: Show ROIs: OFF\n");
			}
			else
			{
				showROIs = true;
				printf("StereoDetector: Show ROIs: ON\n");
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
			ShowImages(true);
			break;
		case ',':
			if(showID > 0)
				showID--;
			ShowImages(true);
			break;
		case 'x':
			log("stop debug mode.");
			single = false;
			break;

		case 'q':
			printf("StereoDetector: Show SEGMENTS\n");
			showType = Z::Gestalt::SEGMENT;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'w':
			printf("StereoDetector: Show LINES\n");
			showType = Z::Gestalt::LINE;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'e':
			printf("StereoDetector: Show COLLINEARITIES\n");
			showType = Z::Gestalt::COLLINEARITY;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'r':
			printf("StereoDetector: Show L-JUNCTIONS\n");
			showType = Z::Gestalt::L_JUNCTION;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 't':
			printf("StereoDetector: Show CLOSURES\n");
			showType = Z::Gestalt::CLOSURE;
			showStereoType = Z::StereoBase::STEREO_CLOSURE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'z':
			printf("StereoDetector: Show RECTANGLES\n");
			showType = Z::Gestalt::RECTANGLE;
			showStereoType = Z::StereoBase::STEREO_RECTANGLE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'u':
			printf("StereoDetector: Show FLAPS\n");
			showType = Z::Gestalt::FLAP;
			showStereoType = Z::StereoBase::STEREO_FLAP;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'i':
			printf("StereoDetector: Show FLAPS_ARI\n");
			showType = Z::Gestalt::FLAP_ARI;
			showStereoType = Z::StereoBase::STEREO_FLAP_ARI;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'o':
			printf("StereoDetector: Show CUBES\n");
			showType = Z::Gestalt::CUBE;
			showStereoType = Z::StereoBase::STEREO_CUBE;
			ShowImages(true);
			WriteVisualObjects();
			break;

		case 'a':
			printf("StereoDetector: Show ARCS\n");
			showType = Z::Gestalt::ARC;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 's':
			printf("StereoDetector: Show ARC-GROUPS\n");
			showType = Z::Gestalt::CONVEX_ARC_GROUP;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'd':
			printf("StereoDetector: Show A-JUNCTIONS\n");
			showType = Z::Gestalt::A_JUNCTION;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'f':
			printf("StereoDetector: Show ELLIPSES\n");
			showType = Z::Gestalt::ELLIPSE;
			showStereoType = Z::StereoBase::STEREO_ELLIPSE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'g':
			printf("StereoDetector: Show EXT-ELLIPSES: not yet implemented\n");
// 			showType = Z::Gestalt::EXTELLIPSE;
// 			showStereoType = Z::StereoBase::UNDEF;
// 			overlays = 204;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'h':
			printf("StereoDetector: Show CYLINDERS: not yet implemented\n");
// 			showType = Z::Gestalt::CYLINDER;
// 			showStereoType = Z::StereoBase::UNDEF;
// 			overlays = 205;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'j':
			printf("StereoDetector: Show CONES: not yet implemented\n");
// 			showType = Z::Gestalt::CONE;
// 			showStereoType = Z::StereoBase::UNDEF;
// 			overlays = 206;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'k':
			printf("StereoDetector: Show SPHERES: not yet implemented\n");
// 			showType = Z::Gestalt::SPHERE;
// 			showStereoType = Z::StereoBase::UNDEF;
// 			overlays = 207;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		default: break;
	}
}


/**
 * @brief Processes the mouse event.
 * Stores found ID of Gestalt in showID. If more than one Gestalt found,
 * take with next pressed button the next Gestalt and so on.
 */
void StereoDetector::MouseEvent()
{
	static int prev_picked = -1, prev_mouseSide = 0;
	static int prev_x = INT_MAX, prev_y = INT_MAX;

	if(mouseX != prev_x || mouseY != prev_y || mouseSide != prev_mouseSide)
		prev_picked = -1;

	showID = score->PickGestaltAt(mouseSide, showType, mouseX, mouseY, prev_picked, false);
	if(showID != -1)
	{
		prev_picked = showID;
		prev_mouseSide = mouseSide;
		prev_x = mouseX;
		prev_y = mouseY;
	}
	else // check again in small surrounding area
	{
		bool found = false;
		int a_end = min((int) ((score->GetMonoCore(mouseSide))->IW())-1, mouseX+1);
		int b_end = min((int) ((score->GetMonoCore(mouseSide))->IH())-1, mouseY+1);
		for(int a = max(0, mouseX-1); a <= a_end && !found; a++)
			for(int b = max(0, mouseY-1); b <= b_end && !found; b++)
			{
				showID = score->PickGestaltAt(mouseSide, showType, a, b, prev_picked, false);
				if(showID != -1)
				{
					found = true;
					prev_picked = showID;
					prev_mouseSide = mouseSide;
					prev_x = mouseX;
					prev_y = mouseY;
				}
			}
	}

	if (showID < 0) return;
	log("show feature with id=%u at position (%u/%u).", showID, mouseX, mouseY);
	ShowImages(true);
}


/**																																							/// TODO Kann man löschen!!!
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


/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
void StereoDetector::GetImages()
{
	if(isFormat7)
	{
		ChangeFormat7Mode(1, 0, 0);
		std::vector<Video::Image> images;
		videoServer->getImages(images);
		image_l = images[0];
		image_r = images[1];

		// Convert images (from Video::Image to iplImage)
		iplImage_l = convertImageToIpl(images[0]);
		iplImage_r = convertImageToIpl(images[1]);
	}
	else
	{
		std::vector<Video::Image> images;
		videoServer->getImages(images);
		image_l = images[0];
		image_r = images[1];

		// Convert images (from Video::Image to iplImage)
		iplImage_l = convertImageToIpl(images[0]);
		iplImage_r = convertImageToIpl(images[1]);
	}

	haveImage = true;
	haveHRImage = false;
	havePrunedImage = false;
}

/**
 * @brief Get high resolution image from video server.
 */
void StereoDetector::GetHRImages()
{
	if(!isFormat7)
	{
		std::vector<Video::Image> images;
	
		haveHRImage = false;
		havePrunedImage = false;

		if(videoServer->getHRImages(images))
		{
			haveHRImage = true;							/// TODO getHRImages liefert nicht richtig true/false zurück!
			// Convert images (from Video::Image to iplImage)
			iplImage_l_hr = convertImageToIpl(images[0]);
			iplImage_r_hr = convertImageToIpl(images[1]);
		}
		else return;
	}
	else log("HR images are not available in Format7-mode.");
}


/**
 * @brief Get pruned image from high resolution image from PG-Cams with Format7-mode.
 * @param roi ID of ROI (region of interest) from the roi-map.
 * @return Returns true for success.
 */
bool StereoDetector::GetPrunedHRImages(int offsetX, int offsetY)
{
	if(isFormat7)
	{
		log("Get pruned image: 1280x960 => 640x480 @ %i, %i", offsetX, offsetY);

		ChangeFormat7Mode(0, offsetX, offsetY);

		std::vector<Video::Image> images;
		videoServer->getImages(images);

		iplImage_l_pr = convertImageToIpl(images[0]);
		iplImage_r_pr = convertImageToIpl(images[1]);

		havePrunedImage = true;
		haveHRImage = false;

		ChangeFormat7Mode(1, 0, 0);
		return true;
	}
	else																																/// TODO noch nicht implementiert.
	{
		log("PG cams not in Format7 mode.");
		return false;
	}
}


/**
 * @brief Change the Format7 mode for the PointGrey cameras (if neccessary).
 * @param mode Which video mode.
 * @param offsetX Offset x-coordinat for mode7 = 0
 * @param offsetY Offset y-coordinat for mode7 = 0
 */
void StereoDetector::ChangeFormat7Mode(int mode, int offsetX, int offsetY)
{
	if(mode != 0 && mode != 1 && !isFormat7) return;

	if(videoServer->getServerName() == "PointGreyServer")
	{
		if(mode == 1 && mode7 == 0)		// change only when not already in mode7=1
		{
			videoServer->changeFormat7Properties(640, 480, 0, 0, 1, 1600);
			mode7 = 1;
		}
		if(mode == 0)
		{
			videoServer->changeFormat7Properties(640, 480, offsetX, offsetY, 0, 1600);
			mode7 = 0;
		}
	}
	else log("only possible with PointGrey stereo cameras.");
}


/**
 * @brief Calculate the window for the pruning from the HR-image with a window size of 640x480.
 * @param roiData The region of interest data.
 * @return Returns false, whan offset cannot be calculated (eg. when no image is captured)
 */
bool StereoDetector::PlausibleROI(ROIData *roiData)
{
	roiData->rect640valid = false;

	if(!haveImage)	return false;

	// calculate scale between stereo and video server (get stereo image => TODO this needs time (do it once))
	Video::Image image;
	getRectImage(0, image);
	roiData->roiScale = image_l.width / image.width;

	int leftShift = image_l.width/6;		// TODO TODO TODO Besser lösen

	/// TODO TODO TODO TODO roiScale/roiScale*2 kann in den nächsten Zeilen nicht stimmen!!!
	roiData->rect640.x = (roiData->rect.x*roiData->roiScale + roiData->rect.width*roiData->roiScale/2 - image_l.width/roiData->roiScale) * 2 - leftShift;
	roiData->rect640.y = (roiData->rect.y*roiData->roiScale + roiData->rect.height*roiData->roiScale/2 - image_l.height/roiData->roiScale) * 2;
	roiData->rect640.width = image_l.width;
	roiData->rect640.height = image_l.height;
	
	if(roiData->rect640.x < 0) roiData->rect640.x = 0;
	if(roiData->rect640.y < 0) roiData->rect640.y = 0;

	if(roiData->rect640.x > image_l.width) return false;
	if(roiData->rect640.y > image_l.height) return false;

	roiData->rect640valid = true;
	return true;
}
}






