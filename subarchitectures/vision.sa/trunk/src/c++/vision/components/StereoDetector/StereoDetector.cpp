/**
 * @file StereoDetector.cpp
 * @author Andreas Richtsfeld
 * @date 2009, 2010
 * @version 0.1
 * @brief Detecting objects with stereo rig for cogx project.
 * 
 * @TODO Destroy cvImages
 */


#include <cast/architecture/ChangeFilterFactory.hpp>

#include <highgui.h>
#include "StereoDetector.h"
#include "StereoBase.h"
#include "Draw.hh"

#include "Mouse.cpp"


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

	nr_p_score = 0;								// start with first processing score
	
	runtime = 1600;								// processing time for left AND right image
	cannyAlpha = 0.8;							// Canny alpha and omega for MATAS canny only! (not for openCV CEdge)
	cannyOmega = 0.001;

	activeReasoner = true;				// activate reasoner
	activeReasonerPlane = true;		// activate plane C
	
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
	showSingleStereo = false;
	single = false;
	showType = Z::Gestalt::SEGMENT;
	showStereoType = Z::StereoBase::STEREO_CLOSURE;
	showSegments = false;
	showROIs = false;
	showReasoner = true;
	showReasonerUnprojected = false;

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
			p_score[0] = new Z::StereoCore(camconfig);
			p_score[1] = new Z::StereoCore(camconfig);
			p_score[2] = new Z::StereoCore(camconfig);
		}
		catch (exception &e)
		{
			printf("StereoDetector::configure: Error during initialisation of stereo core.\n");
			cout << e.what() << endl;
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
	
	reasoner = new Z::Reasoner();
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

	// add change filter for ConvexHull changes
  addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetector>(this, &StereoDetector::receiveConvexHull));

	// initialize openCV windows
  if(showImages) 
	{
		cvNamedWindow("Stereo left", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Stereo right", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("rectified", CV_WINDOW_AUTOSIZE);
// 		cvNamedWindow("Pruned left", CV_WINDOW_AUTOSIZE);
// 		cvNamedWindow("Pruned right", CV_WINDOW_AUTOSIZE);

		cvMoveWindow("Stereo left", 10, 10);
		cvMoveWindow("Stereo right", 680, 10);
		cvMoveWindow("rectified",  1350, 10);
// 		cvMoveWindow("Pruned left",  10, 500);
// 		cvMoveWindow("Pruned right",  680, 500);

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
// 	cvDestroyWindow("Pruned left");
// 	cvDestroyWindow("Pruned right");
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
 *	@brief Receive a new created convex hull. Copy it to a visual object.
 *	@param wmc Working memory change.
 */
void StereoDetector::receiveConvexHull(const cdl::WorkingMemoryChange & _wmc)
{
	log("Process new convex hull");
	ConvexHullPtr chPtr = getMemoryEntry<VisionData::ConvexHull>(_wmc.address);

	// Create a visual object for the plane as mesh and recalculate point sequence 
	// of convex hull in respect to the center.
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
	obj->pose = chPtr->center;
	VEC::Vector3 p;
	std::vector<VEC::Vector3> points;
	for(unsigned i=0; i< chPtr->PointsSeq.size(); i++)
	{
		p.x = chPtr->PointsSeq[i].x - obj->pose.pos.x;	// shift the plane in respect to the pose
		p.y = chPtr->PointsSeq[i].y - obj->pose.pos.y;
		p.z = chPtr->PointsSeq[i].z - obj->pose.pos.z;
		points.push_back(p);
	}
	
	VEC::Vector3 pos;				// center position of the plane
	pos.x = obj->pose.pos.x;
	pos.y = obj->pose.pos.y;
	pos.z = obj->pose.pos.z;
	double radius = chPtr->radius;
	
	if(activeReasonerPlane)
	{
		reasoner->ProcessConvexHull(pos, radius, points);
		if(reasoner->GetPlane(obj))
		{
			planeID = newDataID();
			addToWorkingMemory(planeID, obj);
			log("Wrote new dominant plane as visual object to working memory: %s", planeID.c_str());
		}
	}

// 	printf("  center position: %4.2f / %4.2f / %4.2f\n", chPtr->center.pos.x, chPtr->center.pos.y, chPtr->center.pos.z);
// 	printf("  center rotation matrix:\n    %4.2f / %4.2f / %4.2f\n    %4.2f / %4.2f / %4.2f\n    %4.2f / %4.2f / %4.2f\n", 
// 				 chPtr->center.rot.m00, chPtr->center.rot.m01, chPtr->center.rot.m02, 
// 				 chPtr->center.rot.m10, chPtr->center.rot.m11, chPtr->center.rot.m12, 
// 				 chPtr->center.rot.m20, chPtr->center.rot.m21, chPtr->center.rot.m22);
// 	printf("  radius: %4.2f\n", chPtr->radius);
// 	printf("  plane: %4.2f / %4.2f / %4.2f / %4.2f\n", chPtr->plane.a, chPtr->plane.b, chPtr->plane.c, chPtr->plane.d);
// 	printf("  objects on plane: %u\n", chPtr->Objects.size());
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
	log("Process new images with runtime: %ums", runtime);
//	score->ClearResults();
	p_score[nr_p_score]->ClearResults();
	GetImages();

	log("Got images.");

	try 
	{
		p_score[nr_p_score]->ProcessStereoImage(runtime/2, cannyAlpha, cannyOmega, iplImage_l, iplImage_r);
	}
  catch (exception &e)
  {
		log("StereoDetector::processImage: Unknown exception during processing of stereo images.\n");
    cout << e.what() << endl;
  }

	log("Processed images.");
	score = p_score[nr_p_score];			// copy processed score to main score for displaying
	log("Copied processed score to main.");

	if(activeReasoner)	// do the reasoner things!
	{
		log("Start reasoner.");
		reasoner->Process(score);
		
// 		if(reasoner->Process(score))														/// TODO delete
// 		{
// 			log("Get results.");
// 			Z::Array<VisionData::VisualObjectPtr> objects;
// 			reasoner->GetResults(objects);
// 
// 			log("Write results to working memory!");
// 			WriteToWM(objects);
// 		}
// 		log("End reasoner.");
	}
// 	else WriteVisualObjects();
	
	WriteVisualObjects();
	ShowImages(false);
	
	nr_p_score++;
	if(nr_p_score > 2) nr_p_score=0;
	log("Processing of stereo images ended.");
}

/**
 * @brief Process HR images with pruned LR-images. \n
 * Get pruned HR images, according to the delivered ROIs and process each stereo pair of them.
 */
void StereoDetector::ProcessHRImages()
{
	log("Process new HR- images with runtime: %ums", runtime);

	// Get HR images
	GetHRImages();
	if(!haveHRImage) log("No HR image available.");
	if(!haveHRImage) return;

	score->ClearResults();
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

	score->ClearResults();
	try 
	{
		double ca = 0.4;		// Canny alpha
		double co = 0.001;	// Canny omega
		score->ProcessStereoImage(runtime/2, ca, co, iplImage_l_pr, iplImage_r_pr, oX, oY, sc);
		log("Calculation of pruned stereo images ended!");
	}
  catch (exception &e)
  {
		log("StereoDetector::processPrunedHRImage: Exception during processing of stereo images");
    cout << e.what() << endl;
  }
}

/**
 * @brief Show both stereo images in the openCV windows.
 * @param convertNewIpl Convert image again into iplImage to delete overlays.
 */
void StereoDetector::ShowImages(bool convertNewIpl)
{
	if(!haveImage || !showImages) return;

	if(convertNewIpl)
	{
		iplImage_l = convertImageToIpl(image_l);
		iplImage_r = convertImageToIpl(image_r);
	}

	// segments
	if(showSegments)
		score->DrawMonoResults(Z::Gestalt::SEGMENT, iplImage_l, iplImage_r, true, false, mouseSide, showID, detail);

	// mono results
	if(showDetected || showSingleGestalt) 
	{
		while(!(score->DrawMonoResults(showType, iplImage_l, iplImage_r, showMasked, showSingleGestalt, mouseSide, showID, detail)) && showID > 0)
			showID--;
		if(showSingleGestalt) log("show single mono feature: %u", showID);
	}

	// stereo results (2, 3, 4)
	if(showAllStereoMatched || showStereoMatched || showSingleStereo)
	{
		if(showStereoType != Z::StereoBase::UNDEF || showAllStereoMatched)
			score->DrawStereoResults(showStereoType, iplImage_l, iplImage_r, showAllStereoMatched, showSingleStereo, showID, detail);
		if(showSingleStereo) log("show single stereo match: %u", showID);
	}

	// get rectified image from stereo server
	Video::Image image;
	getRectImage(0, 640, image);						// 0 = left image / 640 = width
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
	
			score->DrawROI(0, roi, 1, rImg, iplImage_r);													/// TODO Wieso braucht man hier rImg???
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
// 		cvShowImage("Pruned left", iplImage_l_pr);
// 		cvShowImage("Pruned right", iplImage_r_pr);
	}

	cvWaitKey(50);	///< TODO TODO wait key to allow openCV to show the images on the window.
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
	
	if(showReasoner)
	{
		Z::Array<VisionData::VisualObjectPtr> objects;
		reasoner->GetResults(objects, showReasonerUnprojected);
// 		log("Write reasoner results to working memory!");
		WriteToWM(objects);
	}
}

/**
 * @brief Write visual objects of different type to the working memory.
 * @param type Type of StereoBase feature to write.
 */
void StereoDetector::WriteToWM(Z::StereoBase::Type type)
{
	static unsigned frameNumber = 0;
// 	static unsigned numStereoObjects = 0;
	VisionData::VisualObjectPtr obj;

	for(int i=0; i<score->NumStereoMatches((Z::StereoBase::Type) type); i++)
	{
		obj = new VisionData::VisualObject;
		bool success = score->GetVisualObject((Z::StereoBase::Type) type, i, obj);

		if(success)
		{
			// add visual object to working memory
			std::string objectID = newDataID();
			objectIDs.push_back(objectID);

// 			VisionData::ReasonerObjectPtr reaObj = new VisionData::ReasonerObject;						/// TODO TODO TODO Write to Reasoner COMPONENT => delete later
// 			reaObj->obj = obj;
// 			reaObj->frameNr = frameNumber;
// 			addToWorkingMemory(objectID, reaObj);

			addToWorkingMemory(objectID, obj);																									/// TODO TODO TODO Write Visual Object!!!

			cvWaitKey(200);	/// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.

			log("Add new visual object to working memory: %s", objectID.c_str());
		}
	}
	
	// Send newFrame command for Reasoner component => TODO delete later
	VisionData::SDReasonerCommandPtr newFrame = new VisionData::SDReasonerCommand;
	newFrame->cmd = VisionData::NEWFRAME;
	addToWorkingMemory(newDataID(), newFrame);
	debug("NewFrame command sent!");																																						/// TODO wird auch gesendet, wenn Ansicht geändert wird!

	frameNumber++;
}

/**																																					TODO New version!
 * @brief Write visual objects to the working memory.
 * First delete all the old ones!
 * @param objects Write this visual objects
 */
void StereoDetector::WriteToWM(Z::Array<VisionData::VisualObjectPtr> objects)
{
	VisionData::VisualObjectPtr obj;
	for(unsigned i=0; i< objects.Size(); i++)
	{
		obj = new VisionData::VisualObject;
		obj = objects[i];
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
		addToWorkingMemory(objectID, obj);

		cvWaitKey(200);			/// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.
		log("New visual object to WM: %s", objectID.c_str());
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
 *   F4 ... Print runtime statistics
 *   F5 ... Refresh display
 *   F9 ... Process single shot \n
 *   F10 .. Process pruned image (Format7 & ROI) \n
 *   F11 .. Process HR image (pruned & ROI)
 * 
 *   key:	1 ... Szene: Show all stereo features on virtual szene (on/off) \n
 *   key:	2 ... Stereo: Show all matched features (on/off) \n
 *   key:	3 ... Stereo: Show matched features (on/off) \n
 *   key:	4 ... Stereo: Show single stereo match (on/off) \n
 *   key:	5 ... Mono: Show all edge segments (on/off) \n
 *   key:	6 ... Mono: Show detected Gestalts (on/off) \n
 *   key:	7 ... Mono: Show the masked Gestalts (on/off) \n
 *   key: 8 ... Mono: Show single Gestalts \n
 *   key: 9 ... Mono: Show ROI windows \n
 *   key:	+ ... Increase degree of detail \n
 *   key:	- ... Decrease degree of detail \n
 *   key:	. ... Increase ID of single showed Gestalt \n
 *   key:	, ... Decrease ID of single showed Gestalt \n
 *   key:	x ... Stop single-shot processing mode.\n\n
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
 *   key:	s ... Show A-JUNCTIONS \n
 *   key:	d ... Show CONVEX ARC-GROUPS \n
 *   key:	f ... Show ELLIPSES \n
 *   key: g ... Show E-JUNCTIONS \n
 *   key:	h ... Show EXT-ELLIPSES: not yet implemented \n
 *   key:	j ... Show CYLINDERS \n
 *   key:	k ... Show CONES \n
 *   key:	l ... Show SPHERES \n
 *   
 *   key: y ... Show REASONER results \n
 *   
 *   key: < ... Switch to older frames \n
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
						"    F3 ... Print information (show single gestalts (5) = on)\n"
						"    F4 ... Print runtime statistics\n"
						"    F5 ... Refresh display\n\n"
						
						"    F9 ... Process single shot\n"
						"    F10 .. Process single shot with region of interest (ROI)\n"
						"    F11 .. Process single shot with HR and ROI\n"

						"    1 ... Szene: Show all stereo features on virtual szene\n"
						"    2 ... Stereo: Show all matched features \n"
						"    3 ... Stereo: Show matched features \n"
						"    4 ... Stereo: Show single stereo match \n"
						"    5 ... Mono: Show all edge segments \n"
						"    6 ... Mono: Show detected Gestalts \n"
						"    7 ... Mono: Show the masked Gestalts \n"
						"    8 ... Mono: Show single Gestalts \n"
						"    9 ... Mono: Show ROI windows\n"
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
						"    s ... Show A-JUNCTIONS\n"
						"    d ... Show CONVEX ARC-GROUPS\n"
						"    f ... Show ELLIPSES\n"
						"    g ... Show E-JUNCTIONS\n"
						"    h ... Show EXT-ELLIPSES: not yet implemented\n"
						"    j ... Show CYLINDERS\n"
						"    k ... Show CONES\n"
						"    l ... Show SPHERES\n\n"

						"    y ... Show REASONER results\n\n"

						"    < ... Switch to older frames\n");

	if (key == 65471 || key == 1114047)	// F2
	{
		const char* text = score->GetGestaltListInfo();
		printf("StereoDetector: got text!\n");
		log("\n%s\n", text);
	}

	if (key == 65472 || key == 1114048)	// F3
	{
		if(showSingleGestalt && showID != -1 && (mouseSide == 0 || mouseSide == 1)) 
		{
			const char* text = (score->GetMonoCore(mouseSide))->GetInfo(showType, showID);
			log("Gestalt infos:\n%s\n", text);
		}
	}

	if (key == 65473 || key == 1114049)	// F4
	{
		score->PrintVCoreStatistics();
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
				log("Szene: Show ALL stereo features at virtual scene: OFF");
			}
			else
			{ 
				showAllStereo = true;
				log("Szene: Show ALL stereo features at virtual scene: ON");
			}
			WriteVisualObjects();
			break;
		case '2':
			if(showAllStereoMatched) 
			{
				showAllStereoMatched = false;
				log("Stereo: Draw ALL MATCHED stereo features: OFF");
			}
			else
			{ 
				showAllStereoMatched = true;
				log("Stereo: Draw ALL MATCHED stereo features: ON");
			}
			if(showImages) ShowImages(true);
			break;
		case '3':
			if(showStereoMatched) 
			{
				showStereoMatched = false;
				log("Stereo: Draw MATCHED stereo features: OFF");
			}
			else
			{ 
				showStereoMatched = true;
				log("Stereo: Draw MATCHED stereo features: ON");
			}
			if(showImages) ShowImages(true);
			break;
		case '4':
			if(showSingleStereo)
			{
				showSingleStereo = false;
				log("Stereo: Show single stereo match: OFF");
			}
			else
			{
				showSingleStereo = true;
				log("Stereo: Show single stereo match: ON");
			}
			ShowImages(true);
			break;
		case '5':
			if(showSegments)
			{
				showSegments = false;
				log("Mono: Show edge segments: OFF");
			}
			else
			{
				showSegments = true;
				log("Mono: Show edge segments: ON");
			}
			ShowImages(true);
			break;
		case '6':
			if(showDetected)
			{
				showDetected = false;
				log("Mono: Draw all detected features: OFF");
			}
			else
			{
				showDetected = true;
				log("Mono: Draw all detected features: ON");
			}
			ShowImages(true);
			break;
		case '7':
			if(showMasked)
			{
				showMasked = false;
				log("Mono: Draw all MASKED features: OFF");
			}
			else
			{
				showMasked = true;
				log("Mono: Draw all MASKED features: ON");
			}
			ShowImages(true);
			break;
		case '8':
			if(showSingleGestalt)
			{
				showSingleGestalt = false;
				log("Mono: Show single Gestalts: OFF");
			}
			else
			{
				showSingleGestalt = true;
				log("Mono: Show single Gestalts: ON");
			}
			ShowImages(true);
			break;
		case '9':
			if(showROIs)
			{
				showROIs = false;
				log("Show ROIs: OFF");
			}
			else
			{
				showROIs = true;
				log("Show ROIs: ON");
			}
			ShowImages(true);
			break;
		case '+':
			if(detail < 15)
			{
				detail++;
				log("Increased degree of display detail to: %u", detail);
				ShowImages(true);
			}
			break;
		case '-':
			if(detail > 0)
			{
				detail--;
				log("Decreased degree of display detail to: %u", detail);
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
			log("Show SEGMENTS");
			showType = Z::Gestalt::SEGMENT;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'w':
			log("Show LINES");
			showType = Z::Gestalt::LINE;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'e':
			log("Show COLLINEARITIES");
			showType = Z::Gestalt::COLLINEARITY;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'r':
			log("Show L-JUNCTIONS");
			showType = Z::Gestalt::L_JUNCTION;
			showStereoType = Z::StereoBase::STEREO_LJUNCTION;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 't':
			log("Show CLOSURES");
			showType = Z::Gestalt::CLOSURE;
			showStereoType = Z::StereoBase::STEREO_CLOSURE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'z':
			log("Show RECTANGLES");
			showType = Z::Gestalt::RECTANGLE;
			showStereoType = Z::StereoBase::STEREO_RECTANGLE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'u':
			log("Show FLAPS");
			showType = Z::Gestalt::FLAP;
			showStereoType = Z::StereoBase::STEREO_FLAP;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'i':
			log("Show FLAPS_ARI");
			showType = Z::Gestalt::FLAP_ARI;
			showStereoType = Z::StereoBase::STEREO_FLAP_ARI;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'o':
			log("Show CUBES");
			showType = Z::Gestalt::CUBE;
			showStereoType = Z::StereoBase::STEREO_CUBE;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'p':
			log("Show CORNERS");
			showType = Z::Gestalt::CORNER;
			showStereoType = Z::StereoBase::STEREO_CORNER;
			ShowImages(true);
			WriteVisualObjects();
			break;

		case 'a':
			log("Show ARCS");
			showType = Z::Gestalt::ARC;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 's':
			log("Show A-JUNCTIONS");
			showType = Z::Gestalt::A_JUNCTION;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'd':
			log("Show CONVEX ARC-GROUPS");
			showType = Z::Gestalt::CONVEX_ARC_GROUP;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'f':
			log("Show ELLIPSES");
			showType = Z::Gestalt::ELLIPSE;
			showStereoType = Z::StereoBase::STEREO_ELLIPSE;
			ShowImages(true);
			WriteVisualObjects();
			break;
			
		case 'g':
			log("Show E-JUNCTIONS");
			showType = Z::Gestalt::E_JUNCTION;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'h':
			log("Show EXT-ELLIPSES: not yet implemented");
// 			showType = Z::Gestalt::EXTELLIPSE;
// 			showStereoType = Z::StereoBase::UNDEF;
// 			overlays = 204;
// 			ShowImages(true);
// 			WriteVisualObjects();
			break;
		case 'j':
			log("Show CYLINDERS");
			showType = Z::Gestalt::CYLINDER;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'k':
			log("Show CONES");
			showType = Z::Gestalt::CONE;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;
		case 'l':
			log("Show CIRCLES");
			showType = Z::Gestalt::CIRCLE;
			showStereoType = Z::StereoBase::UNDEF;
			ShowImages(true);
			WriteVisualObjects();
			break;

		case 'y':
			if(showReasoner)
			{
				showReasoner = false;
				log("Show REASONER results: OFF");
			}
			else
			{
				showReasoner = true;
				log("Show REASONER results:: ON");
			}
			ShowImages(true);
			break;

		case 'c':
			if(showReasonerUnprojected)
			{
				showReasonerUnprojected = false;
				log("Show unprojected REASONER results: OFF");
			}
			else
			{
				showReasonerUnprojected = true;
				log("Show unprojected REASONER results:: ON");
			}
			ShowImages(true);
			break;

		case '<':
			showFrame--;
			if(showFrame < 0) showFrame = 2;
			log("Switch to older frames: %u", showFrame);
			score = p_score[showFrame];
			ShowImages(true);
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
	debug("show feature with id=%u at position (%u/%u).", showID, mouseX, mouseY);
	ShowImages(true);
}

/**																																							/// TODO Kann man löschen!!!
 * @brief Read the SOIs from the working memory and display it.
 */
// void StereoDetector::ReadSOIs()
// {
// 	// read SOIs from working memory
// 	std::vector <VisionData::SOIPtr> sois;
// 	getMemoryEntries<VisionData::SOI>(sois);
// 
// 	printf("Found %u SOIs in working memory:\n", sois.size());
// 
// 	for(unsigned i=0; i<sois.size(); i++)
// 	{
// 		printf("  Sphere: pose: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingSphere.pos.x, sois[i]->boundingSphere.pos.y, sois[i]->boundingSphere.pos.z);
// 		printf("          radius: %4.3f\n", sois[i]->boundingSphere.rad);
// 		printf("     Box: pose: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingBox.pos.x, sois[i]->boundingBox.pos.y, sois[i]->boundingBox.pos.z);
// 		printf("          radius: %4.3f - %4.3f - %4.3f\n", sois[i]->boundingBox.size.x, sois[i]->boundingBox.size.y, sois[i]->boundingBox.size.z);
// 	}
// }


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
	getRectImage(0, 640, image);
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






