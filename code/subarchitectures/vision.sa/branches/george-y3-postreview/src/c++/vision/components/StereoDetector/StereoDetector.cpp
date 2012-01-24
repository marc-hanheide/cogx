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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "StereoDetector.h"
#include "StereoBase.h"
#include "Draw.hh"

#include "Mouse.cpp"
#include "Configuration.cpp"
#include "Receiver.cpp"
#include "GetImages.cpp"


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
 *  @brief Process stereo image pair at stereo core.
 *  Use normal resolution of images (640x480).
 */
void StereoDetector::processImage()
{
  log("Process new images with runtime: %ums", runtime);
//  score->ClearResults();
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
  score = p_score[nr_p_score];      // copy processed score to main score for displaying
  log("Copied processed score to main.");

  if(activeReasoner)  // do the reasoner things!
  {
//     log("Start reasoner.");
//     reasoner->Process(score);
//     log("Reasoner finished.");
//    if(reasoner->Process(score))                            /// TODO delete
//    {
//      log("Get results.");
//      Z::Array<VisionData::VisualObjectPtr> objects;
//      reasoner->GetResults(objects);
// 
//      log("Write results to working memory!");
//      WriteToWM(objects);
//    }
//    log("End reasoner.");
  }
//  else WriteVisualObjects();
  
  
  // Create object representations from new objects!
  try 
  {
    objRep->Process(score);
    DrawIntoTomGine();
  }
  catch (exception &e)
  {
    log("StereoDetector::processImage: Unknown exception during creation of object representations.\n");
    cout << e.what() << endl;
  }

  
  // Write visual objects to working memory and show images!
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

      processPrunedHRImage(roi640.x, roi640.y, 2);    /// TODO Now process the two images
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
      processPrunedHRImage(roi640.x, roi640.y, 2);            /// TODO Add scale (2:1)
printf("ProcessPrunedHRImages 2\n");
    }
    else log("ROI not valid.");
  }

printf("ProcessPrunedHRImages 3\n");
  WriteVisualObjects();
printf("ProcessPrunedHRImages 4\n");
//  ShowImages(false);
printf("ProcessPrunedHRImages 5\n");

  log("processing of pruned stereo images ended.");
}

/**
 *  @brief Process a pruned stereo image pair at stereo core.
 *  @param oX Offset of x-coordinate
 *  @param oY Offset of y-coordinate
 *  @param sc Scale between original and pruned image.
 */
void StereoDetector::processPrunedHRImage(int oX, int oY, int sc)
{
  log("Process pruned image with runtime: %ums", runtime);

  score->ClearResults();
  try 
  {
    double ca = 0.4;    // Canny alpha
    double co = 0.001;  // Canny omega
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
 * @brief Draw into tomGine render machine via the wraper.
 */
void StereoDetector::DrawIntoTomGine()
{
  // clear the render display
  tgRenderer->Clear();

  /// get object graph model
  std::vector< std::vector<cv::Point3d> > first;
  std::vector< std::vector<cv::Point3d> > second;
  std::vector< std::vector<double> > probability;
  std::vector< std::vector<std::string> > link;
  std::vector< std::vector<std::string> > node_0;
  std::vector< std::vector<std::string> > node_1;
  objRep->GetObjectGraphModel(first, second, probability, link, node_0, node_1);

  /// color generation for models
  for(unsigned i=0; i<first.size(); i++)
  {
    uint r = std::rand()%255;
    uint g = std::rand()%255;
    uint b = std::rand()%255;
//     tgRenderer->AddGraphModel(first[i], second[i], probability[i], link[i], node_0[i], node_1[i], r, g, b);
  }
  
  /// get point cloud from point cloud server
  int pointCloudWidth = 320;                                /// TODO get from cast-file!
  int pointCloudHeight = pointCloudWidth *3/4;
  points.resize(0);
  getPoints(true, pointCloudWidth, points);
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  Points2Cloud(points, cloud, colCloud);

//   tgRenderer->SetPointCloud(cloud, colCloud);
  printf("StereoDetector::DrawIntoTomGine: SetPointCloud is antiquated: reimplement!\n");
  
  /// get a registered point cloud, related to the left stereo image.
//   points.resize(0);
//   getPoints(false, pointCloudWidth, points);
// 
//   cogx::Math::Pose3 lokal_kinect_pose;  /// TODO transform only the points of the kinect to the stereo system!
//   transformInverse(camPars[0].pose, camPars[2].pose, lokal_kinect_pose);   // left stereo and kinect camera pose
//   
//   cv::Mat_<cv::Point3f> cloud2;
//   cv::Mat_<cv::Point3f> colCloud2;
//   Points2Cloud(points, cloud2, colCloud2);
// 
//   int end = pointCloudWidth*pointCloudWidth*3/4;  // we transform only the kinect cloud to the coordinate system of the left stereo camera
//   for(unsigned col=0; col<end; col++)
//   {
//     PointCloud::SurfacePoint p;
//     p.p = vector3(cloud2.at<cv::Point3f>(0, col).x,
//                   cloud2.at<cv::Point3f>(0, col).y,
//                   cloud2.at<cv::Point3f>(0, col).z);
//     p.p = transform(lokal_kinect_pose, p.p);
//     cloud2.at<cv::Point3f>(0, col).x = p.p.x;
//     cloud2.at<cv::Point3f>(0, col).y = p.p.y;
//     cloud2.at<cv::Point3f>(0, col).z = p.p.z;
//   }
//   tgRenderer->SetPointCloud(cloud2, colCloud2);
//   
//   
//   /// reproject the kinect point (and stereo point) cloud into a openCV image
//   cast::StereoCamera *sc = new cast::StereoCamera();
//   sc->ReadSVSCalib(stereoconfig);
//   sc->SetInputImageSize(cvSize(pointCloudWidth, pointCloudHeight));  // now set the input image size
//   sc->SetupImageRectification();
// 
//   int i_u, i_v;
//   double u, v;  
//   cv::Mat bImage = cv::Mat_<cv::Vec3b>(pointCloudHeight, pointCloudWidth);
//   for(unsigned row=0; row<pointCloudHeight; row++)
//     for(unsigned col=0; col<pointCloudWidth; col++)
//     { 
//       cv::Point3f point = cloud2.at<cv::Point3f>(0, row*pointCloudWidth+col);
//       sc->ProjectPoint((double)point.x, (double)point.y, (double)point.z, u, v, cogx::Math::LEFT, pointCloudWidth);
//       i_u = (int)u;
//       i_v = (int)v;
//       if(i_u >=0 && i_u < pointCloudWidth && i_v >=0 && i_v < pointCloudHeight)
//       {
//         cv::Vec3b &color = bImage.at<cv::Vec3b>(i_v, i_u);
//         color[0] = (uchar) colCloud2.at<cv::Point3f>(0, row*pointCloudWidth+col).z;
//         color[1] = (uchar) colCloud2.at<cv::Point3f>(0, row*pointCloudWidth+col).y;
//         color[2] = (uchar) colCloud2.at<cv::Point3f>(0, row*pointCloudWidth+col).x;
//       }
//     }
//   IplImage newImg = bImage;
//   cvShowImage("Back projected point cloud", &newImg);
  
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
  getRectImage(0, 640, image);            // 0 = left image / 640 = width
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

      score->DrawROI(0, roi, 1, rImg, iplImage_r);                          /// TODO Wieso braucht man hier rImg???
      if(!havePrunedImage)                                                  /// TODO ist das noch richtig?
      {
  score->DrawROI(0, roi, roiScale, iplImage_l, iplImage_r);
  if(roi640valid)
  {
    score->DrawPrunedROI(0, roi640.x/2, roi640.y/2, iplImage_l, iplImage_r);    // TODO Draw 640x480 ROI for pruned image => /2 kann nicht stimmen
    score->DrawPrunedROI(1, roi640.x/2, roi640.y/2, iplImage_l, iplImage_r);    // TODO Draw 640x480 ROI for pruned image
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
//  cvShowImage("Pruned left", iplImage_l_pr);
//  cvShowImage("Pruned right", iplImage_r_pr);
  }

  cvWaitKey(50);  ///< TODO TODO wait key to allow openCV to show the images on the window.
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
//     if(showStereoType != Z::StereoBase::UNDEF)
//     {
//       WriteToWM(showStereoType);
//      WriteToWM(Z::StereoBase::STEREO_RECTANGLE);                   // TODO hier händisch Rectangles eingefügt!
//     }
    
    if(write_stereo_lines) WriteToWM(Z::StereoBase::STEREO_LINE);
//    if(write_stereo_ljcts) WriteToWM(Z::StereoBase::STEREO_LJUNCTION);      /// TODO ausgeschaltet
    if(write_stereo_ellipses) WriteToWM(Z::StereoBase::STEREO_ELLIPSE);
    if(write_stereo_closures) WriteToWM(Z::StereoBase::STEREO_CLOSURE);
    if(write_stereo_rectangles) WriteToWM(Z::StereoBase::STEREO_RECTANGLE);
    if(write_stereo_flaps) WriteToWM(Z::StereoBase::STEREO_FLAP_ARI);
    if(write_stereo_corners) WriteToWM(Z::StereoBase::STEREO_CORNER);
    
  }
}

/**
 * @brief Write visual objects of a specific type to the working memory.
 * @param type Type of StereoBase feature to write.
 */
void StereoDetector::WriteToWM(Z::StereoBase::Type type)
{
  static unsigned frameNumber = 0;
//  static unsigned numStereoObjects = 0;
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

      addToWorkingMemory(objectID, obj);
      cvWaitKey(200);       /// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.
      log("Add new visual object to working memory: %s", objectID.c_str());
    }
  }
  
  // Send newFrame command for Reasoner component => TODO delete later
//  VisionData::SDReasonerCommandPtr newFrame = new VisionData::SDReasonerCommand;
//  newFrame->cmd = VisionData::NEWFRAME;
//  addToWorkingMemory(newDataID(), newFrame);
//  debug("NewFrame command sent!");              /// TODO wird auch gesendet, wenn Ansicht geändert wird!

  frameNumber++;
}

/**
 * @brief Write visual objects to the working memory.
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

    cvWaitKey(200);     /// TODO HACK TODO HACK TODO HACK TODO HACK => Warten, damit nicht WM zu schnell beschrieben wird.
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
 *   key: 1 ... Szene: Show all stereo features on virtual szene (on/off) \n
 *   key: 2 ... Stereo: Show all matched features (on/off) \n
 *   key: 3 ... Stereo: Show matched features (on/off) \n
 *   key: 4 ... Stereo: Show single stereo match (on/off) \n
 *   key: 5 ... Mono: Show all edge segments (on/off) \n
 *   key: 6 ... Mono: Show detected Gestalts (on/off) \n
 *   key: 7 ... Mono: Show the masked Gestalts (on/off) \n
 *   key: 8 ... Mono: Show single Gestalts \n
 *   key: 9 ... Mono: Show ROI windows \n
 *   key: + ... Increase degree of detail \n
 *   key: - ... Decrease degree of detail \n
 *   key: . ... Increase ID of single showed Gestalt \n
 *   key: , ... Decrease ID of single showed Gestalt \n
 *   key: x ... Stop single-shot processing mode.\n\n
 *
 *   key: q ... Show SEGMENTS \n
 *   key: w ... Show LINES \n
 *   key: e ... Show COLLINEARITIES \n
 *   key: r ... Show L-JUNCTIONS \n
 *   key: t ... Show CLOSURES \n
 *   key: z ... Show RECTANGLES \n
 *   key: u ... Show FLAPS \n
 *   key: i ... Show FLAPS_ARI \n
 *   key: o ... Show CUBES \n
 *
 *   key: a ... Show ARCS \n
 *   key: s ... Show A-JUNCTIONS \n
 *   key: d ... Show CONVEX ARC-GROUPS \n
 *   key: f ... Show ELLIPSES \n
 *   key: g ... Show E-JUNCTIONS \n
 *   key: h ... Show EXT-ELLIPSES: not yet implemented \n
 *   key: j ... Show CYLINDERS \n
 *   key: k ... Show CONES \n
 *   key: l ... Show SPHERES \n
 *   
 *   key: y ... Show REASONER results \n
 *   
 *   key: < ... Switch to older frames \n
 */
void StereoDetector::SingleShotMode()
{
//   if(showKinectImage)
//   {
//     openNI->NextFrame();
//     cv::Mat rgbImg, depImg;
//     if(openNI->GetImages(rgbImg, depImg))
//     {
//       cv::imshow("Kinect image", rgbImg);
//       cv::imshow("Kinect depth image", depImg);
//       DrawIntoTomGine();
//     }
//   }
  
// static bool printit = true;
// if(printit)
//   for(unsigned co=640*14; co<640*15; co++)
//   {
//     printf("%u ", kinectImage->imageData[co]);
//   }
// printit = false;

  if(mouseEvent) MouseEvent();
  mouseEvent = false;

  int key = 0;
  key = cvWaitKey(10);      /// TODO Kurzes wait, damit eingelesen werden kann!

//  if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %c, %i", (char) key, key);
  if (key == 65470 || key == 1114046) // F1
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

  if (key == 65471 || key == 1114047) // F2
  {
    const char* text = score->GetGestaltListInfo();
    log("\n%s\n", text);
  }

  if (key == 65472 || key == 1114048) // F3
  {
    if(showSingleGestalt && showID != -1 && (mouseSide == 0 || mouseSide == 1)) 
    {
      const char* text = (score->GetMonoCore(mouseSide))->GetInfo(showType, showID);
      log("Gestalt infos:\n%s\n", text);
    }
  }

  if (key == 65473 || key == 1114049) // F4
  {
    score->PrintVCoreStatistics();
  }

  if (key == 65474 || key == 1114050) // F5
  {
    if(!haveImage) GetImages();
    if(showImages) ShowImages(true);
  }


  if (key == 65478 || key == 1114054) // F9
  {
    log("process stereo images (single shot).");
    lockComponent();
    processImage();
    unlockComponent();
  }
  if (key == 65479 || key == 1114055) // F10
  {
    log("process pruned stereo images (single shot with ROI).");
    lockComponent();
    ProcessPrunedHRImages();
    unlockComponent();
  }
  if (key == 65480 || key == 1114056) // F11
  {
    log("process HR stereo images (single shot with ROI).");
    lockComponent();
    ProcessHRImages();
    unlockComponent();
  }

//  if (key != -1) log("StereoDetector::SingleShotMode: Pressed key: %i", key);
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
      showStereoType = Z::StereoBase::STEREO_LINE;
      ShowImages(true);
      write_stereo_lines = !write_stereo_lines;
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
      write_stereo_ljcts = !write_stereo_ljcts;
      WriteVisualObjects();
      break;
    case 't':
      log("Show CLOSURES");
      showType = Z::Gestalt::CLOSURE;
      showStereoType = Z::StereoBase::STEREO_CLOSURE;
      ShowImages(true);
      write_stereo_closures = !write_stereo_closures;
      WriteVisualObjects();
      break;
    case 'z':
      log("Show RECTANGLES");
      showType = Z::Gestalt::RECTANGLE;
      showStereoType = Z::StereoBase::STEREO_RECTANGLE;
      ShowImages(true);
      write_stereo_rectangles = !write_stereo_rectangles;
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
      write_stereo_flaps = !write_stereo_flaps;
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
      write_stereo_corners = !write_stereo_corners;
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
      write_stereo_ellipses = !write_stereo_ellipses;
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
//      showType = Z::Gestalt::EXTELLIPSE;
//      showStereoType = Z::StereoBase::UNDEF;
//      overlays = 204;
//      ShowImages(true);
//      WriteVisualObjects();
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

}






