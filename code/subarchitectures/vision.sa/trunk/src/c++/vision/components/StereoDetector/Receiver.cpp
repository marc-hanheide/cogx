/**
 * @file Mouse.cpp
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Functions for mouse handling.
 */

#include "StereoDetector.h"
// #include "StereoBase.h"
#include <VisionData.hpp>

namespace cast
{
  
/**
 *  @brief Receive a changed detection command, written to the working memory
 *  @param wmc Working memory change.
 */
void StereoDetector::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
  if(single) return;    // return if single shot mode is on

  VisionData::StereoDetectionCommandPtr detect_cmd = getMemoryEntry<VisionData::StereoDetectionCommand>(_wmc.address);
  
  log("received detection command ...");
  switch(detect_cmd->cmd){
    case VisionData::SDSTART:
      if(cmd_detect){
  log("stereo detection is already started");
      }else{
  log("starting stereo detection");
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);    /// TODO Hier wird videoServer->startReceiveImages aufgerufen
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
    case VisionData::SDSINGLEHR:                                                    /// TODO Hier wird videoServer->getImage aufgerufen
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
 *  @brief Receive a new created SOI.
 *  @param wmc Working memory change.
 */
void StereoDetector::receiveSOI(const cdl::WorkingMemoryChange & _wmc)
{
//  SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
//  Video::Image image;
//  getRectImage(0, image); // 0 = left image
//  ROIPtr roiPtr = projectSOI(image.camPars, *soi);
//  roi.width = roiPtr->rect.width;
//  roi.height = roiPtr->rect.height;
//  roi.x = roiPtr->rect.pos.x - roi.width/2;
//  roi.y = roiPtr->rect.pos.y - roi.height/2;
//  validROI = true;
// //   log("calculated roi: x=%i, y=%i, width=%i, height=%i", roi.x, roi.y, roi.width, roi.height);
}

/**
 *  @brief Receive a updated SOI.
 *  @param wmc Working memory change.
 */
void StereoDetector::updatedSOI(const cdl::WorkingMemoryChange & _wmc)
{
//  SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
//  Video::Image image;
//  getRectImage(0, image); // 0 = left image
//  ROIPtr roiPtr = projectSOI(image.camPars, *soi);
//  roi.width = roiPtr->rect.width;
//  roi.height = roiPtr->rect.height;
//  roi.x = roiPtr->rect.pos.x - roi.width/2;
//  roi.y = roiPtr->rect.pos.y - roi.height/2;  
//  validROI = true;
// //   log("updated roi: x=%i, y=%i, width=%i, height=%i", roi.x, roi.y, roi.width, roi.height);
}

/**
 *  @brief Receive a deleted SOI.
 *  @param wmc Working memory change.
 */
void StereoDetector::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{
//  printf("SOI deleted!!!\n");
}

/**
 *  @brief Receive a new created ROI.
 *  @param wmc Working memory change.
 */
void StereoDetector::receiveROI(const cdl::WorkingMemoryChange & _wmc)
{
  try
  {
    VisionData::ROIPtr roiPtr = getMemoryEntry<VisionData::ROI>(_wmc.address);
    ROIData roiData;
    roiData.rect.width = roiPtr->rect.width;
    roiData.rect.height = roiPtr->rect.height;
    roiData.rect.x = roiPtr->rect.pos.x - roiData.rect.width/2;
    roiData.rect.y = roiPtr->rect.pos.y - roiData.rect.height/2;  
    PlausibleROI(&roiData);
    rcvROIs[_wmc.address.id] = roiData;
//    log("received roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect.x, roiData.rect.y, roiData.rect.width, roiData.rect.height, _wmc.address.id.c_str());
}
  catch (DoesNotExistOnWMException e)
  {
//    log("ROI with id %s was removed from WM before it could be processed", _wmc.address.id.c_str());
//    log("  => receiving was not possible.");
  }
  catch (std::exception e)
  {
    log("unknown exception during reading from WM.");
  }
}

/**
 *  @brief Receive a updated ROI.
 *  @param wmc Working memory change.
 */
void StereoDetector::updatedROI(const cdl::WorkingMemoryChange & _wmc)
{
  try
  {
    VisionData::ROIPtr roiPtr = getMemoryEntry<VisionData::ROI>(_wmc.address);
    ROIData roiData;
    roiData.rect.width = roiPtr->rect.width;
    roiData.rect.height = roiPtr->rect.height;
    roiData.rect.x = roiPtr->rect.pos.x - roiData.rect.width/2;
    roiData.rect.y = roiPtr->rect.pos.y - roiData.rect.height/2;  
    PlausibleROI(&roiData);
    rcvROIs[_wmc.address.id] = roiData;
  //    log("updated roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect.x, roiData.rect.y, roiData.rect.width, roiData.rect.height, _wmc.address.id.c_str());
}
  catch (DoesNotExistOnWMException e)
  {
//    log("ROI with id %s was removed from WM before it could be processed", _wmc.address.id.c_str());
//    log("  => Update not possible");
  }
  catch (std::exception e)
  {
    log("unknown exception during reading from WM.");
  }
}

/**
 *  @brief Receive a deleted ROI.
 *  @param wmc Working memory change.
 */
void StereoDetector::deletedROI(const cdl::WorkingMemoryChange & _wmc)
{
  try
  {
    rcvROIs.erase(_wmc.address.id);
//    log("deleted roi @ %s", _wmc.address.id.c_str());
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
 *  @brief Receive a new created ProtoObject.
 *  @param wmc Working memory change.
 */
void StereoDetector::receiveProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::ProtoObjectPtr poPtr = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);

  Video::Image img;
  img = poPtr->image;
  poImg = convertImageToIpl(img);

//  Video::ByteSeq data;
//  data = poPtr->mask.data;

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

printf("[StereoDetector::receiveProtoObject] Error: Rewrite function for new opencv2!\n");
/* HACK ARI: Rewrite for new opencv-version
  poPatchImage = cvCreateImage(cvSize(poPtr->mask.width*2, poPtr->mask.height*2), IPL_DEPTH_8U, 3);
  cvSetImageROI(poPatchImage, cvRect( 0, 0, poPtr->mask.width, poPtr->mask.height) );
  cvCopyImage(poImg, poPatchImage);
  cvSetImageROI(poPatchImage, cvRect( poPtr->mask.width, 0, poPtr->mask.width, poPtr->mask.height) );
  cvCvtColor(iplMask, poPatchImage, CV_GRAY2RGB);
//  cvCopyImage(iplMask, poPatchImage);
  cvSetImageROI(poPatchImage, cvRect( 0, poPtr->mask.height, poPtr->mask.width, poPtr->mask.height) );
  cvCopyImage(poImg, poPatchImage);
  cvSetImageROI(poPatchImage, cvRect( poPtr->mask.width, poPtr->mask.height, poPtr->mask.width, poPtr->mask.height) );
  cvCopyImage(poImg, poPatchImage);
  cvResetImageROI(poPatchImage);
*/

//  cvReleaseImage(&poPatchImage);
//  cvReleaseImage(&iplMask);
//  cvCvtColor(segPatch, tetraPatch, CV_GRAY2RGB);
//  cvCvtColor(costPatch, tetraPatch, CV_GRAY2RGB);
//  cvCvtColor(bgCostPatch, tetraPatch, CV_GRAY2RGB);
//  delete []poMask;

  printf("  Proto object received:\n    mask w-h: %u - %u\n", poPtr->mask.width, poPtr->mask.height);
//  printf("    data.size: %u\n", data.size());
}

/**
 *  @brief Receive a new created convex hull. Copy it to a visual object.
 *  @param wmc Working memory change.
 */
void StereoDetector::receiveConvexHull(const cdl::WorkingMemoryChange & _wmc)
{
  log("Process new convex hull");
  VisionData::ConvexHullPtr chPtr = getMemoryEntry<VisionData::ConvexHull>(_wmc.address);

  // Create a visual object for the plane as mesh and recalculate point sequence 
  // of convex hull in respect to the center.
  VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
  obj->pose = chPtr->center;
  VEC::Vector3 p;
  std::vector<VEC::Vector3> points;
  for(unsigned i=0; i< chPtr->PointsSeq.size(); i++)
  {
    p.x = chPtr->PointsSeq[i].x - obj->pose.pos.x;  // shift the plane in respect to the pose
    p.y = chPtr->PointsSeq[i].y - obj->pose.pos.y;
    p.z = chPtr->PointsSeq[i].z - obj->pose.pos.z;
    points.push_back(p);
  }
  
  VEC::Vector3 pos;       // center position of the plane
  pos.x = obj->pose.pos.x;
  pos.y = obj->pose.pos.y;
  pos.z = obj->pose.pos.z;
  double radius = chPtr->radius;
  
  if(activeReasonerPlane)
  {
//     reasoner->ProcessConvexHull(pos, radius, points);
//     if(reasoner->GetPlane(obj))
//     {
//       planeID = newDataID();
//       addToWorkingMemory(planeID, obj);
//       log("Wrote new dominant plane as visual object to working memory: %s", planeID.c_str());
//     }
  }

//  printf("  center position: %4.2f / %4.2f / %4.2f\n", chPtr->center.pos.x, chPtr->center.pos.y, chPtr->center.pos.z);
//  printf("  center rotation matrix:\n    %4.2f / %4.2f / %4.2f\n    %4.2f / %4.2f / %4.2f\n    %4.2f / %4.2f / %4.2f\n", 
//         chPtr->center.rot.m00, chPtr->center.rot.m01, chPtr->center.rot.m02, 
//         chPtr->center.rot.m10, chPtr->center.rot.m11, chPtr->center.rot.m12, 
//         chPtr->center.rot.m20, chPtr->center.rot.m21, chPtr->center.rot.m22);
//  printf("  radius: %4.2f\n", chPtr->radius);
//  printf("  plane: %4.2f / %4.2f / %4.2f / %4.2f\n", chPtr->plane.a, chPtr->plane.b, chPtr->plane.c, chPtr->plane.d);
//  printf("  objects on plane: %u\n", chPtr->Objects.size());
}

/**
 * @brief Receive camera parameters from the camera mount
 */
void StereoDetector::receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc)
{
printf("      StereoDetector::receiveCameraParameters: receive something!!!\n");
  Video::CameraParametersWrapperPtr newCam = getMemoryEntry<Video::CameraParametersWrapper>(_wmc.address);
  // find the camera paramters that need updating and update pose and time stamp
  // Note that we don't change any other (instrinsic) parameters yet as we
  // assume these fixed. At a later stage (with zoom cameras) also the intrinsic
  // parameters might change.
  // Note: if we don't find a camera with matching id in our list, no problem,
  // then these parameters were meant for another video server.
  for(size_t i = 0; i < camPars.size(); i++)
  {
printf("StereoDetector::receiveCameraParameters: newCam->cam.id: %u and camPars[%u]=%u \n", newCam->cam.id, i, camPars[i].id);
    if(newCam->cam.id == camPars[i].id)
    {
printf("StereoDetector::receiveCameraParameters: got new camPars: %4.2f\n", camPars[i].pose.pos.x);
      camPars[i].pose = newCam->cam.pose;
      camPars[i].time = newCam->cam.time;
      break;
    }
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

  lockComponent();
  image_l = images[0];      /// TODO sollte man hier gleich auf iplImage konvertieren?
  image_r = images[1];
  haveImage = true;
  if (cmd_detect) processImage();
  unlockComponent();
}

}
