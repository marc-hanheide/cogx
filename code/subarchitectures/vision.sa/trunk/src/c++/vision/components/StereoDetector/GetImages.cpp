/**
 * @file Mouse.cpp
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Functions to get images for the stereo detector.
 */

#include "StereoDetector.h"

namespace cast
{
	

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
      haveHRImage = true;             /// TODO getHRImages liefert nicht richtig true/false zurück!
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
  else                                              /// TODO noch nicht implementiert.
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
    if(mode == 1 && mode7 == 0)   // change only when not already in mode7=1
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

  if(!haveImage)  return false;

  // calculate scale between stereo and video server (get stereo image => TODO this needs time (do it once))
  Video::Image image;
  getRectImage(0, 640, image);
  roiData->roiScale = image_l.width / image.width;

  int leftShift = image_l.width/6;    // TODO TODO TODO Besser lösen

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