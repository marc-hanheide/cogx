/**
 * @file KinectCore.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Managment of processing kinect data.
 */

#include "KinectCore.h"
#include "Gestalt3D.h"

#include "KinectPatches.h"
#include "KinectSegments.h"
#include "KinectLines.h"
#include "KinectCollinearities.h"
#include "KinectClosures.h"
#include "KinectRectangles.h"

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);

/**
 * @brief Constructor of Kinect Core
 * @param stereocal_file Stereo calibration file
 */
KinectCore::KinectCore() throw(std::runtime_error)
{
  initialized = false;

  // init stereo camera calibration parameters
//   printf("KinectCore::KinectCore: Warning: Antiquated: Use initialisation with openCV xml-files!\n");
//   stereo_cam = new cast::StereoCamera();
//   if(!stereo_cam->ReadSVSCalib(stereocal_file)) throw (std::runtime_error("StereoCore::StereoCore: Cannot open calibration file for stereo camera."));

}


/**
 * @brief Destructor of Kinect Core
 */
KinectCore::~KinectCore()
{
  delete vcore;
  cvReleaseImage(&iplImg);
//   delete stereo_cam;
  for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    if(kinectPrinciples[i]->IsEnabled())
      delete kinectPrinciples[i];
}


/**
 * @brief Initialisation of the Kinect Gestalt clases.
 * Do this initialisation after receiving vcore, iplImg, points.
 */
void KinectCore::InitKinectPrinciples()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    kinectPrinciples[i] = 0;

  // stereoPrinciples all principles
  kinectPrinciples[KinectBase::KINECT_PATCHES] = new KinectPatches(this, vcore, iplImg, points);
  kinectPrinciples[KinectBase::KINECT_SEGMENTS] = new KinectSegments(this, vcore, iplImg, points);
  kinectPrinciples[KinectBase::KINECT_LINES] = new KinectLines(this, vcore, iplImg, points);
  kinectPrinciples[KinectBase::KINECT_COLLINEARITIES] = new KinectCollinearities(this, vcore, iplImg, points);
  kinectPrinciples[KinectBase::KINECT_CLOSURES] = new KinectClosures(this, vcore, iplImg, points);
  kinectPrinciples[KinectBase::KINECT_RECTANGLES] = new KinectRectangles(this, vcore, iplImg, points);
  
  // set principles enabled or disabled
  kinectPrinciples[KinectBase::KINECT_PATCHES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_SEGMENTS]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_LINES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_COLLINEARITIES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_CLOSURES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_RECTANGLES]->EnablePrinciple(true);
  
  initialized = true;
}


/**
 * @brief Create a new 3D Gestalt
 * @param g New 3D Gestalt
 */
void KinectCore::NewGestalt3D(Z::Gestalt3D *g)
{
  g->SetID(NumGestalts3D(g->GetType()));
  Gestalts3D(g->GetType()).PushBack(g);
}
  

/**
 * @brief Clear the vision cores and the used stereo Gestalts.
 */
void KinectCore::ClearResults()
{
  if(initialized)
  {
    vcore->ClearGestalts();
    for(int i = 0; i < KinectBase::MAX_TYPE; i++)
      if(kinectPrinciples[i]->IsEnabled())
        kinectPrinciples[i]->ClearResults();
    for(int i = 0; i < Gestalt3D::MAX_TYPE; i++)
      kinectGestalts[i].Clear();
  }
}


/**
 * @brief Draw results into TomGine render engine.
 * @param tgRenderer TomGine render engine.
 * @param type Type of Gestalt3D to draw.
 */
void KinectCore::DrawGestalts3D(TGThread::TomGineThread *tgRenderer, Gestalt3D::Type type)
{ 
  for(unsigned i=0; i < NumGestalts3D(type); i++)
    Gestalts3D(type, i)->DrawGestalt3D(tgRenderer);
}

/**
 * @brief Print results on console
 * @param type Type of Gestalt3D to draw.
 */
void KinectCore::PrintGestalts3D(Gestalt3D::Type type)
{ 
  for(unsigned i=0; i < NumGestalts3D(type); i++)
    Gestalts3D(type, i)->PrintGestalt3D();
}

/**
 * @brief Set the stereo images.
 * @param side Left or right image
 */
// void KinectCore::SetImages(IplImage *iIl, IplImage *iIr)
// {
//   img_l = iIl;
//   img_r = iIr;
// }


/**
 * @brief Set the active draw area to left or right stereo image.
 * @param side Left or right image of stereo.
 */
// void KinectCore::SetActiveDrawAreaSide(int side)
// {
//   if (side == LEFT) SetActiveDrawArea(img_l);
//   else if (side == RIGHT) SetActiveDrawArea(img_r);
// }


/**
 * @brief Process kinect data
 * @param _iplImage Color image of the kinect camera
 * @param _points Point cloud of the Kinect
 */
void KinectCore::ProcessKinectData(VisionCore *_vcore, IplImage *_iplImg, cv::Mat_<cv::Vec4f> &_points)
{
  vcore = _vcore;
  iplImg = _iplImg;
  points = _points;
  
  /// TODO Initialisierung sollte eigentlich nur einmal sein, danach, dann immer Clear()
  InitKinectPrinciples();

  struct timespec start, current;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  
  try 
  {
    for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    {
      if(kinectPrinciples[i] != 0)
        if(kinectPrinciples[i]->IsEnabled())
        {
          printf("KinectCore::ProcessKinectData: Processing kinect principle: %u\n", i);
          kinectPrinciples[i]->Process();

          clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
          printf("Runtime for processing the kinect principle %u: %4.3f\n", i, timespec_diff(&current, &start));
          start = current;

          printf("KinectCore::ProcessKinectData: Processing kinect principle: %u ended\n", i);
        }
    }
  }
  catch (exception &e)
  {
    printf("KinectCore::ProcessKinectData: Exception during processing of kinect data.");
    std::cout << e.what() << std::endl;
  }
  
  printf("KinectCore::ProcessKinectData: ended\n");
}


/**
 * @brief Get a stereo object as visual object for the CogX cast-framework.
 * @param type Type of stereo object.
 * @param number ID of the stereo object
 * @param obj Visual object as pointer
 * @return Returns true for success.
 */
// #ifdef HAVE_CAST
// bool KinectCore::GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj)
// {
//   return stereoPrinciples[type]->StereoGestalt2VisualObject(obj, id);
// }
// #endif


/**
 * @brief Get the name of a stereo type with a fixed character length.  
 * @param type Stereo type
 * @return Returns the information as string.
 */
const char* KinectCore::GetKinectTypeName(KinectBase::Type type)
{
  const unsigned info_size = 1000;
  static char name_text[info_size] = "";
  int n = 0;

  n += snprintf(name_text + n, info_size - n, "%s: ", KinectBase::TypeName(type));

  for(int i=0; i< (20 - KinectBase::KinectTypeNameLength(type)); i++)
    n += snprintf(name_text + n, info_size -n, " ");
  
  return name_text;
}


/**
 * @brief Get the Gestalt list with Gestalts3D from kinect data.
 * @return Returns the information as character array.
 */
const char* KinectCore::GetGestaltListInfo()
{
  const unsigned info_size = 50000;
  static char info_text[info_size] = "";
  int n = 0;

  n += snprintf(info_text + n, info_size - n, 
    "  KINECT GESTALT LIST\n  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  for(int i=0; i < Z::Gestalt3D::MAX_TYPE; i++)
  {
    n += snprintf(info_text + n, info_size - n, "  %s", GetGestaltTypeName((Gestalt3D::Type) i));
    n += snprintf(info_text + n, info_size - n, "	%i\n", NumGestalts3D((Gestalt3D::Type) i));
  }
//   n += snprintf(info_text + n, info_size - n, "\n  STEREO LIST		STEREO\n  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");           /// TODO Wie siehts mit den Gestalts3D aus?
//   for(unsigned i=0; i< KinectBase::MAX_TYPE; i++)
//   {
//     n += snprintf(info_text + n, info_size - n, "\n  %s", GetKinectTypeName((KinectBase::Type) i));
//     n += snprintf(info_text + n, info_size - n, "%i", NumGestalts3D((KinectBase::Type) i));
//   }
  return info_text;
}

/**
 * @brief Get the name of a Gestalt3D type.  
 * @param type Gestalt3D type
 * @return Returns the information as string.
 */
const char* KinectCore::GetGestaltTypeName(Z::Gestalt3D::Type type)
{
  const unsigned info_size = 10000;
  static char name_text[info_size] = "";
  int n = 0;

  n += snprintf(name_text + n, info_size - n, "%s: ", Z::Gestalt3D::TypeName(type));

  for(int i=0; i< (18 - Z::Gestalt3D::TypeNameLength(type)); i++)
    n += snprintf(name_text + n, info_size -n, " ");
  
  return name_text;
}


/**
 * @brief Draw the mono results into a iplImage
 * @param type Type of stereo object.
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param masked Draw the masked features.
 * @param single Draw only single Gestalt.
 * @param id ID of the mono Gestalt
 * @param detail Degree of detail.
 * @return Returns true for success.
 */
// bool KinectCore::DrawMonoResults(Gestalt::Type type, IplImage *iIl, IplImage *iIr, bool masked, bool single, 
// 				 int singleSide, int id, int detail)
// {
//   SetImages(iIl, iIr);
// 
//   if(!single)
//   for(int side = LEFT; side <= RIGHT; side++)
//   {
//     SetColor(RGBColor::red);
//     SetActiveDrawAreaSide(side);
//     int numGestalts = NumMonoGestalts(type, side);
//     if (id > numGestalts) return false;
// 
//     // draw all Gestalts
//     for(int i=0; i<numGestalts; i++)
//     {
//       if(masked)
//         vcore[side]->Gestalts(type, i)->Draw(detail);	
//       else
//         if (vcore[side]->Gestalts(type, i)->IsUnmasked())
//           vcore[side]->Gestalts(type, i)->Draw(detail);	
//     }
//   }
// 
//   if(single)
//   {
//     SetColor(RGBColor::white);
//     SetActiveDrawAreaSide(singleSide);
//     int numGestalts = NumMonoGestalts(type, singleSide);
//     if (id >= numGestalts) return false;
// 
//     // draw only one gestalt if id is in range of 
//     if(id < numGestalts && id >= 0)
//     {
//       if(masked)
//         vcore[singleSide]->Gestalts(type, id)->Draw(detail);	
//       else
//         if (vcore[singleSide]->Gestalts(type, id)->IsUnmasked())
//           vcore[singleSide]->Gestalts(type, id)->Draw(detail);
//     }
//   }
// 
//   return true;
// }

/**
 * @brief Draw the stereo results into the stereo iplImages.
 * @param type Type of stereo object.
 * @param iIl Left stereo image.
 * @param iIr Right stereo image.
 * @param matched Draw the matched features.
 */
// void KinectCore::DrawStereoResults(StereoBase::Type type, IplImage *iIl, IplImage *iIr, 
// 				 bool showAllStereoMatched, bool single, int id, int detail)
// {
//   SetImages(iIl, iIr);
//   SetColor(RGBColor::blue);
//   if(!single)
//   {
//     if(!showAllStereoMatched)
//     {
//       for(int side = LEFT; side <= RIGHT; side++)
//       {
// 	SetActiveDrawAreaSide(side);
// 	stereoPrinciples[type]->DrawMatched(side, single, id, detail);
//       }
//     }
//     else // show all stereo matched features
//     {
//       for(int i=0; i< StereoBase::MAX_TYPE; i++)
//       {
// 	for(int side = LEFT; side <= RIGHT; side++)
// 	{
// 	  SetActiveDrawAreaSide(side);
// 	  stereoPrinciples[i]->DrawMatched(side, single, id, detail);
// 	}
//       }
//     }
//   }
//   else
//   {
//     for(int side = LEFT; side <= RIGHT; side++)
//     {
//       SetActiveDrawAreaSide(side);
//       stereoPrinciples[type]->DrawMatched(side, single, id, detail);
//     }
//   }
// }



/**
 * @brief Print the statistics from the vision cores.
 */
void KinectCore::PrintVCoreStatistics()
{
  vcore->PrintRunTime();
}

/**
 * @brief Returns id of first gestalt at pixel position (x,y).
 * @param side Left / right side of stereo image pair.
 * @param type Gestalt type.
 * @param x x-coordinate in image pixels.
 * @param y y-coordinate in image pixels.
 * @param start_after Choose only Gestalts with id higher than start_after.
 * @param reject_masked Reject masked Gestalts. 
 * @return Returns the ID of the next Gestalt.
 */
// unsigned KinectCore::PickGestaltAt(int side, Gestalt::Type type, int x, int y, unsigned start_after, bool reject_masked)
// {
//   return vcore[side]->PickGestaltAt(type, x, y, start_after, reject_masked);
// }





} 
