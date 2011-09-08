/**
 * @file KinectCore.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Managment of processing kinect data.
 */

#include <set>

#include "KinectCore.h"
#include "Gestalt3D.h"
#include "Patch3D.h"
#include "Line3D.h"

#include "KinectPatches.h"
#include "KinectSegments.h"
#include "KinectLines.h"
#include "KinectCollinearities.h"
#include "KinectClosures.h"
#include "KinectRectangles.h"

namespace Z
{

inline float GetRandomColor()
{
  RGBValue x;
  x.b = std::rand()%255;
  x.g = std::rand()%255;
  x.r = std::rand()%255;
  x.a = 0.; 
  return x.float_value;
}
  
  
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
 * @brief Give all 3D Gestalts a unique node ID
 */
void KinectCore::SetNodeIDs()
{
printf("KinectCore::SetNodeID: we initialize now only patches and lines!\n");
  unsigned nodeID = 0;
//   for(unsigned i=0; i<Gestalts3D::MAX_TYPE; i++)
//     for(unsigned j=0; j<kinectGestalts[i].Size(); j++)
//       kinectGestalts[i][j]->SetNodeID(nodeID++);

 for(unsigned j=0; j<kinectGestalts[Gestalt3D::PATCH].Size(); j++)
   kinectGestalts[Gestalt3D::PATCH][j]->SetNodeID(nodeID++);
 for(unsigned j=0; j<kinectGestalts[Gestalt3D::LINE].Size()/*5*/; j++)
   kinectGestalts[Gestalt3D::LINE][j]->SetNodeID(nodeID++);
}

void KinectCore::PrintNodeIDs()                                                       /// TODO TODO TODO Delete later!!!
{
  unsigned nodeID = 0;
  for(unsigned i=0; i<KinectBase::MAX_TYPE; i++)
    for(unsigned j=0; j<kinectGestalts[i].Size(); j++)
      printf(" nodeID: %u\n", kinectGestalts[i][j]->GetNodeID());
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
void KinectCore::DrawGestalts3D(TomGine::tgTomGineThread *tgRenderer,
                                Gestalt3D::Type type)
{ 
  for(unsigned i=0; i < NumGestalts3D(type); i++)
    Gestalts3D(type, i)->DrawGestalt3D(tgRenderer);
}


/**
 * @brief Draw results into TomGine render engine.
 * @param image Draw calculated 3D patches to image!
 * @param type Type of Gestalt3D to draw.
 */
void KinectCore::DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, 
                                       Gestalt3D::Type type,
                                       Video::CameraParameters camPars)
{ 
  for(unsigned i=0; i < NumGestalts3D(type); i++)
    Gestalts3D(type, i)->DrawGestalts3DToImage(image, camPars);
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
void KinectCore::Process(VisionCore *_vcore, IplImage *_iplImg, cv::Mat_<cv::Vec4f> &_points)
{
  vcore = _vcore;
  iplImg = _iplImg;
  points = _points;
  
  InitKinectPrinciples();  /// TODO Initialisierung sollte eigentlich nur einmal sein

  try 
  {
    for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    {
      if(kinectPrinciples[i] != 0)
        if(kinectPrinciples[i]->IsEnabled())
        {
//           printf("KinectCore::ProcessKinectData: Processing kinect principle: %u\n", i);
          kinectPrinciples[i]->Process();
//           printf("KinectCore::ProcessKinectData: Processing kinect principle: %u ended\n", i);
        }
    }
  }
  catch (exception &e)
  {
    printf("KinectCore::ProcessKinectData: Exception during processing of kinect data.");
    std::cout << e.what() << std::endl;
  }

  SetNodeIDs(); // give node id's to all kinect-Gestalts
}

/**
 * @brief Get the name of a kinect principle type with a fixed character length.  
 * @param type Kinect principle type
 * @return Returns the information as character field.
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
 * @brief Print the statistics from the vision cores.
 */
void KinectCore::PrintVCoreStatistics()
{
  vcore->PrintRunTime();
}

/**
 * @brief Set object labels for kinect-Gestalts, according to the
 * plane-popout results. Currently for patches and lines!
 * @param pp Plane-Popout
 */
void KinectCore::SetObjectLabels(pclA::PlanePopout *pp)
{
  // set object labels, according to planePopout-SOIs
  unsigned nrPatches = NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p = (Patch3D*) Gestalts3D(Gestalt3D::PATCH, i);
    p->SetObjectLabel(pp->IsInSOI(p->GetCenter3D()));
  }
  unsigned nrLines = NumGestalts3D(Gestalt3D::LINE);
  for(unsigned i=0; i<nrLines; i++)
  {
    Line3D *l = (Line3D*) Gestalts3D(Gestalt3D::LINE, i);
    l->SetObjectLabel(pp->IsInSOI(l->GetCenter3D()));
  }
}

/**
 * @brief Draw Gestalts into TomGine, colored by object labels.
 * @param tgRenderer TomGine render engine.
 */
void KinectCore::DrawObjects3D(TomGine::tgTomGineThread *tgRenderer)
{ 
  std::set<unsigned> objectIDs;
  set<unsigned>::iterator it;

  std::vector<float> color;
  
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < NumGestalts3D(t); i++)
    {
      unsigned pos = 0;
      unsigned objID = Gestalts3D(t, i)->GetObjectLabel();
      if(objID != UNDEF_ID)
      {
        if(objectIDs.find(objID) == objectIDs.end())
        {
          objectIDs.insert(objID);
          float col =  GetRandomColor();
          color.push_back(col);
          pos = color.size()-1;
        }
        else 
        {
          for(it=objectIDs.find(objID); it != objectIDs.begin(); it--)
            pos++;
        }
          
        Gestalts3D(t, i)->DrawGestalt3D(tgRenderer, true, color[pos]);
      }
    }
  }
}


/**
 * @brief Draw features into TomGine, colored by graph-cut labels.
 * @param tgRenderer TomGine render engine.
 */
void KinectCore::DrawGraphCut3D(TomGine::tgTomGineThread *tgRenderer)
{ 
// printf("KinectCore::DrawGraphCut3D: All graph cut groups: %u\n", graphCutGroups.size());
//   for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
//   {
//     Gestalt3D::Type t = (Gestalt3D::Type) type;
//     for(unsigned i=0; i < NumGestalts3D(t); i++)
//     {
//       unsigned label = Gestalts3D(t, i)->GetGraphCutLabel();
// printf("  %u with label: %u\n", Gestalts3D(t, i)->GetNodeID(), label);
//     }
//   }  
//   printf("KinectCore::DrawGraphCut3D: Number of groups: %u\n", graphCutGroups.size());
  
  set<unsigned>::iterator it;
  for(it=graphCutGroups.begin(); it != graphCutGroups.end(); it++)
  {
    float col =  GetRandomColor();
    for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
    {
      Gestalt3D::Type t = (Gestalt3D::Type) type;
      for(unsigned i=0; i < NumGestalts3D(t); i++)
      {
        unsigned label = Gestalts3D(t, i)->GetGraphCutLabel();
        if(label == *it && label != -1)
          Gestalts3D(t, i)->DrawGestalt3D(tgRenderer, true, col);
      }
    }
  }
}

} 
