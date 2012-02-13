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
#include "KinectPclEdges.h"
#include "KinectPclLines.h"
//#include "KinectPclCylinders.h"
#include "KinectPclModels.h"
#include "KinectPclSegments.h"
#include "../../../VisionUtils.h"

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);

/**
 * @brief Constructor of Kinect Core
 * @param stereocal_file Stereo calibration file
 */
KinectCore::KinectCore(Z::VisionCore *vc, double _fx, double _fy, double _cx, double _cy) throw(std::runtime_error)
{
  initialized = false;
  vcore = vc;
  fx = _fx;
  fy = _fy;
  cx = _cx;
  cy = _cy;
  drawNodeID = false;
  valid_normals = false;
  nr_processed_images = 0;
}


/**
 * @brief Destructor of Kinect Core
 */
KinectCore::~KinectCore()
{
  delete vcore;
  cvReleaseImage(&iplImg);
  for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    if(kinectPrinciples[i]->IsEnabled())
      delete kinectPrinciples[i];
}


/**
 * @brief Give all 3D Gestalts a unique node ID
 */
void KinectCore::SetNodeIDs()
{
printf("KinectCore::SetNodeID: we initialize only patches and lines!\n");
  unsigned nodeID = 0;
//   for(unsigned i=0; i<Gestalts3D::MAX_TYPE; i++)
//     for(unsigned j=0; j<kinectGestalts[i].Size(); j++)
//       kinectGestalts[i][j]->SetNodeID(nodeID++);

 for(unsigned j=0; j<kinectGestalts[Gestalt3D::PATCH].Size(); j++)
   kinectGestalts[Gestalt3D::PATCH][j]->SetNodeID(nodeID++);
//  for(unsigned j=0; j<kinectGestalts[Gestalt3D::SEGMENT].Size(); j++)
//    kinectGestalts[Gestalt3D::SEGMENT][j]->SetNodeID(nodeID++);
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
  kinectPrinciples[KinectBase::KINECT_PATCHES] = new KinectPatches(this, vcore);
  kinectPrinciples[KinectBase::KINECT_SEGMENTS] = new KinectSegments(this, vcore);
  kinectPrinciples[KinectBase::KINECT_LINES] = new KinectLines(this, vcore);
  kinectPrinciples[KinectBase::KINECT_COLLINEARITIES] = new KinectCollinearities(this, vcore);
  kinectPrinciples[KinectBase::KINECT_CLOSURES] = new KinectClosures(this, vcore);
  kinectPrinciples[KinectBase::KINECT_RECTANGLES] = new KinectRectangles(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_MODELS] = new KinectPclModels(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_EDGELS] = new KinectPclEdges(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_SEGMENTS] = new KinectPclSegments(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_LINES] = new KinectPclLines(this, vcore);
  
  // set principles enabled or disabled
//   kinectPrinciples[KinectBase::KINECT_PATCHES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_SEGMENTS]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_LINES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_COLLINEARITIES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_CLOSURES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_RECTANGLES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_PCL_MODELS]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_PCL_EDGELS]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_PCL_SEGMENTS]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_PCL_LINES]->EnablePrinciple(true);
  
  initialized = true;
}

/**
 * @brief Initialisation of the Kinect Gestalt clases.
 * Do this initialisation after receiving vcore, iplImg, points.
 */
void KinectCore::InitKinectPrinciplesForSegmenter()
{
  // Add all Gestalt principles we know
  for(int i = 0; i < KinectBase::MAX_TYPE; i++)
    kinectPrinciples[i] = 0;

  // stereoPrinciples all principles
  kinectPrinciples[KinectBase::KINECT_PATCHES] = new KinectPatches(this, vcore);
  kinectPrinciples[KinectBase::KINECT_SEGMENTS] = new KinectSegments(this, vcore);
  kinectPrinciples[KinectBase::KINECT_LINES] = new KinectLines(this, vcore);
  kinectPrinciples[KinectBase::KINECT_COLLINEARITIES] = new KinectCollinearities(this, vcore);
  kinectPrinciples[KinectBase::KINECT_CLOSURES] = new KinectClosures(this, vcore);
  kinectPrinciples[KinectBase::KINECT_RECTANGLES] = new KinectRectangles(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_MODELS] = new KinectPclModels(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_EDGELS] = new KinectPclEdges(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_SEGMENTS] = new KinectPclSegments(this, vcore);
  kinectPrinciples[KinectBase::KINECT_PCL_LINES] = new KinectPclLines(this, vcore);

  // set principles enabled or disabled
//   kinectPrinciples[KinectBase::KINECT_PATCHES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_SEGMENTS]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_LINES]->EnablePrinciple(true);                    /// TODO was tu ich mit den lines??? => Für segments?
//   kinectPrinciples[KinectBase::KINECT_COLLINEARITIES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_CLOSURES]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_RECTANGLES]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_PCL_MODELS]->EnablePrinciple(true);
//   kinectPrinciples[KinectBase::KINECT_PCL_EDGELS]->EnablePrinciple(true);
  kinectPrinciples[KinectBase::KINECT_PCL_SEGMENTS]->EnablePrinciple(true);             /// TODO das hier ist nicht notwendig, oder???
//   kinectPrinciples[KinectBase::KINECT_PCL_LINES]->EnablePrinciple(true);

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
 * @param random_color True for random color
 */
void KinectCore::DrawGestalts3D(TomGine::tgTomGineThread *tgRenderer,
                                Gestalt3D::Type type,
                                bool random_color, 
                                bool use_color, 
                                float color)
{ 
  for(unsigned i=0; i < NumGestalts3D(type); i++)
  {
    Gestalts3D(type, i)->DrawNodeID(drawNodeID);
    Gestalts3D(type, i)->DrawGestalt3D(tgRenderer, random_color, use_color, color);
  }
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
 * @brief Draw Gestalts into TomGine, colored by object labels.
 * @param tgRenderer TomGine render engine.
 */
void KinectCore::DrawObjects3D(TomGine::tgTomGineThread *tgRenderer)
{ 
  std::vector<unsigned> objectIDs;
  std::vector<float> color;
  
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < NumGestalts3D(t); i++)
    {
      bool found = false;
      unsigned pos = 0;
      unsigned objID = Gestalts3D(t, i)->GetObjectLabel();
      
      for(unsigned j=0; j<objectIDs.size(); j++)
        if(objectIDs[j] == objID)
          found = true;
              
      if(!found)
      {
// printf("KinectCore::DrawObjects3D: Got object label: %u\n", objID);
        objectIDs.push_back(objID);
        float col =  GetRandomColor();
        color.push_back(col);
        pos = color.size()-1;
      }
      else 
      {
        for(unsigned j=0; j<objectIDs.size(); j++)
          if(objectIDs[j] == objID)
            pos = j;
      }
// printf("      Got object label: %u with color-pos: %u\n", objID, pos);
      Gestalts3D(t, i)->DrawGestalt3D(tgRenderer, false, true, color[pos]);
    }
  }
}


/**
 * @brief Draw features into TomGine, colored by graph-cut labels.
 * @param tgRenderer TomGine render engine.
 */
void KinectCore::DrawGraphCut3D(TomGine::tgTomGineThread *tgRenderer)
{ 
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
          Gestalts3D(t, i)->DrawGestalt3D(tgRenderer, false, true, col);
      }
    }
  }
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
 * @param _vcore Vision core
 * @param _iplImage Color image of the kinect camera
 * @param _points Point cloud of the Kinect
 */
void KinectCore::Process(IplImage *_iplImg, cv::Mat_<cv::Vec4f> &_points,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud)
{
  valid_normals = false;
  iplImg = _iplImg;
  points = _points;
  pcl_cloud = _pcl_cloud;
  const_pcl_cloud = _pcl_cloud;     // TODO reicht das? => Testen.
//   pcl::copyPointCloud(_pcl_cloud, const_pcl_cloud);
  
  if(!initialized) InitKinectPrinciples();
  
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
 * @brief Get the normals of the point cloud.
 * 640x480: 260ms / 320x240: 64ms
 * @return Returns the normals as cv-Mat
 */
cv::Mat_<cv::Vec4f> KinectCore::GetCvNormals()
{
  if(!valid_normals)
  {
    cv_normals = cv::Mat_<cv::Vec4f>(points.rows, points.cols);
    pclA::NormalsFromSortedPCLCloud(pcl_cloud, pcl_normals, cv_normals, 0.02, 5.0);
    valid_normals = true;
  }
  return cv_normals;
}


/**
 * @brief Get the normals of the point cloud.
 * 640x480: 260ms / 320x240: 64ms
 * @return Returns the normals as pcl-cloud
 */
pcl::PointCloud<pcl::Normal>::Ptr KinectCore::GetPclNormals()
{
  if(!valid_normals)
  {
    cv_normals = cv::Mat_<cv::Vec4f>(points.rows, points.cols);
    pclA::NormalsFromSortedPCLCloud(pcl_cloud, pcl_normals, cv_normals, 0.02, 5.0);
    valid_normals = true;
  }
  return pcl_normals;
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
    "  KINECT GESTALT LIST INFO\n  XXXXXXXXXXXXXXXXXXXXXXXX\n");
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
 * plane-popout results.
 * @param pp Plane-Popout
 */
void KinectCore::SetObjectLabels(pclA::PlanePopout *pp)
{
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < NumGestalts3D(t); i++)
      Gestalts3D(t, i)->SetObjectLabel(pp->IsInSOI(Gestalts3D(t, i)->GetCenter3D()));
  }
}

void KinectCore::PrintNodeIDs()                                                       /// TODO TODO TODO Delete later!!!
{
  unsigned nodeID = 0;
  for(unsigned i=0; i<KinectBase::MAX_TYPE; i++)
    for(unsigned j=0; j<kinectGestalts[i].Size(); j++)
      printf(" nodeID: %u\n", kinectGestalts[i][j]->GetNodeID());
}

/**
 * @brief Check hand-made annotation with graph-cutted results.
 * @param anno Annotation (-1 = unknown / object label = 1,2,3, ...)
 * @return Returns the percentage of correct labeled results.
 */
double KinectCore::CheckAnnotation(std::vector<int> &anno)
{
  unsigned annoMaxLabel = 0;
  for(unsigned i=0; i<anno.size(); i++)
    if(anno[i] != -1)
      if(anno[i] > annoMaxLabel)
        annoMaxLabel = anno[i];
        
  std::vector<double> res_multi;              // results of the object check (anno to real)
  std::vector<double> res_single;             // results of the single object check (anno to real)
  std::vector<double> res_backcheck;          // backcheck (real to anno)
  
  std::vector<int> objects;     // graph cut label (real data) on image plane
  objects.resize(anno.size());
  for(unsigned i=0; i<objects.size(); i++)
    objects[i] = -1;

  int maxGraphCutLabel = 0;
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < NumGestalts3D(t); i++)
    {
      int label = Gestalts3D(t, i)->GetGraphCutLabel();
      if(label > 0)
      {
        if(label > maxGraphCutLabel)
          maxGraphCutLabel = label;
        
        std::vector<int> indices;
        Gestalts3D(t, i)->GetIndices(indices);
        for(unsigned idx=0; idx<indices.size(); idx++)
          objects[(indices[idx])] = label;
      }
    }
  }

// printf(" Anno max label = max gc-label: %u - %u\n", annoMaxLabel, maxGraphCutLabel);

  std::vector< std::pair<int, int> > anno_real_pairs;
//   std::pair<int, int> anno_real_pair;
  for(unsigned lab=1; lab<=annoMaxLabel; lab++)
  {
    int cnt = 0, cnt_sum = 0;
    int labelCnt[maxGraphCutLabel];
    for(unsigned i=0; i<maxGraphCutLabel; i++)
      labelCnt[i]=0;
    
    for(unsigned i=0; i<anno.size(); i++)
    {
      if(anno[i] == lab)
      { 
        cnt_sum++;
        if(objects[i] != -1)
        {
// printf("we have for lab: %u => objects[i]: %u\n", lab, objects[i]);
          cnt++;
// printf("add to labelCnt[%u]++\n", lab);
          labelCnt[objects[i]]++;
        }
      }
    }
// printf("  multi object %u: %u of %u:  %4.4f%\n", lab, cnt, cnt_sum, cnt*100./cnt_sum);
    res_multi.push_back((double) cnt/(double) cnt_sum);

    int maxLabelCnt = 0;
    int maxLabelObject = 0;
    for(unsigned i=0; i<maxGraphCutLabel; i++)
    {
// printf("partial objectness (%u) for object %u: %u of %u (%4.4f%)\n", i, lab, labelCnt[i], cnt_sum, labelCnt[i]*100./cnt_sum);
      if(labelCnt[i] > maxLabelCnt)
      {
        maxLabelCnt = labelCnt[i];
        maxLabelObject = i;
      }
    }
    anno_real_pairs.push_back(std::make_pair(lab, maxLabelObject));
    res_single.push_back((double) maxLabelCnt/(double) cnt_sum);
// printf("=> maximum for (anno=%u) object %u: %u of %u (%4.4f%)\n", lab, maxLabelObject, maxLabelCnt, cnt_sum, (double) maxLabelCnt*100./(double)cnt_sum);
  }
  
  // calculate backcheck
  for(int i=0; i<anno_real_pairs.size(); i++)
  {
    int anno_id = anno_real_pairs[i].first;
    int obj_id = anno_real_pairs[i].second;
    int sum = 0, cnt = 0;
    for(unsigned idx=0; idx<objects.size(); idx++) {
      if(objects[idx] == obj_id) {
        sum++;
        if(anno[idx] == anno_id)
          cnt++;
      }
    }
//     printf("Check #3: anno %u - object %u => %u of %u (%4.4f%)\n", anno_id, obj_id, cnt, sum, (double) cnt*100./sum);
    res_backcheck.push_back((double) cnt/ (double) sum);
  }
  
  double sum_single = 0;
  double sum_multi = 0;
  double sum_backcheck = 0;
  for(unsigned i=0; i< res_single.size(); i++) {
    sum_single += res_single[i];
    sum_multi += res_multi[i];
    sum_backcheck += res_backcheck[i];
  }
  sum_res_multi.push_back(sum_multi/res_multi.size());
  sum_res_single.push_back(sum_single/res_single.size());
  sum_res_backcheck.push_back(sum_backcheck/res_backcheck.size());
  
  if(res_single.size() != res_multi.size() || res_single.size() != res_backcheck.size())
    printf("KinectCore::CheckAnnotation: Warning: result-arrays with different sizes found!\n");
  
  nr_processed_images++;
  return 0.0;
}

void KinectCore::GetAnnotationResults(double &m, double &s, double &b)
{
  m = 0; s = 0; b = 0;
  for(unsigned i=0; i<nr_processed_images; i++) {
    m += sum_res_multi[i];
    s += sum_res_single[i];
    b += sum_res_backcheck[i];
  }
  m /= nr_processed_images;
  s /= nr_processed_images;
  b /= nr_processed_images;
} 


void KinectCore::SetAnnotation(std::vector<int> &anno)
{
  for(int type=0; type < Gestalt3D::MAX_TYPE; type++)
  {
    Gestalt3D::Type t = (Gestalt3D::Type) type;
    for(unsigned i=0; i < NumGestalts3D(t); i++)
      Gestalts3D(t, i)->SetAnnotation(anno);
//       Gestalts3D(t, i)->SetObjectLabel(pp->IsInSOI(Gestalts3D(t, i)->GetCenter3D()));
  }
}


}
