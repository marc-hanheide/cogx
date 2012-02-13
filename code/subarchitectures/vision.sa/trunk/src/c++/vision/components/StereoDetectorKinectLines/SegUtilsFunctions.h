/**
 * @file SegUtilsFunctions.h
 * @author Andreas Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Some additional utils-functions for segmenting-classes
 */

#ifndef SEG_UTILS_FUNCTIONS_H
#define SEG_UTILS_FUNCTIONS_H

#include <opencv2/highgui/highgui.hpp>
#include <v4r/PCLAddOns/PCLCommonHeaders.h>
#include "v4r/TomGine/tgTomGineThread.h"
#include "v4r/NurbsConvertion/NurbsConvertion.h"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"

#include "VisionCore.hh"
#include "StereoCore.h"


namespace cast
{
  
/** Convert a IplImage to a cvMat image. **/
void ConvertImage(IplImage &iplImage, cv::Mat_<cv::Vec3b> &image);

/** Draw NORMALS on the TomGine **/
void DrawNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                 TomGine::tgTomGineThread *tgR,
                 int color);

/** Draw NURBS on the TomGine **/
void DrawNURBS(TomGine::tgTomGineThread *tgR,
               const ON_NurbsSurface &on_surf,
               int color);

/** Returns the canny edges from the vcore as texture vector with a certain width **/
void GetSegmentIndexes(Z::VisionCore *vcore, 
                       std::vector<bool> &_texture,
                       int point_cloud_width);

/** Returns the graph cut group for a certain modelID **/
unsigned WhichGraphCutGroup(unsigned modelID, 
                            std::vector< std::vector<unsigned> > _graphCutGroups);

/** Check over- and undersegmentation of objects **/
void CheckAnnotation(std::vector<surface::SurfaceModel::Ptr> &surfaces,
                     std::vector<int> &anno,
                     std::vector< std::vector<unsigned> > &graphCutGroups);

/** Upscale indices by a factor of 2 **/
std::vector<int> UpscaleIndices(std::vector<int> &_indices,
                                int image_width);

}


#endif
