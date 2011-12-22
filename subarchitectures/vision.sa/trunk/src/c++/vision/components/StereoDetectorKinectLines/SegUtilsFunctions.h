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
#include "v4r/NurbsFitting/NurbsConvertion.h"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"

#include "VisionCore.hh"
#include "StereoCore.h"


namespace cast
{
  
/**
 * @brief Convert a IplImage to a cvMat<Vec3b> image.
 * @param iplImage IplImage source
 * @param matImage cv::Mat image destination
 */
void ConvertImage(IplImage &iplImage, cv::Mat_<cv::Vec3b> &image);

void DrawNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                 TomGine::tgTomGineThread *tgR,
                 int color);

void DrawNURBS(TomGine::tgTomGineThread *tgR,
               const ON_NurbsSurface &on_surf,
               int color);

void GetSegmentIndexes(Z::VisionCore *vcore, 
                       std::vector<bool> &_texture,
                       int point_cloud_width);

unsigned WhichGraphCutGroup(unsigned modelID, 
                            std::vector< std::vector<unsigned> > _graphCutGroups);

void CheckAnnotation(std::vector<cv::Ptr<surface::SurfaceModel> > &surfaces,
                     std::vector<int> &anno,
                     std::vector< std::vector<unsigned> > &graphCutGroups);

}


#endif
