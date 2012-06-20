/**
 * @file SegUtilsFunctions.cpp
 * @author Andreas Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Some additional utils-functions for Segmenting-Classes
 */


#include <vector>
#include "SegUtilsFunctions.h"


namespace cast
{

void ConvertImage(IplImage &iplImage, cv::Mat_<cv::Vec3b> &image)
{
  image = cv::Mat_<cv::Vec3b>(iplImage.height, iplImage.width); 
  for (int v = 0; v < iplImage.height; ++v)
  {
    uchar *d = (uchar*) iplImage.imageData + v*iplImage.widthStep;
    for (int u = 0; u < iplImage.width; ++u, d+=3)
    {
      cv::Vec3b &ptCol = image(v,u);
      ptCol[0] = d[0];
      ptCol[1] = d[1];
      ptCol[2] = d[2];
    }
  }
}

void DrawNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                 TomGine::tgTomGineThread *tgR,
                 int color)
{
  for(unsigned i=0; i<normals->points.size(); i++)
  {
    cv::Vec3f s, e;
    s[0] = pcl_cloud->points[i].x;
    s[1] = pcl_cloud->points[i].y;
    s[2] = pcl_cloud->points[i].z;
    e[0] = pcl_cloud->points[i].x + normals->points[i].normal_x/500.;
    e[1] = pcl_cloud->points[i].y + normals->points[i].normal_y/500.;
    e[2] = pcl_cloud->points[i].z + normals->points[i].normal_z/500.;
    if(color == 1)
      tgR->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 255, 255, 1);
    else 
      tgR->AddLine3D(s[0], s[1], s[2], e[0], e[1], e[2], 255, 0, 0, 1);
  }
}

//void DrawNURBS(TomGine::tgTomGineThread *tgR,
//               const ON_NurbsSurface &on_surf,
//               int color)
//{
//  TomGine::tgModel model;
//  unsigned res = 16;
//  NurbsConvertion::Convert(on_surf, model, res);
//  tgR->AddModel(model);
//}

void GetSegmentIndexes(Z::VisionCore *vcore, 
                       std::vector<bool> &_texture,
                       int point_cloud_width)
{
  double scale = vcore->IW()/point_cloud_width;
  _texture.resize(vcore->IW()*vcore->IH()/(scale*scale));
  for(unsigned i=0; i<_texture.size(); i++)
    _texture[i] = false;

  int nrSegments = vcore->NumGestalts(Z::Gestalt::SEGMENT);
  for(int i=0; i<nrSegments; i++) {
    Z::Segment *s = (Z::Segment*) vcore->Gestalts(Z::Gestalt::SEGMENT, i);
    for(unsigned j=0; j<s->edgels.Size(); j++) {
      int index = (s->edgels[j].p.y * vcore->IW() / scale + s->edgels[j].p.x) / scale;
      _texture[index] = true;
    }
  }
}


unsigned WhichGraphCutGroup(unsigned modelID, std::vector< std::vector<unsigned> > _graphCutGroups)
{
  for(unsigned i=0; i<_graphCutGroups.size(); i++)
    for(unsigned j=0; j<_graphCutGroups[i].size(); j++)
      if(_graphCutGroups[i][j] == modelID)
        return i;
  return 0;
}


std::vector<int> UpscaleIndices(std::vector<int> &_indices,
                    int image_width)
{
printf("SegUtilsFunctions: UpscaleIndices: Check this function!\n");
  std::vector<int> new_indices;
  for(unsigned i=0; i<_indices.size(); i++)
  {
//     int pos_x = _indices[i] % image_width;
//     int pos_y = _indices[i] / image_width;
//     pos = pos_y * image_width * 2 + pos_x * 2;
    
    new_indices.push_back(_indices[i]*2);
    new_indices.push_back(_indices[i]*2 + 1);
    new_indices.push_back(_indices[i]*2 + image_width);
    new_indices.push_back(_indices[i]*2 + image_width + 1);
  }
  return new_indices;
}

}


