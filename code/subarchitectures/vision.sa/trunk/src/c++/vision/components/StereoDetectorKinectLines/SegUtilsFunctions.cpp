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

void DrawNURBS(TomGine::tgTomGineThread *tgR,
               const ON_NurbsSurface &on_surf,
               int color)
{
  TomGine::tgModel model;
  unsigned res = 16;
  NurbsConvertion::Convert(on_surf, model, res);
  tgR->AddModel(model);
}

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

/**
 * TODO We should move that to Annotation
 * @brief Check annotation for evaluation (over-/under-segmentation)
 * @param surfaces Surface patches
 * @param anno Annotation of all pcl_model_indices
 * @param graphCutGroups Graph cut groups of surface patches
 */
void CheckAnnotation(std::vector<surface::SurfaceModel::Ptr> &surfaces,
                     std::vector<int> &anno,
                     std::vector< std::vector<unsigned> > &graphCutGroups)
{
  printf("[SegUtilsFunctions::CheckAnnotation] Move to annotation and cleanup this stuff!\n");

  std::vector<double> res_multi;              // results of the object check (anno to real)
  std::vector<double> res_multi_forward;      // nr true positive points (biggest)
  std::vector<double> res_multi_indices;      // nr overall indices
  static int overallsum_res_forward = 0;      // res_forward over more than one image
  static int overallsum_res_multi = 0;        // res_multi over more than one image

  std::vector<double> res_backcheck;              // backcheck rate (real to anno)
  std::vector<double> res_too_much_pixel;         // nr true positive points (biggest)
                                                  // multi-indices are the same!
  static int overall_res_too_much_pixel = 0;      // number or oversegmentation pixel (more than one image)

//   for(unsigned i=0; i<graphCutGroups.size(); i++) { 
//     printf("%u: ", i);
//     for(unsigned j=0; j<graphCutGroups[i].size(); j++)
//       printf("%u ", graphCutGroups[i][j]);
//     printf("\n");
//   }
    
  // graph cut group label on image plane => objects
  std::vector<int> objects;                   
  objects.resize(anno.size());
  for(unsigned i=0; i<objects.size(); i++)
    objects[i] = -1;
  for(unsigned i=0; i < surfaces.size(); i++)
    for(unsigned j=0; j< surfaces[i]->indices.size(); j++)
      for(unsigned k=0; k < graphCutGroups.size(); k++) 
        for(unsigned l=0; l < graphCutGroups[k].size(); l++) 
          if(graphCutGroups[k][l] == i)
            objects[surfaces[i]->indices[j]] = k;
    

  int annoMaxLabel = 0;                                 /// TODO unused
  std::vector<int> annoLabels;                          // get annoLabels
  for(unsigned i=0; i<anno.size(); i++)
    if(anno[i] != -1)
    {
      bool isAnnoLabel = false;
      for(unsigned j=0; j<annoLabels.size(); j++)
        if(annoLabels[j] == anno[i])
          isAnnoLabel = true;
      if(!isAnnoLabel)
        annoLabels.push_back(anno[i]);
      if(anno[i] > annoMaxLabel)
        annoMaxLabel = anno[i];
    }
    
  int nr_indices_annoLabel[annoLabels.size()];           // Number of points for a certain annoLabel
  for(unsigned i=0; i< annoLabels.size(); i++)
  {
    nr_indices_annoLabel[i] = 0;
    for(unsigned j=0; j< anno.size(); j++)
      if(anno[j] == annoLabels[i])
        nr_indices_annoLabel[i]++;
  }
          
// printf("\nAnno Labels: ");
// for(unsigned i=0; i<annoLabels.size(); i++)
//   printf("%u ", annoLabels[i]);
// printf("\n");


  /// Forward check: annotation => real
  int forwardCheck[graphCutGroups.size()];                // nr of pixels in each graphCutGroup
  for(unsigned i=0; i<annoLabels.size(); i++)             // for each annoLabel
  {
    for(unsigned j=0; j<graphCutGroups.size(); j++)
      forwardCheck[j] = 0;
    
    for(unsigned j=0; j<anno.size(); j++)
      if(anno[j] == annoLabels[i])
        forwardCheck[objects[j]]++;  // == graphCutLabel

    
// printf("graphCutGroups for annoLabel[%u]= %u: %u\n", i, annoLabels[i], nr_indices_annoLabel[i]);
// for(unsigned j=0; j<graphCutGroups.size(); j++)    // anno label 0 is background = uninteresting
//   printf("%u: %u\n", j, forwardCheck[j]);
    
    // calculate biggest forward-value: anno -> real
    int best_graph_cut_group = 0;
    double biggestGroupVal = 0.;
    int biggest_sum_forward = 0;
    for(unsigned j=0; j<graphCutGroups.size(); j++)    // anno label 0 is background = uninteresting
    {
      double groupVal = ((double) forwardCheck[j]) / ((double) nr_indices_annoLabel[i]);
      if(groupVal > biggestGroupVal)
      {
        best_graph_cut_group = j;
        biggestGroupVal = groupVal;
        biggest_sum_forward = forwardCheck[j];
      }
    }
    res_multi.push_back(biggestGroupVal);
    res_multi_forward.push_back(biggest_sum_forward);
    res_multi_indices.push_back(nr_indices_annoLabel[i]);

      
    /// Backcheck: Diese pixel müssen jetzt genauso zusammengerechnet werden!!!
    // get number of pixels of best_graph_cut_group
    int nr_surface_indices = 0;
    for(unsigned j=0; j<graphCutGroups[best_graph_cut_group].size(); j++)
      nr_surface_indices += surfaces[graphCutGroups[best_graph_cut_group][j]]->indices.size();

    // nr_surface_indices - biggestGroupVal => überschüssige pixel
    int too_much_pixel = nr_surface_indices - biggest_sum_forward;

    double undersegmentation = (double) too_much_pixel / (double) nr_indices_annoLabel[i]; 
    res_backcheck.push_back(undersegmentation);
    res_too_much_pixel.push_back((double) too_much_pixel);
  }
  
  int sum_res_multi = 0;
  int sum_res_multi_indices = 0;
  int sum_res_backcheck = 0;
  for(unsigned i=1; i<res_multi.size(); i++)
  {    
    sum_res_multi += res_multi_forward[i];
    sum_res_multi_indices += res_multi_indices[i];
    sum_res_backcheck += res_too_much_pixel[i];
    
    printf("results for forward anno[%u]:  %4.3f (%4.0f / %4.0f)\n", i, res_multi[i], res_multi_forward[i], res_multi_indices[i]);
    printf("results for backward anno[%u]: %4.3f (%4.0f / %4.0f)\n", i, res_backcheck[i], res_too_much_pixel[i], res_multi_indices[i]);
  }


  overallsum_res_multi += sum_res_multi;
  overallsum_res_forward += sum_res_multi_indices;
  overall_res_too_much_pixel += sum_res_backcheck;
  printf("\nSum of forward check:  %u/%u = %4.3f\n", sum_res_multi, sum_res_multi_indices, (double) sum_res_multi / (double) sum_res_multi_indices);
  printf("Sum of backward check: %u/%u = %4.3f\n", sum_res_backcheck, sum_res_multi_indices, (double) sum_res_backcheck / (double) sum_res_multi_indices);
  printf("Overall sum of forward check:  %u/%u = %4.3f\n", overallsum_res_multi, overallsum_res_forward, (double) overallsum_res_multi / (double) overallsum_res_forward);
  printf("Overall sum of backward check: %u/%u = %4.3f\n", overall_res_too_much_pixel, overallsum_res_forward, (double) overall_res_too_much_pixel / (double) overallsum_res_forward);
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


