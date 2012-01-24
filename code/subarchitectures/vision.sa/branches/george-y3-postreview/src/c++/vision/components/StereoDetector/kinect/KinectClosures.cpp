/**
 * @file KinectClosures.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Calculate 3D closures from Kinect data.
 */

#include "KinectCore.h"
#include "KinectClosures.h"
//#include "VisionUtils.h"

#include "Closure3D.h"

namespace Z
{


/**
 * @brief Constructor of class KinectClosures
 * @param kc Kinect core
 * @param vc Vision core
 * @param iplI Ipl-Image
 * @param p Point matrix
 */
KinectClosures::KinectClosures(KinectCore *kc, VisionCore *vc) 
                             : KinectBase(kc, vc)
{
  numClosures = 0;
}

/**
 * @brief Clear all arrays.
 */
void KinectClosures::ClearResults()
{
  numClosures = 0;
}


/**
 * @brief Calculate 3D patches from kinect data.
 */
void KinectClosures::Process()
{
// printf("Process KinectClosures: start!\n");
  int minimum_points_per_plane = 15;
  int point_cloud_width = kcore->GetPointCloudWidth();
  int point_cloud_height = kcore->GetPointCloudHeight();
  double scale = kcore->GetImageWidth()/point_cloud_width;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_closure_points;   // all points inside of the closure

  int nrClosures = vcore->NumGestalts(Z::Gestalt::CLOSURE);
  for(unsigned i=0; i<nrClosures; i++)
  {
    vector<cv::Vec4f> closure_jcts;                       // all jcts of the closure (hull points)
    vector<cv::Vec4f> closure_points;                     // all points inside of the closure

    Z::Closure *c = (Z::Closure*) vcore->Gestalts(Z::Gestalt::CLOSURE, i);
    if(!c->IsMasked())
    {       
// printf("KinectClosures::Process next unmasked closure!\n");
      int minX = 640, maxX = 0;                 // find the min/max x/y-value of closure (bounding box)
      int minY = 480, maxY = 0;
      for(unsigned i=0; i<c->jcts.Size(); i++)  // get each l-junction from the closure and save min/max
      {
        if(c->jcts[i] != 0)
        {
          if(c->jcts[i]->isct.x < minX) minX = c->jcts[i]->isct.x;
          if(c->jcts[i]->isct.x > maxX) maxX = c->jcts[i]->isct.x;
          if(c->jcts[i]->isct.y < minY) minY = c->jcts[i]->isct.y;
          if(c->jcts[i]->isct.y > maxY) maxY = c->jcts[i]->isct.y;
          
          int x = (int)(c->jcts[i]->isct.x/scale + 0.5);
          int y = (int)(c->jcts[i]->isct.y/scale + 0.5);

          if(x>0 && x < point_cloud_width && y > 0 && y < point_cloud_height)
            closure_jcts.push_back(kcore->GetPointCloud().at<cv::Vec4f>(y, x));
        }
      }

// printf("Process KinectClosures: 1 => min/max x/y: %u-%u / %u-%u!\n", minX, maxX, minY, maxY);

      minX = (int) minX/scale+0.5;
      maxX = (int) maxX/scale+0.5;
      minY = (int) minY/scale+0.5;
      maxY = (int) maxY/scale+0.5;
      
      for(int p_x = minX; p_x <= maxX; p_x++)
      {
        for(int p_y = minY; p_y <= maxY; p_y++)
        {
// printf("Process KinectClosures: 1-1 => min/max x/y: %u-%u!\n", p_x, p_y);

          Vector2 p;
          p.x = p_x*scale;    // TODO Wieder rückrechnen auf 640x480 Bilder? Was wenn Bilder andere Größe haben?
          p.y = p_y*scale;
          
          if(p.x < kcore->GetImageWidth())
            if (c->Inside(p))
              closure_points.push_back(kcore->GetPointCloud().at<cv::Vec4f>(p_y, p_x));
        }
      }
// printf("Process KinectClosures: 2 => min/max x/y: %u-%u / %u-%u!\n", minX, maxX, minY, maxY);
      
      // convert cv-vector point cloud to pcl-point cloud
      pclA::ConvertCvVec2PCLCloud(closure_points, pcl_closure_points);
  // int nr_points = pcl_closure_points.size();
      pclA::RemoveZeros(pcl_closure_points);

      if(pcl_closure_points->size() > minimum_points_per_plane)
      {
//        pclA::ConvertCvVec2PCLCloud(closure_jcts, pcl_closure_jcts);
        
        // make singel SAC segmentation with 0.02m distance
        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;                  // model_coefficients for SAC
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;         // planes from SAC
        pclA::SingleSACModelSegmentation(pcl_closure_points, pcl_plane_clouds, model_coefficients, pcl::SACMODEL_PLANE, false, 1.5, 0.02, 100, minimum_points_per_plane);
// printf("Process KinectClosures: after sac 1!\n");
        
  /// TODO TODO TODO TODO Unused: How many points are valid (in %) => indicates something? pruning with this value?
  // int nr_points_wo =pcl_plane_clouds[0]->points.size();
  // printf("nr_points: %u\n", nr_points);
  // printf("nr_points wo: %u\n", nr_points_wo);
  // printf("nr_points %: %4.2f\n", ((double)nr_points_wo) / ((double)nr_points)*100);
        
        if(pcl_plane_clouds.size() > 0 && pcl_plane_clouds[0]->points.size() > minimum_points_per_plane)
        {
          std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_convex_hulls;
          pclA::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_convex_hulls);
// printf("Process KinectClosures: got hull!\n");

          /// TODO Projektion der Eckpunkte (closure junctions): 
          /// Kann nicht funktionieren, weil wir eine Ebene als Ergebnis berechnet haben. Wenn die Tiefe (z-wert) eines 
          /// Punktes nicht passt, dann wird er normal zur Ebene projeziert und nicht vom Sichtstrahl aus! Es müsste eine
          /// Projektion in Richtung Bildpunkt auf die Ebene geben, damit der Wert richtig wäre!!!
  //        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_closure_jcts_clouds;
  //        pcl_closure_jcts_clouds.push_back(pcl_closure_jcts.makeShared());
//   //        pclF::GetProjectedPoints(pcl_closure_jcts_clouds, model_coefficients, pcl_convex_hulls);
  // printf("Size of pcl_convex_hull: %u\n", pcl_convex_hulls[0]->points.size());

          // convert plane point cloud (inliers) to vector point cloud
          std::vector<cv::Vec4f> cv_vec_plane;
          pclA::ConvertPCLCloud2CvVec(pcl_plane_clouds[0], cv_vec_plane);
          
          if(pcl_convex_hulls.size() >0) 
          {
            std::vector<cv::Vec4f> hull_points;
            pclA::ConvertPCLCloud2CvVec(pcl_convex_hulls[0], hull_points, true);   /// TODO With random color

            Z::Closure3D *c3d = new Z::Closure3D(cv_vec_plane, hull_points);
            kcore->NewGestalt3D(c3d);
            numClosures++;
          }
        }
        else printf("KinectClosures::Process: Warning: Could not fit the plane!!!\n");
      }
      else printf("KinectClosures::Process: Warning: Plane without enough points.\n");
    }
    else printf("KinectClosures::Process: Warning: Closure is masked.\n");
  }
}


}








