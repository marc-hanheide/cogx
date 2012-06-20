/**
 * @file KinectRectangles.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Calculate 3D rectangles from Kinect data.
 */

#include "KinectCore.h"
#include "KinectRectangles.h"
//#include "VisionUtils.h"

#include "Rectangle3D.h"

namespace Z
{


/**
 * @brief Constructor of class KinectRectangles
 * @param kc Kinect core
 * @param vc Vision core
 * @param iplI Ipl-Image
 * @param p Point matrix
 */
KinectRectangles::KinectRectangles(KinectCore *kc, VisionCore *vc) 
                : KinectBase(kc, vc)
{
  numRectangles = 0;
}

/**
 * @brief Clear all arrays.
 */
void KinectRectangles::ClearResults()
{
  numRectangles = 0;
}


/**
 * @brief Calculate 3D rectangles from kinect data.
 */
void KinectRectangles::Process()
{
  int minimum_points_per_plane = 15;
  int point_cloud_width = kcore->GetPointCloudWidth();
  int point_cloud_height = kcore->GetPointCloudHeight();
  double scale = kcore->GetImageWidth()/point_cloud_width;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rectangle_points;   // all points inside of the rectangle
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rectangle_jcts;     // all jcts of the rectangle (hull points)

  int nrRectangles = vcore->NumGestalts(Z::Gestalt::RECTANGLE);
  for(unsigned i=0; i<nrRectangles; i++)
  {
// printf("\nKinectRectangles::Process next rectangle!\n");
    vector<cv::Vec4f> rectangle_jcts;                       // all jcts of the rectangle (hull points)
    vector<cv::Vec4f> rectangle_points;                     // all points inside of the rectangle

    Z::Rectangle *r = (Z::Rectangle*) vcore->Gestalts(Z::Gestalt::RECTANGLE, i);
//     if(!r->IsMasked())
    if(true)
    {    
      int minX = 640, maxX = 0;                 // find the min/max x/y-value of closure (bounding box)
      int minY = 480, maxY = 0;
      for(unsigned i=0; i<4; i++)  // get each l-junction from the closure and save min/max
      {
        if(r->isct[i].x < minX) minX = r->isct[i].x;
        if(r->isct[i].x > maxX) maxX = r->isct[i].x;
        if(r->isct[i].y < minY) minY = r->isct[i].y;
        if(r->isct[i].y > maxY) maxY = r->isct[i].y;
        
        int x = (int)(r->isct[i].x/scale + 0.5);
        int y = (int)(r->isct[i].y/scale + 0.5);

        if(x>0 && x < point_cloud_width && y > 0 && y < point_cloud_height)
          rectangle_jcts.push_back(kcore->GetPointCloud().at<cv::Vec4f>(y, x));
      }
        
      minX = (int) minX/scale+0.5;
      maxX = (int) maxX/scale+0.5;
      minY = (int) minY/scale+0.5;
      maxY = (int) maxY/scale+0.5;
      
      for(int p_x = minX; p_x <= maxX; p_x++)
      {
        for(int p_y = minY; p_y <= maxY; p_y++)
        {
          Vector2 p;
          p.x = p_x*scale;
          p.y = p_y*scale;
          if (r->Inside(p))
            rectangle_points.push_back(kcore->GetPointCloud().at<cv::Vec4f>(p_y, p_x));
        }
      }
      
      // convert cv-vector point cloud to pcl-point cloud
      pclA::ConvertCvVec2PCLCloud(rectangle_points, pcl_rectangle_points);
  // int nr_points = pcl_rectangle_points.size();
      pclA::RemoveZeros(pcl_rectangle_points);

      if(pcl_rectangle_points->size() > minimum_points_per_plane)
      {
//        pclA::ConvertCvVec2PCLCloud(rectangle_jcts, pcl_rectangle_jcts);
        
        // make singel SAC segmentation with 0.02m distance
        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;                  // model_coefficients for SAC
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;         // planes from SAC
        pclA::SingleSACModelSegmentation(pcl_rectangle_points, pcl_plane_clouds, model_coefficients, pcl::SACMODEL_PLANE, false, 1.5, 0.02, 100, minimum_points_per_plane);
        
  /// TODO TODO TODO TODO Unused: How many points are valid (in %) => indicates something? pruning with this value?
  // int nr_points_wo =pcl_plane_clouds[0]->points.size();
  // printf("nr_points: %u\n", nr_points);
  // printf("nr_points wo: %u\n", nr_points_wo);
  // printf("nr_points %: %4.2f\n", ((double)nr_points_wo) / ((double)nr_points)*100);
        
        if(pcl_plane_clouds.size() > 0 && pcl_plane_clouds[0]->points.size() > minimum_points_per_plane)
        {
          std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_convex_hulls;
          pclA::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_convex_hulls);

          /// TODO Projektion der Eckpunkte (closure junctions): 
          /// Kann nicht funktionieren, weil wir eine Ebene als Ergebniss berechnet haben. Wenn die Tiefe (z-wert) eines 
          /// Punktes nicht passt, dann wird er normal zur Ebene projeziert und nicht vom Sichtstrahl aus! Es müsste eine
          /// Projektion in Richtung Bildpunkt auf die Ebene geben, damit der Wert richtig wäre!!!
  //        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_closure_jcts_clouds;
  //        pcl_closure_jcts_clouds.push_back(pcl_closure_jcts.makeShared());
  //        pclF::GetProjectedPoints(pcl_closure_jcts_clouds, model_coefficients, pcl_convex_hulls);
  // printf("Size of pcl_convex_hull: %u\n", pcl_convex_hulls[0]->points.size());
          
          // convert plane point cloud (inliers) to vector point cloud
          std::vector<cv::Vec4f> cv_vec_plane;
          pclA::ConvertPCLCloud2CvVec(pcl_plane_clouds[0], cv_vec_plane);
          
          if(pcl_convex_hulls.size() >0) 
          {
            std::vector<cv::Vec4f> hull_points;
            pclA::ConvertPCLCloud2CvVec(pcl_convex_hulls[0], hull_points, true);         /// TODO Random color

            Z::Rectangle3D *r3d = new Z::Rectangle3D(cv_vec_plane, hull_points);
            kcore->NewGestalt3D(r3d);
            numRectangles++;
          }
        }
        else printf("KinectRectangles::Process: Warning: Could not fit the plane!!!\n");
      }
//       else printf("KinectRectangles::Process: Warning: Plane without enough points.\n");
    }
    else printf("KinectRectangles::Process: Warning: Rectangle is masked..\n");
  }
}


}








