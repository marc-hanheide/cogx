/**
 * @file KinectPclSegments.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D segments from Kinect data.
 */

#include "KinectCore.h"
#include "KinectPclSegments.h"
#include "Segment3D.h"

namespace Z
{
  
/* Use recalculation of neighborhood for forground/background recalculation */
static bool USE_FG_BG_RECALCULATION = true;
  
/* Maximum allowed distance between neigboring segment points */
static double MAXIMUM_DISTANCE = 0.05;  // 5cm

/* Ignore the first point, which is out of the maximum np_distance and go further with the next point */
// static bool IGNORE_FIRST_ERROR = true;

/* Maximum allowed z-distance between neighboring foreground pixel and segment pixel */
static double MAXIMUM_NP_DISTANCE = 0.03;  // 5cm


/**
 * @brief Constructor of KinectPclSegments: Calculate stereo matching of Lines
 * @param kc Kinect core
 * @param vc Vision core
 */
KinectPclSegments::KinectPclSegments(KinectCore *kc, VisionCore *vc) 
                 : KinectBase(kc, vc)
{
  numSegments = 0;
  tgEngine = NULL;
  rgbdSegment = NULL;
}

/**
 * @brief Clear all arrays.
 */
void KinectPclSegments::ClearResults()
{
  numSegments = 0;
}

/**
 * @brief Get the right of the pixel-direction if 8-neighborhood
 */
void KinectPclSegments::RightOf(int dirX, int dirY, int &dX, int &dY)
{
  dX = -dirY;
  dY = dirX;
}

void KinectPclSegments::LeftOf(int dirX, int dirY, int &dX, int &dY)
{
  dX = dirY;
  dY = -dirX;
}


/**
 * @brief Calculate 3D segments from kinect data.
 * @param x X-coordinate
 * @param y Y-coordinate
 * @return Returns support vector for color-,depth-, mask- and curvature- edges
 */
cv::Vec4f KinectPclSegments::GetSupportVector(int x, int y)
{
  cv::Vec4f edge_support;
  edge_support[0] = color_edges(y, x);
  edge_support[1] = depth_edges(y, x);
  edge_support[2] = mask_edges(y, x);
  edge_support[3] = curvature_edges(y, x);
  return edge_support;
}

/**
 * @brief Calculate 3D segments from kinect data.
 */
void KinectPclSegments::Process()
{
  double fx, fy, cx, cy;
  kcore->GetScaledCameraParameters(fx, fy, cx, cy);
  int pc_width = kcore->GetPointCloudWidth();
  int pc_height = kcore->GetPointCloudHeight();
  int rows = pc_height;
  int cols = pc_width;
  double scale = kcore->GetScale();
  
  // ------------------------------------------------------------------- //
  // Get tomGine edge-images with color, depth, mask and curvature edges //
  // ------------------------------------------------------------------- //
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc;
  cv::Mat_<cv::Vec4f> normals = kcore->GetCvNormals();
  pclA::CopyPointCloud(kcore->GetPclCloud(), pcl_pc);

  float zMin = 0.3;                   // 0.3
  float zMax = 2.3;                   // 1.3
  float weightColorEdge = 0.05;       // 0.05
  float weightDepthEdge = 10;         // 4
  float weightCurvatureEdge = 40.;    // 40
  float weightMaskEdge = 0.8;         // 0.8
  RGBDSegment::Parameter p = RGBDSegment::Parameter(zMin, zMax, weightColorEdge, weightDepthEdge,
                          weightCurvatureEdge, weightMaskEdge, 50.0, 25.0, 0.02, 20, 1);

  if(tgEngine == NULL)
    tgEngine = new TomGine::tgEngine(cols, rows);
  if(rgbdSegment == NULL)
    rgbdSegment = new RGBDSegment(tgEngine, p);

  // Filter points with z-value
  pclA::FilterZ(pcl_pc, zMin, zMax);
  
  color_edges = cv::Mat_<float>(rows, cols);
  depth_edges = cv::Mat_<float>(rows, cols);
  curvature_edges = cv::Mat_<float>(rows, cols);
  mask_edges = cv::Mat_<float>(rows, cols);
  edges = cv::Mat_<float>(rows, cols);
  
  // get image and mask
  cv::Mat_<cv::Vec3b> image = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertPCLCloud2Image(pcl_pc, image);
  cv::Mat_<uchar> mask = cv::Mat_<uchar>(rows, cols);
  pclA::ConvertPCLCloud2Mask(pcl_pc, mask, false, false);
 
  cv::Mat_<float> depth = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> curvature = cv::Mat_<float>(rows, cols);
  rgbdSegment->DetectEdges(kcore->GetPointCloud(), normals, image, mask, depth, curvature, color_edges, depth_edges, curvature_edges, mask_edges, edges);

  // --------------------------------------------------------------------------------- //
  // Calculate now the segments and estimate color, depth, mask and curvature support! //
  // --------------------------------------------------------------------------------- //
  cv::Vec4f last_point;
  int nrSegments = vcore->NumGestalts(Z::Gestalt::SEGMENT);
  for(unsigned i=0; i<nrSegments; i++)
  {
    bool firstPoint = true;

    Z::Segment *s = (Z::Segment*) vcore->Gestalts(Z::Gestalt::SEGMENT, i);
    VEC::Vector2 edgePoint2D, nextEdgePoint2D;
    std::vector<cv::Vec4f> edgels3d;
    std::vector<int> indexes;                    // 2D image indexes
    std::vector<cv::Vec4f> edgels3dEdgeSupport;  // color, depth, mask, curvature

    for(unsigned j=0; j<s->edgels.Size(); j++)
    {
      edgePoint2D = s->edgels[j].p / scale;
// printf("original edgePoint2D: %4.1f-%4.1f\n", edgePoint2D.x, edgePoint2D.y);
      int dirX, dirY;
      if(j < s->edgels.Size()-1)
      {
        nextEdgePoint2D = s->edgels[j+1].p / scale;
        dirX = (int) ((nextEdgePoint2D.x - edgePoint2D.x)*scale);
        dirY = (int) ((nextEdgePoint2D.y - edgePoint2D.y)*scale);
      }
      else 
      {
        nextEdgePoint2D = s->edgels[j-1].p / scale;   // point back to former pixel
        dirX = (int) (edgePoint2D.x - nextEdgePoint2D.x)*scale;     // change direction, otherwise left is right and right is left.
        dirY = (int) (edgePoint2D.y - nextEdgePoint2D.y)*scale;
      }
// printf("next: %u of %u => dir: %i - %i\n", j, s->edgels.Size(), dirX, dirY);

//       if((int) edgePoint2D.x >= 0 && (int) edgePoint2D.x < pc_width && (int) edgePoint2D.y >= 0. && (int) edgePoint2D.y < pc_height)  // TODO weg mit dieser Abfrage => Kann nicht vorkommen?
//       {
        cv::Vec4f point3D = kcore->GetPointCloud().at<cv::Vec4f>(edgePoint2D.y, edgePoint2D.x);
        if(point3D[2] > 0.001 && point3D[2] == point3D[2])    // Zero, NAN or invalid negative value?
        {
          bool sucessful_fg_bg_recalc = false;
          if(!firstPoint && USE_FG_BG_RECALCULATION)
          {
            // calculate, if left or right point is nearer to the cam (z-coordinate)
            int rdX, rdY; // right dx, dy
            int ldX, ldY; // left dx, dy
            RightOf(dirX, dirY, rdX, rdY);
            LeftOf(dirX, dirY, ldX, ldY);
            cv::Vec4f nearerPointToCam;
            cv::Vec4f rightPoint3D = kcore->GetPointCloud().at<cv::Vec4f>(edgePoint2D.y + rdY, edgePoint2D.x + rdX);
            cv::Vec4f leftPoint3D = kcore->GetPointCloud().at<cv::Vec4f>(edgePoint2D.y + ldY, edgePoint2D.x + ldX);
  
            bool valid_points = true;
            if(rightPoint3D[0] != rightPoint3D[0])
            {
              if(leftPoint3D[0] != leftPoint3D[0])        // both are not valid
                valid_points = false;
              else nearerPointToCam = leftPoint3D;        // left is valid
            } 
            else if(leftPoint3D[0] != leftPoint3D[0])     // right is valid
            {
              nearerPointToCam = rightPoint3D;
            }
            else                                          // both are valid
            {
              if(rightPoint3D[2] < leftPoint3D[2])
                nearerPointToCam = rightPoint3D;
              else 
                nearerPointToCam = leftPoint3D;
            }
            
            if(valid_points)
            {
              if((point3D[2] - nearerPointToCam[2]) > MAXIMUM_NP_DISTANCE)
              {
// printf("Point3D: %4.3f - %4.3f - %4.3f\n", point3D[0], point3D[1], point3D[2]);
// printf("XXXXXXXXXXXXXXXXXXX Valid points and NP: %4.3f\n", point3D[2] - nearerPointToCam[2]);
    // printf("######### KinectPclSegments: Recalculate point with fg-bg distance: %4.3f!\n", nearerPointToCam[2] - point3D[2]);
                double camRay_x, camRay_y;
                camRay_x = (float)(edgePoint2D.x-cx)/fx;
                camRay_y = (float)(edgePoint2D.y-cy)/fy;
                point3D[0] = camRay_x * nearerPointToCam[2];
                point3D[1] = camRay_y * nearerPointToCam[2];
                point3D[2] = nearerPointToCam[2];
//                point3D[3] = 1.;                                /// TODO Red color!
                sucessful_fg_bg_recalc = true;

              }
    //         else
    //          printf("KinectPclSegments: Now we have a big problem: %4.3f!\n", nearerPoint[2] - point3D[2]);
            }
          }
  
          if(firstPoint)
          {
// printf("  edgel %u added\n", j);
            edgels3d.push_back(point3D);
            int pos = (int) edgePoint2D.y*pc_width + edgePoint2D.x;
// printf("edgePoint2D: %4.1f-%4.1f\n", edgePoint2D.x, edgePoint2D.y);
// printf(" push_back: %u\n", pos);
            indexes.push_back(pos);
            edgels3dEdgeSupport.push_back(GetSupportVector(edgePoint2D.x, edgePoint2D.y));
            last_point = point3D;
            firstPoint = false;
          }
          else
          {
            double dist = fabs(point3D[0] - last_point[0]) + fabs(point3D[1] - last_point[1]) + fabs(point3D[2] - last_point[2]);
            if(dist != 0.) // same 3D point
            {  
              if(dist < MAXIMUM_DISTANCE)
              {
  // printf("  edgel %u added\n", j);
                edgels3d.push_back(point3D);
                int pos = (int) edgePoint2D.y*pc_width + edgePoint2D.x;
  // printf("edgePoint2D: %4.1f-%4.1f\n", edgePoint2D.x, edgePoint2D.y);

/*                if(indexes[indexes.size()-1] != pos){  // push back when other point found*/
  // printf(" push_back: %u\n", pos);
                indexes.push_back(pos);
                edgels3dEdgeSupport.push_back(GetSupportVector(edgePoint2D.x, edgePoint2D.y));
              }
              else /// FEHLER GEFUNDEN
              {
                if(edgels3d.size() > 1)
                {
                  
  // if(indexes.size() == 86)
  // {
  //   for(unsigned i=0; i<indexes.size(); i++)
  //     printf("seg indexes: %u => %u - %u\n", indexes[i], indexes[i]%320, indexes[i]/320);
  // }

  // printf("    => new segment, because of broken edge => no first error\n");
                  Z::Segment3D *s3d = new Z::Segment3D(s->ID(), edgels3d, indexes, edgels3dEdgeSupport);
                  kcore->NewGestalt3D(s3d);
                }
                edgels3d.clear();
                indexes.clear();
                edgels3dEdgeSupport.clear();
                last_point = point3D;
              }
              last_point = point3D;
            }
          }
        }
//         else printf(" zero, nan or negative point: %4.3f-%4.3f-%4.3f\n", point3D[0], point3D[1], point3D[2]);
//       }
//       else // That should not happen, otherwise its wrong!!!
//         printf("\n\nKinectPclSegments::Process: point is out of the image space: %u-%u\n\n\n", (int) edgePoint2D.x, (int) edgePoint2D.y);
    }
    
    if(edgels3d.size() > 1)
    {
// if(indexes.size() == 86)
// {
//   for(unsigned i=0; i<indexes.size(); i++)
//     printf("seg indexes: %u => %u - %u\n", indexes[i], indexes[i]%320, indexes[i]/320);
// }
// printf("    => new segment, because end of edge reached: %u edgels\n", edgels3d.size());
      Z::Segment3D *s3d = new Z::Segment3D(s->ID(), edgels3d, indexes, edgels3dEdgeSupport);
      kcore->NewGestalt3D(s3d);
    }
  }
}


}








