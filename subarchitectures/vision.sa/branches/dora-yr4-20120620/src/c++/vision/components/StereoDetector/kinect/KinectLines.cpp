/**
 * @file KinectLines.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D lines from Kinect data.
 */

// // #include <time.h>
#include "KinectCore.h"
#include "KinectLines.h"
// #include "StereoTypes.h"
#include "Line3D.h"

namespace Z
{

static double MAXIMUM_DISTANCE = 0.05;

/**
 * @brief Constructor of StereoLines: Calculate stereo matching of Lines
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
KinectLines::KinectLines(KinectCore *kc, VisionCore *vc) 
                       : KinectBase(kc, vc)
{
  numLines = 0;
}


/**
 * @brief Draw matched Lines.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
// void StereoLines::DrawMatched(int side, bool single, int id, int detail)
// {
//   if(single)
//   {
//     if(id < 0 || id >= lineMatches)
//     {
//       printf("StereoLines::DrawMatched: warning: id out of range!\n");
//       return;
//     }
//     DrawSingleMatched(side, id, detail);
//   }
//   else
//     for(int i=0; i< lineMatches; i++)
//       DrawSingleMatched(side, i, detail);
// }

/**
 * @brief Draw single matched line.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
// void StereoLines::DrawSingleMatched(int side, int id, int detail)
// {
//   lines[side][id].Draw(detail);
// }


/**
 * @brief Clear all arrays.
 */
void KinectLines::ClearResults()
{
  numLines = 0;
}


/**
 * @brief Calculate 3D lines from kinect data.
 */
void KinectLines::Process()
{
  cv::Vec4f last_point;
  int point_cloud_width = kcore->GetPointCloudWidth();
  int point_cloud_height = kcore->GetPointCloudHeight();
  double scale = kcore->GetImageWidth()/point_cloud_width;
  int nrLines = vcore->NumGestalts(Z::Gestalt::LINE);

// printf("KinectLines::Process: We have %u lines!\n", nrLines);
// printf("KinectLines::Process: point-cloud: %u-%u\n", point_cloud_width, point_cloud_height);

  for(unsigned i=0; i<nrLines; i++)
  {
    bool firstPoint = true;

    Z::Line *l = (Z::Line*) vcore->Gestalts(Z::Gestalt::LINE, i);
    VEC::Vector2 edgePoint2D;
    std::vector<cv::Vec4f> edgels3d;

// printf("\nline: %u\n", i);
    for(unsigned j=l->idx[0]; j<l->idx[1]; j++)
    {
      edgePoint2D = l->seg->edgels[j].p / scale;

      if((int) edgePoint2D.x >= 0 && (int) edgePoint2D.x < point_cloud_width && (int) edgePoint2D.y >= 0. && (int) edgePoint2D.y < point_cloud_height)
      {
        cv::Vec4f point3D = kcore->GetPointCloud().at<cv::Vec4f>(edgePoint2D.y, edgePoint2D.x);

        if(point3D[2] > 0.001)    // z-value is positive
        {
          if(firstPoint)
          {
// printf("  edgel %u added\n", j);
            edgels3d.push_back(point3D);
            last_point = point3D;
            firstPoint = false;
          }
          else
          {
            double dist = fabs(point3D[0] - last_point[0]) + fabs(point3D[1] - last_point[1]) + fabs(point3D[2] - last_point[2]);

            if(dist < MAXIMUM_DISTANCE)
            {
// printf("  edgel %u added\n", j);
              edgels3d.push_back(point3D);
            }
            else 
            {
              if(edgels3d.size() > 1)
              {
// printf("    => new segment, because of broken edge\n");

                // when start and end point is different
                if((edgels3d[0][0] - edgels3d[edgels3d.size()-1][0]) != 0 ||
                   (edgels3d[0][1] - edgels3d[edgels3d.size()-1][1]) != 0 ||
                   (edgels3d[0][2] - edgels3d[edgels3d.size()-1][2]) != 0)
                {
// printf("p: %4.3f-%4.3f-%4.3f\n", edgels3d[0][0] - edgels3d[edgels3d.size()-1][0], edgels3d[0][1] - edgels3d[edgels3d.size()-1][1], edgels3d[0][2] - edgels3d[edgels3d.size()-1][2]);
                  Z::Line3D *l3d = new Z::Line3D(l->ID(), edgels3d);
                  kcore->NewGestalt3D(l3d);
                }
              }
              edgels3d.clear();
              last_point = point3D;
            }
            last_point = point3D;
          }
        }
//         else printf("StereoDetectorKinectLines::calc3DSegements: zero point: %4.3f-%4.3f-%4.3f\n", point.x, point.y, point.z);
      }
      else
        printf("StereoDetectorKinectLines::calc3DSegements: point is out of the image space: %u-%u\n", (int) edgePoint2D.x, (int) edgePoint2D.y);
    }
    
    if(edgels3d.size() > 1)
    {          
      if((edgels3d[0][0] - edgels3d[edgels3d.size()-1][0]) != 0 ||
         (edgels3d[0][1] - edgels3d[edgels3d.size()-1][1]) != 0 ||
         (edgels3d[0][2] - edgels3d[edgels3d.size()-1][2]) != 0)
      {
// printf("p: %4.3f-%4.3f-%4.3f\n", edgels3d[0][0] - edgels3d[edgels3d.size()-1][0], edgels3d[0][1] - edgels3d[edgels3d.size()-1][1], edgels3d[0][2] - edgels3d[edgels3d.size()-1][2]);
// printf("    => new segment, because end of edge reached\n");
        Z::Line3D *l3d = new Z::Line3D(l->ID(), edgels3d);
        kcore->NewGestalt3D(l3d);
      }
    }
  }
}


}








