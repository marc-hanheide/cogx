/**
 * @file KinectSegments.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D segments from Kinect data.
 */

#include "KinectCore.h"
#include "KinectSegments.h"
//#include "VisionUtils.h"
#include "Segment3D.h"

namespace Z
{

static double MAXIMUM_DISTANCE = 0.05;


/**
 * @brief Constructor of StereoLines: Calculate stereo matching of Lines
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
KinectSegments::KinectSegments(KinectCore *kc, VisionCore *vc) 
                             : KinectBase(kc, vc)
{
  numSegments = 0;
}

/**
 * @brief Clear all arrays.
 */
void KinectSegments::ClearResults()
{
  numSegments = 0;
}


/**
 * @brief Calculate 3D segments from kinect data.
 */
void KinectSegments::Process()
{
  cv::Vec4f last_point;
  int pc_width = kcore->GetPointCloudWidth();
  int pc_height = kcore->GetPointCloudHeight();
  
  double scale = kcore->GetImageWidth()/pc_width;
  int nrSegments = vcore->NumGestalts(Z::Gestalt::SEGMENT);
  
  for(unsigned i=0; i<nrSegments; i++)
  {
    bool firstPoint = true;

    Z::Segment *s = (Z::Segment*) vcore->Gestalts(Z::Gestalt::SEGMENT, i);
    VEC::Vector2 edgePoint2D;
    std::vector<cv::Vec4f> edgels3d;
    std::vector<int> indexes;
    std::vector<cv::Vec4f> edgels3dEdgeSupport;   /// TODO Be careful: EMPTY !!!

// printf("\nsegment: %u\n", i);
    for(unsigned j=0; j<s->edgels.Size(); j++)
    {
      edgePoint2D = s->edgels[j].p / scale;

      if((int) edgePoint2D.x >= 0 && (int) edgePoint2D.x < pc_width && 
         (int) edgePoint2D.y >= 0. && (int) edgePoint2D.y < pc_height)
      {
        cv::Vec4f point3D = kcore->GetPointCloud().at<cv::Vec4f>(edgePoint2D.y, edgePoint2D.x);

        if(point3D[2] > 0.001)    // z-value is positive
        {
          if(firstPoint)
          {
// printf("  edgel %u added\n", j);
            edgels3d.push_back(point3D);
            indexes.push_back(edgePoint2D.y*pc_width + edgePoint2D.x);
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
              indexes.push_back(edgePoint2D.y*pc_width + edgePoint2D.x);
            }
            else 
            {
              if(edgels3d.size() > 1)
              {
// printf("    => new segment, because of broken edge\n");
                Z::Segment3D *s3d = new Z::Segment3D(s->ID(), edgels3d, indexes, edgels3dEdgeSupport);
                kcore->NewGestalt3D(s3d);
              }
              edgels3d.clear();
              indexes.clear();
              last_point = point3D;
            }
            last_point = point3D;
          }
        }
//         else printf("StereoDetectorKinectLines::calc3DSegements: zero point: %4.3f-%4.3f-%4.3f\n", point.x, point.y, point.z);
      }
      else
        printf("KinectSegments::calc3DSegements: point is out of the image space: %u-%u\n", (int) edgePoint2D.x, (int) edgePoint2D.y);
    } 
    
    if(edgels3d.size() > 1)
    {
// printf("    => new segment, because end of edge reached\n");
      Z::Segment3D *s3d = new Z::Segment3D(s->ID(), edgels3d, indexes, edgels3dEdgeSupport);
      kcore->NewGestalt3D(s3d);
    }
  }
}


}








