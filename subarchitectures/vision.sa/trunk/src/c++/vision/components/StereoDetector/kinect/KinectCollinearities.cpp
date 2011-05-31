/**
 * @file KinectCollinearities.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D collinearities from Kinect data.
 */

#include "KinectCollinearities.h"
#include "KinectCore.h"
// #include "StereoTypes.h"
#include "Collinearity3D.h"

namespace Z
{

static double MAXIMUM_DISTANCE = 0.05;

/**
 * @brief Constructor of StereoLines: Calculate stereo matching of Lines
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
KinectCollinearities::KinectCollinearities(KinectCore *kc, VisionCore *vc, IplImage *iplI, cv::Mat_<cv::Vec4f> &p) 
                       : KinectBase(kc, vc, iplI, p)
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
// void KinectCollinearities::DrawMatched(int side, bool single, int id, int detail)
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
// void KinectCollinearities::DrawSingleMatched(int side, int id, int detail)
// {
//   lines[side][id].Draw(detail);
// }


/**
 * @brief Clear all arrays.
 */
void KinectCollinearities::ClearResults()
{
  numLines = 0;
}

/**
 *  @brief Calculate a single 3D Line from the kinect depth data.
 *  @param line
 *  @param width_2d Width of the image from the camera
 *  @param width_3d Width of the point cloud
 *  @param line_start
 *  @param line_end
 *  
 *  TODO we need still the kinect point cloud global
 */
// bool KinectCollinearities::calc3DLine(Z::Line *line, double width_2d, double width_3d, cv::Point3f &line_start, cv::Point3f &line_end)
// {
//   VEC::Vector2 start, end;          // start/end point of line in 2D image
//   double scale = width_2d/width_3d;
//   
//   start = line->point[0];
//   end = line->point[1];
//   start = start/scale;
//   end = end/scale;
//   
//   int position = (int) ((int)(start.y)*width_3d + (int)(start.x));
//   int next_position = (int) ((int)(end.y)*width_3d + (int)(end.x));
//   line_start = points.at<cv::Point3f>(0, position);
//   line_end = points.at<cv::Point3f>(0, next_position);
//   
//   if(line_start.z < 0.001 || line_end.z < 0.001)
//     return false;
//   else return true;
// }

/**
 * @brief Check the line for "holes". Go from edgel to edgel and check distance.
 * When distance is too big, split the line and take only a part of the line
 */
bool KinectCollinearities::CheckLineValidity(Z::Collinearity *col, Z::Line3D *line[2])
{
// printf("\nKinectCollinearities::CheckLineValidity: start!\n");

  bool succeed[2] = {false, false};
  cv::Vec4f last_point;
  bool firstPoint = true;
  std::vector<cv::Vec4f> edgels3d;
  
  for(unsigned side=0; side <= 1; side++)   // get left/right line
  {
    bool line_finished = false;
    edgels3d.clear();

    Z::Line *l = col->line[side];
    unsigned start_index = col->line[side]->idx[col->near_point[side]];
    unsigned end_index = col->line[side]->idx[Other(col->near_point[side])];

    int crem = 1;   // increment or decrement
    if(start_index > end_index) crem = -1;
// printf(" Start/end index: %u to %u\n", start_index, end_index);

    for(unsigned idx = start_index; idx != end_index+crem; idx += crem)
    {
      VEC::Vector2 edgePoint2D = col->line[side]->seg->edgels[idx].p / scale;
      cv::Vec4f point3D = points.at<cv::Vec4f>(edgePoint2D.y, edgePoint2D.x);

      if(point3D[2] > 0.001 && !line_finished)    // z-value is positive
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
// printf("    => new line, because of broken edge => finished\n");
//                 line0 = new Z::Line3D(col->line[0]->ID(), edgels3d);
              line_finished = true;
            }
//               edgels3d.clear();
//               last_point = point3D;
          }
          last_point = point3D;
        }
      }
    }

    if(edgels3d.size() > 1 /*&& line_finished*/)
    {
// printf("    => new line, because end of edge reached\n");
      line[side] = new Z::Line3D(col->line[0]->ID(), edgels3d);
      succeed[side] = true;
    }
  }  
  
  if(succeed[0] && succeed[1]) return true;
  return false;
}


/**
 * @brief Calculate 3D lines from kinect data.
 */
void KinectCollinearities::Process()
{
  bool succeed3D = true;
  
  int point_cloud_width = points.cols;
  int point_cloud_height = points.rows;
  scale = iplImg->width/point_cloud_width;
  
  // Get collinearities
  unsigned nrCols = vcore->NumGestalts(Z::Gestalt::COLLINEARITY);
  for(unsigned idx=0; idx<nrCols; idx++)
  {
// printf("\n NOW new line!");

    // Check, if the two lines are valid!
    Z::Collinearity *col = (Z::Collinearity*) vcore->Gestalts(Z::Gestalt::COLLINEARITY, idx);
    Z::Line3D *line[2];
    succeed3D = CheckLineValidity(col, line);

    if(succeed3D)
    {
// printf("Succeed:\n points:\n %4.3f-%4.3f-%4.3f and\n %4.3f-%4.3f-%4.3f and\n %4.3f-%4.3f-%4.3f\n", 
//              line[0]->point[0][0], line[0]->point[0][1], line[0]->point[0][2], 
//              line[0]->point[1][0], line[0]->point[1][1], line[0]->point[1][2], 
//              line[1]->point[1][0], line[1]->point[1][1], line[1]->point[1][2]);
             
      double distance = pow(line[0]->point[0][0] - line[1]->point[0][0], 2) +
                        pow(line[0]->point[0][1] - line[1]->point[0][1], 2) +
                        pow(line[0]->point[0][2] - line[1]->point[0][2], 2);
      distance = sqrt(distance);
      if(distance > MAXIMUM_DISTANCE)
      {
// printf("distance between end points for col %u: %4.4f\n", idx, distance);
        succeed3D = false;
      }
                        
// TODO Wieder aktivieren: Compare direction of lines!
      cv::Point3f dir0, dir1;
      dir0.x = line[0]->point[0][0] - line[0]->point[1][0];
      dir0.y = line[0]->point[0][1] - line[0]->point[1][1];
      dir0.z = line[0]->point[0][2] - line[0]->point[1][2];
      dir1.x = line[1]->point[1][0] - line[1]->point[0][0];
      dir1.y = line[1]->point[1][1] - line[1]->point[0][1];
      dir1.z = line[1]->point[1][2] - line[1]->point[0][2];

      VEC::Vector3 d0, d1;              // direction of lines as Vector3
      double n0 = cv::norm(dir0);       // norm of line
      d0.x = dir0.x/n0;
      d0.y = dir0.y/n0;
      d0.z = dir0.z/n0;
      double n1 = cv::norm(dir1);       // norm of line
      d1.x = dir1.x/n1;
      d1.y = dir1.y/n1;
      d1.z = dir1.z/n1;

      // compare difference of directions
      VEC::Vector3 dir_diff = d0 - d1;
      if(fabs(dir_diff.x) > M_PI/8. && fabs(dir_diff.y) > M_PI/8. && fabs(dir_diff.z) > M_PI/8.)  // TODO Maximum angle between lines => Pi/8 = 22,5°
      {
// printf("difference in angle for col %u: %4.4f / %4.4f / %4.4f\n", idx, dir_diff.x, dir_diff.y, dir_diff.z);
        succeed3D = false;      
      }

      if(succeed3D)
      {
        Z::Collinearity3D *c3d = new Z::Collinearity3D(col->ID(), line);
        kcore->NewGestalt3D(c3d);
      }
    }
  }
}


}








