/**
 * @file KinectLines.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D lines from Kinect data.
 */

#ifndef Z_KINECT_LINES_H
#define Z_KINECT_LINES_H

// #include <time.h>
// #include <vector>
// #include <list>
// #include <map>

#include "KinectBase.h"
#include "VisionCore.hh"

// #include "CreateMSLD.hh"
// #include "vs3/Line.hh"
// #include "math/Vector.hh"
// #include "math/Math.hh"

// #include "StereoCamera.h"
// #include "Line3D.h"

#include "Line.hh"


namespace Z
{

/**
 * @brief Class KinectLines: Calculate 3D lines from kinect data.
 */
class KinectLines : public KinectBase
{
private:
  int numLines;              ///< Number of extracted 3D lines
  
//   Array<TmpLine> lines[2];    ///< Tmp. lines from the vision cores.
//   P::CreateMSLD createMSLD;
// 
// #ifdef HAVE_CAST
//   bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
// #endif
// 
//   unsigned EpipolarSanityCheck(unsigned idx, std::map<float, unsigned> &match_map, unsigned nr = 3);
//   void ExtendDescriptors(unsigned idx, std::map<float, unsigned> &match_map, unsigned nr = 3);
//   void MatchLines(std::vector< std::vector<float> > descr_left, 
// 	          std::vector< std::vector<float> > descr_right, 
//                   std::vector< std::pair<unsigned, unsigned> > &matches);
//   void ProcessMSLD(std::vector< std::vector<float> > *descriptors);
//   void SortLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, std::vector< std::pair<unsigned, unsigned> > &matches);
//   void Calculate3DLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, int &matches);
//   bool Prune3DLines(Line3D *line3d);
//   void DrawSingleMatched(int side, int id, int detail);
//   void GetUnsplitedLines();

public:
  KinectLines(KinectCore *kc, VisionCore *vc, IplImage *iplI, cv::Mat_<cv::Vec4f> &p);
  ~KinectLines() {}

  int Num3DGestalts() {return numLines;}
  void Get3DGestalt(Array<double> &values, int id) {}

//  Line* Lines2D(int side, int id) {return Lines(vcore[side], id);}       ///< Return lines from left/right image

//  int NumStereoMatches() {return lineMatches;}                           ///< Return number of stereo matches
//   void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  
};

}

#endif
