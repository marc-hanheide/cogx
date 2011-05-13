/**
 * @file StereoLines.h
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Stereo calculation of lines with MSLD-descriptor.
 */

#ifndef Z_STEREO_LINES_HH
#define Z_STEREO_LINES_HH

#include <time.h>
#include <vector>
#include <list>
#include <map>

#include "CreateMSLD.hh"
#include "vs3/Line.hh"
#include "math/Vector.hh"
#include "math/Math.hh"

#include "StereoBase.h"
#include "StereoCamera.h"
#include "Line3D.h"


namespace Z
{

/**
 * @brief Class TmpLine
 */
class TmpLine
{
private:
  bool rectified;
  
public:
  Vertex2D point2D[2];              ///< Start/End point of the 2D line.
  unsigned vs3ID;                   ///< ID of the vs3 line
//  Vector2 dir[2];                ///< Angles of the two arms

  TmpLine() {}
  TmpLine(Line *line);
  void Draw(int detail);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  bool IsValid() {return true;}                         // TODO is always valid
};


/**
 * @brief Class StereoLines: Try to match lines from the stereo images with msld-algorithm.
 */
class StereoLines : public StereoBase
{
private:

  Array<TmpLine> lines[2];    ///< Tmp. lines from the vision cores.
  int lineMatches;            ///< Number of stereo matched lines
  P::CreateMSLD createMSLD;

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  unsigned EpipolarSanityCheck(unsigned idx, std::map<float, unsigned> &match_map, unsigned nr = 3);
  void ExtendDescriptors(unsigned idx, std::map<float, unsigned> &match_map, unsigned nr = 3);
  void MatchLines(std::vector< std::vector<float> > descr_left, 
	          std::vector< std::vector<float> > descr_right, 
                  std::vector< std::pair<unsigned, unsigned> > &matches);
  void ProcessMSLD(std::vector< std::vector<float> > *descriptors);
  void SortLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, std::vector< std::pair<unsigned, unsigned> > &matches);
  void Calculate3DLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, int &matches);
  bool Prune3DLines(Line3D *line3d);
  void DrawSingleMatched(int side, int id, int detail);
  void GetUnsplitedLines();

public:
  StereoLines(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
  ~StereoLines() {}

  int NumLines2D(int side) {return lines[side].Size();}                  ///< Return number of tmp. 2D lines
  Line* Lines2D(int side, int id) {return Lines(vcore[side], id);}       ///< Return lines from left/right image

  int NumStereoMatches() {return lineMatches;}                           ///< Return number of stereo matches
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif
