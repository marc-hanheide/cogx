/**
 * @file StereoEllipses.h
 * @author Andreas Richtsfeld
 * @date Dec. 2009, June 2010
 * @version 0.1
 * @brief Stereo calculation of ellipses.
 */

#ifndef Z_STEREO_ELLIPSES_HH
#define Z_STEREO_ELLIPSES_HH

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 * @brief Class TmpEllipse
 */
class TmpEllipse
{
public:
  unsigned id;                                          ///< ID from the vision core																		/// TODO TODO TODO Überall integrieren zum Nachverfolgen
  Vertex2D center;                                      ///< Center point of the ellipse (fix point)
  Vertex2D hullPoint[6];                                ///< Points on the ellipse (for rectifying and refitting an ellipse)
  bool isLeftOfCenter[6];                               ///< True, if hullPoint is left of center
  double a, b, phi;                                     ///< Ellipse parameters

  TmpEllipse() {}
  TmpEllipse(Ellipse *ellipse);
  void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
//   bool IsAtPosition(int x, int y) const;
//   void Fuddle(unsigned off0);
  bool IsValid() {return true;/*surf.is_valid;*/}        /// TODO Wenn nach initialisieren valid, dann wird zu ellipses[2] hinzugefügt
};


/**																														/// TODO Base class for all 3D-Gestalts?
 * @brief Class Ellipse3D
 * NOTE: Hypothesis of a circle!
 */
class Ellipse3D
{
public:
  int tmpID;                                             ///< Position of the 2D features in ellipses[2]				// TODO TODO überall integrieren, zum nachverfolgen?
  Vertex3D center;                                       ///< 3D center point of the circle with the normal
  double radius;                                         ///< Radius of the circle
  double significance;                                   ///< Significance value of the estimated circle
  double x_e, y_e, a_e, b_e, phi_e;                      ///< New fitted ellipse in left rectified image
  double k, d[6];						///< gradient and offest of turned straigt line
  double y_g[6];						///< y value of the lines from the right ellipse hull points (is also y of solution)
  double x[6][2];						///< Solutions of ellipse-line intersection
  Vertex2D leftHullPoint[6];					///< The solutions (x[i][j], y_g[i]) (i=0,..,6 / j=0,1)
  bool reliableCPoint[6];					///< Which circle points are reliable
  unsigned nrReliableCPoints;					///< Number of reliable circle points
  Vertex3D cPoints[6];						///< Real circle points in 3D
  double distance[6];						///< Distance between center and cPoints
  double deviation[6];						///< Deviation between distance and mean (radius)
  unsigned nrCPointsCalc;					///< Number of calculated circle points (cPointsCalc)
  Vertex3D cPointsCalc[20];					///< Calculated points on a "real" circle.

private:	
  void FitRectifiedEllipse(TmpEllipse &tmpEll);
  void SolveEllipseLineIntersection(TmpEllipse &left, TmpEllipse &right);
  bool CheckReliabilityOfResults();
  void CalculateCirclePoints(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right);
  bool CheckGeometry();
  void RefineVertices();
  void CalculateCircleProperties();
  bool SanityOK();
  void CalculateSignificance();

public:
  bool Reconstruct(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right);
  double Compare(Ellipse3D &ell);
};


/**
 * @brief Class StereoEllipses: Try to match ellipses from the stereo images.
 */
class StereoEllipses : public StereoBase
{
private:

  Array<TmpEllipse> ellipses[2];				///< 2D ellipses (tmp.) from mono image (left/right) (=> sorted with ellipse3ds)
  Array<Ellipse3D> ellipse3ds;					///< Stereo matched 3D ellipses
  int candMatches;											///< Number of matched ellipse candidates from left/right image
  int ellMatches;												///< Number of stereo matched ellipses (== ellipse3ds.Size())
	
#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
  void RecalculateCoordsystem(Ellipse3D &ellipse, Pose3 &pose);

  double MatchingScoreEllipse(TmpEllipse &left_ell, TmpEllipse &right_ell);
  unsigned FindMatchingEllipse(TmpEllipse &left_ell, Array<TmpEllipse> &right_ell, unsigned l);
  void MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches);
  void Calculate3DEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches, Array<Ellipse3D> &ellipse3ds);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoEllipses(VisionCore *vc[2], StereoCamera *sc);
  ~StereoEllipses() {}

  int NumEllipsesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::ELLIPSE);}		///< 
  int NumEllipsesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::ELLIPSE);}	///< 

  const TmpEllipse &Ellipses2D(int side, int i);
  const Ellipse3D &Ellipses(int i) {return ellipse3ds[i];}												///< Return a matched stereo ellipse.
  int NumStereoMatches() {return ellMatches;}																			///< Return the number of stereo matched ellipses
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif
