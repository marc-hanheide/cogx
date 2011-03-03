/**
 * @file StereoEllipses.h
 * @author Andreas Richtsfeld
 * @date Dec. 2009, June 2010
 * @version 0.1
 * @brief Stereo calculation of ellipses.
 */

#ifndef Z_STEREO_ELLIPSES_HH
#define Z_STEREO_ELLIPSES_HH

#include <map>
#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Ellipse3D.h"
#include "Ellipse.hh"

namespace Z
{

/**
 * @brief Class TmpEllipse
 */
class TmpEllipse
{
private:

  
public:
  unsigned vs3ID;                                       ///< ID from the vision core							/// TODO Werte private machen!
  Vertex2D center;                                      ///< Center point of the ellipse (fix point)
  double a, b, phi;                                     ///< Ellipse parameters
  Vertex2D hullPoint[6];                                ///< Points on the ellipse (for rectifying and refitting an ellipse)
  bool isLeftOfCenter[6];                               ///< True, if hullPoint is left of center

  TmpEllipse() {}
  TmpEllipse(Ellipse *ellipse);
  void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsValid() {return true;}        /// TODO Wenn nach initialisieren valid, dann wird zu ellipses[2] hinzugefügt
};

/**
 * @brief Class TmpEllipse3D
 * 
 */
class TmpEllipse3D
{
public:
  unsigned vs3ID[2];                                     ///< The vs3 IDs of the left and right ellipse
  
  double x_e, y_e, a_e, b_e, phi_e;                      ///< New rectified ellipse parameters from left image ellipse
  
  // values for right 3D matches
  double k, d[6];                                        ///< gradient and offest of turned straigt line
  double y_g[6];                                         ///< y value of the lines from the right ellipse hull points (is also y of solution)
  double x[6][2];                                        ///< Solutions of ellipse-line intersection (6 points: x1,2) (2 solutions)

  unsigned nrReliableCPoints;                            ///< Number of reliable circle points
  bool reliableCPoint[6];                                ///< Which circle points are reliable

  Vertex3D center;                                       ///< 3D center point of the circle with the normal
  Vertex2D leftHullPoint[6];                             ///< The solutions (x[i][j], y_g[i]) (i=0,..,6 / j=0,1)
  Vertex3D cPoints[6];                                   ///< Real circle points in 3D

  double distance[6];                                    ///< Distance between center and cPoints
  double deviation[6];                                   ///< Deviation between distance and mean (radius)

  double radius;                                         ///< Radius of the circle
  unsigned nrCPointsCalc;                                ///< Number of calculated circle points (cPointsCalc)
  Vertex3D cPointsCalc[20];                              ///< Calculated points on a "real" circle.

  double significance;                                   ///< Significance value of the estimated circle

public:
  unsigned tmpEllID[2];                                  ///< The originally ID in the ellipses[] Array
  double y_dist;                                         ///< Deviation of the matched center point
  bool valid;

public:
  TmpEllipse3D() {}
  void FitRectifiedEllipse(TmpEllipse &tmpEllLeft, unsigned numPoints);
  void SolveEllipseLineIntersection(TmpEllipse &right);
  bool CheckReliabilityOfResults();
  void CalculateCirclePoints(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right);
  bool CheckGeometry();
  void RefineVertices();
  void CalculateCircleProperties();
  bool SanityOK();
  void CalculateSignificance();
  bool Reconstruct(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right);
  
  double GetSignificance(){return significance;}
  unsigned GetVs3ID(unsigned side){return vs3ID[side];}
};


/**
 * @brief Class StereoEllipses: Try to match ellipses from the stereo images.
 */
class StereoEllipses : public StereoBase
{
private:

  Array<TmpEllipse> ellipses[2];                           ///< 2D ellipses (tmp.) from mono image (left/right)
  Array<TmpEllipse3D> ellipses3D;                          ///< Calculated tmp. 3D ellipses						/// TODO für jede linke ellipse eine?
//  int candMatches;                                         ///< Number of matched ellipse candidates from left/right image
  int ellMatches;                                          ///< Number of stereo matched ellipses (== ellipse3ds.Size())

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  void MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map);
  unsigned Calculate3DEllipses(Array<TmpEllipse3D> &ellipses3D, Array<TmpEllipse> &left_ell, 
			       Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoEllipses(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc);
  ~StereoEllipses() {}

  int NumEllipsesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::ELLIPSE);}		///< TODO Rename: NumEllipses2D(unsigned side)
  int NumEllipsesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::ELLIPSE);}	///< 

  const TmpEllipse &Ellipses2D(int side, int i);
//  const Ellipse3D &Ellipses(int i) {return ellipse3ds[i];}                              ///< Return a matched stereo ellipse.
  
  int NumStereoMatches() {return ellMatches;}                                           ///< Return the number of stereo matched ellipses
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif
