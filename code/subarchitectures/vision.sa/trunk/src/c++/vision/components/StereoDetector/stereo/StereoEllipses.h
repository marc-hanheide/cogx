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
#include "StereoCamera.h"
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
  bool valid;
  
public:
  unsigned vs3ID;                                       ///< ID from the vision core							/// TODO Werte private machen!
  Vertex2D center;                                      ///< Center point of the ellipse (fix point)
  double a, b, phi;                                     ///< Ellipse parameters
  Vertex2D hullPoint[6];                                ///< Points on the ellipse (for rectifying and refitting an ellipse)
  bool isLeftOfCenter[6];                               ///< True, if hullPoint is left of center

  TmpEllipse() {valid = false;}
  TmpEllipse(Ellipse *ellipse);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  bool IsValid() {return valid;}        /// TODO Wenn nach initialisieren valid, dann wird zu ellipses[2] hinzugefügt
};

/**
 * @brief Class TmpEllipse3D
 * 
 */
class TmpEllipse3D
{
private:
  
  unsigned vs3ID[2];                                     ///< The vs3 IDs of the left and right ellipse
  unsigned tmpEllID[2];                                  ///< The originally ID in the ellipses[] Array
  double y_dist;                                         ///< Deviation of the matched center point from y-coordinate
  bool valid;                                            ///< Valid 3D ellipse

  // values for right 3D matches
  double k, d[6];                                        ///< gradient and offest of turned straigt line
  
  double distance[6];                                    ///< Distance between center and cPoints

  double constructionSignificance;                       ///< Significance value, derived during construction of 3D circle
  double significance;                                   ///< Significance value of the estimated circle

  Vertex3D center3D;                                     ///< 3D center point of the circle with the normal
  double radius3D;                                       ///< Radius of the circle

public:																			/// TODO Privatisieren
  double y_g[6];                                         ///< y value of the lines from the right ellipse hull points (is also y of solution)
  double x[6][2];                                        ///< Solutions of ellipse-line intersection (6 points: x1,2) (2 solutions)

  double x_r, y_e, a_e, b_e, phi_e;                      ///< New rectified ellipse parameters from image ellipse

  Vertex2D leftHullPoint[6];                             ///< The solutions (x[i][j], y_g[i]) (i=0,..,6 / j=0,1)
  unsigned nrReliableCPoints;                            ///< Number of reliable circle points
  bool reliableCPoint[6];                                ///< Which circle points are reliable
  Vertex3D cPoints[6];                                   ///< Calculated points

private:
  void FitRectifiedEllipse(TmpEllipse &tmpEllLeft, unsigned numPoints);
  void SolveEllipseLineIntersection(TmpEllipse &right);
  bool CheckReliabilityOfResults();
  void CalculateCirclePoints(cast::StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right);
  bool CheckCircleGeometry();
  void RefineVertices();
  void CalculateCircleProperties();
  bool SanityOK();
  void CalculateSignificance(double significance2D);
  
public:
  TmpEllipse3D() {significance = 0.0; valid = false;}
  
  bool Reconstruct(cast::StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right, double significance2D);
  
  double GetSignificance(){return significance;}
  unsigned GetVs3ID(unsigned side){return vs3ID[side];}
  void SetValidation(bool v) {valid = v;}
  bool IsValid() {return valid;}
  void SetParameter(unsigned el, unsigned er, double yd) {tmpEllID[LEFT] = el; tmpEllID[RIGHT] = er; y_dist = yd;}
  unsigned GetTmpEllID(unsigned side) {return tmpEllID[side];}
  
  Vertex3D GetCenter(){return center3D;}
  double GetRadius(){return radius3D;}
};


/**
 * @brief Class StereoEllipses: Try to match ellipses from the stereo images.
 */
class StereoEllipses : public StereoBase
{
private:
  Array<TmpEllipse> ellipses[2];                           ///< 2D ellipses (tmp.) from mono image (left/right)
  Array<TmpEllipse3D> ellipses3D;                          ///< Calculated tmp. 3D ellipses						/// TODO für jede linke ellipse eine? Braucht man die danach noch mal?
  int ellMatches;                                          ///< Number of stereo matched ellipses (== ellipse3ds.Size())

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  double Calculate2DSignificance(double match_distance, TmpEllipse &left_ell,  TmpEllipse &right_ell);
  void MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map);
  void BackCheck(std::map<double, unsigned> *match_map, unsigned map_size);
  unsigned Calculate3DEllipses(Array<TmpEllipse3D> &ellipses3D, Array<TmpEllipse> &left_ell, 
			       Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoEllipses(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
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
