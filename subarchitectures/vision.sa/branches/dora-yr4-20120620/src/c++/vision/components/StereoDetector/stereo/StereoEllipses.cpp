/**
 * @file StereoEllipses.cpp
 * @author Andreas Richtsfeld
 * @date December 2009
 * @version 0.1
 * @brief Stereo calculation of ellipses.
 */

#include "StereoEllipses.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpEllipse --------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpEllipse
 * @param ellipse Object detector ellipse
 */
TmpEllipse::TmpEllipse(Ellipse *ellipse)
{
  vs3ID = ellipse->ID();
  center.p = ellipse->center;
  
  // calculate points on the ellipse hull
  for(unsigned i=0; i<6; i++)
  {
    hullPoint[i].p.x = center.p.x + ellipse->a*cos(2*M_PI*i/6.)*cos(ellipse->phi) - ellipse->b*sin(2*M_PI*i/6.)*sin(ellipse->phi);
    hullPoint[i].p.y = center.p.y + ellipse->a*cos(2*M_PI*i/6.)*sin(ellipse->phi) + ellipse->b*sin(2*M_PI*i/6.)*cos(ellipse->phi);
  }
  
  a = ellipse->a;
  b = ellipse->b;
  phi = ellipse->phi;
  
  valid = true;
}

/**
 * @brief Recalculate all ellipse parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpEllipse::RePrune(int oX, int oY, int sc)
{
  printf("TmpEllipse::RePrune: Not yet implemented!\n");
// 	surf.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpEllipse and estimate left/right hullPoints
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpEllipse::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  center.Rectify(stereo_cam, side);
  for(unsigned i=0; i<6; i++)
    hullPoint[i].Rectify(stereo_cam, side);

  for(unsigned i=0; i<6; i++)
  {
    if((hullPoint[i].pr.x - center.pr.x) < 0)
      isLeftOfCenter[i] = true;
    else
      isLeftOfCenter[i] = false;
  }
}

/**
 * @brief Returns true, if ellipse is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if ellipse is at this position.
 */
// bool TmpEllipse::IsAtPosition(int x, int y) const
// {
//   return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y);
// }



//-------------------------------------------------------------------//
//-------------------------- TmpEllipse3D ---------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * @param left Tmp. ellipse from the left image
 * @param numPoints Defines the number of hull points for later 3D matching
 */
void TmpEllipse3D::FitRectifiedEllipse(TmpEllipse &left, unsigned numPoints)
{
  // first match an ellipse in some left rectified ellipse points
  CvPoint2D32f *points = 0;
  CvBox2D params;

  // some rectified points of left ellipse
  points = new CvPoint2D32f[numPoints];
  for(unsigned i=0; i<numPoints; i++)
  {
    points[i].x = left.hullPoint[i].pr.x;
    points[i].y = left.hullPoint[i].pr.y;
  }
  
  // fit new ellipse to rectified points of ellipse and estimate parameters
printf("[StereoEllipses::TmpEllipse3D::FitRectifiedEllipse] ERROR: Antiquated! Update this function to new opencv version!\n");
//  cvFitEllipse(points, numPoints, &params);
  x_r = params.center.x;
  y_e = params.center.y;
  // box size is double the axis lengths
  a_e = params.size.width/2.;
  b_e = params.size.height/2.;
  // note: the angle returned is in degrees!
  phi_e = ScaleAngle_0_pi(params.angle*M_PI/180.);        /// HACK
  // note: for unknown reasons sometimes a < b!
  if(a_e < b_e)
  {
    swap(a_e, b_e);
    phi_e = ScaleAngle_0_pi(phi_e + M_PI_2);              /// HACK
  }
  delete[] points;
}


/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * First, shift ellipse to the middle, 
 * @param right Right tmp. ellipse
 */
void TmpEllipse3D::SolveEllipseLineIntersection(TmpEllipse &right)
{
  double y_gv[6];                               // y-values of the straight line, shifted by ellipse center.
  double qe_a, qe_b[6], qe_c[6];                // values of the quadratic equation

  k = -tan(phi_e);                              // rotate the straight line (around zero-point)
  qe_a = 1 + (k*k*a_e*a_e/(b_e*b_e));           // a parameter of quadratic equation
  for(unsigned i=0; i<6; i++)                   /// TODO man könnte die numPoints angeben wie bei FitRectifiedEllipse
  {
    y_g[i] = right.hullPoint[i].pr.y;           // get a y-coordinate from a point on the right ellipse
    y_gv[i] = y_g[i] - y_e;                     // shift the line (y_e = center.y)
    d[i] = y_gv[i]/cos(phi_e);                  // offset of straight line
    qe_b[i] = 2*k*d[i]*a_e*a_e/(b_e*b_e);
    qe_c[i] = -(a_e*a_e)*(1 - (d[i]*d[i]/(b_e*b_e)));
    x[i][0] = x_r + ((-qe_b[i] - sqrt(qe_b[i]*qe_b[i] - 4*qe_a*qe_c[i]))/(2*qe_a));
    x[i][1] = x_r + ((-qe_b[i] + sqrt(qe_b[i]*qe_b[i] - 4*qe_a*qe_c[i]))/(2*qe_a));
//     printf("Result: x[%u]: %4.2f / %4.2f\n",i, x[i][0], x[i][1]);
  }
}

/**
 * @brief Check the results of the solved ellipse-line intersections and prune wrong results.			/// TODO Bad results => Check again!
 * Only results in the image plane and != nan are valid.
 * @return True, if more than three reliable cPoints.
 */
bool TmpEllipse3D::CheckReliabilityOfResults()
{
  nrReliableCPoints = 0;
  for(unsigned i=0; i<6; i++)
  {
    if(isnan(x[i][0]) || isnan(x[i][1]) || (x[i][0] < 0 && x[i][1] < 0) || (x[i][0] > 640 && x[i][1] > 640))
      reliableCPoint[i] = false;
    else 
    {
      reliableCPoint[i] = true;
      nrReliableCPoints++;
    }
  }
// printf("TmpEllipse3D::CheckReliabilityOfResults: %u\n", nrReliableCPoints);
  if(nrReliableCPoints < 4) return false;
  else return true;
}


/**
 * @brief Calculate the 3D points from the results of the ellipse-line intersection.
 * @param stereo_cam Stereo camera paramters and functions.
 * @param left Left tmp. ellipse
 * @param right Right tmp. ellipse
 */
void TmpEllipse3D::CalculateCirclePoints(cast::StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right)
{
  center3D.Reconstruct(stereo_cam, left.center, right.center);
  for(unsigned i=0; i<6; i++)
  {
    if(reliableCPoint[i])
    {
      if(right.isLeftOfCenter[i])
	leftHullPoint[i].pr.x = x[i][0];
      else
	leftHullPoint[i].pr.x = x[i][1];

      leftHullPoint[i].pr.y = y_g[i];
      cPoints[i].Reconstruct(stereo_cam, leftHullPoint[i], right.hullPoint[i]);
    }
  }
}

/**
 * @brief Check the results of the calculated 3D circle points (cPoints).
 * To get a reliable result, we need at least 3 reliable points on the circle.
 * Calculate the construction significance from the deviations.
 * @return Return true, if geometry of a circle is reliable.
 */
bool TmpEllipse3D::CheckCircleGeometry()
{
//  double maxDeviationFactor = 2.;										/// TODO we allow everything! Unused variable

  // calculate mean distance between center and points
  radius3D = 0.;
  for(unsigned i=0; i<6; i++)
  {
    if(reliableCPoint[i])
    {
      distance[i] = cPoints[i].Distance(center3D);
      radius3D += distance[i];
    }
  }
  radius3D /= (double) nrReliableCPoints;

  // The max. deviation of a point may be half of the radius (1/maxD...).
  double deviation[6];
  for(unsigned i=0; i<6; i++)
    if(reliableCPoint[i])
      deviation[i] = radius3D - distance[i];
  
  // prune cPoints with "too" high deviation
//   for(unsigned i=0; i<6; i++)
//   {
//     if(reliableCPoint[i])
//     {
//       if(fabs(deviation[i]*maxDeviationFactor) > meanDistance)
//       {
// 	reliableCPoint[i] = false;
// 	nrReliableCPoints--;
//       }
//     }
//   }

  // calculate 3D construction significance
  double deviationSum = 0.;
  for(unsigned i=0; i<6; i++)
    if(reliableCPoint[i])
      deviationSum += fabs(deviation[i]);
  deviationSum /= nrReliableCPoints;
  constructionSignificance = 1 - deviationSum/radius3D;
      
  if(nrReliableCPoints < 3)
    printf("##################################### TmpEllipse3D::CheckGeometry: Geometry is not good!\n");

//   if(nrReliableCPoints < 3) return false;
//   else return true;
  return true;
}


/**
 * @brief Span one plane into the circle points (with center) and project all resulting \n
 * points to this plane. Calculate the normal direction of the circle plane.
 */
void TmpEllipse3D::RefineVertices()
{
  // copy the reliable cPoints and the center to Vertex3D array
  int nrVertices = nrReliableCPoints + 1;	// circle points + center
  Vertex3D vertex[nrVertices];
  int j=0;
  for(unsigned i=0; i<6; i++)
  if(reliableCPoint[i])
  {
    vertex[j] = cPoints[i];
    j++;
  }
  vertex[nrVertices-1] = center3D;

  // create matrices and make cvSVD
  CvMat *A = cvCreateMat(nrVertices, 4, CV_32FC1);
  CvMat *W = cvCreateMat(4, 1, CV_32FC1);
  CvMat *VT = cvCreateMat(4, 4, CV_32FC1);
  for(int i = 0; i < nrVertices; i++)
  {
    cvmSet(A, i, 0, vertex[i].p.x);
    cvmSet(A, i, 1, vertex[i].p.y);
    cvmSet(A, i, 2, vertex[i].p.z);
    cvmSet(A, i, 3, 1.);
  }
  // solve for a x + b y + c z + d = 0
  // subject to constraint |a, b, c, d| = 1
  cvSVD(A, W, NULL, VT, CV_SVD_MODIFY_A | CV_SVD_V_T);
	
  // solution is last row of VT (last column of V)
  // plane normal vector, not normalised yet
  Vector3 n(cvmGet(VT, 3, 0), cvmGet(VT, 3, 1), cvmGet(VT, 3, 2));
  double d = cvmGet(VT, 3, 3)/Norm(n);
  n = Normalise(n);
  // some plane point (in this case the plane point closest to origin)
  Vector3 p = -d*n;
  for(int i = 0; i < nrVertices; i++)
  {
    // project vertex point to plane
    vertex[i].p -= n*Dot(vertex[i].p - p, n);
    vertex[i].n = n;
  }

  cvReleaseMat(&A);
  cvReleaseMat(&W);
  cvReleaseMat(&VT);
	
  // copy the results back to cPoints (and center3D)!
  j=0;
  for(unsigned i=0; i<6; i++)
  if(reliableCPoint[i])
  {
// printf("TmpEllipse3D::RefineVertices: Distance[%u]: %4.3f\n", i, vertex[j].Distance(cPoints[i]));
    cPoints[i] = vertex[j];
    j++;
  }
// printf("TmpEllipse3D::RefineVertices: Distance[center]: %4.3f\n",  vertex[nrVertices-1].Distance(center));

  // copy the center point with the calculated NORMAL back to center3D
  center3D = vertex[nrVertices-1];
}


/**													// TODO Die sollte man eigentlich nur für die Anzeige berechnen!!!
 * @brief Calculate the radius (as mean), the calculated circle points and \n
 * the significance of the circle (from the deviation)
 */
void TmpEllipse3D::CalculateCircleProperties()
{
//   // calculate circle plane vectors
//   Vector3 u,v;
//   // rotate u 90 degree to center
//   u.x = center3D.n.x;
//   u.y = center3D.n.z;
//   u.z = -center3D.n.y;
//   // v = n x u => 3rd orthogonal vector
//   v.x = center3D.n.y*u.z - center3D.n.z*u.y;
//   v.y = center3D.n.z*u.x - center3D.n.x*u.z;
//   v.z = center3D.n.x*u.y - center3D.n.y*u.x;
//   
//   // calculate cPointsCalc
//   nrCPointsCalc = 20;		// set number of cPointsCalc
//   for(unsigned i=0; i<nrCPointsCalc; i++)
//   {
//     cPointsCalc[i].p.x = center3D.p.x + radius3D*u.x*cos(i*2*M_PI/(double)nrCPointsCalc) + radius3D*v.x*sin(i*2*M_PI/(double)nrCPointsCalc);
//     cPointsCalc[i].p.y = center3D.p.y + radius3D*u.y*cos(i*2*M_PI/(double)nrCPointsCalc) + radius3D*v.y*sin(i*2*M_PI/(double)nrCPointsCalc);
//     cPointsCalc[i].p.z = center3D.p.z + radius3D*u.z*cos(i*2*M_PI/(double)nrCPointsCalc) + radius3D*v.z*sin(i*2*M_PI/(double)nrCPointsCalc);
//   }
}

/**
 * @brief Calculate the significance of the estimated circle hypothesis.
 * The significance is calculated from the deviations between distance of the
 * hull points to the center and the radius. [is between 0...1?]
 * This significance is for detection of CIRCLES!
 * @param significance2D 2D significance from matching algorithm.
 */
void TmpEllipse3D::CalculateSignificance(double significance2D)
{
  significance = constructionSignificance * significance2D;
//   printf("  TmpEllipse3D::CalcSig: sig2D: %4.3f / con_sig: %4.2f / sig: %4.2f of ell: %u-%u\n", significance2D, constructionSignificance, significance, vs3ID[0], vs3ID[1]);
}
	
/**
 * @brief Check Sanity of 3D Ellipse
 * Check the distance from the camera to the object (z-coordinate)
 * @return Return true, if sanity check was ok.
 */
bool TmpEllipse3D::SanityOK()
{
  // center should be inside the space of interest (SOI)
  if(center3D.p.z < SC_MIN_DIST_Z || center3D.p.z > SC_MAX_DIST_Z)
  {
    printf("Ellipse3D::SanityOK: not ok: %4.2f => Error?\n", center3D.p.z);
    return false;
  }
  return true;
}

/**
 * @brief Reconstruct a circle in 3D from the ellipse projections in 2D of left/right image.
 * @param stereo_cam Stereo camera paramters and functions.
 * @param left Left tmp. ellipse
 * @param right Right tmp. ellipse
 * @param y_dist Distance between the center points
 * @return Return true, if reconstruction was ok.
 */
bool TmpEllipse3D::Reconstruct(cast::StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right, double significance2D)
{
  vs3ID[LEFT] = left.vs3ID;
  vs3ID[RIGHT] = right.vs3ID;
  FitRectifiedEllipse(left, 6);
  SolveEllipseLineIntersection(right);
  if(!CheckReliabilityOfResults()) return false;
  CalculateCirclePoints(stereo_cam, left, right);
  if(!CheckCircleGeometry()) return false;
  RefineVertices();
  CalculateCircleProperties();
  CalculateSignificance(significance2D);
  return(SanityOK());
}


//-------------------------------------------------------------------//
//------------------------- StereoEllipses --------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoEllipses: Calculate stereo matching of ellipses.
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 * @param sc Stereo camera paramters and functions.
 */
StereoEllipses::StereoEllipses(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
{
  vcore[LEFT] = vc[LEFT];
  vcore[RIGHT] = vc[RIGHT];
  stereo_cam = sc;
  ellMatches = 0;
}

/**
 * @brief Draw the matched ellipses and additional information:
 * @param side Left/right side of stereo images
 * @param single Show only one single result
 * @param id ID of the single matched result
 * @param detail Degree of detail
 */
void StereoEllipses::DrawMatched(int side, bool single, int id, int detail)
{
  if(single)
    DrawSingleMatched(side, id, detail);
  else
    for(int i=0; i<ellMatches; i++)
      DrawSingleMatched(side, i, detail);
}

/**															TODO Diese Implementierungen sollten eigentlich mit den Daten in den tmpXX sein!
 * @brief Draw single matched ellipses.
 * @param side Left/right side of stereo images.
 * @param id ID of the ellipse for ellipse3ds & ellipses
 */
void StereoEllipses::DrawSingleMatched(int side, int id, int detail)
{
  if(detail == 0) 		// draw then original ellises of the matching ellipses in 2D
  {
    if(id >= 0 && id < ellMatches)
    {
      if(side == LEFT)		// rectified new ellipse
      {
	DrawEllipse2D(ellipses[LEFT][id].center.p.x, ellipses[LEFT][id].center.p.y, 
		      ellipses[LEFT][id].a, ellipses[LEFT][id].b, ellipses[LEFT][id].phi, RGBColor::blue);
      }
      if(side == RIGHT)		// rectified new ellipse
      {
	DrawEllipse2D(ellipses[RIGHT][id].center.p.x, ellipses[RIGHT][id].center.p.y,
		      ellipses[RIGHT][id].a, ellipses[RIGHT][id].b, ellipses[RIGHT][id].phi, RGBColor::blue);
      }
    }
  }
  
  if(detail == 1)		// draw ellipses and hull points (rectified)
  {
    if(id >= 0 && id < ellMatches)
    {
      if(side == LEFT)          // old unrectified ellipse on left side with hull points
      {
	DrawEllipse2D(ellipses[LEFT][id].center.p.x, ellipses[LEFT][id].center.p.y,
		      ellipses[LEFT][id].a, ellipses[LEFT][id].b, ellipses[LEFT][id].phi, RGBColor::blue);
	for(unsigned i=0; i<6; i++)
	{
	  DrawPoint2D(ellipses[side][id].hullPoint[i].p.x, ellipses[side][id].hullPoint[i].p.y, RGBColor::red);
	  DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
	}
      }
      else                      // right ellipse and rectified points of right ellipse
      {
	DrawEllipse2D(ellipses[RIGHT][id].center.p.x, ellipses[RIGHT][id].center.p.y,
		      ellipses[RIGHT][id].a, ellipses[RIGHT][id].b, ellipses[RIGHT][id].phi, RGBColor::blue);
	for(unsigned i=0; i<6; i++)
	{
	  DrawPoint2D(ellipses[side][id].hullPoint[i].p.x, ellipses[side][id].hullPoint[i].p.y, RGBColor::red);
	  DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
	}
      }
    }
  }
 
  if(detail == 2)		// draw resulting ellipse
  {
    if(id >= 0 && id < ellMatches) 
    {
      if(side == LEFT)          // rectified new ellipse on left si-de
      {
	DrawEllipse2D(ellipses3D[id].x_r, ellipses3D[id].y_e,
		      ellipses3D[id].a_e, ellipses3D[id].b_e, 
		      ellipses3D[id].phi_e, RGBColor::blue);
	for(unsigned i=0; i<6; i++)
	{
	  DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
	}
      }
      else                      // rectified points of right ellipse
      {
	DrawEllipse2D(ellipses[RIGHT][id].center.p.x, ellipses[RIGHT][id].center.p.y,
		      ellipses[RIGHT][id].a, ellipses[RIGHT][id].b, ellipses[RIGHT][id].phi, RGBColor::blue);
	for(unsigned i=0; i<6; i++)
	{
	  DrawPoint2D(ellipses[side][id].hullPoint[i].p.x, ellipses[side][id].hullPoint[i].p.y, RGBColor::red);
	  DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
	}
      }
    }
  }
  
//   if(detail == 2 || detail == 3) // draw the ellipse candidates, before 3D calculated
//   {
//     if(id >= 0 && id < ellMatches) 
//     {
//       DrawEllipse2D(ellipses[side][id].center.p.x, ellipses[side][id].center.p.y,
// 		    ellipses[side][id].a, ellipses[side][id].b, ellipses[side][id].phi, RGBColor::blue);
//     }
//   }
// 

  if(detail == 3)
  {
    if(id >= 0 && id < ellMatches) 
    {
      if(side == LEFT)
      {
	// rectified new ellipse on left image
	DrawEllipse2D(ellipses3D[id].x_r, ellipses3D[id].y_e, 
		      ellipses3D[id].a_e, ellipses3D[id].b_e, 
		      ellipses3D[id].phi_e, RGBColor::blue);

	// draw epipolar lines (y_g[i]).
	for(unsigned i=0; i<6; i++)
	{
	  DrawLine2D(0, ellipses3D[id].y_g[i], 640, ellipses3D[id].y_g[i], RGBColor::red);
	}

	// draw solutions
	for(unsigned i=0; i<6; i++)
	{
// 	  DrawLine2D(ellipses3D[id].leftHullPoint[i].pr.x-5, ellipses3D[id].y_g[i]-5, 
// 		     ellipses3D[id].leftHullPoint[i].pr.x+5, ellipses3D[id].y_g[i]+5, RGBColor::blue);
// 	  DrawLine2D(ellipses3D[id].leftHullPoint[i].pr.x+5, ellipses3D[id].y_g[i]-5, 
// 		     ellipses3D[id].leftHullPoint[i].pr.x-5, ellipses3D[id].y_g[i]+5, RGBColor::blue);
// 	  DrawLine2D(ellipses3D[id].leftHullPoint[i].pr.x-5, ellipses3D[id].y_g[i]-5, 
// 		     ellipses3D[id].leftHullPoint[i].pr.x+5, ellipses3D[id].y_g[i]+5, RGBColor::blue);
// 	  DrawLine2D(ellipses3D[id].leftHullPoint[i].pr.x+5, ellipses3D[id].y_g[i]-5, 
// 		     ellipses3D[id].leftHullPoint[i].pr.x-5, ellipses3D[id].y_g[i]+5, RGBColor::blue);
	  
	  // results of ellipse-line equation!
	  DrawPoint2D(ellipses3D[id].leftHullPoint[i].pr.x, ellipses3D[id].y_g[i], RGBColor::yellow);		/// excact point!
	  char id_str[20];
	  snprintf(id_str, 20, "%u", i);
	  DrawText2D(id_str, ellipses3D[id].leftHullPoint[i].pr.x+8, ellipses3D[id].y_g[i], RGBColor::blue);
	}
      }

      if(side == RIGHT)
      {
	// draw epipolar lines
	for(unsigned i=0; i<6; i++)
	  DrawLine2D(0, ellipses[side][id].hullPoint[i].pr.y, 640, ellipses[side][id].hullPoint[i].pr.y, RGBColor::red);

	// rectified points of right ellipse
	for(unsigned i=0; i<6; i++)
	{
	  DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
	  char id_str[20];
	  snprintf(id_str, 20, "%u", i);
	  DrawText2D(id_str, ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::blue);
	}
      }
    }
  }
  
  if(detail == 0)		// draw the candidates, before 3D calculated, with center point and id
  {
    if(id >= 0 && id < ellMatches) 
    {
      DrawPoint2D(ellipses[side][id].center.p.x, ellipses[side][id].center.p.y, RGBColor::blue);
      char id_str[20];
      snprintf(id_str, 20, "%u", ellipses[side][id].vs3ID);
      DrawText2D(id_str, ellipses[side][id].center.p.x, ellipses[side][id].center.p.y, RGBColor::blue);
    }
  }
}

/**
 * @brief Convert ellipse from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector's ellipse.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoEllipses::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
  obj->model = new VisionData::GeometryModel;
  Ellipse3D *ell3D = Ellipses3D(score, id);
  
  VisionData::Vertex v;
  VisionData::Face f;
//   Ellipse3D ellipse = Ellipses(id);
// 
  // Recalculate pose of vertices (relative to the pose of the ell == cog)
  Pose3 pose;
  // we use the center of the ellipse as feature center!
  pose.pos.x = (ell3D->GetCenter()).p.x;
  pose.pos.y = (ell3D->GetCenter()).p.y;
  pose.pos.z = (ell3D->GetCenter()).p.z;
  pose.rot.x = 0.;	// set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. feature
  Pose3 inv = pose.Inverse();

  // add center point to the model
  cogx::Math::Pose3 cogxPose;
  cogxPose.pos.x = pose.pos.x;
  cogxPose.pos.y = pose.pos.y;
  cogxPose.pos.z = pose.pos.z;
  obj->pose = cogxPose;
  
  // calculate circle plane vectors
  Vector3 vec_u,vec_v;
  // rotate u 90 degree to center
  vec_u.x = (ell3D->GetCenter()).n.x;
  vec_u.y = (ell3D->GetCenter()).n.z;
  vec_u.z = -(ell3D->GetCenter()).n.y;
  // v = n x u => 3rd orthogonal vector
  vec_v.x = (ell3D->GetCenter()).n.y*vec_u.z - (ell3D->GetCenter()).n.z*vec_u.y;
  vec_v.y = (ell3D->GetCenter()).n.z*vec_u.x - (ell3D->GetCenter()).n.x*vec_u.z;
  vec_v.z = (ell3D->GetCenter()).n.x*vec_u.y - (ell3D->GetCenter()).n.y*vec_u.x;
  
  unsigned nrCPointsCalc = 20;                  // Number of calculated circle points (cPointsCalc)
  Vertex3D cPointsCalc[20];                     // Calculated points on a "real" circle.
  for(unsigned i=0; i<nrCPointsCalc; i++)
  {
    cPointsCalc[i].p.x = (ell3D->GetCenter()).p.x + ell3D->GetRadius()*vec_u.x*cos(i*2*M_PI/(double)nrCPointsCalc) + ell3D->GetRadius()*vec_v.x*sin(i*2*M_PI/(double)nrCPointsCalc);
    cPointsCalc[i].p.y = (ell3D->GetCenter()).p.y + ell3D->GetRadius()*vec_u.y*cos(i*2*M_PI/(double)nrCPointsCalc) + ell3D->GetRadius()*vec_v.y*sin(i*2*M_PI/(double)nrCPointsCalc);
    cPointsCalc[i].p.z = (ell3D->GetCenter()).p.z + ell3D->GetRadius()*vec_u.z*cos(i*2*M_PI/(double)nrCPointsCalc) + ell3D->GetRadius()*vec_v.z*sin(i*2*M_PI/(double)nrCPointsCalc);
  }
  // recalculate the vertices, relative to new center point
  for(unsigned i=0; i<nrCPointsCalc; i++)
    cPointsCalc[i].p = cPointsCalc[i].p - pose.pos;
  
  // recalculate the vectors to the vertices from new center point
  // => center
  Vector3 xavier((ell3D->GetCenter()).p.x, (ell3D->GetCenter()).p.y, (ell3D->GetCenter()).p.z);			/// TODO TODO Was macht diese Berechnung? => Wenn man weglöscht, dann ist nur mehr eine Ellipse in der VirtualSzene
  xavier = inv.Transform(xavier);
// ellipse.center.p = inv.Transform(p);

  // => cPointsCalc
  Vector3 cP[nrCPointsCalc];
  for(unsigned i = 0; i < nrCPointsCalc; i++)
  {
    cP[i] = Vector3(cPointsCalc[i].p.x, cPointsCalc[i].p.y, cPointsCalc[i].p.z);
    cP[i] = inv.Transform(cP[i]);
  }

  // copy center point to a vertex
  unsigned maxNr = -1;
  v.pos.x = 0.0; //(ell3D->GetCenter()).p.x;
  v.pos.y = 0.0; //(ell3D->GetCenter()).p.y;
  v.pos.z = 0.0; //(ell3D->GetCenter()).p.z;
  v.normal.x = (ell3D->GetCenter()).n.x;
  v.normal.y = (ell3D->GetCenter()).n.y;
  v.normal.z = (ell3D->GetCenter()).n.z;
  obj->model->vertices.push_back(v);

    
//   // => cPoints
// //   for(unsigned i = 0; i < 6; i++)
// //   {
// //     if(ellipse.reliableCPoint)
// //     {
// //       Vector3 p(ellipse.cPoints[i].p.x,	ellipse.cPoints[i].p.y,	ellipse.cPoints[i].p.z);
// //       ellipse.cPoints[i].p = inv.Transform(p);
// //     }
// //   }
//   // ++++++++++++++++++++++++++++++++++ real Points +++++++++++++++++++++++++++++++++ //
//   // create vertices (relative to the 3D center point)
// // 	for(unsigned i=0; i<6; i++)
// // 	{
// // 		if(ellipse.reliableCPoint[i])
// // 		{
// // 			v.pos.x = ellipse.cPoints[i].p.x;
// // 			v.pos.y = ellipse.cPoints[i].p.y;
// // 			v.pos.z = ellipse.cPoints[i].p.z;
// // 			obj->model->vertices.push_back(v);
// // 		}
// // 	}
// // 
// // 	// create faces
// // 	for(unsigned i=1; i<ellipse.nrReliableCPoints; i++)
// // 	{
// // 		f.vertices.push_back(0);
// // 		f.vertices.push_back(i);
// // 		f.vertices.push_back(i+1);
// // 		obj->model->faces.push_back(f);
// // 		f.vertices.clear();
// // 	}
// // 	maxNr = ellipse.nrReliableCPoints;
// // 	f.vertices.push_back(0);
// // 	f.vertices.push_back(maxNr);
// // 	f.vertices.push_back(1);
// // 	obj->model->faces.push_back(f);
// // 	f.vertices.clear();

  
  // ++++++++++++++++++++++++++++++++++ calc. Points +++++++++++++++++++++++++++++++++ //
  // create vertices (relative to the 3D center point)
  for(unsigned i=0; i<nrCPointsCalc; i++)
  {
    v.pos.x = cPointsCalc[i].p.x;
    v.pos.y = cPointsCalc[i].p.y;
    v.pos.z = cPointsCalc[i].p.z;
    obj->model->vertices.push_back(v);
  }

  // create faces
  for(unsigned i=1; i<nrCPointsCalc; i++)
  {
    f.vertices.push_back(maxNr+1);
    f.vertices.push_back(maxNr+1+i);
    f.vertices.push_back(maxNr+1+i+1);
    obj->model->faces.push_back(f);
    f.vertices.clear();
  }
  f.vertices.push_back(maxNr+1+0);
  f.vertices.push_back(maxNr+1+nrCPointsCalc);
  f.vertices.push_back(maxNr+1+1);
  obj->model->faces.push_back(f);
  f.vertices.clear();

  obj->detectionConfidence = ell3D->GetSignificance(); // 0.0;			// TODO detection confidence

  return true;
}
#endif

/**
 * @brief Calculate the significance of the matched result.
 * @param match_distance Distance between matching center points.
 * @param left_ell Left ellipse candidates
 * @param right_ell Right ellipse candidate
 */
double StereoEllipses::Calculate2DSignificance(double match_distance, TmpEllipse &left_ell,  TmpEllipse &right_ell)
{
  double sig = 1 - match_distance / SC_MAX_DELTA_V_POINT;
  double axis_deviation = 0;
  if(left_ell.a < right_ell.a)
    axis_deviation = left_ell.a/right_ell.a;
  else
    axis_deviation = right_ell.a/left_ell.a;
  sig = sig*axis_deviation;

// printf("Calculate2DSignificance: sig: %4.2f of %u-%u\n", sig, left_ell.vs3ID, right_ell.vs3ID);
  return sig;
}

/**
 * @brief Match left and right ellipses from an stereo image pair and create a match map for each left ellipse with the best
 * fitting ellipses of the right image.
 * @param left_ell Array of all ellipses from left stereo image
 * @param right_ell Array of all ellipses from right stereo image.
 * @param match_map Map for each left ellipse with the best fitting right ellipses (significance / right ellipse id)
 */
void StereoEllipses::MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map)
{
  for(unsigned i=0; i < left_ell.Size(); i++)
  {
    for(unsigned j=0; j < right_ell.Size(); j++)
    {
      double match = MatchingScorePoint(left_ell[i].center, right_ell[j].center);
      if(match != HUGE)
      {
	double sig = Calculate2DSignificance(match, left_ell[i], right_ell[j]);
	double size = (left_ell[i].a + right_ell[j].a)/2.;
// printf("MatchEllipses: size: %4.3f\n", size);
// 	if(sig > SC_MIN_2D_SIGNIFICANCE && size > SC_MIN_AXIS_SIZE)  // delete the really bad 2D results
	if((sig > SC_MIN_2D_ELL_SIGNIFICANCE && size > SC_MIN_AXIS_SIZE) || !SC_USE_ELL_THRESHOLDS)  // delete the really bad 2D results
	{
	  std::pair<double, unsigned> pair(sig, right_ell[j].vs3ID);
	  match_map[i].insert(pair);
	}
      }
    }    
  }
}

/**
 * @brief Each right ellipse can have only one best matching left ellipse.
 * Delete double assigned ones.
 * @param match_map Match map for ellipses
 * @param map_size Size of the match_map
 */
void StereoEllipses::BackCheck(std::map<double, unsigned> *match_map, unsigned map_size)
{
  unsigned nrMatches[map_size];
  for(unsigned i=0; i< map_size; i++)
    nrMatches[i] = match_map[i].size();

  bool solved = false;
  while(!solved)
  {
    solved = true;
    std::map<double, unsigned>::iterator it_i;
    std::map<double, unsigned>::iterator it_j;

    for (unsigned i=0; i<map_size; i++)
    {    
      for(unsigned j=0; j<map_size; j++)
      {
	if(i != j && nrMatches[i] > 0 && nrMatches[j] > 0)
	{
	  it_i = match_map[i].end(); it_i--;
	  it_j = match_map[j].end(); it_j--;
	  
	  if((*it_i).second == (*it_j).second)
	  {
	    solved = false;
	    if((*it_i).first > (*it_j).first)  // delete smaller value
	    {
	      match_map[j].erase(it_j);
	      nrMatches[j]--;
	    }
	    else
	    {
	      match_map[i].erase(it_i);
	      nrMatches[i]--;
	    }
	  }
	}
      }
    }
  }
  
  /// PRINT match map
//   for(unsigned i=0; i<map_size; i++)
//   {
//     std::map<double, unsigned>::iterator it;
//     if(match_map[i].size() > 0)
//     {
//       it = match_map[i].end(); it--;
//       printf("MatchMap after: %4.3f with %u / %u\n", (*it).first, i, (*it).second);
//     }
//   }
}

/**
 * @brief Calculate 3D ellipses from matched ellipses.
 * @param left_ell Array of all ellipses from left stereo image.
 * @param right_ell Array of all ellipses from right stereo image.
 * @param match_map 
 * @return Returns the number of matched ellipses.
 */
unsigned StereoEllipses::Calculate3DEllipses(Array<TmpEllipse3D> &ellipses3D, Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, std::map<double, unsigned> *match_map)
{
  Array<TmpEllipse> left, right;
  unsigned maxSize = 5;							// TODO maximum size of 5!
  std::map<double, unsigned> new_match_map[left_ell.Size()]; 
  
  unsigned nrMatches = 0;
  std::map<double, unsigned>::iterator it;
  TmpEllipse3D tmpEll3D[left_ell.Size()][maxSize];
  
  for(unsigned i=0; i<left_ell.Size(); i++)
  {
    unsigned nrResults = match_map[i].size();
    if(nrResults > maxSize) nrResults = maxSize;
    if(nrResults > 0)
    {
      it = match_map[i].end();
      for(unsigned j=0; j<nrResults; j++)
      {
	it--;
	bool reconstruct = tmpEll3D[i][j].Reconstruct(stereo_cam, left_ell[i], right_ell[(*it).second], (*it).first);
	if(reconstruct)
	{ 
	  std::pair<double, unsigned> pair(tmpEll3D[i][j].GetSignificance(), (*it).second);
	  new_match_map[i].insert(pair);
	  tmpEll3D[i][j].SetParameter(i, (*it).second, (*it).first);
	} 
	tmpEll3D[i][j].SetValidation(reconstruct);
      }
    }
  }
  
  // only one to one assignments between left and right ellipses
  BackCheck(new_match_map, left_ell.Size());

  // Create new stereo ellipses and store in the arrays
  for(unsigned i=0; i<left_ell.Size(); i++)
  {
    if(new_match_map[i].size() > 0)
    {
      it = new_match_map[i].end(); it--;
      for(unsigned k=0; k<maxSize; k++)
      {
	if(tmpEll3D[i][k].IsValid())
	{
	  if(tmpEll3D[i][k].GetVs3ID(RIGHT) == (*it).second)
	  {
	    ellipses3D.PushBack(tmpEll3D[i][k]);
	    Ellipse3D *ell3D = new Ellipse3D(tmpEll3D[i][k].GetVs3ID(LEFT), (*it).second, tmpEll3D[i][k].GetCenter(), 
					     tmpEll3D[i][k].GetRadius(), tmpEll3D[i][k].GetSignificance());
	    score->NewGestalt3D(ell3D);    
	  
// printf("Calculate3DEllipses: 3Dsig: %4.3f and ids: %u, %u\n", tmpEll3D[i][k].GetSignificance(), tmpEll3D[i][k].GetTmpEllID(LEFT), tmpEll3D[i][k].GetTmpEllID(RIGHT));
	    
	    left.PushBack(left_ell[tmpEll3D[i][k].GetTmpEllID(LEFT)]);
	    right.PushBack(right_ell[tmpEll3D[i][k].GetTmpEllID(RIGHT)]);
	    nrMatches++;
	  }
	}
      }
    }
  }

  left_ell = left;
  right_ell = right;
    
  return nrMatches;
}

/**
 * @brief Delete all arrays ...
 */
void StereoEllipses::ClearResults()
{
  ellipses[LEFT].Clear();
  ellipses[RIGHT].Clear();
  ellMatches = 0;
}

/**
 * @brief Match and calculate 3D ellipses from 2D ellipses.
 */
void StereoEllipses::Process()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::ELLIPSE); i++)
    {
      Ellipse *core_ellipse = (Ellipse*)vcore[side]->Gestalts(Gestalt::ELLIPSE, i);
//      if(!vcore[side]->use_masking || !core_ellipse->IsMasked())	/// TODO use_masking ist immer true => damit immer true!
//       {
	TmpEllipse ellipse(core_ellipse);
	if(ellipse.IsValid())
	  ellipses[side].PushBack(ellipse);
//       }
    }
    /// TODO Insert RePrune
//   if(pPara.pruning)
//     for(unsigned i = 0; i < cubes[side].Size(); i++)
//       cubes[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);

    for(unsigned i = 0; i < ellipses[side].Size(); i++)    // Rectify ellipse.
      ellipses[side][i].Rectify(stereo_cam, side);
  }

// struct timespec start, end;
// clock_gettime(CLOCK_REALTIME, &start);
  
  // define match map with significance value as key and right ellipse id
  std::map<double, unsigned> match_map[ellipses[LEFT].Size()];
  MatchEllipses(ellipses[LEFT], ellipses[RIGHT], match_map);

// clock_gettime(CLOCK_REALTIME, &end);
// cout<<"StereoEllipses::Process: Time to match [s]: " << timespec_diff(&end, &start) << endl;

// struct timespec cstart, cend;
// clock_gettime(CLOCK_REALTIME, &cstart);
  
  ellMatches = Calculate3DEllipses(ellipses3D, ellipses[LEFT], ellipses[RIGHT], match_map);

// clock_gettime(CLOCK_REALTIME, &cend);
// cout<<"StereoEllipses::Process: Time to calculate 3D ellipses [s]: " << timespec_diff(&cend, &cstart) << endl;

}


/**
 * @brief Match and calculate 3D ellipses from 2D ellipses using a pruned image pair.
 * @param oX Offset in x-coordinate
 * @param oY Offset in y-coordinate
 * @param sc Scale factor
 */
    void StereoEllipses::Process(int oX, int oY, int sc)
{
  pPara.pruning = true;
  pPara.offsetX = oX;
  pPara.offsetY = oY;
  pPara.scale = sc;
  Process();
  pPara.pruning = false;
}


}








