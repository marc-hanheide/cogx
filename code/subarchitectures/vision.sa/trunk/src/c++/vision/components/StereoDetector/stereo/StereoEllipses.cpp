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
	id = ellipse->ID();
	center.p = ellipse->center;
	
	// calculate points on the ellipse hull
	for(unsigned i=0; i<6; i++)
	{
		hullPoint[i].p.x = center.p.x + ellipse->a*cos(2*M_PI*i/6.)*cos(ellipse->phi) 
																	- ellipse->b*sin(2*M_PI*i/6.)*sin(ellipse->phi);
		hullPoint[i].p.y = center.p.y + ellipse->a*cos(2*M_PI*i/6.)*sin(ellipse->phi) 
																	+ ellipse->b*sin(2*M_PI*i/6.)*cos(ellipse->phi);
		if((hullPoint[i].p.x - center.p.x) < 0)
			isLeftOfCenter[i] = true;
		else
			isLeftOfCenter[i] = false;
	}
	
	a = ellipse->a;
	b = ellipse->b;
	phi = ellipse->phi;
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
 * @brief Rectify TmpEllipse
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpEllipse::Rectify(StereoCamera *stereo_cam, int side)
{
	center.Rectify(stereo_cam, side);
	for(unsigned i=0; i<6; i++)
		hullPoint[i].Rectify(stereo_cam, side);
}

/**																										/// TODO TODO not yet implemented!
 * @brief Refine TmpEllipse
 */
void TmpEllipse::Refine()
{
// 	printf("TmpEllipse::Refine: Not yet implemented!\n");
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
//---------------------------- Ellipse3D ----------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * @param tmpEll Tmp. ellipse
 */
void Ellipse3D::FitRectifiedEllipse(TmpEllipse &tmpEll)
{
	// first match an ellipse in some left rectified ellipse points
	CvPoint2D32f *points = 0;
	CvBox2D params;
	unsigned numPoints = 6;

	// some rectified points of left ellipse
	points = new CvPoint2D32f[numPoints];
	for(unsigned i=0; i<numPoints; i++)
	{
		points[i].x = tmpEll.hullPoint[i].pr.x;
		points[i].y = tmpEll.hullPoint[i].pr.y;
	}
	
	// fit new ellipse to rectified points of ellipse and estimate parameters
	cvFitEllipse(points, numPoints, &params);
	x_e = params.center.x;
	y_e = params.center.y;
	// box size is double the axis lengths
	a_e = params.size.width/2.;
	b_e = params.size.height/2.;
	// note: the angle returned is in degrees!
	phi_e = ScaleAngle_0_pi/*ScaleAngle_mpi_pi*/(params.angle*M_PI/180.);		/// HACK
	// note: for unknown reasons sometimes a < b!
	if(a_e < b_e)
	{
		swap(a_e, b_e);
		phi_e = ScaleAngle_0_pi/*ScaleAngle_mpi_pi*/(phi_e + M_PI_2);					/// HACK
	}
	delete[] points;
}

/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * First, shift ellipse to the middle, 
 * @param left Left tmp. ellipse
 * @param right Right tmp. ellipse
 */
void Ellipse3D::SolveEllipseLineIntersection(TmpEllipse &left, TmpEllipse &right)
{
	double y_gv[6];												// y-values of the straight line, shifted by ellipse center.
	double qe_a, qe_b[6], qe_c[6];				// values of the quadratic equation

	k = -tan(phi_e);											// rotate the straight line (around zero-point)
	qe_a = 1 + (k*k*a_e*a_e/(b_e*b_e));		// a parameter of quadratic equation
	for(unsigned i=0; i<6; i++)
	{
		y_g[i] = right.hullPoint[i].pr.y;		// get a y-coordinate from a point on the right ellipse
		y_gv[i] = y_g[i] - y_e;							// shift the line
		d[i] = y_gv[i]/cos(phi_e);					// offset of straight line
		qe_b[i] = 2*k*d[i]*a_e*a_e/(b_e*b_e);
		qe_c[i] = -(a_e*a_e)*(1 - (d[i]*d[i]/(b_e*b_e)));
		x[i][0] = x_e + ((-qe_b[i] - sqrt(qe_b[i]*qe_b[i] - 4*qe_a*qe_c[i]))/(2*qe_a));
		x[i][1] = x_e + ((-qe_b[i] + sqrt(qe_b[i]*qe_b[i] - 4*qe_a*qe_c[i]))/(2*qe_a));
	}
}

/**
 * @brief Check the results of the solved ellipse-line intersections and prune wrong results.
 * @return True, if more than three reliable cPoints.
 */
bool Ellipse3D::CheckReliabilityOfResults()
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
	if(nrReliableCPoints < 4) return false;
	else return true;
}


/**
 * @brief Calculate the 3D points from the results of the ellipse-line intersection.
 * @param stereo_cam Stereo camera paramters and functions.
 * @param left Left tmp. ellipse
 * @param right Right tmp. ellipse
 */
void Ellipse3D::CalculateCirclePoints(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right)
{
	center.Reconstruct(stereo_cam, left.center, right.center);
	for(unsigned i=0; i<6; i++)
	{
		if(reliableCPoint[i])
		{
			if(right.isLeftOfCenter[i])
			{
				leftHullPoint[i].pr.x = x[i][0];
				leftHullPoint[i].pr.y = y_g[i];
				cPoints[i].Reconstruct(stereo_cam, leftHullPoint[i], right.hullPoint[i]);
			}
			else
			{
				leftHullPoint[i].pr.x = x[i][1];
				leftHullPoint[i].pr.y = y_g[i];
				cPoints[i].Reconstruct(stereo_cam, leftHullPoint[i], right.hullPoint[i]);
			}
		}
	}
}


/**
 * @brief Check the results of the calculated 3D circle points (cPoints).
 * To get a reliable result, we need at least 3 reliable points on the circle.
 * @return Return true, if geometry of a circle is reliable.
 */
bool Ellipse3D::CheckGeometry()
{
	// calculate mean
	double mean = 0.;
	for(unsigned i=0; i<6; i++)
	{
		if(reliableCPoint[i])
		{
			distance[i] = cPoints[i].Distance(center);
			mean += distance[i];
		}
	}
	mean /= (double) nrReliableCPoints;
	
	// The max. deviation of a point may be half of the radius (1/maxD...).
	double maxDeviationFactor = 2.;
	for(unsigned i=0; i<6; i++)
		if(reliableCPoint[i])
			deviation[i] = mean - distance[i];

	// prune cPoints with "too" high deviation
	for(unsigned i=0; i<6; i++)
	{
		if(reliableCPoint[i])
		{
			if(fabs(deviation[i]*maxDeviationFactor) > mean)
			{
				reliableCPoint[i] = false;
				nrReliableCPoints--;
			}
		}
	}

	if(nrReliableCPoints < 3) return false;
	else return true;
}



/**
 * @brief Span one plane into the circle points (with center) and project all resulting \n
 * points to this plane. Calculate the normal direction of the circle plane.
 */
void Ellipse3D::RefineVertices()
{
	// copy the reliable cPoints and the center to Vertex3D array
	int nrVertices = nrReliableCPoints + 1;		// circle points + center
	Vertex3D vertex[nrVertices];
	int j=0;
	for(unsigned i=0; i<6; i++)
		if(reliableCPoint[i])
		{
			vertex[j] = cPoints[i];
			j++;
		}
	vertex[nrVertices-1] = center;

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
	
	// copy the results back to cPoints (and center)!
	j=0;
	for(unsigned i=0; i<6; i++)
		if(reliableCPoint[i])
		{
			cPoints[i] = vertex[j];
			j++;
		}
	center = vertex[nrVertices-1];
}


/**
 * @brief Calculate the radius (as mean), the calculated circle points and \n
 * the significance of the circle (from the deviation)
 */
void Ellipse3D::CalculateCircleProperties()
{
	// calculate radius
	radius = 0.;
	for(unsigned i=0; i<6; i++)
		if(reliableCPoint[i])
			radius += distance[i];
	radius /= (double) nrReliableCPoints;
	
	// calculate circle plane vectors
	Vector3 u,v;
	// rotate u 90 degree to center
	u.x = center.n.x;
	u.y = center.n.z;
	u.z = -center.n.y;
	// v = n x u => 3rd orthogonal vector
	v.x = center.n.y*u.z - center.n.z*u.y;
	v.y = center.n.z*u.x - center.n.x*u.z;
	v.z = center.n.x*u.y - center.n.y*u.x;
	
	// calculate cPointsCalc
	nrCPointsCalc = 20;		// set number of cPointsCalc
	for(unsigned i=0; i<nrCPointsCalc; i++)
	{
		cPointsCalc[i].p.x = center.p.x + radius*u.x*cos(i*2*M_PI/(double)nrCPointsCalc) + radius*v.x*sin(i*2*M_PI/(double)nrCPointsCalc);
		cPointsCalc[i].p.y = center.p.y + radius*u.y*cos(i*2*M_PI/(double)nrCPointsCalc) + radius*v.y*sin(i*2*M_PI/(double)nrCPointsCalc);
		cPointsCalc[i].p.z = center.p.z + radius*u.z*cos(i*2*M_PI/(double)nrCPointsCalc) + radius*v.z*sin(i*2*M_PI/(double)nrCPointsCalc);
	}
}


/**
 * @brief Calculate the significance of the estimated circle hypothesis.
 * The significance is calculated from the deviations between distance of the
 * hull points to the center and the radius. [is between 0...1?]
 */
void Ellipse3D::CalculateSignificance()
{
	// calculate significance
	double deviationSum = 0.;
	for(unsigned i=0; i<6; i++)
		if(reliableCPoint[i])
			deviationSum += fabs(deviation[i]);
	deviationSum /= nrReliableCPoints;
	significance = 1 - deviationSum/radius;
}
	
/**
 * @brief Check Sanity of 3D Ellipse
 * Check the distance from the camera to the object (z-coordinate)
 * @return Return true, if sanity check was ok.
 */
bool Ellipse3D::SanityOK()
{
	// center should be inside the space of interest (SOI)
	if(center.p.z < SC_MIN_DIST_Z || center.p.z > SC_MAX_DIST_Z)
	{
		printf("Ellipse3D::SanityOK: not ok: %4.2f => Error?\n", center.p.z);
		return false;
	}
	return true;
}

/**
 * @brief Reconstruct a circle in 3D from the ellipse projections in 2D of left/right image.
 * @param stereo_cam Stereo camera paramters and functions.
 * @param left Left tmp. ellipse
 * @param right Right tmp. ellipse
 * @return Return true, if reconstruction was ok.
 */
bool Ellipse3D::Reconstruct(StereoCamera *stereo_cam, TmpEllipse &left, TmpEllipse &right)
{
	FitRectifiedEllipse(left);
	SolveEllipseLineIntersection(left, right);
	if(!CheckReliabilityOfResults()) return false;
	CalculateCirclePoints(stereo_cam, left, right);
	if(!CheckGeometry()) return false;
	RefineVertices();
	CalculateCircleProperties();
	CalculateSignificance();
  return(SanityOK());
}


/**
 * @brief Compare two ellipses for filtering.
 * @param ell 3D ellipse to compare
 * @return Return certainty value for equality [0,100]
 */
double Ellipse3D::Compare(Ellipse3D &ell)
{
	/// use center and radius
	
	/// distance between center points
	double dist = Length(center.p - ell.center.p) + fabs(radius-ell.radius);
	
// 	printf("  Ellipse3D::Compare: dist: %4.2f\n", dist);
	
	return dist;
}
	
//-------------------------------------------------------------------//
//------------------------- StereoEllipses --------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoEllipses: Calculate stereo matching of ellipses.
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 * @param sc Stereo camera paramters and functions.
 */
StereoEllipses::StereoEllipses(VisionCore *vc[2], StereoCamera *sc) : StereoBase()
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
		for(int i=0; i<candMatches; i++)
			DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched ellipses.
 * @param side Left/right side of stereo images.
 * @param id ID of the ellipse for ellipse3ds & ellipses
 */
void StereoEllipses::DrawSingleMatched(int side, int id, int detail)
{
	if(detail == 0) 			// draw then original ellises of the matching ellipses in 2D
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
	
	if(detail == 1)				// draw resulting ellipse
	{
		if(id >= 0 && id < ellMatches) 
		{
			if(side == LEFT)		// rectified new ellipse
			{
				DrawEllipse2D(ellipse3ds[id].x_e, ellipse3ds[id].y_e, 
											ellipse3ds[id].a_e, ellipse3ds[id].b_e, 
											ellipse3ds[id].phi_e, RGBColor::blue);				
				DrawEllipse2D(ellipse3ds[id].x_e, ellipse3ds[id].y_e, 
											ellipse3ds[id].a_e, ellipse3ds[id].b_e, 
											ellipse3ds[id].phi_e, RGBColor::blue);
				for(unsigned i=0; i<6; i++)
					DrawPoint2D(ellipses[side][id].hullPoint[i].p.x, ellipses[side][id].hullPoint[i].p.y, RGBColor::yellow);
			}
			else								// rectified points of right ellipse
				for(unsigned i=0; i<6; i++)
				{
					DrawPoint2D(ellipses[side][id].hullPoint[i].p.x, ellipses[side][id].hullPoint[i].p.y, RGBColor::blue);
					DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);
				}
		}
	}
	
	if(detail == 2 || detail == 3)		// draw the ellipse candidates, before 3D calculated
	{
		if(id >= 0 && id < candMatches) 
		{
			DrawEllipse2D(ellipses[side][id].center.p.x, ellipses[side][id].center.p.y,
										ellipses[side][id].a, ellipses[side][id].b, ellipses[side][id].phi, RGBColor::blue);
		}
	}

	if(detail == 3)		// draw the candidates, before 3D calculated, with center point and id
	{
		if(id >= 0 && id < candMatches) 
		{
			DrawPoint2D(ellipses[side][id].center.p.x, ellipses[side][id].center.p.y, RGBColor::blue);
			char id_str[20];
			snprintf(id_str, 20, "%u", ellipses[side][id].id);
			DrawText2D(id_str, ellipses[side][id].center.p.x, ellipses[side][id].center.p.y, RGBColor::blue);
		}
	}

	if(detail == 4)
	{
		if(id >= 0 && id < ellMatches) 
		{
			if(side == LEFT)
			{
				// rectified new ellipse
				DrawEllipse2D(ellipse3ds[id].x_e, ellipse3ds[id].y_e, 
											ellipse3ds[id].a_e, ellipse3ds[id].b_e, 
											ellipse3ds[id].phi_e, RGBColor::blue);

				// draw epipolar lines
				for(unsigned i=0; i<6; i++)
				{
					DrawLine2D(0, ellipse3ds[id].y_g[i], 640, ellipse3ds[id].y_g[i], RGBColor::red);
				}

				// draw solutions
				for(unsigned i=0; i<6; i++)
				{
					DrawLine2D(ellipse3ds[id].leftHullPoint[i].pr.x-5, ellipse3ds[id].y_g[i]-5, 
										ellipse3ds[id].leftHullPoint[i].pr.x+5, ellipse3ds[id].y_g[i]+5, RGBColor::blue);
					DrawLine2D(ellipse3ds[id].leftHullPoint[i].pr.x+5, ellipse3ds[id].y_g[i]-5, 
										ellipse3ds[id].leftHullPoint[i].pr.x-5, ellipse3ds[id].y_g[i]+5, RGBColor::blue);
					DrawLine2D(ellipse3ds[id].leftHullPoint[i].pr.x-5, ellipse3ds[id].y_g[i]-5, 
										ellipse3ds[id].leftHullPoint[i].pr.x+5, ellipse3ds[id].y_g[i]+5, RGBColor::blue);
					DrawLine2D(ellipse3ds[id].leftHullPoint[i].pr.x+5, ellipse3ds[id].y_g[i]-5, 
										ellipse3ds[id].leftHullPoint[i].pr.x-5, ellipse3ds[id].y_g[i]+5, RGBColor::blue);

					char id_str[20];
					snprintf(id_str, 20, "%u", i);
					DrawText2D(id_str, ellipse3ds[id].leftHullPoint[i].pr.x+5, ellipse3ds[id].y_g[i], RGBColor::blue);
				}
			}

			if(side == RIGHT)
			{
				// draw epipolar lines
				for(unsigned i=0; i<6; i++)
					DrawLine2D(0, ellipses[side][id].hullPoint[i].pr.y, 640, ellipses[side][id].hullPoint[i].pr.y, RGBColor::red);
				
				for(unsigned i=0; i<6; i++)
				{
					// rectified points of right ellipse
					DrawPoint2D(ellipses[side][id].hullPoint[i].pr.x+5, ellipses[side][id].hullPoint[i].pr.y, RGBColor::yellow);

					char id_str[20];
					snprintf(id_str, 20, "%u", i);
					DrawText2D(id_str, ellipses[side][id].hullPoint[i].pr.x+5, ellipses[side][id].hullPoint[i].pr.y, RGBColor::blue);
				}
			}
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
	VisionData::Vertex v;
	VisionData::Face f;
	Ellipse3D ellipse = Ellipses(id);

	// Recalculate pose of vertices (relative to the pose of the flap == cog)
	Pose3 pose;
	RecalculateCoordsystem(ellipse, pose);

	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = pose.pos.x;
	cogxPose.pos.y = pose.pos.y;
	cogxPose.pos.z = pose.pos.z;
	obj->pose = cogxPose;

	unsigned maxNr = -1;
	v.pos.x = ellipse.center.p.x;
	v.pos.y = ellipse.center.p.y;
	v.pos.z = ellipse.center.p.z;
	obj->model->vertices.push_back(v);

	// ++++++++++++++++++++++++++++++++++ real Points +++++++++++++++++++++++++++++++++ //
	// create vertices (relative to the 3D center point)
// 	for(unsigned i=0; i<6; i++)
// 	{
// 		if(ellipse.reliableCPoint[i])
// 		{
// 			v.pos.x = ellipse.cPoints[i].p.x;
// 			v.pos.y = ellipse.cPoints[i].p.y;
// 			v.pos.z = ellipse.cPoints[i].p.z;
// 			obj->model->vertices.push_back(v);
// 		}
// 	}
// 
// 	// create faces
// 	for(unsigned i=1; i<ellipse.nrReliableCPoints; i++)
// 	{
// 		f.vertices.push_back(0);
// 		f.vertices.push_back(i);
// 		f.vertices.push_back(i+1);
// 		obj->model->faces.push_back(f);
// 		f.vertices.clear();
// 	}
// 	maxNr = ellipse.nrReliableCPoints;
// 	f.vertices.push_back(0);
// 	f.vertices.push_back(maxNr);
// 	f.vertices.push_back(1);
// 	obj->model->faces.push_back(f);
// 	f.vertices.clear();

	
	// ++++++++++++++++++++++++++++++++++ calc. Points +++++++++++++++++++++++++++++++++ //
	// create vertices (relative to the 3D center point)
	for(unsigned i=0; i<ellipse.nrCPointsCalc; i++)
	{
		v.pos.x = ellipse.cPointsCalc[i].p.x;
		v.pos.y = ellipse.cPointsCalc[i].p.y;
		v.pos.z = ellipse.cPointsCalc[i].p.z;
		obj->model->vertices.push_back(v);
	}

	// create faces
	for(unsigned i=1; i<ellipse.nrCPointsCalc; i++)
	{
		f.vertices.push_back(maxNr+1);
		f.vertices.push_back(maxNr+1+i);
		f.vertices.push_back(maxNr+1+i+1);
		obj->model->faces.push_back(f);
		f.vertices.clear();
	}
	f.vertices.push_back(maxNr+1+0);
	f.vertices.push_back(maxNr+1+ellipse.nrCPointsCalc);
	f.vertices.push_back(maxNr+1+1);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	obj->detectionConfidence = 1.0;						// TODO detection confidence is always 1

	return true;
}
#endif

/**
 * @brief Try to find a "natural" looking coordinate system for a ellipse.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity as position and set orientation to identity.
 * @param ellipse 3D ellipse
 * @param pose calculated pose
 */
void StereoEllipses::RecalculateCoordsystem(Ellipse3D &ellipse, Pose3 &pose)
{
	// we use the center of the ellipse as feature center!
  pose.pos.x = ellipse.center.p.x;
  pose.pos.y = ellipse.center.p.y;
  pose.pos.z = ellipse.center.p.z;

	// set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.x = 0.;
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. feature
  Pose3 inv = pose.Inverse();

	// recalculate the vectors to the vertices from new center point
	// => center
	Vector3 p(ellipse.center.p.x,	ellipse.center.p.y,	ellipse.center.p.z);
	ellipse.center.p = inv.Transform(p);

	// => cPoints
	for(unsigned i = 0; i < 6; i++)
	{
		if(ellipse.reliableCPoint)
		{
			Vector3 p(ellipse.cPoints[i].p.x,	ellipse.cPoints[i].p.y,	ellipse.cPoints[i].p.z);
			ellipse.cPoints[i].p = inv.Transform(p);
		}
	}
	
	// => cPointsCalc
	for(unsigned i = 0; i < ellipse.nrCPointsCalc; i++)
	{
			Vector3 p(ellipse.cPointsCalc[i].p.x,	ellipse.cPointsCalc[i].p.y,	ellipse.cPointsCalc[i].p.z);
			ellipse.cPointsCalc[i].p = inv.Transform(p);
	}
}

/**
 * @brief Calculate the matching score of two ellipses.
 * @param left_ell Left tmp. ellipse
 * @param right_ell Right tmp. ellipse
 * @return Returns the vertical deviation of the center point (distance \n
 * to epipolar line)
 */
double StereoEllipses::MatchingScoreEllipse(TmpEllipse &left_ell, TmpEllipse &right_ell)
{
	// do not compare ellipses with "too" big deviation in size
	double ratio_a = left_ell.a/right_ell.a;
	double ratio_b = left_ell.b/right_ell.b;
	if(ratio_a < 0.5 || ratio_a > 1.5 || ratio_b < 0.5 || ratio_b > 1.5)
		return HUGE;

	// get vertical deviation (distance to epipolar line)
	double match = MatchingScorePoint(left_ell.center, right_ell.center);

	// no match if vertical deviation is bigger than main axis length
	if (match != HUGE && match > left_ell.a && match > right_ell.a)
		match = HUGE;

	return match;
}

/**
 * @brief Find right best matching ellipse for given left ellipse, begining at position l of right ellipse array.
 * @param left_ell Tmp. ellipse of left stereo image.
 * @param right_ell Array of all ellipses from right stereo image.
 * @param l Begin at position l of right ellipse array
 * @return Returns position of best matching right ellipse from the right_ell array.
 */
unsigned StereoEllipses::FindMatchingEllipse(TmpEllipse &left_ell, Array<TmpEllipse> &right_ell, unsigned l)
{
	double match, best_match = HUGE;
	unsigned j, j_best = UNDEF_ID;

	for(j = l; j < right_ell.Size(); j++)
	{
    match = MatchingScoreEllipse(left_ell, right_ell[j]);

		if(match < best_match)
		{
			best_match = match;
			j_best = j;
		}
	}
	return j_best;
}

/**
 * @brief Match left and right ellipses from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_ell Array of all ellipses from left stereo image (matching ellipses get sorted to the beginning of the array.)
 * @param right_ell Array of all ellipses from right stereo image.
 * @param matches Number of matched ellipses (sorted to the beginning of the arrays).
 */
void StereoEllipses::MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches)
{
  unsigned j, l = 0, u = left_ell.Size();
  for(; l < u && l < right_ell.Size();)
  {
    j = FindMatchingEllipse(left_ell[l], right_ell, l);
    if(j != UNDEF_ID)
    {
      right_ell.Swap(l, j);
      l++;
    }
    else
    {
      left_ell.Swap(l, u-1);
      u--;
    }
  }
  u = min(u, right_ell.Size());
  matches = u;
}


/**
 * @brief Calculate 3D ellipses from matched ellipses.
 * @param left_ell Array of all ellipses from left stereo image.
 * @param right_ell Array of all ellipses from right stereo image.
 * @param matches Number of matched ellipses.
 * @param ellipse3ds Array of calculated 3d ellipses.
 */
void StereoEllipses::Calculate3DEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches, Array<Ellipse3D> &ellipse3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Ellipse3D ellipse3d;
// 		bool reconstructed = ellipse3d.Reconstruct(stereo_cam, left_ell[i], right_ell[i]);
		if(ellipse3d.Reconstruct(stereo_cam, left_ell[i], right_ell[i]))
		{
			ellipse3d.tmpID = i;																																		/// TODO add to all stereo principles!
			ellipse3ds.PushBack(ellipse3d);
			i++;
    }
    // move unacceptable features to the end
    else
    {
      left_ell.Swap(i, u-1);
      right_ell.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}

/**
 * @brief Delete all arrays ...
 */
void StereoEllipses::ClearResults()
{
	ellipse3ds.Clear();
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
			if(!vcore[side]->use_masking || !core_ellipse->IsMasked())													/// TODO use_masking ist immer true => damit immer true!
			{
				TmpEllipse ellipse(core_ellipse);
				if(ellipse.IsValid())
					ellipses[side].PushBack(ellipse);
			}
		}
		
		/// TODO Insert RePrune
// 		if(pPara.pruning)
// 			for(unsigned i = 0; i < cubes[side].Size(); i++)
// 				cubes[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);

		// Rectify ellipse points.
		for(unsigned i = 0; i < ellipses[side].Size(); i++)
			ellipses[side][i].Rectify(stereo_cam, side);
		for(unsigned i = 0; i < ellipses[side].Size(); i++)
			ellipses[side][i].Refine();
	}

  // do stereo matching and depth calculation
	ellMatches = 0;	
	MatchEllipses(ellipses[LEFT], ellipses[RIGHT], ellMatches);
	candMatches = ellMatches;
	Calculate3DEllipses(ellipses[LEFT], ellipses[RIGHT], ellMatches, ellipse3ds);
}


/**
 * @brief Match and calculate 3D rectangles from 2D rectangles.
 * @param side LEFT/RIGHT image of stereo.images.
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








