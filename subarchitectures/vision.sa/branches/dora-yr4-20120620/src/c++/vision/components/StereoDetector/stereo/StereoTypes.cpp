/**
 * @file StereoTypes.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated features.
 */

#include "StereoTypes.h"

namespace Z
{
  
  
//-----------------------------------------------------------------//
//------------------------ static helpers -------------------------//
//-----------------------------------------------------------------//

/**
 * @brief Add edgels from a segment edgel list to an OpenCV point array.
 * @																																												/// TODO
 */
static void AddEdgels(CvPoint2D32f *edgels, int &num_edgels, const Array<Edgel> &new_edgels, unsigned start_idx, unsigned end_idx)
{
  for(unsigned i = start_idx; i <= end_idx; i++)
    edgels[num_edgels++] = cvPoint2D32f(new_edgels[i].p.x, new_edgels[i].p.y);
}


/**
 * @brief Remove short lines from array of lines, dependent to the longest line (10% frontier)
 * Used to sort out the shortest lines from an array of lines (closures) 
 * @param lines Array of lines
 */
static void RefineLines(vector<HelpLine> &lines)
{
  // minimum ratio of length to maximum length
  static double LENGTH_THR_FACTOR = 0.1;
  vector<Vector2> points(lines.size());
  vector<double> lengths(lines.size());
  bool done = false;
  while(!done)
  {
    double length_max = 0.;
    bool erased_short_line = false;
    for(unsigned i = 0; i < lines.size(); i++)
    {
      // line i-1 -> point i -> line i
      unsigned j = (i != 0 ? i - 1 : lines.size() - 1);
      points[i] = LineIntersection(lines[j].p, lines[j].d,
                                   lines[i].p, lines[i].d);
    }
    for(unsigned i = 0; i < lines.size(); i++)
    {
      // point i -> line i -> point i+1
      unsigned j = (i < lines.size() - 1 ? i + 1 : 0);
      lengths[i] = Distance(points[i], points[j]);
      length_max = max(length_max, lengths[i]);
    }
    for(unsigned i = 0; i < lines.size() && !erased_short_line; i++)
    {
      // don't erase if only 3 lines left
      if(lengths[i] < length_max*LENGTH_THR_FACTOR && lines.size() > 3)
      {
        lines.erase(lines.begin() + i);
        erased_short_line = true;
      }
    }
    done = !erased_short_line;
  }
}


//----------------------------------------------------------------//
//-------------------------- Vertex2D ----------------------------// 
//----------------------------------------------------------------//
/**
 * @brief Recalculate the point, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void Vertex2D::RePrune(int oX, int oY, int sc)
{
	p.x = (oX + p.x)/sc;
	p.y = (oY + p.y)/sc;
}

/**
 * @brief Rectify 2D point
 * @param stereo_cam Stereo camera paramters and functions.
 * @param side LEFT/RIGHT side of stereo rig.
 */
void Vertex2D::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  stereo_cam->RectifyPoint(p.x, p.y, pr.x, pr.y, side);
  rectified_valid = true;
}

/**
 * // TODO Not implementation yet.
 * @brief Refine tmp. surface
 */
void Vertex2D::Refine()
{
// 	printf("StereoBase:Vertex2D::Refine: Not yet implemented!\n");
}

/**
 * @brief Returns true, if the point is at this position in the image.
 * @param x X-coordinate in the image
 * @param y Y-coordinate in the image
 * @return Returns true if the point is at this position in the image.
 */
bool Vertex2D::IsAtPosition(int x, int y) const
{
	if(fabs(p.x-x) < 2 && fabs(p.y-y) <2) return true;
  return false;
}

/**
 * @brief Draw the surface unrectified.
 * @param col RGB color
 */
void Vertex2D::Draw()
{
  DrawPoint2D(p.x, p.y);
}

//----------------------------------------------------------------//
//---------------------------- Surf2D ----------------------------//
//----------------------------------------------------------------//
/**
 * @brief Init tmp. surface with data of a rectangle.
 * Set the size of the point-vectors (p, pr) and copy corner coordinates to p.
 * @param p Points to init the surface.
 */
void Surf2D::Init(vector<Vector2> points)
{
  p.resize(points.size());
  pr.resize(points.size());
  
  for(unsigned i=0; i<points.size(); i++)
    p[i] = points[i];

  is_valid = true;
}

/**
 * @brief Init 2D surface with data from a closure.
 * 												TODO Why calculation with edgels ...? Whats going on here?
 * @param clos Closure to init surface
 */
void Surf2D::Init(Closure *clos)
{
  // array of edgel points, 10000 should be enough, i.e. a random segfault will
  // appear at some possibly distant point in the future :)
  CvPoint2D32f edgels[10000];
  int num_edgels = 0;
  vector<HelpLine> lines;
  unsigned first_l_jct = 0, i = 0;
  bool full_round = false;

  is_valid = false;

  // Notes:
  // The closure has two arrays jcts and colls of same size. If a
  // junction between two consecutive lines is an L-jct, jct[i] points to the
  // according L-jct and coll[i] == 0. Otherwise jct[i] == 0 and coll[i] points
  // to the respective collinearity.
  //
  // lines (and junctions) are in counter-clockwise order
  //
  // jct i is the jct between line i-1 and line i
  // so i-1 is the LEFT and i the RIGHT line of L-jct i
  
  // move to the first L-jct
  while(i < clos->jcts.Size() && clos->jcts[i] == 0)
    i++;
  // note: in case clos is a circle, we have only collinearities!
  if(i == clos->jcts.Size())
    return;
  first_l_jct = i;

  while(!full_round)
  {
    // add edgels of RIGHT line of L-jct i, i.e. line i
    VisibleLine *line = (VisibleLine*)clos->lines[i];
    AddEdgels(edgels, num_edgels, line->seg->edgels, line->idx[START], line->idx[END]);
    i = clos->jcts.CircularNext(i);

    // if we have reached the next L-jct, our "straight" line is complete
    // fit line to edgels
    if(clos->jcts[i] != 0)
    {
      float line_params[4];
      CvMat tmp = cvMat(num_edgels, 1, CV_32FC2, edgels);
      cvFitLine(&tmp, CV_DIST_L2, 0, 0.01, 0.01, line_params);
      lines.push_back(HelpLine(line_params[2], line_params[3],
                              line_params[0], line_params[1]));
      // and start new line
      num_edgels = 0;
    }

    // if we have come round
    if(i == first_l_jct)
      full_round = true;
  }
  // we can't do anything with less than 3 lines
  if(lines.size() < 3)
    return;

  // remove short lines
  RefineLines(lines);

  // calculate corner points
  p.resize(lines.size());
  pr.resize(lines.size());
  for(i = 0; i < lines.size(); i++)
  {
    unsigned j = (i < lines.size() - 1 ? i + 1 : 0);
    p[i] = LineIntersection(lines[i].p, lines[i].d, lines[j].p, lines[j].d);
  }

  is_valid = true;
}


/**
 * @brief Init tmp. surface with data of a rectangle.
 * Set the size of the point-vectors (p, pr) and copy corner coordinates to p.
 * @param rectangle Rectangle to init the surface.
 */
void Surf2D::Init(Rectangle *rectangle)
{
  p.resize(4);
  pr.resize(4);

  for(unsigned i=0; i<4; i++)
    p[i] = rectangle->isct[i];

  is_valid = true;
}


/**
 * @brief Init tmp. surface with data of a cube.
 * Set the size of the point-vectors (p, pr) and copy corner coordinates to p.
 * @param cube Cube to init the surface.
 * @param int Top (0), Right (1) or Left (2) side of the cube.
 * TODO Surfaces of cubes may be ambiguous.
 */
void Surf2D::Init(Cube *cube, int side)
{
// printf("StereoBase: Surf2D::Init(Cube *cube, int side): Implemented, but untested\n");

  p.resize(4);
  pr.resize(4);

  if(side == 0)	// => first rectangle 0-1-2-3
  {
    p[0] = cube->cornerPoint[0];
    p[1] = cube->cornerPoint[1];
    p[2] = cube->cornerPoint[2];
    p[3] = cube->cornerPoint[3];
  }
  else if(side == 1)	// => second rectangle: 0-3-4-5
  {
    p[0] = cube->cornerPoint[0];
    p[1] = cube->cornerPoint[3];
    p[2] = cube->cornerPoint[4];
    p[3] = cube->cornerPoint[5];
  }
  else if(side == 2)	// => third rectangle: 0-5-6-1
  {
    p[0] = cube->cornerPoint[0];
    p[1] = cube->cornerPoint[5];
    p[2] = cube->cornerPoint[6];
    p[3] = cube->cornerPoint[1];
  }
  else
    printf("Surf2D::Init: False cube side.\n");

  is_valid = true;
}


/**
 * @brief Shift points (and rectified ones) offs to the left: p[i] <- p[i+offs]
 * @param offs Number of point shifts.
 */
void Surf2D::ShiftPointsLeft(unsigned offs)
{
  assert(p.size() == pr.size()); 
  vector<Vector2> t(p.size());
  for(unsigned i = 0; i < p.size(); i++)
    t[i] = p[(i + offs)%p.size()];
  p = t;
  for(unsigned i = 0; i < p.size(); i++)
    t[i] = pr[(i + offs)%p.size()];
  pr = t;
}


/**
 * @brief Recalculate the surface points, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void Surf2D::RePrune(int oX, int oY, int sc)
{
	for(unsigned i = 0; i < p.size(); i++)
	{
		p[i].x = (oX + p[i].x)/sc;
		p[i].y = (oY + p[i].y)/sc;
	}
}

/**
 * @brief Rectify tmp. surface
 * @param stereo_cam Stereo camera with paramters and functions.
 * @param side LEFT/RIGHT side of stereo rig.
 */
void Surf2D::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  for(unsigned i = 0; i < p.size(); i++)
    stereo_cam->RectifyPoint(p[i].x, p[i].y, pr[i].x, pr[i].y, side);
}

/**
 * // TODO Not implementation yet.
 * @brief Refine tmp. surface
 */
void Surf2D::Refine()
{
// 	printf("StereoBase:Surf2D::Refine: Not yet implemented!\n");
}

/**
 * @brief Returns true, if the surface is at this position in the image.
 * @param x X-coordinate in the image
 * @param y Y-coordinate in the image
 * @return Returns true if surface is at this position in the image.
 */
bool Surf2D::IsAtPosition(int x, int y) const
{
  const double GRAB_DIST = 2.;
  try
  {
    int ncorners = p.size();
    for(int i = 0; i < ncorners; i++)
    {
      int j = (i < ncorners - 1 ? i + 1 : 0);
      Vector2 d = Normalise(p[j] - p[i]);
      if(AbsDistPointToLine(Vector2((double)x, (double)y), p[i], d) <= GRAB_DIST)
				return true;
    }
  }
	catch (exception &e)																																		/// TODO delete, if not occours
  {
		// normalise might divide by zero, ignore and later return false TODO
		printf("StereoBase:Surf2D::IsAtPosition: Exception: Normalise-Problem?\n");
    cout << e.what() << endl;
  }
  return false;
}


/**
 * @brief Draw the surface unrectified.
 * @param col RGB color
 */
void Surf2D::Draw(unsigned detail)
{
  unsigned m, s = p.size();
  for(unsigned i = 0; i < s; i++)
  {
    m = i+1; 
    if(s <= m) m = 0;
    DrawLine2D(p[i].x, p[i].y, p[m].x, p[m].y);

    if(detail == 2)
    {
      char text[20];
      snprintf(text, 20, "%u", i);
      DrawText2D(text, p[i].x, p[i].y-5, RGBColor::blue);
    }
  }
}

//--------------------------------------------------------------//
//-------------------------- Vertex3D --------------------------//
//--------------------------------------------------------------//
/**
 * @brief Check whether some sanity checks hold.
 * @return Returns true, if x,y,z-distances are within the defined 
 * SOI boundaries.
 */
bool Vertex3D::SanityOK()
{
  bool sanity = true;
  if (p.x < SC_MIN_DIST_X || p.x > SC_MAX_DIST_X) sanity = false;
  else if(p.y < SC_MIN_DIST_Y || p.y > SC_MAX_DIST_Y) sanity = false;
  else if(p.z < SC_MIN_DIST_Z || p.z > SC_MAX_DIST_Z) sanity = false;
  
  return sanity;
}

/**
 * @brief Reconstruct surface in 3D space.
 * E.g. wrong matches tend to produce very elongated surfaces.
 * @param left Left tmp. surface
 * @param right Right tmp. surface
 * @return Return true for success.
 */
bool Vertex3D::Reconstruct(cast::StereoCamera *stereo_cam, Vertex2D &left, Vertex2D &right)
{
  stereo_cam->ReconstructPoint(left.pr.x, left.pr.y, left.pr.x - right.pr.x, p.x, p.y, p.z);

  n.x = 1.;
  n.y = 0.;
  n.z = 0.;

  // check sanity
  if(SanityOK())
    return true;
  else
    return false;
}

/**
 * @brief Recunstruct surface in 3D space.
 * E.g. wrong matches tend to produce very elongated surfaces.
 * @param left Left tmp. surface
 * @param right Right tmp. surface
 * @return Return true for success.
 */
double Vertex3D::Distance(Vertex3D point)
{
  double x = point.p.x - p.x;
  double y = point.p.y - p.y;
  double z = point.p.z - p.z;
  
  return sqrt(x*x + y*y + z*z);
}


//----------------------------------------------------------------//
//---------------------------- Surf3D ----------------------------//
//----------------------------------------------------------------//
/**
 * @brief Check whether normal vectors are consistent.
 * In the ideal case all normal vectors should be equal.
 * Reject any surface, where normal vector deviates significantly from
 * the mean.
 * @return Return false, if normals vector deviates significantly from the mean.
 */
bool Surf3D::NormalsOK()
{
  unsigned n = vertices.Size();
  Vector3 mean_normal = Vector3(0., 0., 0.);
  bool ok = true;

  for(unsigned j = 0; j < n; j++)
    mean_normal += vertices[j].n;
  mean_normal /= (double)n;
  for(unsigned j = 0; j < n; j++)
    if(acos(Dot(mean_normal, vertices[j].n)) > SC_MAX_NORMAL_DEVIATION)
      ok = false;
  return ok;
}

/**
 * @brief Check whether some minimum size requirement is met.
 * @return Return false, if minimum size 
 */
bool Surf3D::SizeOK()
{
  unsigned n = vertices.Size();
  double circ = 0.;
  for(unsigned j = 0; j < n; j++)
  {
    unsigned j_n = (j < n - 1 ? j + 1 : 0);
    circ += Length(vertices[j_n].p - vertices[j].p);
  }
  return circ >= SC_MIN_CIRC;
}

/**
 * @brief Check whether some sanity checks hold.
 * E.g. wrong matches tend to produce very elongated surfaces.
 * @return Return true, if the length of a surface line, is within
 * the defined maximum length.
 */
bool Surf3D::SanityOK()
{
  unsigned n = vertices.Size();
  bool ok = true;
  for(unsigned j = 0; j < n; j++)
  {
    unsigned j_n = (j < n - 1 ? j + 1 : 0);
    if(Length(vertices[j_n].p - vertices[j].p) > SC_MAX_LENGTH)
      ok = false;
  }
  return ok;
}


/**
 * @brief Fit a proper plane through the vertex points and project points onto that plane.
 * TODO TODO TODO TODO This works only with sufaces with 4 vertices, otherwise you will get 
 * an error message.
 * TODO TODO TODO Auf beliebige Anzahl an vertices ausbauen (Matrizengröße anpassen.)
 */
void Surf3D::RefineVertices()
{
	if(vertices.Size() != 4)
	{
		printf("Surf3D::RefineVertices: try to refine surface with more than four vertices.\n");
		return;
	}

  CvMat *A = cvCreateMat((int)vertices.Size(), 4, CV_32FC1);
  CvMat *W = cvCreateMat(4, 1, CV_32FC1);
  CvMat *VT = cvCreateMat(4, 4, CV_32FC1);
  for(int i = 0; i < (int)vertices.Size(); i++)
  {
    cvmSet(A, i, 0, vertices[i].p.x);
    cvmSet(A, i, 1, vertices[i].p.y);
    cvmSet(A, i, 2, vertices[i].p.z);
    cvmSet(A, i, 3, 1.);
//     printf("A[d,...]: %.3f %.3f %.3f %.3f\n", cvmGet(A, i, 0), cvmGet(A, i, 1), cvmGet(A, i, 2), cvmGet(A, i, 3));
  }

  // solve for a x + b y + c z + d = 0
  // subject to constraint |a, b, c, d| = 1
  cvSVD(A, W, NULL, VT, CV_SVD_MODIFY_A | CV_SVD_V_T);
  // solution is last row of VT (last column of V)
//   printf("VT: %.3f %.3f %.3f %.3f\n", cvmGet(VT, 3, 0), cvmGet(VT, 3, 1),
//       cvmGet(VT, 3, 2), cvmGet(VT, 3, 3));
  // plane normal vector, not normalised yet
  Vector3 n(cvmGet(VT, 3, 0), cvmGet(VT, 3, 1), cvmGet(VT, 3, 2));
//   printf("n: [%.3f %.3f %.3f]\n", n.x, n.y, n.z);
  // normalise plane equation

  double d = cvmGet(VT, 3, 3)/Norm(n);
  n = Normalise(n);
  // some plane point (in this case the plane point closest to origin)
  Vector3 p = -d*n;
  for(int i = 0; i < (int)vertices.Size(); i++)
  {
    // project vertex point to plane
    vertices[i].p -= n*Dot(vertices[i].p - p, n);
    vertices[i].n = n;
  }

  cvReleaseMat(&A);
  cvReleaseMat(&W);
  cvReleaseMat(&VT);
}


/**
 * @brief Recunstruct surface in 3D space.
 * E.g. wrong matches tend to produce very elongated surfaces.
 * @param left Left tmp. surface
 * @param right Right tmp. surface
 * @param refine Try to fit vertices to a plane? The surface may have more than 4 vertices,
 * but only the first four will be considered.
 * @return Return true for success.
 */
bool Surf3D::Reconstruct(cast::StereoCamera *stereo_cam, Surf2D &left, Surf2D &right, bool refine)
{
  assert(left.pr.size() == right.pr.size());
  unsigned n = left.pr.size();

  vertices.Resize(n);
  for(unsigned j = 0; j < n; j++)
  {
    stereo_cam->ReconstructPoint(left.pr[j].x, left.pr[j].y, left.pr[j].x - right.pr[j].x,
        vertices[j].p.x, vertices[j].p.y, vertices[j].p.z);
  }
  for(unsigned j = 0; j < n; j++)
  {
    unsigned j_p = (j > 0 ? j - 1 : n - 1);
    unsigned j_n = (j < n - 1 ? j + 1 : 0);
    Vector3 a = vertices[j_p].p - vertices[j].p;
    Vector3 b = vertices[j_n].p - vertices[j].p;
    vertices[j].n = Normalise(Cross(a, b));
  }
  if(NormalsOK() && SizeOK() && SanityOK())
  {
    if(refine) RefineVertices();
    return true;
  }
  else
  {
    return false;
  }
}




}


