/**
 * $Id: StereoCore.cc,v 1.3 2008/10/07 20:21:37 mz Exp $
 *
 * @author Michael Zillich
 * @date September 2006
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "Closure.hh"
#include "Segment.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Flap.hh"
#include "StereoCore.hh"
#include "Draw.hh"

#define ZSC_DEBUG

namespace Z
{

extern void SetActiveDrawArea(IplImage *iI);

struct TmpLine
{
  Vector2 p;  // some point on the line
  Vector2 d;  // direction of the line
  TmpLine(float px, float py, float dx, float dy) : p(px, py), d(dx, dy) {}
};

/**
 * Add edgels from a segment edgel list to an OpenCV point array.
 */
static void AddEdgels(CvPoint2D32f *edgels, int &num_edgels,
    const Array<Edgel> &new_edgels, unsigned start_idx, unsigned end_idx)
{
  for(unsigned i = start_idx; i <= end_idx; i++)
    edgels[num_edgels++] = cvPoint2D32f(new_edgels[i].p.x, new_edgels[i].p.y);
}

/**
 * Remove short lines from array of lines.
 */
static void RefineLines(vector<TmpLine> &lines)
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

void TmpSurf::Init(Closure *clos)
{
  // array of edgel points, 10000 should be enough, i.e. a random segfault will
  // appear at some possibly distant point in the future :)
  CvPoint2D32f edgels[10000];
  int num_edgels = 0;
  vector<TmpLine> lines;
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
    AddEdgels(edgels, num_edgels, line->seg->edgels, line->idx[START],
        line->idx[END]);
    i = clos->jcts.CircularNext(i);

    // if we have reached the next L-jct, our "straight" line is complete
    // fit line to edgels
    if(clos->jcts[i] != 0)
    {
      float line_params[4];
      CvMat tmp = cvMat(num_edgels, 1, CV_32FC2, edgels);
      cvFitLine(&tmp, CV_DIST_L2, 0, 0.01, 0.01, line_params);
      lines.push_back(TmpLine(line_params[2], line_params[3],
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

  id = clos->ID();

  is_valid = true;
}

/**
 * Shift points (and rectified ones) offs to the left:
 * p[i] <- p[i+offs]
 */
void TmpSurf::ShiftPointsLeft(unsigned offs)
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

void TmpSurf::Rectify(StereoCamera *stereo_cam, int side)
{
  for(unsigned i = 0; i < p.size(); i++)
    stereo_cam->RectifyPoint(p[i].x, p[i].y, pr[i].x, pr[i].y, side);
}

void TmpSurf::Refine()
{
}

bool TmpSurf::IsAtPosition(int x, int y) const
{
  const double GRAB_DIST = 2.;
  try
  {
    int ncorners = p.size();
    for(int i = 0; i < ncorners; i++)
    {
      int j = (i < ncorners - 1 ? i + 1 : 0);
      Vector2 d = Normalise(p[j] - p[i]);
      if(AbsDistPointToLine(Vector2((double)x, (double)y), p[i], d)
         <= GRAB_DIST)
        return true;
    }
  }
  catch(Except &e)
  {
    // normalise might divide by zero, ignore and later return false
  }
  return false;
}

/**
 * Check whether normal vectors are consistent.
 * In the ideal case all normal vectors should be equal.
 * Reject any surface, where normal vector deviates significantly from
 * the mean.
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
 * Check whether some minimum size requirement is met.
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
 * Check whether some sanity checks hold.
 * E.g. wrong matches tend to produce very elongated surfaces.
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
 * Fit a proper plane through the vertex points and project points onto that
 * plane.
 */
void Surf3D::RefineVertices()
{
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

bool Surf3D::Reconstruct(TmpSurf &left, TmpSurf &right, StereoCamera *cam)
{
  assert(left.pr.size() == right.pr.size());
  unsigned n = left.pr.size();

  vertices.Resize(n);
  for(unsigned j = 0; j < n; j++)
  {
    cam->ReconstructPoint(left.pr[j].x, left.pr[j].y,
        left.pr[j].x - right.pr[j].x,
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
    RefineVertices();
    return true;
  }
  else
  {
    return false;
  }
}

TmpFlap::TmpFlap(Flap *flap)
{
  surf[0].Init(flap->clos[0]);
  surf[1].Init(flap->clos[1]);
}

void TmpFlap::Rectify(StereoCamera *stereo_cam, int side)
{
  surf[0].Rectify(stereo_cam, side);
  surf[1].Rectify(stereo_cam, side);
}

void TmpFlap::Refine()
{
  surf[0].Refine();
  surf[1].Refine();
}

bool TmpFlap::IsAtPosition(int x, int y) const
{
  return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y);
}

/**
 * If a flap from the right image was matched with a flap from the left image we
 * typically have to shift the point arrays of the right surfaces to align with
 * the left points and maybe swap surface 0 and 1 of the right flap.
 * Note that we first shift points and then swap surfaces.
 */
void TmpFlap::Fuddle(unsigned off0, unsigned off1, bool swap)
{
  surf[0].ShiftPointsLeft(off0);
  surf[1].ShiftPointsLeft(off1);
  if(swap)
  {
    TmpSurf t = surf[1];
    surf[1] = surf[0];
    surf[0] = t;
  }
}

bool Flap3D::Reconstruct(TmpFlap &left, TmpFlap &right, StereoCamera *cam)
{
  bool ok0 = surf[0].Reconstruct(left.surf[0], right.surf[0], cam);
  bool ok1 = surf[1].Reconstruct(left.surf[1], right.surf[1], cam);
  return ok0 && ok1;
}

StereoCore::StereoCore(const string &stereocal_file)
    throw(Except)
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    // create vision core, don't specify config file, we'll configure ourselves
    vcore[side] = new VisionCore();
    // hardwire the gestalt principles we need, saves loading a config file
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_RECTANGLES);
    vcore[side]->EnableGestaltPrinciple(GestaltPrinciple::FORM_FLAPS);
  }
  stereo_cam = new StereoCamera();
  stereo_cam->ReadSVSCalib(stereocal_file);
  nmatches = 0;
}

StereoCore::~StereoCore()
{
  delete vcore;
  delete stereo_cam;
}

int StereoCore::NumSurfaces2D(int side)
{
  assert(side == LEFT || side == RIGHT);
  return surfs[side].Size();
}

const TmpSurf &StereoCore::Surfaces2D(int side, int i)
{
  assert(side == LEFT || side == RIGHT);
  return surfs[side][i];
}

int StereoCore::NumFlaps2D(int side)
{
  assert(side == LEFT || side == RIGHT);
  return flaps[side].Size();
}

const TmpFlap &StereoCore::Flaps2D(int side, int i)
{
  assert(side == LEFT || side == RIGHT);
  return flaps[side][i];
}

/**
 * match_offs  offset to be added to the indices of the right points in order to
 *             match with the left points.
 *             I.e. left_surf.p[i] corresponds to right_surf.p[i + match_offs]
 *             (of course with proper modulo)
 */
double StereoCore::MatchingScore(TmpSurf &left_surf, TmpSurf &right_surf,
    unsigned &match_offs)
{
  static const double MAX_DELTA_V = 10.;   // TODO: nasty threshold
  static const double MIN_DISPARITY = 0.; // TODO: obtain from config file?
  double sumv, sumv_min = HUGE, dv, dv_max, du, du_min;
  unsigned ncorners = left_surf.pr.size(), i, j, offs;
  match_offs = UNDEF_ID;
  if(left_surf.pr.size() == right_surf.pr.size())
  {
    for(offs = 0; offs < ncorners; offs++)
    {
      sumv = 0.;
      dv_max = 0.;
      du_min = HUGE;
      for(i = 0; i < ncorners; i++)
      {
        j = (i + offs)%ncorners;

        // distance in y-dir (should be zero)
        dv = fabs(left_surf.pr[i].y - right_surf.pr[j].y);
        sumv += dv;
        dv_max = fmax(dv_max, dv);
        // distance in x-dir = disparity, must be > 0
        du = left_surf.pr[i].x - right_surf.pr[j].x;
        du_min = fmin(du_min, du);
      }
      if(dv_max < MAX_DELTA_V && du_min > MIN_DISPARITY)
        if(sumv < sumv_min)
        {
          sumv_min = sumv;
          match_offs = offs;
        }
    }
  }
  return sumv_min;
}

// unsigned StereoCore::FindMatchingSurf(TmpSurf &left_surf, 
//     Array<TmpSurf> &right_surfs, unsigned l)
// {
//   double match, best_match = HUGE;
//   unsigned j, j_best = UNDEF_ID, offs, offs_best = UNDEF_ID;
//   for(j = l; j < right_surfs.Size(); j++)
//   {
// printf(" right: %u\n", j);
//     match = MatchingScore(left_surf, right_surfs[j], offs);
//     if(match < best_match)
//     {
//       best_match = match;
//       j_best = j;
//       offs_best = offs;
//     }
//   }
//   if(j_best != UNDEF_ID)
//   {
//     right_surfs[j_best].ShiftPointsLeft(offs_best);
//   }
//   return j_best;
// }

// void StereoCore::MatchSurfaces(Array<TmpSurf> &left_surfs, Array<TmpSurf> &right_surfs, int &matches)
// {
//   unsigned j, l = 0, u = left_surfs.Size();
//   for(; l < u && l < right_surfs.Size();)
//   {
//     j = FindMatchingSurf(left_surfs[l], right_surfs, l);
//     // found a matching right, move it to same index position as left
//     if(j != UNDEF_ID)
//     {
//       right_surfs.Swap(l, j);
//       l++;
//     }
//     // found no right, move left to end and decrease end
//     else
//     {
//       left_surfs.Swap(l, u-1);
//       u--;
//     }
//   }
//   u = min(u, right_surfs.Size());
//   matches = u;
// }

void StereoCore::Calculate3DSurfs(Array<TmpSurf> &left_surfs,
    Array<TmpSurf> &right_surfs, int &matches, Array<Surf3D> &surf3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Surf3D surf3d;
    if(surf3d.Reconstruct(left_surfs[i], right_surfs[i], stereo_cam))
    {
      surf3ds.PushBack(surf3d);
      i++;
    }
    // move unacceptable surfs to the end
    else
    {
      left_surfs.Swap(i, u-1);
      right_surfs.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}

double StereoCore::MatchingScore(TmpFlap &left_flap, TmpFlap &right_flap, unsigned &off_0, unsigned &off_1, bool &cross)
{
  // _s .. straight (left flap surf 0 matches right flap surf 0)
  // _x .. crossed (left flap surf 0 matches right flap surf 1)
  unsigned off_s0, off_s1, off_x0, off_x1;
  double sc_s = MatchingScore(left_flap.surf[0], right_flap.surf[0], off_s0) +
                MatchingScore(left_flap.surf[1], right_flap.surf[1], off_s1);
  double sc_x = MatchingScore(left_flap.surf[0], right_flap.surf[1], off_x1) +
                MatchingScore(left_flap.surf[1], right_flap.surf[0], off_x0);

  // if flaps match "straight"
  if(sc_s < sc_x)
  {
    cross = false;
    off_0 = off_s0;
    off_1 = off_s1;
    return sc_s;
  }
  // else if flaps match "crossed"
  else
  {
    cross = true;
    off_0 = off_x0;
    off_1 = off_x1;
    return sc_x;
  }
}

unsigned StereoCore::FindMatchingFlap(TmpFlap &left_flap, Array<TmpFlap> &right_flaps, unsigned l)
{
// printf("    FindMatchingFlap:\n");
  double match, best_match = HUGE;
  unsigned j, j_best = UNDEF_ID;
  unsigned off_0, off_1, off_0_best = 0, off_1_best = 0;
  bool cross = false, cross_best = false;
  for(j = l; j < right_flaps.Size(); j++)
  {
    match = MatchingScore(left_flap, right_flaps[j], off_0, off_1, cross);
    if(match < best_match)
    {
      best_match = match;
      j_best = j;
      cross_best = cross;
      off_0_best = off_0;
      off_1_best = off_1;
    }
  }
  if(j_best != UNDEF_ID)
  {
    right_flaps[j_best].Fuddle(off_0_best, off_1_best, cross_best);
  }
  return j_best;
}

void StereoCore::MatchFlaps(Array<TmpFlap> &left_flaps, Array<TmpFlap> &right_flaps, int &matches)
{
  unsigned j, l = 0, u = left_flaps.Size();
  for(; l < u && l < right_flaps.Size();)
  {
    j = FindMatchingFlap(left_flaps[l], right_flaps, l);
    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_flaps.Swap(l, j);
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_flaps.Swap(l, u-1);
      u--;
    }
  }
  u = min(u, right_flaps.Size());
  matches = u;
}

void StereoCore::Calculate3DFlaps(Array<TmpFlap> &left_flaps,
    Array<TmpFlap> &right_flaps, int &matches, Array<Flap3D> &flap3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Flap3D flap3d;
    bool ok0 = flap3d.surf[0].Reconstruct(left_flaps[i].surf[0], right_flaps[i].surf[0], stereo_cam);
    bool ok1 = flap3d.surf[1].Reconstruct(left_flaps[i].surf[1], right_flaps[i].surf[1], stereo_cam);
    if(ok0 && ok1)
    {
      flap3ds.PushBack(flap3d);
      i++;
    }
    // move unacceptable flaps to the end
    else
    {
      left_flaps.Swap(i, u-1);
      right_flaps.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}

void StereoCore::ClearResults()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    vcore[side]->ClearGestalts();
    surfs[side].Clear();
    flaps[side].Clear();
  }
  surf3ds.Clear();
  flap3ds.Clear();
  nmatches = 0;
}

/**
 * @brief Process stereo image 
 * @param runtime_ms  granted runtime in [ms] for each image
 */
void StereoCore::ProcessStereoImage(/*const IplImage *left_img, const IplImage *right_img,*/ int runtime_ms)
{
  // clear previous results
  ClearResults();

  // do monocular processing for each side
  for(int side = LEFT; side <= RIGHT; side++)
  {
    vcore[side]->NewImage(side == LEFT ? /*left_img : right_img*/ img_l : img_r);
    vcore[side]->ProcessImage(runtime_ms);

		DrawFlaps();

    // note: the awkward Gestalt::FLAP thingy is necessary because the global
    // NumFlaps() and Flaps() collide with StereoCores respective methods.
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::FLAP); i++)
    {
      Flap *core_flap = (Flap*)vcore[side]->Gestalts(Gestalt::FLAP, i);
      if(!vcore[side]->use_masking || !core_flap->IsMasked())
      {
        TmpFlap flap(core_flap);
        if(flap.IsValid())
          flaps[side].PushBack(flap);
      }
    }
    for(unsigned i = 0; i < flaps[side].Size(); i++)
      flaps[side][i].Rectify(stereo_cam, side);
    for(unsigned i = 0; i < flaps[side].Size(); i++)
      flaps[side][i].Refine();
  }


  // do stereo matching and depth calculation
  nmatches = 0;
  MatchFlaps(flaps[LEFT], flaps[RIGHT], nmatches);
  Calculate3DFlaps(flaps[LEFT], flaps[RIGHT], nmatches, flap3ds);

// 	PrintResults();		// HACK Print corner points of the surfaces (left and right)
  printf("flaps left/right: %d %d\n", flaps[LEFT].Size(), flaps[RIGHT].Size());  // HACK
  printf("nmatches: %d\n", nmatches); // HACK
}

/**
 * @brief Draw flaps as overlay over the stereo image
 */
void StereoCore::DrawFlaps()
{
	SetActiveDrawAreaSide(0);			// left stereo image is draw area
	for(int i=0; i<NumFlapsLeft2D(); i++)
	{
		if (vcore[LEFT]->Gestalts(Gestalt::FLAP, i)->IsUnmasked())
  		vcore[LEFT]->Gestalts(Gestalt::FLAP, i)->Draw();	
	}
	SetActiveDrawAreaSide(1);		// right stereo image is draw area
	for(int i=0; i<NumFlapsRight2D(); i++)
	{
		if (vcore[RIGHT]->Gestalts(Gestalt::FLAP, i)->IsUnmasked())
  		vcore[RIGHT]->Gestalts(Gestalt::FLAP, i)->Draw();	
	}
}

/**
 * @brief Draw flaps as overlay over the stereo image
 * @param iIl left (openCV) iplImage of stereo rig
 * @param iIr	right  (openCV) iplImage of stereo rig
 */
void StereoCore::SetIplImages(IplImage *iIl, IplImage *iIr)
{
	img_l = iIl;
	img_r = iIr;
}

/**
 * @brief Set the active draw area to left or right stereo image.
 * @param side Left or right image
 */
void StereoCore::SetActiveDrawAreaSide(int side)
{
	if (side == 0) SetActiveDrawArea(img_l);
	if (side == 1) SetActiveDrawArea(img_r);
}

/**
 * @brief Print the corner points of the calculated flaps.
 */
void StereoCore::PrintResults()
{
	printf("RESULTS:  \n");
	for(unsigned i=0; i<flaps[LEFT].Size(); i++)
	{
		for(unsigned j=0; j<flaps[LEFT][i].surf[0].pr.size(); j++)
			printf("Points left, flap %u, surf 0:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
				i, flaps[LEFT][i].surf[0].p[j].x, flaps[LEFT][i].surf[0].p[j].y, flaps[LEFT][i].surf[0].pr[j].x, flaps[LEFT][i].surf[0].pr[j].y);
		for(unsigned j=0; j<flaps[LEFT][i].surf[1].pr.size(); j++)
			printf("Points left, flap %u, surf 1:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
				i, flaps[LEFT][i].surf[1].p[j].x, flaps[LEFT][i].surf[1].p[j].y, flaps[LEFT][i].surf[1].pr[j].x, flaps[LEFT][i].surf[1].pr[j].y);
	}
	for(unsigned i=0; i<flaps[RIGHT].Size(); i++)
	{
		for(unsigned j=0; j<flaps[RIGHT][i].surf[0].pr.size(); j++)
			printf("Points right, flap %u, surf 0:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
				i, flaps[RIGHT][i].surf[0].p[j].x, flaps[RIGHT][i].surf[0].p[j].y, flaps[RIGHT][i].surf[0].pr[j].x, flaps[RIGHT][i].surf[0].pr[j].y);
		for(unsigned j=0; j<flaps[RIGHT][i].surf[1].pr.size(); j++)
			printf("Points right, flap %u, surf 1:	p: %4.2f / %4.2f	pr: %4.2f / %4.2f\n", 
				i, flaps[RIGHT][i].surf[1].p[j].x, flaps[RIGHT][i].surf[1].p[j].y, flaps[RIGHT][i].surf[1].pr[j].x, flaps[RIGHT][i].surf[1].pr[j].y);
	}
}

}

