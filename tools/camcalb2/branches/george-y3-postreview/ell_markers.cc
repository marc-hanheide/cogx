 /**
 * $Id$
 *
 * @author  Michael Zillich,
 *      <A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 * @date August 2006
 *
 * TODO: make finding of lines robust, avoid max_dist
 */

#include <math.h>
#include <assert.h>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>
extern "C" {
#include "canny.h"
#include "vector2.h"
}
#include "ell_markers.hh"

using namespace std;

static const double two_pi = 2.*M_PI;

struct RGBColor
{
  unsigned char r, g, b;
};

static inline CvPoint2D64f operator+(CvPoint2D64f a, CvPoint2D64f b)
{
  CvPoint2D64f c = {a.x + b.x, a.y + b.y};
  return c;
}

static inline CvPoint2D64f operator-(CvPoint2D64f a, CvPoint2D64f b)
{
  CvPoint2D64f c = {a.x - b.x, a.y - b.y};
  return c;
}

static inline float Length(CvPoint2D64f p)
{
  return sqrt(p.x*p.x + p.y*p.y);
}

static inline float Cross(CvPoint2D64f a, CvPoint2D64f b)
{
  return a.x*b.y - a.y*b.x;
}

/// Returns signed distance of point q from line defined by points p1 and p2.
static inline float DistPointToLine(CvPoint2D64f p1, CvPoint2D64f p2,
    CvPoint2D64f q)
{
  CvPoint2D64f dir = p2 - p1;
  CvPoint2D64f q_to_line = q - p1;
  return Cross(q_to_line, dir)/Length(dir);
}

/// Check if point q lies on line p1-p2, where max_dist is the maximum distance
/// allowed.
static inline bool InLine(CvPoint2D64f p1, CvPoint2D64f p2, CvPoint2D64f q,
    float max_dist)
{
  return fabs(DistPointToLine(p1, p2, q)) <= max_dist;
}

static inline float Dist(float x1, float y1, float x2, float y2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

static inline float Dist(CvPoint2D64f p1, CvPoint2D64f p2)
{
  return Dist(p1.x, p1.y, p2.x, p2.y);
}

static inline CvPoint2D64f Rotate(CvPoint2D64f a, float phi)
{
  double si = sin(phi), co = cos(phi);
  CvPoint2D64f b = {co*a.x - si*a.y, si*a.x + co*a.y};
  return b;
}

/**
 * Transform a point from image to ellipse co-ordinates.
 */
static inline CvPoint2D64f TransformToEllipse(float x, float y, float phi,
    CvPoint2D64f p)
{
  CvPoint2D64f c = {x, y};
  return Rotate(p - c, -phi);
}

/**
 * Approximation of the shortest absolute distance of a point to a centered,
 * axis-parallel ellipse.
 */
static float DistPointEllCentAxPar(float a, float b, CvPoint2D64f p)
{
  float x2, y2, n, d;
  float a2 = a*a, b2 = b*b;
  float a4 = a2*a2, b4 = b2*b2;
  if(fpclassify(p.x) == FP_ZERO && fpclassify(p.y) == FP_ZERO)
    return b;
  x2 = p.x*p.x;
  y2 = p.y*p.y;
  n = fabs(x2/a2 + y2/b2 - 1.);
  d = 2.*sqrt(x2/a4 + y2/b4);
  return n/d;
}

/**
 * Approximation of the shortest abolute distance of a point to an ellipse.
 * The method used is gradient-weighted algebraic distance, which requires that
 * the ellipse be centered in the origin and the big axis a lies on
 * the x-axis. I.e. the point x,y must first be transformed to ellipse
 * co-ordinates. This is done inside the Distance function. So You don't have
 * to worry about that.
 */
static inline float DistPointEll(float x, float y, float a, float b, float phi,
    CvPoint2D64f p)
{
  return DistPointEllCentAxPar(a, b, TransformToEllipse(x, y, phi, p));
}

/// sort points in ascending x order
static int CmpX(const void *a, const void *b)
{
  if( ((EllipsePair*)a)->center.x < ((EllipsePair*)b)->center.x )
    return -1;  // a is first
  else
    return 1;   // b is first
}

/// sort points in descending x order
static int CmpNX(const void *a, const void *b)
{
  if( ((EllipsePair*)a)->center.x > ((EllipsePair*)b)->center.x )
    return -1;  // a is first
  else
    return 1;   // b is first
}

/// sort points in ascending y order
static int CmpY(const void *a, const void *b)
{
  if( ((EllipsePair*)a)->center.y < ((EllipsePair*)b)->center.y )
    return -1;  // a is first
  else
    return 1;   // b is first
}

/// sort points in descending y order
static int CmpNY(const void *a, const void *b)
{
  if( ((EllipsePair*)a)->center.y > ((EllipsePair*)b)->center.y )
    return -1;  // a is first
  else
    return 1;   // b is first
}

static bool operator<(const EllipsePair &a, const EllipsePair &b)
{
  return a.size < b.size;
}

/**
 * Scale angle to [0..2pi[
 */
static double ScaleAngle_0_2pi(double a)
{
  while(a >= two_pi) a -= two_pi;
  while(a < 0.) a += two_pi;
  return a;
}

/**
 * Returns whether two pixels are neighbours. Note: we allow a gap of 1 pixel.
 */
static bool AreNeighbours(t_V2i *a, t_V2i *b)
{
  return abs(a->x - b->x) <= 2 && abs(a->y - b->y) <= 2;
}

static bool IsClosedSegment(t_LL edgels)
{
  return AreNeighbours((t_V2i*)FirstElmLL(edgels), (t_V2i*)LastElmLL(edgels));
}

static bool FitEllipse(t_LL edgels, double &x, double &y, double &a,
    double &b, double &phi)
{
  int n = SizeLL(edgels), i;
  CvMat *points = 0;
  CvBox2D params;
  void *void_edgel;
  t_V2i *edgel;
  float sup = 0.0;
  const float MAX_DIST = 1.0;
  const float MIN_SUP = 0.95;

  // we need ad least 6 poitns for ellipse fitting
  if(n < 6)
    return false;
  points = cvCreateMat(n, 1, CV_32FC2);
  i = 0;
  ForeachLL_M(edgels, void_edgel)
  {
    edgel = (t_V2i*)void_edgel;
    // note that we swap x and y! (the Canny implementation uses a strange
    // co-ordinate system..)
    cvSet1D(points, i, cvScalar(edgel->y, edgel->x));
    i++;
  }
  params = cvFitEllipse2(points);
  x = params.center.x;
  y = params.center.y;
  // box size is double the axis lengths
  a = params.size.width/2.;
  b = params.size.height/2.;
  // note: the angle returned is in degrees!
  phi = ScaleAngle_0_2pi(params.angle*M_PI/180.);
  // note: for unknown reasons sometimes a < b!
  if(a < b)
  {
    swap(a, b);
    phi = ScaleAngle_0_2pi(phi + M_PI_2);
  }
  // calculate relative support
  for(i = 0; i < n; i++)
  {
    CvScalar tmp = cvGet1D(points, i);
    CvPoint2D64f p;
    p.x = tmp.val[0];
    p.y = tmp.val[1];
    if(fabs(DistPointEll(x, y, a, b, phi, p)) <= MAX_DIST)
      sup += 1.;
  }
  sup /= (float)n;
  cvReleaseMat(&points);
  return sup >= MIN_SUP;
}

static t_graphGR CreateEdgeGraph(char *rgb24, int width, int height)
{
  float alpha = 3.00;  // filter width (these are "good" values)  1.50
  float omega = 0.001; // filter parameter  0.001
  float high_thresh, low_thresh;
  RGBColor *pixbuf = (RGBColor*)rgb24;
  FARY *grey_img = 0;
  FARY *grad_dir = 0;
  FARY *grad_mag = 0;
  BARY *edge_img = 0;
  t_graphGR edge_graph;

  grey_img = makFary(height, width);
  for(int y = 0; y < height; y++)
    for(int x = 0; x < width; x++)
    {
      RGBColor *pix = &pixbuf[y*width + x];
      grey_img->el[y][x] = (float)(pix->r + pix->g + pix->b)/3.;
    }
  grad_dir = makFary(height, width);
  grad_mag = FindGradientDir(grey_img, alpha, omega, grad_dir,
                             &high_thresh, &low_thresh);
  edge_img = HysteresisThresh(grad_mag, high_thresh, low_thresh);
  edge_graph = LinkEdges(edge_img);
  destAry((ARY*)grey_img);
  destAry((ARY*)grad_dir);
  destAry((ARY*)grad_mag);
  destAry((ARY*)edge_img);
  return edge_graph;
}

static void FindEllipses(t_graphGR edge_graph, vector<Ellipse> &ellipses)
{
  void *void_edge;

  // fit ellipses to all closed edges
  ForeachLL_M(edge_graph.edges, void_edge)
  {
    t_edgeGR edge = (t_edgeGR)void_edge;
    t_LL edgels = (t_LL)edge->att;
    if(IsClosedSegment(edgels))
    {
      double x, y, a, b, phi;
      if(FitEllipse(edgels, x, y, a, b, phi))
        ellipses.push_back(Ellipse(x, y, a, b, phi));
    }
  }
}

/**
 * Filter out ellipses not belonging to the calibration pattern.
 * Accept only ellipses which have a matching concentric ellipse.
 */
static void FilterEllipses(vector<Ellipse> &ells, vector<EllipsePair> &pairs)
{
  // maximum distance allowed for the centers of two concentric ellipses
  static const float MAX_DIST = 1;
  // invariant: ellipses up to (excluding) l have a Matching concentric ellipse
  // (where matching ellipses are 0,1 and 2,3 etc.). ellipses after u have No
  // matching ellipse. ellipses in between are unknown.
  // MMMMMMMMMM??????????????????NNNNNNN
  //           l    i->         u
  // initialisation: l = 0, u = size - 1
  // stop condition: l > u
  int l = 0;
  int u = ells.size() - 1;
  int i;
  while(l <= u)
  {
    i = l + 1;
    while(i <= u &&
          Dist(ells[l].x, ells[l].y, ells[i].x, ells[i].y) > MAX_DIST)
      i++;
    if(i <= u)
    {
      // swap so that l,l+1 are a matching pair
      swap(ells[l+1], ells[i]);
      l += 2;
    }
    else
    {
      // swap so that l (now known to have no match) lies beyond u
      swap(ells[l], ells[u]);
      u--;
    }
  }
  for(i = 0; i < l; i+=2)
    pairs.push_back(EllipsePair(ells[i], ells[i+1]));
  sort(pairs.begin(), pairs.end());
  reverse(pairs.begin(), pairs.end());
}

/**
 * Return whether the lines of rectangular grid of points[l] to points [u-1] are
 * horizontal or vertical.
 */
bool LinesHorizontal(EllipsePair *points, int l, int u)
{
  // get direction of first line from two of its points
  CvPoint2D64f dir = points[l+1].center - points[l].center;
  return fabs(dir.x) > fabs(dir.y);
}

/**
 * Sort points on line in ascending x order (or decending y order, depending on
 * orientation of line).
 * points to sort are points[l] .. points[u-1]
 */
void SortLine(EllipsePair *points, int l, int u)
{
  // if rows are rather horizontal, sort after x
  if(LinesHorizontal(points, l, u))
    qsort(&points[l], u - l, sizeof(EllipsePair), CmpX);
  else
    qsort(&points[l], u - l, sizeof(EllipsePair), CmpNY);
}

/**
 * Sort points on line and check whether point distances are
 * approximately equal.  If not then a wrong background point was falsely
 * attributed to the line, return false.
 * Points to check are points[l] .. points[u-1]
 */
bool SortCheckLine(EllipsePair *points, int l, int u)
{
  SortLine(points, l, u);
  // get min and max distance
  float min = Dist(points[l+1].center, points[l].center);
  float max = min;
  for(int i = l + 1; i < u - 1; i++)
  {
    float d = Dist(points[i+1].center, points[i].center);
    if(d < min) min = d;
    if(d > max) max = d;
  }
  // if min and max differ by more than a factor 2, we have very likely a
  // false point in the line, reject it
  // (note: 2 is an educated guess ;)
  if(max > 2.*min)
    return false;
  else
    return true;
}

/**
 * Arrange points in lines.
 * When returning successfully the first rows*cols points will be arranged
 * to form a rectangular rows x cols matrix.
 * Each row is sorted internally, rows themselves must still be sorted.
 * Points not fitting in the rectangular grid will be moved to the end of the
 * array.
 */
bool ArrangeInLines(EllipsePair *points, int npoints, int rows, int cols)
{
  // the outer loop steadily increases the distance threshold for points
  // being on a line.
  // so for images with a small calibration pattern and thus close ellipses, no
  // spurious lines will be found because of a too tolerant threshold.
  // and for images with strong distortions (where points in fact are not
  // really on a line) the relaxed threshold will eventully still allow
  // formation of lines.
  bool found = false;
  float line_max_dist = 1.;
  float line_max_dist_limit = 10.;
  srand(117);
  for(; line_max_dist <= line_max_dist_limit && !found; line_max_dist += 1.)
  {
    // randomly sample lines using 2 points, then check if the correct number of
    // points (namely 'cols' points) lie on it and have approximately equal
    // distances. if yes, swallow these points and search for next line.
    // a maximum of 'max_iter' iterations is performed.
    //
    // invariant: points up to (excluding) l form complete lines with 'cols'
    // points and are grouped in chunks of 'cols' points. points l and l+1 form
    // the currently evaluated line. points up to (excluding) p form partial
    // line with some points.
    // CCCCCCCPPPP??????????????????
    //        l   p                 u
    // initialisation: l = 0, p = 0, u = size
    // stop condition: l >= u - 1 or max. iterations reached
    int max_iter = 1000;
    int iter = 0;
    int l = 0;
    int p = 0;
    int u = npoints;
    int i, j, k;
    while(l < u - 1 && iter < max_iter)
    {
      p = l;
      i = p + rand()%(u - p);   // i = [p, u-1]
      swap(points[p++], points[i]);
      j =  p + rand()%(u - p);  // j = [p, u-1]
      swap(points[p++], points[j]);
      for(k = p; k < u; k++)
        if(InLine(points[l].center, points[l+1].center, points[k].center,
              line_max_dist))
          swap(points[p++], points[k]);
      // if we have a complete and valid line, advance l
      if(p - l == cols && SortCheckLine(points, l, p))
        l += cols;
      iter++;
    }
    // if all points form complete lines, we are done
    if(l == rows*cols)
      found = true;
  }
  return found;
}

/**
 * Sort rows of matrix after y (or x, depending on the orientation of the lines
 * defined by the rows).
 */
void SortRows(EllipsePair *points, int rows, int cols)
{
  // we have all rows (sorted internally), now lets sort the rows 
  // check direction of first (i.e. any) line
  // if rows are rather horizontal
  if(LinesHorizontal(points, 0, cols))
    // sort rows in negativenegative  y
    qsort(points, rows, cols*sizeof(EllipsePair), CmpNY);
  else
    // sort rows in x
    qsort(points, rows, cols*sizeof(EllipsePair), CmpNX);
}

/**
 * Order ellipses in a rectangular array corresponding to the pattern model.
 */
bool OrderEllipses(vector<EllipsePair> &in, vector<EllipsePair> &out,
  Model &model)
{
  // first make sure we have enough points to begin with
  if((int)in.size() < model.NumPoints())
    return false;
  int npoints = in.size();
  int rows = model.ny;
  int cols = model.nx;
  EllipsePair points[npoints];

  out.clear();
  // copy the STL vector to a regular C array as this is easier to work with
  for(int i = 0; i < npoints; i++)
    points[i] = in[i];
  if(ArrangeInLines(points, npoints, rows - 1, cols))
  {
    SortRows(points, rows - 1, cols);
    EllipsePair *rectblock = points;
    EllipsePair *shortline = points + (rows-1)*cols;
    // now identify the short line in the rest of points
    if(ArrangeInLines(shortline, npoints - (rows-1)*cols,
       1, cols - 1))
    {
      // check which end of the short line is aligned with which end of the
      // rectangular block: top-left aligned or bottom-right aligned
      float d1 = fabs(DistPointToLine(rectblock[0].center,
          rectblock[cols*(rows-2)].center, shortline[0].center));
      float d2 = fabs(DistPointToLine(rectblock[cols-1].center,
          rectblock[cols-1+cols*(rows-2)].center, shortline[cols-2].center));
      // if bottom-right aligned
      if(d2 < d1)
      {
        // the short lines lies "below" the block, so the image is rotated
        // all orderings have to be reversed
        int b = 0, t = rows - 2;  // bottom, top
        int l, r;  // left, right
        while(b < t)
        {
          for(int i = 0; i < cols; i++)
            swap(rectblock[b*cols + i], rectblock[t*cols + i]);
          b++;
          t--;
        }
        for(b = 0; b < rows - 1; b++)
        {
          l = 0;
          r = cols - 1;
          while(l < r)
          {
            swap(rectblock[b*cols + l], rectblock[b*cols + r]);
            l++;
            r--;
          }
        }
        l = 0;
        r = cols - 2;
        while(l < r)
        {
          swap(shortline[l], shortline[r]);
          l++;
          r--;
        }
      }
      // else the short lines lies "above" the block, as in the model -> we can
      // just copy points
      for(int i = 0; i < (rows - 1)*cols; i++)
        out.push_back(rectblock[i]);
      for(int i = 0; i < cols - 1; i++)
        out.push_back(shortline[i]);
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

/// find white point halfway between center and inner ellipse
static CvPoint FindWhitePoint(const EllipsePair &ellp)
{
  Ellipse ell;
  // inner (smaller) ellipse
  if(ellp.ell1.a < ellp.ell2.a)
    ell = ellp.ell1;
  else
    ell = ellp.ell2;
  // some point on major axis, inside ellipse
  return cvPoint((int)(ell.x + ell.a*cos(ell.phi)*0.66),
                 (int)(ell.y + ell.a*sin(ell.phi)*0.66));
}

/**
 * Find area centers of blobs at ellipse centers.
 */
void FindBlobCenters(char *rgb24, int width, int height, vector<EllipsePair> &ells)
{
  // grey scale image
  IplImage *grey = cvCreateImage(cvSize(width, height), 8, 1);
  char *greyData = (char*)grey->imageData;
  for(int y = 0; y < height; y++)
    for(int x = 0; x < width; x++)
    {
      RGBColor c = (RGBColor&)rgb24[y*width + x];
      greyData[y*grey->widthStep + x] = (c.r + c.g + c.b)/3;
    }

  // mask image
  IplImage *mask = cvCreateImage(cvSize(width+2, height+2), 8, 1);
  memset(mask->imageData, 0, (width+2)*(height+2));

  for(unsigned i = 0; i < ells.size(); i++)
  {
    CvMoments moments;
    CvMat subimg;
    CvConnectedComp ccomp;
    // black center point
    CvPoint c = cvPoint((int)ells[i].center.x, (int)ells[i].center.y);
    unsigned char black = (unsigned char)greyData[c.y*grey->widthStep + c.x];
    // white point halfway between center and smaller ellipse rim
    CvPoint w = FindWhitePoint(ells[i]);
    unsigned char white = (unsigned char)greyData[w.y*grey->widthStep + w.x];
    cvFloodFill(grey, c, cvScalarAll(1),
        cvScalarAll(black), cvScalarAll((white - black)/2),
        &ccomp, 8 | CV_FLOODFILL_FIXED_RANGE | CV_FLOODFILL_MASK_ONLY, mask);
    // note that mask image has a border of one pixel
    ccomp.rect.x += 1;
    ccomp.rect.y += 1;
    cvGetSubRect(mask, &subimg, ccomp.rect);
    cvMoments(&subimg, &moments, 1);
    double m_00 = cvGetSpatialMoment(&moments, 0, 0);
    double m_10 = cvGetSpatialMoment(&moments, 1, 0);
    double m_01 = cvGetSpatialMoment(&moments, 0, 1);
    // get area center, add roi offset and subtract 1 pixel (the mask border)
    ells[i].center.x = m_10/m_00 + ccomp.rect.x - 1;
    ells[i].center.y = m_01/m_00 + ccomp.rect.y - 1;
  }

  cvReleaseImage(&grey);
  cvReleaseImage(&mask);
}

void FindCorrespondences(vector<EllipsePair> &in, vector<EllipsePair> &out,
  Model &model)
{
  float h[9];
  CvMat H = cvMat(3, 3, CV_32FC1, h);
  CvPoint2D64f obj[4], img[4];
  CvMat obj_mat = cvMat(4, 2, CV_32FC1, obj);
  CvMat img_mat = cvMat(4, 2, CV_32FC1, img);
  int trials = 2*(model.nx - 1) + 2*(model.ny - 1) - 1;

  // find convex hull of points
  // ...

  for(int i = 0; i < trials; i++)
  {
    // ...
  }
}

/**
 * Detect rectangular grid of ellipse markers (which are rings, i.e. pairs of
 * concentric circles).
 * @param rgb24  image buffer containing RGBRGBRGB... in chunks of 24 bits
 * @param width  width of image buffer
 * @param height  height of image buffer
 * @param model  calibration model containing numbers and spacings of points
 * @param centers  array of detected ellipse centers, ordered in rectangular
 *                 grid
 * @param out_ells  all detected ellipses (interesting for debug purposes only)
 * @param out_pairs  all detected pairs of ellipses, i.e. markers (interesting
 *                   for debug purposes only)
 */
bool DetectEllipseMarkers(char *rgb24, int width, int height,
  Model &model, vector<CvPoint2D64f> &centers, vector<Ellipse> &out_ells,
  vector<EllipsePair> &out_pairs, vector<CvPoint2D64f> &all_edgels)
{
  bool success = false;
  vector<Ellipse> ellipses;
  vector<EllipsePair> pairs;
  vector<EllipsePair> ordered_pairs;

  t_graphGR edge_graph = CreateEdgeGraph(rgb24, width, height);
  FindEllipses(edge_graph, ellipses);
  {
    // HACK
    void *void_edge;
    ForeachLL_M(edge_graph.edges, void_edge)
    {
      t_edgeGR edge = (t_edgeGR)void_edge;
      t_LL edgels = (t_LL)edge->att;
      void *void_edgel;
      t_V2i *edgel;
      ForeachLL_M(edgels, void_edgel)
      {
        edgel = (t_V2i*)void_edgel;
        CvPoint2D64f e;
        e.x = edgel->y;
        e.y = edgel->x;
        all_edgels.push_back(e);
      }
    }
  }
  DestGR(edge_graph);
  if((int)ellipses.size() >= 2*model.NumPoints())
  {
    FilterEllipses(ellipses, pairs);
    if((int)pairs.size() >= model.NumPoints())
    {
      if(OrderEllipses(pairs, ordered_pairs, model))
      {
        // Does not work properly
        // FindBlobCenters(rgb24, width, height, ordered_pairs);
        for(unsigned i = 0; i < ordered_pairs.size(); i++)
          centers.push_back(ordered_pairs[i].center);
        success = true;
      }
    }
  }
  out_ells = ellipses;
  out_pairs = pairs;
  return success;
}

