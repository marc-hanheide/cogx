/**
 * @file FormSegments.cc
 * @author Zillich
 * @date Februar 2007
 * @version 0.1
 * @brief Form edge segments with Matas canny edge detector.
 **/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include "canny.h"
#include "vector2.h"
}
#include "VisionCore.hh"
#include "Color.hh"
#include "Draw.hh"
#include "Segment.hh"
#include "Line.hh"
#include "Arc.hh"
#include "FormSegments.hh"
#include "rosin_lines.hh"
#include "rosin_arcline.hh"

namespace Z
{

IdImage *FormSegments::edge_img = 0;
float *FormSegments::dir_img = 0;
int FormSegments::num_edgels = 0;

static int CmpSegments(const void *a, const void *b)
{
  if( Segments(*(unsigned*)a)->sig > Segments(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormSegments::FormSegments(Config *cfg)
: GestaltPrinciple(cfg)
{
  done = false;
  cur_img = 0;
  next_principles.PushBack(FORM_LINES);
  next_principles.PushBack(FORM_ARCS);
}

FormSegments::~FormSegments()
{
  delete edge_img;
  delete[] dir_img;
}

void FormSegments::Reset(const Image *img)
{
  done = false;
  cur_img = img;
  if(cur_img == 0)
    throw Except(__HERE__, "NULL image");
  if(edge_img != 0 &&
     !(edge_img->width == (int)cur_img->Width() &&
       edge_img->height == (int)cur_img->Height()))
  {
    delete edge_img;
    edge_img = 0;
    delete[] dir_img;
    dir_img = 0;
  }
  if(edge_img == 0)
  {
    edge_img = new IdImage(cur_img->Width(), cur_img->Height());
    dir_img = new float[cur_img->Width()*cur_img->Height()];
  }
  else
  {
    edge_img->Clear();
    for(int i = 0; i < (int)cur_img->Width()*(int)cur_img->Height(); i++)
      dir_img[i] = 0.;
  }
}

/**
 * The first step after acquiring an image is to calculate Canny edges and save
 * all edge segments in a segment array.
 */
void FormSegments::Operate(bool incremental)
{
  StartRunTime();
  // note: we only want to run this once for repeated calls to Operate()
  if(!done)
  {
    Create();
    Rank();
    for(unsigned i = 0; i < NumSegments(); i++)
      DrawToEdgeImage(i);
    done = true;
  }
  StopRunTime();
}

void FormSegments::Create()
{
  CreateSegmentsMatas();
  //SegmentArcsLines();
}

void FormSegments::SegmentArcsLines()
{
  const int MAX_PIXELS = 5000;
  const int MAX_LINES = 100;
  /* note: list indexing goes from 1 (not 0!) to SIZE-1 */
  /* original list of pixels */
  float x_c[MAX_PIXELS];
  float y_c[MAX_PIXELS];
  float x_start[MAX_LINES], y_start[MAX_LINES];
  float x_end[MAX_LINES], y_end[MAX_LINES];
  float sigs[MAX_LINES];
  int line_start[MAX_LINES], line_end[MAX_LINES];
  int n_lines = MAX_LINES;
  LineSegmenter ls;
  ArcLineSegmenter als;

  for(unsigned rank = 0; rank < NumSegments(); rank++)
  {
    unsigned seg = RankedGestalts(Gestalt::SEGMENT, rank);
    for(unsigned i = 1; i <= Z::Segments(seg)->edgels.Size(); i++)
    {
      x_c[i] = Z::Segments(seg)->edgels[i - 1].p.x;
      y_c[i] = Z::Segments(seg)->edgels[i - 1].p.y;
    }
    n_lines = MAX_LINES;
    if(ls.segment_lines(x_c, y_c, Z::Segments(seg)->edgels.Size(),
        x_start, y_start, x_end, y_end, line_start, line_end, sigs, &n_lines))
    {
      for(int i = 0; i < n_lines; i++)
      {
        // note: lines of length 2 often cause problems
        if(line_end[i] - line_start[i] + 1 >= 3)
          NewGestalt(new Line(seg, line_start[i] - 1, line_end[i] - 1));
      }
      als.segment_arcs_lines(x_start, y_start, x_end, y_end, sigs, n_lines);
    }
  }
}

/**
 * Create an image where each edge is drawn with its ID.
 * This image is later used for determining the colours left/right of lines.
 */
void FormSegments::DrawToEdgeImage(unsigned seg)
{
  Segment *s = Segments(seg);
  for(unsigned i = 0; i < s->edgels.Size(); i++)
  {
    int x = (int)s->edgels[i].p.x;
    int y = (int)s->edgels[i].p.y;
    edge_img->SetPixel(x, y, s->ID());
    dir_img[edge_img->width*y + x] = s->edgels[i].dir;
  }
}

/**
 * Canny edge detection using code by Matas et al.
 */
void FormSegments::CreateSegmentsMatas()
{
//  float alpha = 0.6;  	// 0.8 // filter width (these are "good" values)  1.50
//  float omega = 0.001; 	// 0.3 // filter parameter  0.001
  float high_thresh, low_thresh;
  RGBColor *rgb_img = (RGBColor*)cur_img->Data();
  FARY *grey_img = 0;
  FARY *grad_dir = 0;
  FARY *grad_mag = 0;
  BARY *edge_img = 0;
  t_graphGR edge_graph;
  void *void_edge;
  Array<Edgel> edgel_arr;
  double avg_len = 0.;

  if(cur_img == 0)
    return;

  // calculate Canny edges
  grey_img = makFary(cur_img->Height(), cur_img->Width());
  for(int y = 0; y < (int)cur_img->Height(); y++)
    for(int x = 0; x < (int)cur_img->Width(); x++)
    {
      RGBColor *pix = &rgb_img[y*cur_img->Width() + x];
      grey_img->el[y][x] = (float)(pix->r + pix->g + pix->b)/3.;
    }
  grad_dir = makFary(cur_img->Height(), cur_img->Width());
  grad_mag = FindGradientDir(grey_img, alpha, omega, grad_dir,
                             &high_thresh, &low_thresh);
  edge_img = HysteresisThresh(grad_mag, high_thresh, low_thresh);
  edge_graph = LinkEdges(edge_img);

  // first calculate edgel probability, i.e. the probability that a pixel is an
  // edgel: total number of edgels by total number of pixels.
  // (needed for calculation of significances of newly created edges)
  num_edgels = 0;
  ForeachLL_M(edge_graph.edges, void_edge)
  {
    t_edgeGR edge = (t_edgeGR)void_edge;
    t_LL edgels = (t_LL)edge->att;
    void *void_edgel;
    ForeachLL_M(edgels, void_edgel)
      num_edgels++;
  }
  VisionCore::p_e = (double)num_edgels/
    (double)(cur_img->Height()*cur_img->Width());

  // create segments from edge graph
  ForeachLL_M(edge_graph.edges, void_edge)
  {
    t_edgeGR edge = (t_edgeGR)void_edge;
    t_LL edgels = (t_LL)edge->att;
    void *void_edgel;
    t_V2i *edgel;
    unsigned i = 0;
    edgel_arr.Resize(SizeLL(edgels));
    ForeachLL_M(edgels, void_edgel)
    {
      edgel = (t_V2i*)void_edgel;
      // note that we swap x and y! (the Canny implementation uses a strange
      // co-ordinate system)
      edgel_arr[i].p.x = edgel->y;
      edgel_arr[i].p.y = edgel->x;
      edgel_arr[i].dir = ScaleAngle_0_2pi(grad_dir->el[edgel->x][edgel->y]);
      i++;
    }
    avg_len += edgel_arr.Size();
    NewGestalt(new Segment(edgel_arr));
  }
  // probability of an edgel, given that the previous pixel was an edgel
  avg_len /= NumSegments();
  VisionCore::p_ee = 1. - 1./avg_len;

  destAry((ARY*)grey_img);
  destAry((ARY*)grad_dir);
  destAry((ARY*)grad_mag);
  destAry((ARY*)edge_img);
  DestGR(edge_graph);
}

void FormSegments::Rank()
{
  RankGestalts(Gestalt::SEGMENT, CmpSegments);
}

void FormSegments::SetCanny(int a, int o)
{
	alpha = (float) a/1000.;
	omega = (float) o/1000.;
}
}
