/**
 * $Id: Line.cc,v 1.27 2007/03/25 21:35:57 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Segment.hh"
#include "Line.hh"
#include "TJunction.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "FormSegments.hh"
#include "FormLines.hh"
#include "FormJunctions.hh"
#include "FormCorners.hh"
#include <set>

namespace Z
{

// HACK: region of interest: these values are good for "looking into the door"
// of office_wall
static const Vector2 roi = Vector2(400., 650.);
static const double roi_s = 200.;
static const RGBColor red(255, 0, 0); //(160, 93, 103);  // good
static const RGBColor blue(0, 0, 255); //(94, 139, 216);  // good
static const RGBColor coi = red;     // color of interest
// HACK END

LineBase::LineBase()
  : Gestalt(LINE)
{
  for(unsigned i = START; i <= END; i++)
  {
    t_jct[i] = UNDEF_ID;
  }
}

void LineBase::CalculateParameters()
{
  Vector2 v = point[END] - point[START];
  dir = Normalise(v);
  tang[START] = -dir;
  tang[END] = dir;
  phi = ScaleAngle_0_2pi(PolarAngle(dir));
  len = v.Length();
  // Hessian parameters
  /*if(IsZero(dir.x))
  {
    if(start_point.x >= 0.)
    {
      s = start_point.x;
      theta = 3*M_PI_2;
    }
    else
    {
      s = -start_point.x;
      theta = M_PI_2;
    }
  }
  else
  {
    double y = start_point.y - dir.y*start_point.x/dir.x;  // point on y axis
    theta = ScaleAngle_0_2pi(atan(dir.y/dir.x));
    s = y*cos(theta);
    if(s < 0.)
    {
      s = -s;
      theta = ScaleAngle_0_2pi(theta + M_PI);
    }
  }*/
}

void LineBase::DrawArrow()
{
  double len_2 = 4.; // length/2
  double wid_2 = 3.; // width/2
  Vector2 tip = (point[START] + point[END])/2. + dir*len_2;
  Vector2 left = (point[START] + point[END])/2. - dir*len_2 +
    dir.NormalAntiClockwise()*wid_2;
  Vector2 right = (point[START] + point[END])/2. - dir*len_2 +
    dir.NormalClockwise()*wid_2;
  DrawLine2D(left.x, left.y, tip.x, tip.y, RGBColor::red);
  DrawLine2D(right.x, right.y, tip.x, tip.y, RGBColor::red);
}

void LineBase::Draw(int detail)
{
  DrawLine2D(point[START].x, point[START].y, point[END].x, point[END].y,
      RGBColor::red);
  if(detail >= 1)
  {
    char id_str[20];
    Vector2 mid_point = (point[START] + point[END])/2. + Vector2(4., -4.);
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, mid_point.x, mid_point.y, RGBColor::red);
    DrawArrow();
  }
  /*if(detail >= 2)
  {
    // draw T-junctions
    if(t_jct[START] != UNDEF_ID)
      TJunctions(t_jct[START])->Draw(0);
    if(t_jct[END] != UNDEF_ID)
      TJunctions(t_jct[END])->Draw(0);

    // draw L-junctions
    for(unsigned end = START; end <= END; end++)
      for(unsigned side = LEFT; side <= RIGHT; side++)
        for(unsigned i = 0; i < l_jct[end][side].Size(); i++)
          LJunctions(l_jct[end][side][i])->Draw(0);
  }*/
}

void LineBase::DrawInfo()
{
  char str[100];
  FillRect2D(0., 0., 0.5, 1., mean_col[LEFT]);
  DrawText2D("left", 0.1, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", mean_col[LEFT].r, mean_col[LEFT].g,
      mean_col[LEFT].b);
  DrawText2D(str, 0.1, 0.6, RGBColor::green);
  FillRect2D(0.5, 0., 1., 1., mean_col[RIGHT]);
  DrawText2D("right", 0.6, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", mean_col[RIGHT].r, mean_col[RIGHT].g,
      mean_col[RIGHT].b);
  DrawText2D(str, 0.6, 0.6, RGBColor::green);
}

const char* LineBase::GetInfo()
{
/*  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%s(%.0f %.0f)-(%.0f %.0f) length: %f\nphi: %f\nneighbours: %d\n",
      Gestalt::GetInfo(),
      point[START].x, point[START].y, point[END].x, point[END].y, Length(),
      phi, neighbors.size());
  return info_text;
*/
  set<unsigned>::iterator it;
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size,
      "%s(%.0f %.0f)-(%.0f %.0f) length: %f\nphi: %f\n",
      Gestalt::GetInfo(),
      point[START].x, point[START].y, point[END].x, point[END].y, Length(),
      phi);

  n += snprintf(info_text + n, info_size - n, "neighbors: ");
  for (it=neighbors.begin(); it!=neighbors.end(); it++)
    n += snprintf(info_text + n, info_size - n, " %i", *it);
  n += snprintf(info_text + n, info_size - n, "\n");

  return info_text;
}

double LineBase::MinEndpointDistance(const LineBase *l)
{
  double dist = HUGE;
  for(unsigned i = START; i <= END; i++)
    for(unsigned j = START; j <= END; j++)
      dist = fmin(dist, Distance(point[i], l->point[j]));
  return dist;
}

double LineBase::DistanceToPoint(const Vector2 &q)
{
  Vector2 start_to_q = q - point[START];
  double s = Dot(start_to_q, dir);
  if(s >= 0. && s <= Length())
    return fabs(Cross(start_to_q, dir));
  else
    return Distance(q, point[(s<0. ? START : END)]);
}

/**
 * Add an L-junction.
 */
void LineBase::AddLJunction(unsigned end, unsigned side, unsigned jct)
{
  unsigned osLines = l_jct[0][0].Size()+l_jct[0][1].Size();	// old lines (start)
  unsigned oeLines = l_jct[1][0].Size()+l_jct[1][1].Size(); // old lines (end)

  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < l_jct[end][side].Size())
  {
    // rank according to accidentalness
    if(LJunctions(jct)->acc < LJunctions(l_jct[end][side][i])->acc)
    {
      l_jct[end][side].InsertBefore(i, jct);
      return;
    }
    i++;
  }
  l_jct[end][side].PushBack(jct);
  neighbors.insert(LJunctions(jct)->line[Other(side)]);
  
  // TODO ARI:
  // InformNewCandidate in FormCorners
  unsigned nsLines = l_jct[0][0].Size()+l_jct[0][1].Size();	// new lines (start)
  unsigned neLines = l_jct[1][0].Size()+l_jct[1][1].Size();	// new lines (end)
  if((nsLines - osLines) > 0 && nsLines > 1)
	Principles(GestaltPrinciple::FORM_CORNERS)->InformNewCandidate(id, START);
  if((neLines - oeLines) > 0 && neLines > 1)
	Principles(GestaltPrinciple::FORM_CORNERS)->InformNewCandidate(id, END);
}

/**
 * Add an Collinearity
 */
void LineBase::AddCollinearity(unsigned end, unsigned co)
{
  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < coll[end].Size())
  {
    // rank according to significance
    if(Collinearities(co)->sig > Collinearities(coll[end][i])->sig)
    {
      coll[end].InsertBefore(i, co);
      return;
    }
    i++;
  }
  coll[end].PushBack(co);
  neighbors.insert(Collinearities(co)->OtherLine(id));
}

/**
 * Add a passive T-junction.
 */
void LineBase::AddPassiveTJunction(unsigned end, unsigned side, unsigned jct)
{
  // TODO: this insertion has bad O(n^2) complexity, but n is typically never
  // larger than 10, so we can live with it.
  unsigned i = 0;
  while(i < pt_jct[end][side].Size())
  {
    // rank according to accidentalness
    if(TJunctions(jct)->acc < TJunctions(pt_jct[end][side][i])->acc)
    {
      pt_jct[end][side].InsertBefore(i, jct);
      return;
    }
    i++;
  }
  pt_jct[end][side].PushBack(jct);
  // note that any T-jct Tabc implies L-jcts and collinearities Lab, Lca and
  // Cbc, so we don't need to update neighborhood here, as it is handled by L
  // and C.
}

/**
 * Add an E-Junction.
 */
void LineBase::AddEJunction(unsigned end, unsigned eJct)
{
  e_jct[end].PushBack(eJct);	
}

void LineBase::AddExtEllipse(unsigned eEll)
{
  if(!extEllipses.Contains(eEll))
	extEllipses.PushBack(eEll);
}

void LineBase::AddCylinder(unsigned cyl)
{
  cylinders.PushBack(cyl);
}

void LineBase::AddRectangle(unsigned rect)
{
  rects.PushBack(rect);
}

/**
 * Add a cube.
 */
void LineBase::AddCube(unsigned cube)
{
  cubes.PushBack(cube);
}

Line::Line(unsigned seg_id, unsigned i, unsigned j)
{
  defer_vote = id;
  next = UNDEF_ID;
  seg = seg_id;
  idx[START] = i;
  idx[END] = j;
  label = 0;
  Recalc();
}

void Line::Recalc()
{
  point[START] = Segments(seg)->edgels[idx[START]].p;
  point[END] = Segments(seg)->edgels[idx[END]].p;
  CalculateParameters();
  CalculateColors();
  CalculateSignificance();
  CalculateWeight();
}

const char* Line::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
  n += snprintf(info_text + n, info_size, "%sseg: %u\n"
      "label: %d energy: %f stability: %f\n\n"
      "T-Jcts: %d %d\n",
      LineBase::GetInfo(), seg, label, energy, stability,
      t_jct[START], t_jct[END]);

  n += snprintf(info_text + n, info_size - n, "L-Jcts: ");
  for (unsigned i=0; i<l_jct[0][0].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "SL(%u) ", l_jct[0][0][i]);
  for (unsigned i=0; i<l_jct[0][1].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "SR(%u) ", l_jct[0][1][i]);
  for (unsigned i=0; i<l_jct[1][0].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "EL(%u) ", l_jct[1][0][i]);
  for (unsigned i=0; i<l_jct[1][1].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "ER(%u) ", l_jct[1][1][i]);

  n += snprintf(info_text + n, info_size - n, "\nE-Jcts: ");
  for (unsigned i=0; i<e_jct[0].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "S(%u) ", e_jct[0][i]);
  for (unsigned i=0; i<e_jct[1].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "E(%u) ", e_jct[1][i]);

  n += snprintf(info_text + n, info_size - n, "\nColls: ");
  for (unsigned i=0; i<coll[0].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "S(%u) ", coll[0][i]);
  for (unsigned i=0; i<coll[1].Size(); i++)
	n += snprintf(info_text + n, info_size -n, "E(%u) ", coll[1][i]);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "Closures:");
  for(unsigned i = 0; i < closures.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", closures[i]);
  n += snprintf(info_text + n, info_size - n, "\n");
  
  n += snprintf(info_text + n, info_size - n, "Corners:");
  for(unsigned i = 0; i < corners.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", corners[i]);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "ExtEllipses:");
  for(unsigned i = 0; i < extEllipses.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", extEllipses[i]);
  n += snprintf(info_text + n, info_size - n, "\n");
  
  n += snprintf(info_text + n, info_size - n, "Cylinders:");
  for(unsigned i = 0; i < cylinders.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", cylinders[i]);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "Rectangles:");
  for(unsigned i = 0; i < rects.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", rects[i]);
  n += snprintf(info_text + n, info_size - n, "\n");
  
  n += snprintf(info_text + n, info_size - n, "Cubes:");
  for(unsigned i = 0; i < cubes.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %u", cubes[i]);
  n += snprintf(info_text + n, info_size - n, "\n");
  
  return info_text;
}

/**
** Get All L-Junctions from a line
*/
void Line::GetAllLJunctions(Array<unsigned> *lj)
{
  Array<unsigned> ljc = *lj;
	
  for(unsigned i=0; i<l_jct[0][0].Size(); i++)
	ljc.PushBack(l_jct[0][0][i]);
  for(unsigned i=0; i<l_jct[0][1].Size(); i++)
	ljc.PushBack(l_jct[0][1][i]);
  for(unsigned i=0; i<l_jct[1][0].Size(); i++)
	ljc.PushBack(l_jct[1][0][i]);
  for(unsigned i=0; i<l_jct[1][1].Size(); i++)
	ljc.PushBack(l_jct[1][1][i]);
  
  *lj = ljc;
}

/**
** Get all L-Junctions from one end [START/END] of the line
*/
void Line::GetLJunctions(unsigned end, Array<unsigned> &lj)
{
  for(unsigned i=0; i<l_jct[end][0].Size(); i++)
		lj.PushBack(l_jct[end][0][i]);
  for(unsigned i=0; i<l_jct[end][1].Size(); i++)
		lj.PushBack(l_jct[end][1][i]);
}

void Line::GetAllCollinearities(Array<unsigned> *col)
{
  Array<unsigned> c = *col;
  
  for(unsigned i=0; i<coll[0].Size(); i++)
	c.PushBack(coll[0][i]);
  for(unsigned i=0; i<coll[1].Size(); i++)
	c.PushBack(coll[1][i]);
	
  *col = c;
}

// Get all L-Junctions from one end [START/END] of the line
void Line::GetCollinearities(unsigned end, Array<unsigned> &c)
{
  for(unsigned i=0; i<coll[end].Size(); i++)
	c.PushBack(coll[end][i]);
}

void Line::GetAllExtEllipses(Array<unsigned> *ee)
{
  Array<unsigned> extEll = *ee;
	
  for(unsigned i=0; i<extEllipses.Size(); i++)
	extEll.PushBack(extEllipses[i]);
  
  *ee = extEll;
}

void Line::DrawVotes()
{
  VoteImage *vi = FormJunctions::vote_img;
  unsigned baseIndex = FormJunctions::baseIndex;
  unsigned baseOffset = FormJunctions::baseOffset;	
	
  // return if no vote_img or id is from split line 
  // note: if id > baseOffset => id is from a split line, because baseOffset
  // is the number of lines from first operation
  if(vi == 0 || id > baseOffset)
    return;
  for(int x = 0; x < vi->width; x++)
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        if(el->id/baseIndex == id)
        {
          unsigned vtype = el->id%baseIndex;
          switch(vtype)
          {
            case VOTE_TS:
            case VOTE_NLS:
            case VOTE_NRS:
              DrawPoint2D(x, y, RGBColor::magenta);
              break;
            case VOTE_TE:
            case VOTE_NLE:
            case VOTE_NRE:
              DrawPoint2D(x, y, RGBColor::cyan);
              break;
            default:
              DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
}

void Line::Draw(int detail)
{
  if(detail <= 1)
    LineBase::Draw(detail);
  if(detail == 2)
    for(unsigned i = idx[START]; i <= idx[END]; i++)
      DrawPoint2D((int)Segments(seg)->edgels[i].p.x,
          (int)Segments(seg)->edgels[i].p.y, RGBColor::green);
  if(detail == 3)
  {
    // TODO: avoid double code , use same as CalculateColors
    const unsigned NUM_SAMPLES = 16;
    unsigned i, j, s, e;
    int x1, y1, x2, y2;
    Vector2 n;
    RGBColor col;
    // sample at regular intervals
    for(j = 0; j < NUM_SAMPLES; j++)
    {
      s = idx[START] + (idx[END] - idx[START])/4;
      e = idx[START] + ((idx[END] - idx[START])*3)/4;
      i = s + ((e - s)*j)/(NUM_SAMPLES - 1);
      // find the points halfway between line and next edge
      // left
      n = dir.NormalAntiClockwise();
      x1 = (int)(Segments(seg)->edgels[i].p.x + 2.*n.x);
      y1 = (int)(Segments(seg)->edgels[i].p.y + 2.*n.y);
      if(FormSegments::edge_img->FindLineEnd(x1, y1,
          x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      {
        DrawPoint2D(x2, y2, RGBColor::blue);
        DrawPoint2D((x1+x2)/2, (y1+y2)/2, RGBColor::cyan);
      }
      // right
      n = -n;
      x1 = (int)(Segments(seg)->edgels[i].p.x + 2.*n.x);
      y1 = (int)(Segments(seg)->edgels[i].p.y + 2.*n.y);
      if(FormSegments::edge_img->FindLineEnd(x1, y1,
          x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      {
        DrawPoint2D(x2, y2, RGBColor::blue);
        DrawPoint2D((x1+x2)/2, (y1+y2)/2, RGBColor::cyan);
      }
    }
  }
  if(detail == 4)
    DrawVotes();
  if(detail == 10)
  {
    DrawPoint2D(point[START].x, point[START].y, RGBColor::red);
    DrawPoint2D(point[END].x, point[END].y, RGBColor::red);
  }
}

bool Line::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = idx[START]; i <= idx[END]; i++)
    if(IsEqual(xd, Segments(seg)->edgels[i].p.x) &&
       IsEqual(yd, Segments(seg)->edgels[i].p.y))
      return true;
  return false;
}

void Line::CalculateSignificance()
{
/*  int k = (int)Length();
  int l = (int)Length();  // or the whole visible line?
  sig = Significance(2, k, l, VisionCore::p_e);*/
	sig = -log(pow(VisionCore::p_ee, (double)NumEdgels()));
}

/**
 * Sample some points at either side and calculate mean colors.
 */
void Line::CalculateColors()
{
  const unsigned NUM_SAMPLES = 16;
  unsigned i, j, s, e;
  int x1, y1, x2, y2;
  int rl = 0, gl = 0, bl = 0;
  int rr = 0, gr = 0, br = 0;
  Vector2 n;
  RGBColor col;
  // sample at regular intervals
  for(j = 0; j < NUM_SAMPLES; j++)
  {
    s = idx[START] + (idx[END] - idx[START])/4;
    e = idx[START] + ((idx[END] - idx[START])*3)/4;
    i = s + ((e - s)*j)/(NUM_SAMPLES - 1);
    // Find the points halfway between line and next edge. Search next edge at
    // max l pixels away, where l is line length.
    // left
    n = dir.NormalAntiClockwise();
    x1 = (int)(Segments(seg)->edgels[i].p.x + 2.*n.x);
    y1 = (int)(Segments(seg)->edgels[i].p.y + 2.*n.y);
    if(FormSegments::edge_img->FindLineEnd(x1, y1,
        x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      col = VisionCore::Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    rl += (int)col.r;
    gl += (int)col.g;
    bl += (int)col.b;
    // right
    n = -n;
    x1 = (int)(Segments(seg)->edgels[i].p.x + 2.*n.x);
    y1 = (int)(Segments(seg)->edgels[i].p.y + 2.*n.y);
    if(FormSegments::edge_img->FindLineEnd(x1, y1,
        x1 + (int)(len*n.x), y1 + (int)(len*n.y), &x2, &y2))
      col = VisionCore::Pixel((x1+x2)/2, (y1+y2)/2);
    else
      col = RGBColor::black;
    rr += (int)col.r;
    gr += (int)col.g;
    br += (int)col.b;
  }
  mean_col[LEFT].r = rl/NUM_SAMPLES; 
  mean_col[LEFT].g = gl/NUM_SAMPLES; 
  mean_col[LEFT].b = bl/NUM_SAMPLES; 
  mean_col[RIGHT].r = rr/NUM_SAMPLES; 
  mean_col[RIGHT].g = gr/NUM_SAMPLES; 
  mean_col[RIGHT].b = br/NUM_SAMPLES; 
}

void Line::CalculateWeight()
{
  weight = 0.;
  // using gaussian roi
//  for(unsigned i = idx[START]; i <= idx[END]; i++)
//    weight += BivariateGaussianPDF(roi.x, roi.y, roi_s,
//       Segments(seg)->edgels[i].p.x, Segments(seg)->edgels[i].p.y);
  // using color
  double dist =
    min(Dist(coi, mean_col[LEFT]), Dist(coi, mean_col[RIGHT]));
  weight = Length()/max(1., dist);
}

}
