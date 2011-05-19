/**
 * $Id: LJunction.cc,v 1.19 2007/04/14 20:50:59 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"

namespace Z
{

/**
 * Returns true if junction b is inside a and vice versa.
 */
bool Inside(LJunction* a, LJunction* b)
{
  Vector2 ab = b->isct - a->isct;
  // TODO: cross(p,q) > 0 if q LEFT to p!!! check!!!
  // b must be right of left arm (note: cross(p,q) > 0 if q right to p)
  if(Cross(a->dir[LEFT], ab) < 0.)
    return false;
  // and left of right arm
  if(Cross(a->dir[RIGHT], ab) > 0.)
    return false;
  // note: Cross(ba,..) < 0 equals Cross(ab,..) > 0
  if(Cross(b->dir[LEFT], ab) > 0.)
    return false;
  if(Cross(b->dir[RIGHT], ab) < 0.)
    return false;
  return true;
}

/**
 * Returns true if junction b is inside junction a.
 */
bool Opposing(LJunction* a, LJunction* b)
{
  // line ab joining junction a and b
  Vector2 ab = b->isct - a->isct;
  // projections of line ab onto directions of junctions
  double la[2], lb[2];
  la[LEFT] = Dot(a->dir[LEFT], ab);
  la[RIGHT] = Dot(a->dir[RIGHT], ab);
  // note: Dot(ba,..) = -Dot(ab,..)
  lb[LEFT] = -Dot(b->dir[LEFT], ab);
  lb[RIGHT] = -Dot(b->dir[RIGHT], ab);
  // the larger projection defines whether ab coincides more with arm on left
  // or right side
  int side_a = (fabs(la[LEFT]) > fabs(la[RIGHT]) ? LEFT : RIGHT);
  int side_b = (fabs(lb[LEFT]) > fabs(lb[RIGHT]) ? LEFT : RIGHT);
  // junctions may only be joined left-right or right-left
  // and projections along chosen side must be positive
  if(side_a != side_b && la[side_a] > 0. && lb[side_b] > 0.)
  {
    // what remains is to check whether the (detected, visible) line or lines
    // making up the arms of the junctions lie inside the area defined by the
    // opposing junctions (note there are special cases where this is not true).
    // Check whether the nearer point of one junction lies inside the other
    // junction. I.e. the far end of one junction may stick out of the other
    // junction.
    // TODO: use near_point
    Vector2 v[2];
    v[START] = b->line[side_b]->point[START] - a->isct;
    v[END] = b->line[side_b]->point[END] - a->isct;
    if(max(Dot(v[START], a->dir[side_a]),
           Dot(v[END], a->dir[side_a])) < 0.)
      return false;
    v[START] = a->line[side_a]->point[START] - b->isct;
    v[END] = a->line[side_a]->point[END] - b->isct;
    if(max(Dot(v[START], b->dir[side_b]),
           Dot(v[END], b->dir[side_b])) < 0.)
      return false;
    return true;
  }
  return false;
}

/**
 * i, j .. lines
 * inter .. intersection point
 * TODO: simplify, clean up
 */
LJunction::LJunction(VisionCore *c, Line *line_i, Line *line_j,
    int end_i, int end_j, const Vector2 &inter)
  : Gestalt(c, L_JUNCTION)
{
  // find whether line directions point to or away from the junction
  // i.e. whether the junction is nearer to the start or the end of the lines
  double sense_i = (end_i == START ? 1. : -1.);
  double sense_j = (end_j == START ? 1. : -1.);
  // note: tangents from end points to intersection, junction dirs point in the
  // opposite direction
  // angle bisection is direction of L-junction
  dir_i = Normalise(-line_i->tang[end_i] - line_j->tang[end_j]);
  isct = inter;
  // TODO: use LeftOf or RightOf here
  // if line i is clockwise to dir it is the right line
  if(Cross(dir_i, -line_i->tang[end_i]) < 0.)
  {
    line[LEFT] = line_j;
    line[RIGHT] = line_i;
    sense[LEFT] = sense_j;
    sense[RIGHT] = sense_i;
  }
  else
  {
    line[LEFT] = line_i;
    line[RIGHT] = line_j;
    sense[LEFT] = sense_i;
    sense[RIGHT] = sense_j;
  }
  for(int side = LEFT; side <= RIGHT; side++)
  {
    dir[side] = sense[side]*line[side]->dir;
    near_point[side] = (sense[side] > 0. ? START : END);
    dist[side] = Distance(isct, line[side]->point[near_point[side]]);
  }
  int left_inside = RIGHT ^ near_point[LEFT];
  int right_inside = LEFT ^ near_point[RIGHT];
  // note: colors falling into the same bin could in the worst case still be
  // separated by bin diameter -> assume bin diameter as min distance
  col_dist = fmax(sqrt(3.), Dist(line[LEFT]->MeanCol(left_inside),
      line[RIGHT]->MeanCol(right_inside)));
  r = fmax(1., Distance(line[LEFT]->point[near_point[LEFT]],
        line[RIGHT]->point[near_point[RIGHT]]));
  CalculateSignificance();
  for(int side = LEFT; side <= RIGHT; side++)
  {
    // Store this junction at the near end of each line.
    // For the left line this reads e.g.:
    // My START end is the LEFT side of junction 'id'.
    line[side]->AddLJunction(near_point[side], side, this);
  }
  // TODO: reduce gap if both ends have T-jct (as in collinearity)
}

void LJunction::Recalc()
{
  // nothing to recalc right now
  // TODO: check this properly
}

void LJunction::Draw(int detail)
{
  if(detail >= 1)
  {
    line[LEFT]->Draw(detail - 1);
    line[RIGHT]->Draw(detail - 1);
  }
  if(detail >= 2)
  {
    char id_str[20];
    Vector2 v = isct - 10.*dir_i;
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, v.x, v.y, RGBColor::blue);
    /*v = isct + 10*dir[LEFT];
    DrawText2D("L", v.x, v.y, RGBColor::blue);
    v = isct + 10*dir[RIGHT];
    DrawText2D("R", v.x, v.y, RGBColor::blue);*/
  }
  // draw arms of junction
  Vector2 pl = line[LEFT]->point[near_point[LEFT]];
  Vector2 pr = line[RIGHT]->point[near_point[RIGHT]];
  DrawLine2D(isct.x, isct.y, pl.x, pl.y, RGBColor::cyan);
  DrawLine2D(isct.x, isct.y, pr.x, pr.y, RGBColor::magenta);
  // draw intersection point
  DrawPoint2D(isct.x, isct.y, RGBColor::blue);
}

void LJunction::DrawInfo()
{
  char str[100];
  int left_inside = RIGHT ^ near_point[LEFT];
  int right_inside = LEFT ^ near_point[RIGHT];
  RGBColor col[2];
  col[LEFT] = line[LEFT]->MeanCol(left_inside);
  col[RIGHT] = line[RIGHT]->MeanCol(right_inside);
  FillRect2D(0., 0., 0.5, 1., col[LEFT]);
  DrawText2D("left", 0.1, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", col[LEFT].r, col[LEFT].g, col[LEFT].b);
  DrawText2D(str, 0.1, 0.6, RGBColor::green);
  FillRect2D(0.5, 0., 1., 1., col[RIGHT]);
  DrawText2D("right", 0.6, 0.8, RGBColor::green);
  snprintf(str, 100, "%d %d %d", col[RIGHT].r, col[RIGHT].g, col[RIGHT].b);
  DrawText2D(str, 0.6, 0.6, RGBColor::green);
}

const char* LJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%sleft: %u right: %u\ndist: %f col_dist: %f\nangle: %.3f\n",
      Gestalt::GetInfo(), line[LEFT]->ID(), line[RIGHT]->ID(), r, col_dist,
      OpeningAngle());
  // PrintSig();
  return info_text;
}

bool LJunction::IsAtPosition(int x, int y)
{
  return line[LEFT]->IsAtPosition(x, y) || line[RIGHT]->IsAtPosition(x, y);
}

/**
 * Calculate accidentalness and significance.
 * Assume endpoints are distributed following a Poisson process with parameter
 * p_lep in space R2 (p_lep = prob. of line endpoints).
 * Be r the distance between endpoints of left and right line.
 * Calculate the probability that there are one or more endpoints in area A =
 * pi*r^2. Which is the same as 1 - probability that there is no endpoint in
 * area A.
 * TODO: check and fix the above descriptive text to match the actual and
 * correct code below.
 */
void LJunction::CalculateSignificance()
{
  double n = 2.*(double)core->NumGestalts(Gestalt::LINE);
  // proximity factor
  double f_prox = CircleArea(r)/((double)core->ImageArea());
  // color factor
#ifdef USE_COLOR
  double f_col = SphereVolume(col_dist)/pow(256., 3.);
#else
  double f_col = 1.;
#endif

  acc = 1. - exp(-n*f_prox*f_col);
  acc = min(acc, 1.);
  sig = -log(acc);
  sig = max(sig, 1e-6);  // TODO: properly
}

void LJunction::PrintSig()
{
  // proximity factor
  double f_prox = CircleArea(r)/(double)core->ImageArea();
  // color factor
  double f_col = SphereVolume(col_dist)/pow(256., 3.);
  // we want L-junctions to be close to 90 degrees
  // TODO: base angular uncertainty on shorter line
  // to avoid dphi = 0 and thus acc = 0, assume an angular uncertainty for line
  // 0 of one pixel over line length
  double d_phi = AngleBetweenLines(line[LEFT]->phi + M_PI_2, line[RIGHT]->phi);
  // orientation factor
  double f_ori = d_phi/M_PI_2;

  printf("f_prox: %f  f_col: %f  f_ori: %f\n", f_prox, f_col, f_ori);
}

double LJunction::Gap(int side)
{
  return Distance(isct, line[side]->point[near_point[side]]);
}

double LJunction::OpeningAngle()
{
  return acos(Dot(dir[LEFT], dir[RIGHT]));
}

}

