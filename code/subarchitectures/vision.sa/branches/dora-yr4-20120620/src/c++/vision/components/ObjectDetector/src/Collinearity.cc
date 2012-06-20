/**
 * $Id: Collinearity.cc,v 1.19 2007/04/14 20:50:59 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "TJunction.hh"
#include "Collinearity.hh"

namespace Z
{

/**
 * TODO: use one function recalc()
 */
Collinearity::Collinearity(unsigned i, unsigned j, unsigned end_i,
    unsigned end_j)
  : Gestalt(COLLINEARITY)
{
  double sense_i = (end_i == START ? 1. : -1.);
  double sense_j = (end_j == START ? 1. : -1.);

  // line 0 is to be the longer, line 1 the shorter line
  if(Lines(i)->Length() >= Lines(j)->Length())
  {
    line[0] = i;
    line[1] = j;
    near_point[0] = end_i;
    near_point[1] = end_j;
	sense[0] = sense_i;
	sense[1] = sense_j;
  }
  else
  {
    line[0] = j;
    line[1] = i;
    near_point[0] = end_j;
    near_point[1] = end_i;
	sense[0] = sense_j;
	sense[1] = sense_i;
  }
  gap = max(1., Distance(Lines(line[0])->point[near_point[0]],
        Lines(line[1])->point[near_point[1]]));
  vertex = (Lines(line[0])->point[near_point[0]] + 
            Lines(line[1])->point[near_point[1]])/2.;

  for(unsigned side = LEFT; side <= RIGHT; side++)
  {
	Lines(line[side])->AddCollinearity(near_point[side], id);
    dir[side] = sense[side]*Lines(line[side])->dir;
  }

  CorrectGap();
  CalculateColors();
  CalculateSignificance();
}

void Collinearity::Recalc()
{
  // line 0 is to be the longer, line 1 the shorter line
  if(Lines(line[0])->Length() < Lines(line[1])->Length())
  {
    Swap(line[0], line[1]);
    Swap(near_point[0], near_point[1]);
  }
  gap = max(1., Distance(Lines(line[0])->point[near_point[0]],
        Lines(line[1])->point[near_point[1]]));
  vertex = (Lines(line[0])->point[near_point[0]] + 
            Lines(line[1])->point[near_point[1]])/2.;
  CorrectGap();
  CalculateColors();
  CalculateSignificance();
}

void Collinearity::Draw(int detail)
{
  Vector2 p[2];
  if(detail >= 1)
    for(unsigned i = 0; i <= 1; i++)
      Lines(line[i])->Draw(detail - 1);
  if(detail >= 2)
  {
    char id_str[20];
    Vector2 v = vertex + Vector2(10., 10.);
    snprintf(id_str, 20, "%u", id);
    DrawText2D(id_str, v.x, v.y, RGBColor::dark_yellow);
  }
  for(unsigned i = 0; i <= 1; i++)
    p[i] = Lines(line[i])->point[near_point[i]];
  DrawLine2D(p[0].x, p[0].y, p[1].x, p[1].y, RGBColor::dark_yellow);
  DrawPoint2D(vertex.x, vertex.y, RGBColor::blue);
}

void Collinearity::DrawInfo()
{
  char str[100];
  // longer line 0 defines what is left and right
  // if line 0 and 1 point the same way, left 0 = left, else left 0 = right
  unsigned left_1 = (near_point[0] != near_point[1] ? LEFT : RIGHT);
  RGBColor col[2][2];  // line 0/1 and left/right
  col[0][LEFT] = Lines(line[0])->MeanCol(LEFT);
  col[0][RIGHT] = Lines(line[0])->MeanCol(RIGHT);
  col[1][LEFT] = Lines(line[1])->MeanCol(left_1);
  col[1][RIGHT] = Lines(line[1])->MeanCol(Other(left_1));
    
  FillRect2D(0., 0., 0.5, 0.5, col[0][LEFT]);
  snprintf(str, 100, "0 left: %d %d %d", col[0][LEFT].r, col[0][LEFT].g,
      col[0][LEFT].b);
  DrawText2D(str, 0.05, 0.1, RGBColor::green);

  FillRect2D(0., 0.5, 0.5, 1., col[1][LEFT]);
  snprintf(str, 100, "1 left: %d %d %d", col[1][LEFT].r, col[1][LEFT].g,
      col[1][LEFT].b);
  DrawText2D(str, 0.05, 0.6, RGBColor::green);

  FillRect2D(0.5, 0., 1., 0.5, col[0][RIGHT]);
  snprintf(str, 100, "0 right: %d %d %d", col[0][RIGHT].r, col[0][RIGHT].g,
      col[0][RIGHT].b);
  DrawText2D(str, 0.55, 0.1, RGBColor::green);

  FillRect2D(0.5, 0.5, 1., 1., col[1][RIGHT]);
  snprintf(str, 100, "1 right: %d %d %d", col[1][RIGHT].r, col[1][RIGHT].g,
      col[1][RIGHT].b);
  DrawText2D(str, 0.55, 0.6, RGBColor::green);
}

const char* Collinearity::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "%slines: %u %u\ngap: %f (corrected %f)\ncol dist: %f\nangle: %f\n",
      Gestalt::GetInfo(), line[0], line[1], gap, gap_cor, col_dist,
		OpeningAngle());
  return info_text;
}

bool Collinearity::IsAtPosition(int x, int y)
{
  return Lines(line[0])->IsAtPosition(x, y) ||
    Lines(line[1])->IsAtPosition(x, y);
}

/**
 * Calculate color difference.
 */
void Collinearity::CalculateColors()
{
  double dist[2];
  // longer line 0 defines what is left and right
  // if line 0 and 1 point the same way, left 0 = left, else left 0 = right
  unsigned left_1 = (near_point[0] != near_point[1] ? LEFT : RIGHT);
  dist[LEFT] = Dist(Lines(line[0])->MeanCol(LEFT),
                    Lines(line[1])->MeanCol(left_1));
  dist[RIGHT] = Dist(Lines(line[0])->MeanCol(RIGHT),
                     Lines(line[1])->MeanCol(Other(left_1)));
  // take the smaller of the two color distances
  // note: colors falling into the same bin could in the worst case still be
  // separated by bin diameter -> assume bin diameter as min distance
  col_dist = max(sqrt(3.), min(dist[LEFT], dist[RIGHT]));
}

/**
 * Calculate accidentalness and significance.
 * Assume endpoints are distributed following a Poisson process with parameter
 * p_lep in space R2 (p_lep = prob. of line endpoints).
 * Be g the vector from the endpoint of the longer line 0 to the endpoint of the
 * shorter line 1, r its length and theta its angle with line 1.
 * Calculate the probability that there are one or more endpoints in area A =
 * 2*theta*r^2. Which is the same as 1 - probability that there is no endpoint
 * in area A.
 * We chose the wedge with width 2*theta instead of the full circle because we
 * are also interested in the direction of the endpoint, not just its distance.
 * Be dphi the angular difference between line 0 and line 1.
 * Then multiply with the probability that line 0 falls within +/-dphi.
 * P = P(endpoint falls in area)*P(angle falls in interval)
 * Note that the lengths of the lines play no role.
 * TODO: check and fix the above descriptive text to match the actual and
 * correct code below.
 */
void Collinearity::CalculateSignificance()
{
  double n = 2.*(double)NumLines();
  double r = gap_cor;  // gap corrected for occlusion
  double alpha, beta;
  double f_prox;    // proximity factor
  double f_ori;     // orientation factor
  double f_col;     // color factor
  // gap vector pointing from line 0 to 1
  Vector2 g = Lines(line[1])->point[near_point[1]] -
    Lines(line[0])->point[near_point[0]];
  // vector along line 0 pointing towards gap
  Vector2 a = Normalise(Lines(line[0])->point[near_point[0]] -
     Lines(line[0])->point[Other(near_point[0])]);
  // vector along line 1 pointing towards gap
  Vector2 b = Normalise(Lines(line[1])->point[near_point[1]] -
     Lines(line[1])->point[Other(near_point[1])]);

  // To avoid 0 angles and thus acc = 0 (sig -> inf) we always assume an angular
  // uncertainty of 1 pixel over line length
  // If there is a gap measure angles of between lines and gap
  if(g.Length() > 1.)
  {
    g.Normalise();
    alpha = fmax(atan(1./Lines(line[0])->Length()), acos(Dot(a, g)));
    beta = fmax(atan(1./Lines(line[1])->Length()), acos(Dot(b, -g)));
  }
  // else measure angles between lines themselves
  else
  {
    alpha = fmax(atan(1./Lines(line[0])->Length()), acos(Dot(a, -b)));
    beta = fmax(atan(1./Lines(line[1])->Length()), acos(Dot(b, -a)));
  }

  // proximity factor
  f_prox = alpha*r*r/(double)VisionCore::ImageArea();
  // orientation factor
  f_ori = beta/M_PI;
  // color factor
#ifdef USE_COLOR
  f_col = SphereVolume(col_dist)/pow(256., 3.);
#else
  f_col = 1.;
#endif

  acc = 1. - exp(-n*f_prox*f_ori*f_col);
  acc = min(acc, 1.);  // TODO: what's that for??
  sig = -log(acc);
  sig = max(sig, 1e-6);  // TODO: properly
}

void Collinearity::CorrectGap()
{
  unsigned t[2];
  t[0] = Lines(line[0])->t_jct[near_point[0]];
  t[1] = Lines(line[1])->t_jct[near_point[1]];
  // if there are T-junctions at both ends of the collinearity reduce the actual
  // gap by the distance between T-junctions
  if(t[0] != UNDEF_ID && t[1] != UNDEF_ID)
  {
    double d = Distance(TJunctions(t[0])->isct, TJunctions(t[1])->isct);
    gap_cor = max(1., gap - d);
  }
  else
    gap_cor = gap;
}

double Collinearity::OpeningAngle()
{
  return acos(Dot(dir[LEFT], dir[RIGHT]));
}

unsigned Collinearity::WhichLineIs(unsigned l) throw(Except)
{
  if(l == line[0])
    return 0;
  else if(l == line[1])
    return 1;
  else
    throw Except(__HERE__, "line %u is not part of collinearity %u", l, id);
}

unsigned Collinearity::OtherLine(unsigned l)
{
  return line[Other(WhichLineIs(l))];
}

unsigned Collinearity::WhichEndIs(unsigned l)
{
  return near_point[WhichLineIs(l)];
}

}
