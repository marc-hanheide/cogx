/**
 * $Id$
 * Michael Zillich
 */

#include "Line.hh"


namespace P 
{

const static int SAMPLE_DISTANCE = 5;


Line::Line()
 : id(0), nb(0), seg(0), sig(0.), vec(0), size(0)
{
}

Line::Line(Segment *_seg, unsigned i, unsigned j)
 : id(0), nb(0), sig(0), vec(0), size(0)
{
  seg = _seg;
  idx[START] = i;
  idx[END] = j;
  Recalc();
}

Line::~Line()
{
  if (vec!=0) delete[] vec;
}

void Line::CalculateParameters()
{
  Vector2 v = point[END] - point[START];
  dir = Normalise(v);
  tang[START] = -dir;
  tang[END] = dir;
  phi = ScaleAngle_0_2pi(PolarAngle(dir));
  len = v.Length();
}

void Line::CalculateParameters2()
{
  double l[3];

  p = MidPoint(point[0],point[1]);

  LinePts2Para(&point[0].x, &point[1].x, l);

  if (!IsZero(l[2]))
  {
    ln.x = l[0]/l[2];
    ln.y = l[1]/l[2];  
  }
  else
  {
    //cout<<"Zero line!!!"<<endl;
    ln.x = 0.;
    ln.y = 0.;
  }
}

void Line::DrawArrow(IplImage *img)
{
  double len_2 = 4.; // length/2
  double wid_2 = 3.; // width/2
  Vector2 tip = (point[START] + point[END])/2. + dir*len_2;
  Vector2 left = (point[START] + point[END])/2. - dir*len_2 +
    dir.NormalAntiClockwise()*wid_2;
  Vector2 right = (point[START] + point[END])/2. - dir*len_2 +
    dir.NormalClockwise()*wid_2;
  SDraw::DrawLine(img, left.x, left.y, tip.x, tip.y, CV_RGB(255,0,0));
  SDraw::DrawLine(img, right.x, right.y, tip.x, tip.y, CV_RGB(255,0,0));
}

void Line::Draw(IplImage *img, int detail, CvScalar col, int size)
{
  SDraw::DrawLine(img, point[START].x, point[START].y, point[END].x, point[END].y, col, size);
  if(detail >= 1)
  {
    char id_str[20];
    Vector2 mid_point = (point[START] + point[END])/2. + Vector2(4., -4.);
    snprintf(id_str, 20, "%u", id);
    SDraw::WriteText(img, id_str, mid_point.x, mid_point.y, CV_RGB(255,0,0));
    DrawArrow(img);
  }
}

double Line::MinEndpointDistance(const Line *l)
{
  double dist = HUGE;
  for(unsigned i = START; i <= END; i++)
    for(unsigned j = START; j <= END; j++)
      dist = fmin(dist, Distance(point[i], l->point[j]));
  return dist;
}

double Line::DistanceToPoint(const Vector2 &q)
{
  Vector2 start_to_q = q - point[START];
  double s = Dot(start_to_q, dir);
  if(s >= 0. && s <= Length())
    return fabs(Cross(start_to_q, dir));
  else
    return Distance(q, point[(s<0. ? START : END)]);
}

void Line::Recalc()
{
  point[START] = seg->edgels[idx[START]].p;
  point[END] = seg->edgels[idx[END]].p;
  CalculateParameters();
}

/**
 * Uses a prior image (e.g. colour or depth prior...)
 * to compute a significance.
 * Prior must be IPL_DEPTH_8U
 */
void Line::CalculateSignificance(IplImage *prior, double norm)
{
  unsigned sum, num;
  double mean1, mean2;
  Vector2 sdist, p1, p2;

  sdist = dir.Normal() * SAMPLE_DISTANCE;

  p1 = point[END] + sdist;
  p2 = point[START] + sdist;
  SampleLine(prior, p1.x, p1.y, p2.x, p2.y, sum, num);
  if (num>0)
    mean1 = (double)sum/(double)num;
  else
  {
    sig=0.;
    return;
  }

  p1 = point[END] - sdist;
  p2 = point[START] - sdist;
  SampleLine(prior, p1.x, p1.y, p2.x, p2.y, sum, num);
  if (num>0)
  {
    mean2 = (double)sum/(double)num;
    sig = (mean1>mean2?(mean1-mean2)/norm:(mean2-mean1)/norm);
  }else
    sig=0.;
}

/**
 * Accumulate the colour along a one channel IPL_DEPTH_8U image
 */
void Line::SampleLine(IplImage *prior, int x1, int y1, int x2, int y2, unsigned &sum, unsigned &num)
{
  sum=0; num=0;

  if (!IsImage8UC1(prior))
    return;

  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(prior->width-1, prior->height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
  // first octant bresenham
    err = -dx/2;
    do
    {
      sum+=GetPx8UC1(prior, x, y);
      num++;
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
        {
          // make line dense
          sum+=GetPx8UC1(prior, x, y);
          num++;
        }
      }
      x += inc_x;
    } while(x != x2); // TODO: x2 is not coloured!
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      sum+=GetPx8UC1(prior, x, y);
      num++;
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
        {
          // make line dense
          sum+=GetPx8UC1(prior, x, y);
          num++;
        }
      }
      y += inc_y;
    } while(y != y2);
  }
}

/**
 * Accumulate the colour along a one channel IPL_DEPTH_16S image
 */
void Line::SampleLine16SC1(IplImage *grad, int x1, int y1, int x2, int y2, int &sum, int &num)
{
  sum=0; num=0;

  if (!IsImage16SC1(grad))
    return;

  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(grad->width-1, grad->height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
  // first octant bresenham
    err = -dx/2;
    do
    {
      sum+=GetPx16SC1(grad, x, y);
      num++;
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
        {
          // make line dense
          sum+=GetPx16SC1(grad, x, y);
          num++;
        }
      }
      x += inc_x;
    } while(x != x2); // TODO: x2 is not coloured!
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      sum+=GetPx16SC1(grad, x, y);
      num++;
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
        {
          // make line dense
          sum+=GetPx16SC1(grad, x, y);
          num++;
        }
      }
      y += inc_y;
    } while(y != y2);
  }
}



bool Line::IsAtPosition(int x, int y)
{
  double xd = (double)x, yd = (double)y;
  for(unsigned i = idx[START]; i <= idx[END]; i++)
    if(IsEqual(xd, seg->edgels[i].p.x) &&
       IsEqual(yd, seg->edgels[i].p.y))
      return true;
  return false;
}

/**
 * Detete an array of segments
 */
void DeleteLines(P::Array<Line *> &lines)
{
  for (unsigned i=0; i<lines.Size(); i++)
    delete lines[i];
  lines.Clear();
}

}

