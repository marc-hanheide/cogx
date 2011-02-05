/**
 * @author Michael Zillich
 * @date September 2010
 */

#include <ios>
#include <cv.h>
#include "PlanarPatch.h"

using namespace cogx::Math;

bool PlanarPatch::isPointInside(Vector3 &q) const
{
  int i, j;
  const std::vector<Vector2> &h = projectedConvexHull;
  Vector3 pp = transformInverse(pose, q);
  Vector2 p = vector2(pp.x, pp.y);
  for(i = 0, j = h.size() - 1; i < h.size(); j = i++)
  {
    Vector2 a = h[i] - h[j];
    Vector2 b = p - h[j];
    if(!leftOf(a, b))
      return false;
  }
  return true;

  /*int i, j;
  bool c = false;
  const std::vector<Vector2> &h = projectedConvexHull;
  Vector3 p = transformInverse(pose, q);
  for(i = 0, j = h.size() - 1; i < h.size(); j = i++)
  {
    if((h[i].y<=p.y) && (p.y< (h[j].x - h[i].x) * (p.y - h[i].y) / (h[j].y - h[i].y) + h[i].x))
      c = !c;
  }
  return c;*/
}
