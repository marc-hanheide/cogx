/**
 * @file StereoFlapsAri.cpp
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of flaps, based on rectangles.
 */

#include <vector>
#include "StereoFlapsAri.h"

namespace Z
{

//-----------------------------------------------------------------//
//---------------------------- TmpFlapAri ----------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpFlapAri
 * Construct TmpFlapAri from the six isct-points of FlapAri
 * @param flap vs3 flap
 */
TmpFlapAri::TmpFlapAri(FlapAri *flap)
{
  id2D = flap->ID();
  std::vector<Vector2> p0, p1;
  int j;
  for(unsigned i=0; i<4; i++)
  {
    p0.push_back(flap->isct[i]);
    j=i+3; if(j>5) j=0;
    p1.push_back(flap->isct[j]);
  }
  surf[0].Init(p0);   // isct[0/1/2/3]
  surf[1].Init(p1);   // isct[3/4/5/0]
}

/**
 * @brief Recalculate all flap parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpFlapAri::RePrune(int oX, int oY, int sc)
{
  surf[0].RePrune(oX, oY, sc);
  surf[1].RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpFlapAri
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpFlapAri::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  surf[0].Rectify(stereo_cam, side);
  surf[1].Rectify(stereo_cam, side);
}

/**
 * @brief Refine TmpFlapAri
 */
void TmpFlapAri::Refine()
{
  surf[0].Refine();
  surf[1].Refine();
}

/**
 * @brief Returns true, if flap is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if flap is at this position.
 */
bool TmpFlapAri::IsAtPosition(int x, int y) const
{
  return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y);
}

/**
 * @brief If a flap from the right image was matched with a flap from the left image we
 * typically have to shift the point arrays of the right surfaces to align with
 * the left points and maybe swap surface 0 and 1 of the right flap.
 * Note that we first shift points and then swap surfaces.
 * @param off0 TODO Shift points ???
 * @param off1 TODO Shift points ???
 * @param swap Swap the the two surfaces 
 */
void TmpFlapAri::Fuddle(unsigned off0, unsigned off1, bool swap)
{
  surf[0].ShiftPointsLeft(off0);
  surf[1].ShiftPointsLeft(off1);
  if(swap)
  {
    Surf2D t = surf[1];
    surf[1] = surf[0];
    surf[0] = t;
  }
}

//------------------------------------------------------------------//
//--------------------------- StereoFlaps --------------------------//
//------------------------------------------------------------------//
/**
 * @brief Constructor of StereoFlaps: Calculate stereo matching of flaps
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 * @param sc Stereo camera parameters
 */
StereoFlapsAri::StereoFlapsAri(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
{
  vcore[LEFT] = vc[LEFT];
  vcore[RIGHT] = vc[RIGHT];
  stereo_cam = sc;
  flapMatches = 0;
}

/**
 * @brief Number of Flaps in 2D 
 * @param side LEFT/RIGHT side of stereo rig.
 */
int StereoFlapsAri::NumFlaps2D(int side)
{
  assert(side == LEFT || side == RIGHT);
  return flaps[side].Size();
}

/**
 * @brief Delivers 2D tmp. flap.
 * @param side LEFT/RIGHT side of stereo rig.
 * @param i Position of the flap in the array
 */
const TmpFlapAri &StereoFlapsAri::Flaps2D(int side, int i)
{
  assert(side == LEFT || side == RIGHT);
  return flaps[side][i];
}

/**
 * @brief Draw matched closures.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoFlapsAri::DrawMatched(int side, bool single, int id, int detail)
{
  if(single)
  {
    if(id < 0 || id >= flapMatches)
    {
      printf("StereoClosures::DrawMatched: warning: id out of range!\n");
      return;
    }
    DrawSingleMatched(side, id, detail);
  }
  else
    for(int i=0; i< flapMatches; i++)
      DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched closure.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoFlapsAri::DrawSingleMatched(int side, int id, int detail)
{
  flaps[side][id].surf[0].Draw(detail);
  flaps[side][id].surf[1].Draw(detail); 
}


/**
 * @brief Convert flap from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector flap.
 * @return Return true for success.
 */
#ifdef HAVE_CAST
bool StereoFlapsAri::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
  obj->model = new VisionData::GeometryModel;
  Flap3D *flap = Flaps3D(score, id);

  // Recalculate pose of vertices (relative to the center of gravity == COG)
  Pose3 pose;
  Vector3 c(0., 0., 0.);
  int cnt = 0;
  Vertex3D point[6];
  if(!flap->GetPoints(point)) 
    printf("StereoFlapsAri::StereoGestalt2VisualObject: Warning: No flap corner points calculated!\n");
  for(int i = 0; i < 6; i++)   // find the center of gravity
  {
    c += point[i].p;
    cnt++;
  }
  
printf("StereoFlapsAri::StereoGestalt2VisualObject: point: %4.2f/%4.2f/%4.2f\n", point[0].p.x, point[0].p.y, point[0].p.z);
  c /= (double)cnt;
  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;
  pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

  // add center point to the model
  cogx::Math::Pose3 cogxPose;
  cogxPose.pos.x = pose.pos.x;
  cogxPose.pos.y = pose.pos.y;
  cogxPose.pos.z = pose.pos.z;
  obj->pose = cogxPose;

  // recalculate the vectors to the vertices from new center point
  for(int i = 0; i < 6; i++)
  {
    Vector3 p(point[i].p.x,
	      point[i].p.y,
	      point[i].p.z);
    p = inv.Transform(p);
    
    VisionData::Vertex v;
    v.pos.x = p.x;
    v.pos.y = p.y;
    v.pos.z = p.z;
    obj->model->vertices.push_back(v);
  }

  // create vertices (relative to the 3D center point)
  VisionData::Face f;
  f.vertices.push_back(0);
  f.vertices.push_back(1);
  f.vertices.push_back(2);
  f.vertices.push_back(3);
  obj->model->faces.push_back(f);
  f.vertices.clear();
  f.vertices.push_back(3);
  f.vertices.push_back(4);
  f.vertices.push_back(5);
  f.vertices.push_back(0);
  obj->model->faces.push_back(f);
  f.vertices.clear();
  
  obj->detectionConfidence = 1.0;                                // TODO detection confidence is always 1

//   obj->model = new VisionData::GeometryModel;
//   Flap3D *flap = Flaps3D(score, id);
// 
//   // Recalculate pose of vertices (relative to the center of gravity == COG)
//   Pose3 pose;
//   Vector3 c(0., 0., 0.);
//   int cnt = 0;
//   for(int i = 0; i <= 1; i++)   // find the center of gravity
//   {
//     for(unsigned j = 0; j < flap->surf[i].vertices.Size(); j++)
//     {
//       c += flap->surf[i].vertices[j].p;
//       cnt++;
//     }
//   }
//   c /= (double)cnt;
//   pose.pos.x = c.x;
//   pose.pos.y = c.y;
//   pose.pos.z = c.z;
//   pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
//   pose.rot.y = 0.;
//   pose.rot.z = 0.;
// 
//   // invert to get pose of world w.r.t. flap
//   Pose3 inv = pose.Inverse();
// 
//   // add center point to the model
//   cogx::Math::Pose3 cogxPose;
//   cogxPose.pos.x = pose.pos.x;
//   cogxPose.pos.y = pose.pos.y;
//   cogxPose.pos.z = pose.pos.z;
//   obj->pose = cogxPose;
// 
//   // recalculate the vectors to the vertices from new center point
//   for(int i = 0; i <= 1; i++)
//   {
//     for(unsigned j = 0; j < flap->surf[i].vertices.Size(); j++)
//     {
//       Vector3 p(flap->surf[i].vertices[j].p.x,
//                 flap->surf[i].vertices[j].p.y,
//                 flap->surf[i].vertices[j].p.z);
//       p = inv.Transform(p);
//       
//       VisionData::Vertex v;
//       v.pos.x = p.x;
//       v.pos.y = p.y;
//       v.pos.z = p.z;
//       obj->model->vertices.push_back(v);
//     }
//   }
// 
//   // create vertices (relative to the 3D center point)
//   VisionData::Face f;
//   for(unsigned i=0; i<=1; i++)	// LEFT/RIGHT rectangle of flap
//   {
//     for(unsigned j=0; j<flap->surf[i].vertices.Size(); j++)
//     {
//       f.vertices.push_back(j+(i*4));
//     }
//     obj->model->faces.push_back(f);
//     f.vertices.clear();
//   }
// 
//   obj->detectionConfidence = 1.0;                                // TODO detection confidence is always 1
  return true;
}
#endif



/**
 * @brief Calculate matching score for flaps
 * @param left_flap Left tmp. flap
 * @param right_flap Right tmp. flap
 * @param off_0 TODO Offset ???
 * @param off_1 TODO Offset ???
 * @param cross true, if match is "crossed" and not "straight"
 */
double StereoFlapsAri::MatchingScore(TmpFlapAri &left_flap, TmpFlapAri &right_flap, unsigned &off_0, unsigned &off_1, bool &cross)
{
  // _s .. straight (left flap surf 0 matches right flap surf 0)
  // _x .. crossed (left flap surf 0 matches right flap surf 1)
  unsigned off_s0, off_s1, off_x0, off_x1;
  double sc_s = MatchingScoreSurf(left_flap.surf[0], right_flap.surf[0], off_s0) +
                MatchingScoreSurf(left_flap.surf[1], right_flap.surf[1], off_s1);
  double sc_x = MatchingScoreSurf(left_flap.surf[0], right_flap.surf[1], off_x1) +
                MatchingScoreSurf(left_flap.surf[1], right_flap.surf[0], off_x0);

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

/**																																			/// TODO StereoFlaps verschieben
 * @brief Find right best matching flap for given left flaps, begining at position l of right flap array.
 * @param left_flap Tmp. flap of left stereo image.
 * @param right_flaps Array of all flaps from right stereo image.
 * @param l Begin at position l of right flap array
 * @return Returns position of best matching right flap from the right flap array.
 */
unsigned StereoFlapsAri::FindMatchingFlap(TmpFlapAri &left_flap, Array<TmpFlapAri> &right_flaps, unsigned l)
{
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
  
  // NOTE: The corners of the flaps are ordered (inner corner and than clockwise 6 points => therefore it is
  // not possible that a fuddling is necessary. Nevertheless a warning.
//   if(j_best != UNDEF_ID)	
//   {
//     right_flaps[j_best].Fuddle(off_0_best, off_1_best, cross_best);
//   }
  if(off_0_best != 0 || off_1_best !=0 || cross_best)
  {
    printf("StereoFlapsAri::FindMatchingFlap: Warning: fuddling necessary and not done.\n");
    printf("                                  offset left/right: %u/%u and crossing: %u\n", off_0_best, off_1_best, cross_best);
  }
  return j_best;
//   return -1;
}


/**
 * @brief Match left and right flaps from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_flaps Array of all flaps from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_flaps Array of all flaps from right stereo image.
 * @param matches Number of matched flaps (sorted to the beginning of the arrays).
 */
void StereoFlapsAri::MatchFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &matches)
{
  unsigned j, l = 0, u = left_flaps.Size();
  for(; l < u && l < right_flaps.Size();)
  {
    j = FindMatchingFlap(left_flaps[l], right_flaps, l);
    if(j != UNDEF_ID)    // found a matching right, move it to same index position as left
    {
      right_flaps.Swap(l, j);
      l++;
    }
    else                 // found no right, move left to end and decrease end
    {
      left_flaps.Swap(l, u-1);
      u--;
    }
  }
  u = min(u, right_flaps.Size());
  matches = u;
}


/**
 * @brief Calculate 3D flaps from matched flaps.
 * @param left_flaps Array of all flaps from left stereo image.
 * @param right_flaps Array of all flaps from right stereo image.
 * @param matches Number of matched flaps.
 * @param flap3ds Array of calculated 3d flaps.
 */
void StereoFlapsAri::Calculate3DFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &matches)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Flap3D *flap3d = new Flap3D();
    bool ok0 = flap3d->surf[0].Reconstruct(stereo_cam, left_flaps[i].surf[0], right_flaps[i].surf[0], true);
    bool ok1 = flap3d->surf[1].Reconstruct(stereo_cam, left_flaps[i].surf[1], right_flaps[i].surf[1], true);
    if(ok0 && ok1)
    {
      flap3d->CalcIdealFlap();
      score->NewGestalt3D(flap3d);
      i++;
    }
    else    // move unacceptable flaps to the end
    {
      left_flaps.Swap(i, u-1);
      right_flaps.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}

/**
 * @brief Delete all arrays ...
 */
void StereoFlapsAri::ClearResults()
{
  flaps[LEFT].Clear();
  flaps[RIGHT].Clear();
  flapMatches = 0;
}

/**
 * @brief Match and calculate 3D flaps from 2D flaps.
 */
void StereoFlapsAri::Process()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::FLAP_ARI); i++)
    {
      FlapAri *core_flap = (FlapAri*)vcore[side]->Gestalts(Gestalt::FLAP_ARI, i);
      if(!vcore[side]->use_masking || !core_flap->IsMasked())
      {
	TmpFlapAri flap(core_flap);
	if(flap.IsValid())
	  flaps[side].PushBack(flap);
      }
    }
    if(pPara.pruning)
      for(unsigned i = 0; i < flaps[side].Size(); i++)
	flaps[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
    for(unsigned i = 0; i < flaps[side].Size(); i++)
      flaps[side][i].Rectify(stereo_cam, side);
    for(unsigned i = 0; i < flaps[side].Size(); i++)
      flaps[side][i].Refine();
}

  // do stereo matching and depth calculation
  flapMatches = 0;
  MatchFlaps(flaps[LEFT], flaps[RIGHT], flapMatches);
  Calculate3DFlaps(flaps[LEFT], flaps[RIGHT], flapMatches);
}


/**
 * @brief Match and calculate 3D flaps from 2D flaps.
 */
void StereoFlapsAri::Process(int oX, int oY, int sc)
{
  pPara.pruning = true;
  pPara.offsetX = oX;
  pPara.offsetY = oY;
  pPara.scale = sc;
  Process();
  pPara.pruning = false;
}

}








