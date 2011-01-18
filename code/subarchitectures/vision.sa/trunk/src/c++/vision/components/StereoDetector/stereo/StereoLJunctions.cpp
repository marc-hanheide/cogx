/**
 * @file StereoLJunctions.cpp
 * @author Andreas Richtsfeld
 * @date Juni 2010
 * @version 0.1
 * @brief Stereo calculation of l-junctions.
 */

#include "StereoLJunctions.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpLJunction -------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpRectangle
 * @param flap vs3 flap
 */
TmpLJunction::TmpLJunction(LJunction *ljct)
{
  point2D.p.x = ljct->isct.x;
  point2D.p.y = ljct->isct.y;
}


/**
 * @brief Recalculate all rectangle parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpLJunction::RePrune(int oX, int oY, int sc)
{
  point2D.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpLJunction
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpLJunction::Rectify(StereoCamera *stereo_cam, int side)
{
  point2D.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpLJunction
 */
void TmpLJunction::Refine()
{
  point2D.Refine();
}

/**
 * @brief Returns true, if junction is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if junction is at this position.
 */
bool TmpLJunction::IsAtPosition(int x, int y) const
{
  return point2D.IsAtPosition(x, y);
}


//-------------------------------------------------------------------//
//------------------------- StereoLJunctions ------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoLJunctions: Calculate stereo matching of L-junctions
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoLJunctions::StereoLJunctions(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc) : StereoBase(sco)
{
  vcore[LEFT] = vc[LEFT];
  vcore[RIGHT] = vc[RIGHT];
  stereo_cam = sc;
  ljctMatches = 0;
}


/**
 * @brief Draw matched L-junctions.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoLJunctions::DrawMatched(int side, bool single, int id, int detail)
{
// printf("StereoLJunctions::DrawMatched!\n");
  if(single)
  {
    if(id < 0 || id >= ljctMatches)
    {
      printf("StereoLJunctions::DrawMatched: warning: id out of range!\n");
      return;
    }
    DrawSingleMatched(side, id, detail);
  }
  else
    for(int i=0; i< ljctMatches; i++)
      DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched closure.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoLJunctions::DrawSingleMatched(int side, int id, int detail)
{
  ljcts[side][id].point2D.Draw();
}

/**
 * @brief Convert rectangle from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector rectangle.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoLJunctions::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
// printf("StereoLJunctions::StereoGestalt2VisualObject: Not yet implemented.\n");														/// TODO TODO TODO Not yet implemented!
// 	obj->model = new VisionData::GeometryModel;
// 	Rectangle3D rectangle = Rectangles(id);
// 
// 	// Recalculate pose of vertices (relative to the pose of the flap == cog)
// 	Pose3 pose;
// 	RecalculateCoordsystem(rectangle, pose);
// 
// 	// add center point to the model
// 	cogx::Math::Pose3 cogxPose;
// 	cogxPose.pos.x = pose.pos.x;
// 	cogxPose.pos.y = pose.pos.y;
// 	cogxPose.pos.z = pose.pos.z;
// 	obj->pose = cogxPose;
// 
// 	// create vertices (relative to the 3D center point)
// 	for(unsigned i=0; i<rectangle.surf.vertices.Size(); i++)		// TODO Rectangle hat 4 L-Junctions!!! nicht immer richtig!
// 	{
// 		VisionData::Vertex v;
// 		v.pos.x = rectangle.surf.vertices[i].p.x;
// 		v.pos.y = rectangle.surf.vertices[i].p.y;
// 		v.pos.z = rectangle.surf.vertices[i].p.z;
// 		obj->model->vertices.push_back(v);
// 	}
// 
// 	// add faces to the vision model
// 	VisionData::Face f;
// 	f.vertices.push_back(0);
// 	f.vertices.push_back(1);
// 	f.vertices.push_back(2);
// 	f.vertices.push_back(3);
// 	obj->model->faces.push_back(f);
// 	f.vertices.clear();
// 
// 	obj->detectionConfidence = 1.0;															// TODO detection confidence is always 1

	return false;
}
#endif

/**
 * @brief Try to find a "natural" looking coordinate system for a flap.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity of the corner points as position and set orientation to identity.
 * @param rectangle 3D rectangle
 * @param pose calculated pose
 */
void StereoLJunctions::RecalculateCoordsystem(LJunction3D &ljct, Pose3 &pose)
{
printf("StereoLJunctions::RecalculateCoordsystem: Not yet implemented!\n");
//   Vector3 c(0., 0., 0.);
//   int cnt = 0;
//   // find the center of gravity
// 	for(unsigned i = 0; i < rectangle.surf.vertices.Size(); i++)
// 	{
// 		c += rectangle.surf.vertices[i].p;
// 		cnt++;
// 	}
// 
//   c /= (double)cnt;
//   pose.pos.x = c.x;
//   pose.pos.y = c.y;
//   pose.pos.z = c.z;
// 
// 	// set the orientation to identity, i.e. parallel to world coordinate system
//   pose.rot.x = 0.;
//   pose.rot.y = 0.;
//   pose.rot.z = 0.;
// 
//   // invert to get pose of world w.r.t. flap
//   Pose3 inv = pose.Inverse();
// 
// 	// recalculate the vectors to the vertices from new center point
// 	for(unsigned i = 0; i < rectangle.surf.vertices.Size(); i++)
// 	{
// 		Vector3 p(rectangle.surf.vertices[i].p.x,
// 							rectangle.surf.vertices[i].p.y,
// 							rectangle.surf.vertices[i].p.z);
// 		rectangle.surf.vertices[i].p = inv.Transform(p);
// 	}
}


/**
 * @brief Find right best matching rectangle for given left rectangles, begining at position l of right rectangle array.
 * @param left_rect Tmp. rectangle of left stereo image.
 * @param right_rects Array of all rectangles from right stereo image.
 * @param l Begin at position l of right rectangle array
 * @return Returns position of best matching right rectangle from the right_rects array.
 */
unsigned StereoLJunctions::FindMatchingLJunction(TmpLJunction &left_ljct, Array<TmpLJunction> &right_ljcts, unsigned l)
{
  double match, best_match = HUGE;
  unsigned j, j_best = UNDEF_ID;				// we start at j and try to find j_best (!=UNDEF_ID)

  for(j = l; j < right_ljcts.Size(); j++)
  {
    match = MatchingScorePoint(left_ljct.point2D, right_ljcts[j].point2D);

// printf("      match = %6.5f\n", match);
// if (match < HUGE)
// 	printf("  found matching score of right rect %u\n", j);

    if(match < best_match)
    {
      best_match = match;
      j_best = j;
    }
}
  return j_best;
}


/**
 * @brief Match left and right rectangles from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_rects Array of all rectangles from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_rects Array of all rectangles from right stereo image.
 * @param matches Number of matched rectangles (sorted to the beginning of the arrays).
 */
void StereoLJunctions::MatchLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches)
{
// printf("StereoLJunctions::MatchLJunctions: start:\n");

  unsigned j, l = 0, u = left_ljcts.Size();
  for(; l < u && l < right_ljcts.Size();)
  {
    j = FindMatchingLJunction(left_ljcts[l], right_ljcts, l);

    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_ljcts.Swap(l, j);				// change found right_ljcts[j] at same position than left_ljcts ==> l
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_ljcts.Swap(l, u-1);			// change found left_ljcts[l] to last position
      u--;
    }
  }
  u = min(u, right_ljcts.Size());
  matches = u;
}


/**
 * @brief Calculate 3D points from matched l-junctions.
 * @param left_ljcts Array of all l-junctions from left stereo image.
 * @param right_rects Array of all l-junctions from right stereo image.
 * @param matches Number of matched points.
 * @param ljct3ds Array of calculated 3d l-junctions.
 */
void StereoLJunctions::Calculate3DLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches, Array<LJunction3D> &ljct3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    LJunction3D ljct3d;
    if (ljct3d.point3D.Reconstruct(stereo_cam, left_ljcts[i].point2D, right_ljcts[i].point2D))
    {
      ljct3ds.PushBack(ljct3d);
      i++;
    }
    // move unacceptable points to the end
    else
    {
      left_ljcts.Swap(i, u-1);
      right_ljcts.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}


/**
 * @brief Clear all arrays.
 */
void StereoLJunctions::ClearResults()
{
  ljcts[LEFT].Clear();
  ljcts[RIGHT].Clear();
  ljct3ds.Clear();
  ljctMatches = 0;
}


/**
 * @brief Match and calculate 3D rectangles from 2D rectangles.
 * @param side LEFT/RIGHT image of stereo.images.
 */
void StereoLJunctions::Process()
{
// printf("StereoLJunctions::Process: implemented!\n");
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::L_JUNCTION); i++)
    {
      LJunction *core_ljct = (LJunction*)vcore[side]->Gestalts(Gestalt::L_JUNCTION, i);
      if(!vcore[side]->use_masking || !core_ljct->IsMasked())
      {
	TmpLJunction ljct(core_ljct);
	if(ljct.IsValid())
	   ljcts[side].PushBack(ljct);
      }
    }
    if(pPara.pruning)
      for(unsigned i = 0; i < ljcts[side].Size(); i++)
	ljcts[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
    for(unsigned i = 0; i < ljcts[side].Size(); i++)
	ljcts[side][i].Rectify(stereo_cam, side);
    for(unsigned i = 0; i < ljcts[side].Size(); i++)
	ljcts[side][i].Refine();
  }

  // do stereo matching and depth calculation
  ljctMatches = 0;
// printf("StereoLJunctions::Process: left: %u - right: %u\n", ljcts[LEFT].Size(), ljcts[RIGHT].Size());
  MatchLJunctions(ljcts[LEFT], ljcts[RIGHT], ljctMatches);
// printf("MatchedLJunctions: %u\n", ljctMatches);
  Calculate3DLJunctions(ljcts[LEFT], ljcts[RIGHT], ljctMatches, ljct3ds);
}


/**
 * @brief Match and calculate 3D rectangles from 2D rectangles.
 * @param side LEFT/RIGHT image of stereo.images.
 */
void StereoLJunctions::Process(int oX, int oY, int sc)
{
  pPara.pruning = true;
  pPara.offsetX = oX;
  pPara.offsetY = oY;
  pPara.scale = sc;
  Process();
  pPara.pruning = false;
}


}








