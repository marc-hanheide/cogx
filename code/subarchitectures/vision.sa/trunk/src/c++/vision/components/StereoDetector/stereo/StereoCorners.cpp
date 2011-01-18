/**
 * @file StereoCorners.h
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Stereo calculation of corners. Corners are intersections of 3 lines 
 * (logic: 2 l-junctions with one sharing arm)
 */

#include "StereoCorners.h"

namespace Z
{

//--------------------------------------------------------------//
//-------------------------- TmpCorner -------------------------//
//--------------------------------------------------------------//
/**
 * @brief Constructor TmpCorner
 * @param corner vs3 corner
 */
TmpCorner::TmpCorner(Corner *corner)
{
  point2D.p.x = corner->isct.x;
  point2D.p.y = corner->isct.y;
}


/**
 * @brief Recalculate all rectangle parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpCorner::RePrune(int oX, int oY, int sc)
{
  point2D.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpCorner
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpCorner::Rectify(StereoCamera *stereo_cam, int side)
{
  point2D.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpCorner
 */
void TmpCorner::Refine()
{
  point2D.Refine();
}

/**
 * @brief Returns true, if corner junction is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if corner junction is at this position.
 */
bool TmpCorner::IsAtPosition(int x, int y) const
{
  return point2D.IsAtPosition(x, y);
}


//----------------------------------------------------------------//
//------------------------- StereoCorners ------------------------//
//----------------------------------------------------------------//
/**
 * @brief Constructor of StereoFlaps: Calculate stereo matching of flaps
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoCorners::StereoCorners(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc) : StereoBase(sco)
{
  vcore[LEFT] = vc[LEFT];
  vcore[RIGHT] = vc[RIGHT];
  stereo_cam = sc;
  cornerMatches = 0;
}


/**
 * @brief Draw matched corners.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoCorners::DrawMatched(int side, bool single, int id, int detail)
{
// printf("StereoLJunctions::DrawMatched!\n");
  if(single)
  {
    if(id < 0 || id >= cornerMatches)
    {
      std::printf("StereoCorners::DrawMatched: warning: id out of range!\n");
      return;
    }
    DrawSingleMatched(side, id, detail);
  }
  else
    for(int i=0; i< cornerMatches; i++)
      DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched closure.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoCorners::DrawSingleMatched(int side, int id, int detail)
{
  corners[side][id].point2D.Draw();
}

/**
 * @brief Convert corner from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector rectangle.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoCorners::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
printf("StereoCorners::StereoGestalt2VisualObject: Not yet implemented.\n");														/// TODO TODO TODO Not yet implemented!
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
void StereoCorners::RecalculateCoordsystem(Corner3D &corner, Pose3 &pose)
{
std::printf("StereoCorners::RecalculateCoordsystem: Not yet implemented!\n");
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
 * @brief Find right best matching corner for given left corner, begining at position l of right corner array.
 * @param left_rect Tmp. rectangle of left stereo image.
 * @param right_rects Array of all rectangles from right stereo image.
 * @param l Begin at position l of right rectangle array
 * @return Returns position of best matching right rectangle from the right_rects array.
 */
unsigned StereoCorners::FindMatchingCorner(TmpCorner &left_corner, Array<TmpCorner> &right_corners, unsigned l)
{
  double match, best_match = HUGE;
  unsigned j, j_best = UNDEF_ID;				// we start at j and try to find j_best (!=UNDEF_ID)

  for(j = l; j < right_corners.Size(); j++)
  {
    match = MatchingScorePoint(left_corner.point2D, right_corners[j].point2D);

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
 * @brief Match left and right corner from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_corners Array of all corners from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_corners Array of all corners from right stereo image.
 * @param matches Number of matched corners (sorted to the beginning of the arrays).
 */
void StereoCorners::MatchCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches)
{
// printf("StereoLJunctions::MatchLJunctions: start:\n");

  unsigned j, l = 0, u = left_corners.Size();
  for(; l < u && l < right_corners.Size();)
  {
    j = FindMatchingCorner(left_corners[l], right_corners, l);

    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_corners.Swap(l, j);				// change found right_ljcts[j] at same position than left_ljcts ==> l
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_corners.Swap(l, u-1);			// change found left_ljcts[l] to last position
      u--;
    }
  }
  u = std::min(u, right_corners.Size());
  matches = u;
}


/**
 * @brief Calculate 3D points from matched corners.
 * @param left_corners Array of all corners from left stereo image.
 * @param right_corners Array of all corners from right stereo image.
 * @param matches Number of matched points.
 * @param corner3ds Array of calculated 3d corners.
 */
void StereoCorners::Calculate3DCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Corner3D *corner3d = new Corner3D();
    if (corner3d->point3D.Reconstruct(stereo_cam, left_corners[i].point2D, right_corners[i].point2D))
    {
      score->NewGestalt3D(corner3d);
      i++;
    }
    // move unacceptable points to the end
    else
    {
      left_corners.Swap(i, u-1);
      right_corners.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}


/**
 * @brief Clear all arrays.
 */
void StereoCorners::ClearResults()
{
  corners[LEFT].Clear();
  corners[RIGHT].Clear();
  cornerMatches = 0;
}


/**
 * @brief Match and calculate 3D corners from 2D corners.
 * @param side LEFT/RIGHT image of stereo.images.
 */
void StereoCorners::Process()
{
// printf("StereoLJunctions::Process: implemented!\n");
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::CORNER); i++)
    {
      Corner *core_corner = (Corner*)vcore[side]->Gestalts(Gestalt::CORNER, i);
      if(!vcore[side]->use_masking || !core_corner->IsMasked())
      {
	TmpCorner corner(core_corner);
	if(corner.IsValid())
	   corners[side].PushBack(corner);
      }
    }
    if(pPara.pruning)
      for(unsigned i = 0; i < corners[side].Size(); i++)
	corners[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
    for(unsigned i = 0; i < corners[side].Size(); i++)
	corners[side][i].Rectify(stereo_cam, side);
    for(unsigned i = 0; i < corners[side].Size(); i++)
	corners[side][i].Refine();
  }

  // do stereo matching and depth calculation
  cornerMatches = 0;
// printf("StereoLJunctions::Process: left: %u - right: %u\n", ljcts[LEFT].Size(), ljcts[RIGHT].Size());
  MatchCorners(corners[LEFT], corners[RIGHT], cornerMatches);
// printf("MatchedLJunctions: %u\n", ljctMatches);
  Calculate3DCorners(corners[LEFT], corners[RIGHT], cornerMatches);
}


/**
 * @brief Match and calculate 3D corners from 2D corners.
 * @param oX Offset in x-direction
 * @param oY Offset in y-direction
 * @param sc Scale
 */
void StereoCorners::Process(int oX, int oY, int sc)
{
  pPara.pruning = true;
  pPara.offsetX = oX;
  pPara.offsetY = oY;
  pPara.scale = sc;
  Process();
  pPara.pruning = false;
}


}








