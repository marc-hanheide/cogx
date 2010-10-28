/**
 * @file StereoRectangles.cpp
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of rectangles.
 */

#include "StereoRectangles.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpRectangle -------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpRectangle
 * @param flap vs3 flap
 */
TmpRectangle::TmpRectangle(Rectangle *rectangle)
{
	surf.Init(rectangle);
}


/**
 * @brief Recalculate all rectangle parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpRectangle::RePrune(int oX, int oY, int sc)
{
	surf.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpFlap
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpRectangle::Rectify(StereoCamera *stereo_cam, int side)
{
	surf.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpFlap
 */
void TmpRectangle::Refine()
{
  surf.Refine();
}

/**
 * @brief Returns true, if rectangle is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if flap is at this position.
 */
bool TmpRectangle::IsAtPosition(int x, int y) const
{
  return surf.IsAtPosition(x, y);
}

/**
 * @brief If a rectangle from the right image was matched with a rectangle from the left image we
 * typically have to shift the point arrays of the right surfaces to align with
 * the left points.
 * @param off0 TODO Shift points off0 to the left
 */
void TmpRectangle::Fuddle(unsigned off0)
{
  surf.ShiftPointsLeft(off0);
}


//-------------------------------------------------------------------//
//------------------------- StereoRectangles ------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoFlaps: Calculate stereo matching of flaps
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoRectangles::StereoRectangles(VisionCore *vc[2], StereoCamera *sc) : StereoBase()
{
	vcore[LEFT] = vc[LEFT];
	vcore[RIGHT] = vc[RIGHT];
	stereo_cam = sc;
  rectMatches = 0;
}

/**
 * @brief Draw matched rectangles.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoRectangles::DrawMatched(int side, bool single, int id, int detail)
{
	if(single)
	{
		if(id < 0 || id >= rectMatches)
		{
			printf("StereoClosures::DrawMatched: warning: id out of range!\n");
			return;
		}
		DrawSingleMatched(side, id, detail);
	}
	else
		for(int i=0; i< rectMatches; i++)
			DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched closure.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoRectangles::DrawSingleMatched(int side, int id, int detail)
{
	rectangles[side][id].surf.Draw(detail);
}

/**
 * @brief Convert rectangle from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector rectangle.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoRectangles::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
	obj->model = new VisionData::GeometryModel;
	Rectangle3D rectangle = Rectangles(id);

	// Recalculate pose of vertices (relative to the pose of the flap == cog)
	Pose3 pose;
	RecalculateCoordsystem(rectangle, pose);

	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = pose.pos.x;
	cogxPose.pos.y = pose.pos.y;
	cogxPose.pos.z = pose.pos.z;
	obj->pose = cogxPose;

	// create vertices (relative to the 3D center point)
	for(unsigned i=0; i<rectangle.surf.vertices.Size(); i++)		// TODO Rectangle hat 4 L-Junctions!!! nicht immer richtig!
	{
		VisionData::Vertex v;
		v.pos.x = rectangle.surf.vertices[i].p.x;
		v.pos.y = rectangle.surf.vertices[i].p.y;
		v.pos.z = rectangle.surf.vertices[i].p.z;
		obj->model->vertices.push_back(v);
	}

	// add faces to the vision model
	VisionData::Face f;
	f.vertices.push_back(0);
	f.vertices.push_back(1);
	f.vertices.push_back(2);
	f.vertices.push_back(3);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	obj->detectionConfidence = 1.0;															// TODO detection confidence is always 1

	return true;
}
#endif

/**
 * @brief Try to find a "natural" looking coordinate system for a flap.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity of the corner points as position and set orientation to identity.
 * @param rectangle 3D rectangle
 * @param pose calculated pose
 */
void StereoRectangles::RecalculateCoordsystem(Rectangle3D &rectangle, Pose3 &pose)
{
  Vector3 c(0., 0., 0.);
  int cnt = 0;
  // find the center of gravity
	for(unsigned i = 0; i < rectangle.surf.vertices.Size(); i++)
	{
		c += rectangle.surf.vertices[i].p;
		cnt++;
	}

  c /= (double)cnt;
  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;

	// set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.x = 0.;
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

	// recalculate the vectors to the vertices from new center point
	for(unsigned i = 0; i < rectangle.surf.vertices.Size(); i++)
	{
		Vector3 p(rectangle.surf.vertices[i].p.x,
							rectangle.surf.vertices[i].p.y,
							rectangle.surf.vertices[i].p.z);
		rectangle.surf.vertices[i].p = inv.Transform(p);
	}
}


/**
 * @brief Find right best matching rectangle for given left rectangles, begining at position l of right rectangle array.
 * @param left_rect Tmp. rectangle of left stereo image.
 * @param right_rects Array of all rectangles from right stereo image.
 * @param l Begin at position l of right rectangle array
 * @return Returns position of best matching right rectangle from the right_rects array.
 */
unsigned StereoRectangles::FindMatchingRectangle(TmpRectangle &left_rect, Array<TmpRectangle> &right_rects, unsigned l)
{
// printf("    FindMatchingRectangle:\n");
	double match, best_match = HUGE;
	unsigned j, j_best = UNDEF_ID;
	unsigned off_0, off_0_best = 0;

// 	bool cross = false, cross_best = false;
	for(j = l; j < right_rects.Size(); j++)
	{
// 		match = MatchingScore(left_rect, right_rects[j], off_0);
// printf("    find matching score of right rect %u\n", j);
    match = MatchingScoreSurf(left_rect.surf, right_rects[j].surf, off_0);
// printf("      match = %6.5f\n", match);
		if(match < best_match)
		{
			best_match = match;
			j_best = j;
// 			cross_best = cross;
			off_0_best = off_0;
// 			off_1_best = off_1;
		}
	}
	if(j_best != UNDEF_ID)
	{
		right_rects[j_best].Fuddle(off_0_best);
	}
	return j_best;
}


/**
 * @brief Match left and right rectangles from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_rects Array of all rectangles from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_rects Array of all rectangles from right stereo image.
 * @param matches Number of matched rectangles (sorted to the beginning of the arrays).
 */
void StereoRectangles::MatchRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches)
{
  unsigned j, l = 0, u = left_rects.Size();
  for(; l < u && l < right_rects.Size();)
  {
    j = FindMatchingRectangle(left_rects[l], right_rects, l);

		// found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_rects.Swap(l, j);			// change right_rects[j] to same place as left_rects (==>l)
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_rects.Swap(l, u-1);		// change left_rects[l] to last place (u-1) and do not consider in future.
      u--;
    }
  }
  u = min(u, right_rects.Size());
  matches = u;
}


/**
 * @brief Calculate 3D rectangles from matched rectangles.
 * @param left_rects Array of all flaps from left stereo image.
 * @param right_rects Array of all flaps from right stereo image.
 * @param matches Number of matched rectangles.
 * @param rectangle3ds Array of calculated 3d flaps.
 */
void StereoRectangles::Calculate3DRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches, Array<Rectangle3D> &rectangle3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Rectangle3D rectangle3d;
		if (rectangle3d.surf.Reconstruct(stereo_cam, left_rects[i].surf, right_rects[i].surf, true))
		{
      rectangle3ds.PushBack(rectangle3d);
      i++;
    }
    // move unacceptable flaps to the end
    else
    {
      left_rects.Swap(i, u-1);
      right_rects.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}


/**
 * @brief Delete all arrays ...
 */
void StereoRectangles::ClearResults()
{
	rectangles[LEFT].Clear();
	rectangles[RIGHT].Clear();
	rectangle3ds.Clear();
	rectMatches = 0;
}


/**
 * @brief Match and calculate 3D rectangles from 2D rectangles.
 */
void StereoRectangles::Process()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
		for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::RECTANGLE); i++)
		{
			Rectangle *core_rectangle = (Rectangle*)vcore[side]->Gestalts(Gestalt::RECTANGLE, i);
			if(!vcore[side]->use_masking || !core_rectangle->IsMasked())
			{
				TmpRectangle rectangle(core_rectangle);
				if(rectangle.IsValid())
					rectangles[side].PushBack(rectangle);
			}
		}
		if(pPara.pruning)
			for(unsigned i = 0; i < rectangles[side].Size(); i++)
				rectangles[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
		for(unsigned i = 0; i < rectangles[side].Size(); i++)
			rectangles[side][i].Rectify(stereo_cam, side);
		for(unsigned i = 0; i < rectangles[side].Size(); i++)
			rectangles[side][i].Refine();
	}

  // do stereo matching and depth calculation
	rectMatches = 0;
	MatchRectangles(rectangles[LEFT], rectangles[RIGHT], rectMatches);
	Calculate3DRectangles(rectangles[LEFT], rectangles[RIGHT], rectMatches, rectangle3ds);
}


/**
 * @brief Match and calculate 3D rectangles from 2D rectangles.
 * @param oX Offset in x-direction.
 * @param oY Offset in y-direction
 * @param sc Scale factor.
 */
void StereoRectangles::Process(int oX, int oY, int sc)
{
	pPara.pruning = true;
	pPara.offsetX = oX;
	pPara.offsetY = oY;
	pPara.scale = sc;
	Process();
	pPara.pruning = false;
}


}








