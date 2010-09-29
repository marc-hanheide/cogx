/**
 * @file StereoClosures.cpp
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of closures.
 */

#include "StereoClosures.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpClosures --------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpClosure
 * @param closure Object detector closure
 */
TmpClosure::TmpClosure(Closure *closure)
{
	surf.Init(closure);
}

/**
 * @brief Recalculate all closure parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpClosure::RePrune(int oX, int oY, int sc)
{
	surf.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpClosure
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpClosure::Rectify(StereoCamera *stereo_cam, int side)
{
	surf.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpClosure
 */
void TmpClosure::Refine()
{
	surf.Refine();
}

/**
 * @brief Returns true, if closure is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if closure is at this position.
 */
// bool TmpFlap::IsAtPosition(int x, int y) const
// {
//   return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y);
// }

/**
 * @brief If a closure from the right image was matched with a closure from the left image we
 * typically have to shift the point arrays of the right surface to align with
 * the left points.
 * @param off0 Shift points off0 to the left
 */
void TmpClosure::Fuddle(unsigned off0)
{
	surf.ShiftPointsLeft(off0);
}


//-------------------------------------------------------------------//
//------------------------- StereoClosures --------------------------//
//-------------------------------------------------------------------//

/**
 * @brief Constructor of StereoClosures: Calculate stereo matching of closures
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoClosures::StereoClosures(VisionCore *vc[2], StereoCamera *sc) : StereoBase()
{
	vcore[LEFT] = vc[LEFT];
	vcore[RIGHT] = vc[RIGHT];
	stereo_cam = sc;
  closMatches = 0;
}

/**
 * @brief Draw matched closures.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoClosures::DrawMatched(int side, bool single, int id, int detail)
{
	if(single)
	{
		if(id < 0 || id >= closMatches)
		{
			printf("StereoClosures::DrawMatched: warning: id out of range!\n");
			return;
		}
		DrawSingleMatched(side, id, detail);
	}
	else
		for(int i=0; i< closMatches; i++)
			DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched closure.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoClosures::DrawSingleMatched(int side, int id, int detail)
{
	closures[side][id].surf.Draw(detail);
}


/**
 * @brief Convert closure from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector closure.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoClosures::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
	obj->model = new VisionData::GeometryModel;
	Closure3D closure = Closures(id);

	// Recalculate pose of vertices (relative to the pose of the flap == cog)
	Pose3 pose;
	RecalculateCoordsystem(closure, pose);

	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = pose.pos.x;
	cogxPose.pos.y = pose.pos.y;
	cogxPose.pos.z = pose.pos.z;
	obj->pose = cogxPose;

	// create vertices (relative to the 3D center point)
	// How many vertices?
	for(unsigned i=0; i<closure.surf.vertices.Size(); i++)
	{
		VisionData::Vertex v;
		v.pos.x = closure.surf.vertices[i].p.x;
		v.pos.y = closure.surf.vertices[i].p.y;
		v.pos.z = closure.surf.vertices[i].p.z;
		obj->model->vertices.push_back(v);
	}

	VisionData::Face f;
	for(unsigned j=0; j<closure.surf.vertices.Size(); j++)
		f.vertices.push_back(j);
	obj->model->faces.push_back(f);
	f.vertices.clear();
	obj->detectionConfidence = 1.0;						// TODO detection confidence is always 1

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
void StereoClosures::RecalculateCoordsystem(Closure3D &closure, Pose3 &pose)
{
  Vector3 c(0., 0., 0.);
  int cnt = 0;
  // find the center of gravity
	for(unsigned i = 0; i < closure.surf.vertices.Size(); i++)
	{
		c += closure.surf.vertices[i].p;
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
	for(unsigned i = 0; i < closure.surf.vertices.Size(); i++)
	{
		Vector3 p(closure.surf.vertices[i].p.x,
							closure.surf.vertices[i].p.y,
							closure.surf.vertices[i].p.z);
		closure.surf.vertices[i].p = inv.Transform(p);
	}
}


/**
 * @brief Find right best matching closure for given left closures, begining at position l of right closure array.
 * @param left_clos Tmp. closure of left stereo image.
 * @param right_clos Array of all closures from right stereo image.
 * @param l Begin at position l of right closure array
 * @return Returns position of best matching right closure from the right_clos array.
 */
unsigned StereoClosures::FindMatchingClosure(TmpClosure &left_clos, Array<TmpClosure> &right_clos, unsigned l)
{
	double match, best_match = HUGE;
	unsigned j, j_best = UNDEF_ID;
	unsigned off_0, off_0_best = 0;

// 	bool cross = false, cross_best = false;
	for(j = l; j < right_clos.Size(); j++)
	{
// 		match = MatchingScore(left_rect, right_rects[j], off_0);
// printf("    find matching score of right rect %u\n", j);
    match = MatchingScoreSurf(left_clos.surf, right_clos[j].surf, off_0);
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
		right_clos[j_best].Fuddle(off_0_best);
	}
	return j_best;
}


/**
 * @brief Match left and right rectangles from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_rects Array of all rectangles from left stereo image (matching rectangles get sorted to the beginning of the array.)
 * @param right_rects Array of all rectangles from right stereo image.
 * @param matches Number of matched rectangles (sorted to the beginning of the arrays).
 */
void StereoClosures::MatchClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches)
{
  unsigned j, l = 0, u = left_clos.Size();
  for(; l < u && l < right_clos.Size();)
  {
    j = FindMatchingClosure(left_clos[l], right_clos, l);
// printf("  left: %u - best right found: %u:", l, j);
    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
// printf(" MATCH!\n");
      right_clos.Swap(l, j);				/// wechsle gefundenes right_rects[j] an selbe Stelle wie left_rects ==> l
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
// printf(" No match!\n");
      left_clos.Swap(l, u-1);				/// wechsle left_rects[l] an die letzte Stelle u-1 und mach u um eins kleiner
      u--;
    }
  }
  u = min(u, right_clos.Size());
  matches = u;
}


/**
 * @brief Calculate 3D closures from matched closures.
 * @param left_clos Array of all closures from left stereo image.
 * @param right_clos Array of all closurs from right stereo image.
 * @param matches Number of matched closures.
 * @param closure3ds Array of calculated 3d closures.
 */
void StereoClosures::Calculate3DClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches, Array<Closure3D> &closure3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Closure3D closure3d;
		if (closure3d.surf.Reconstruct(stereo_cam, left_clos[i].surf, right_clos[i].surf, false))
		{
			closure3ds.PushBack(closure3d);
			i++;
    }
    // move unacceptable closures to the end
    else
    {
      left_clos.Swap(i, u-1);
      right_clos.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}


/**
 * @brief Delete all arrays ...
 */
void StereoClosures::ClearResults()
{
	closure3ds.Clear();
	closures[LEFT].Clear();
	closures[RIGHT].Clear();
	closMatches = 0;
}


/**
 * @brief Match and calculate 3D closures from 2D closures.
 */
void StereoClosures::Process()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
		for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::CLOSURE); i++)
		{
			Closure *core_closure = (Closure*)vcore[side]->Gestalts(Gestalt::CLOSURE, i);
			if(!vcore[side]->use_masking || !core_closure->IsMasked())
			{
				TmpClosure closure(core_closure);
				if(closure.IsValid())
					closures[side].PushBack(closure);
			}
		}
		if(pPara.pruning)
			for(unsigned i = 0; i < closures[side].Size(); i++)
				closures[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
		for(unsigned i = 0; i < closures[side].Size(); i++)
			closures[side][i].Rectify(stereo_cam, side);
		for(unsigned i = 0; i < closures[side].Size(); i++)
			closures[side][i].Refine();
	}

  // do stereo matching and depth calculation
	closMatches = 0;
	MatchClosures(closures[LEFT], closures[RIGHT], closMatches);
	Calculate3DClosures(closures[LEFT], closures[RIGHT], closMatches, closure3ds);
}


/**
 * @brief Match and calculate 3D closures from 2D closures.
 */
void StereoClosures::Process(int oX, int oY, int sc)
{
	pPara.pruning = true;
	pPara.offsetX = oX;
	pPara.offsetY = oY;
	pPara.scale = sc;
	Process();
	pPara.pruning = false;
}


}








