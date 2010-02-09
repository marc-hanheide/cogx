/**
 * @file StereoEllipses.cpp
 * @author Andreas Richtsfeld
 * @date December 2009
 * @version 0.1
 * @brief Stereo calculation of ellipses.
 */

#include "StereoEllipses.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpEllipse --------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpEllipse
 * @param ellipse Object detector ellipse
 */
TmpEllipse::TmpEllipse(Ellipse *ellipse)
{
	surf.Init(ellipse);
}

/**
 * @brief Rectify TmpEllipse
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpEllipse::Rectify(StereoCamera *stereo_cam, int side)
{
	surf.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpEllipse
 */
void TmpEllipse::Refine()
{
	surf.Refine();
}

/**
 * @brief Returns true, if ellipse is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if ellipse is at this position.
 */
// bool TmpFlap::IsAtPosition(int x, int y) const
// {
//   return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y);
// }

/**
 * @brief If a ellipse from the right image was matched with a ellipse from the left image we
 * typically have to shift the point arrays of the right surface to align with
 * the left points.
 * @param off0 Shift points off0 to the left
 */
void TmpEllipse::Fuddle(unsigned off0)
{
	surf.ShiftPointsLeft(off0);
}


//-------------------------------------------------------------------//
//------------------------- StereoEllipses --------------------------//
//-------------------------------------------------------------------//

/**
 * @brief Constructor of StereoEllipses: Calculate stereo matching of ellipses.
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoEllipses::StereoEllipses(VisionCore *vc[2], StereoCamera *sc) : StereoBase()
{
	vcore[LEFT] = vc[LEFT];
	vcore[RIGHT] = vc[RIGHT];
	stereo_cam = sc;
  ellMatches = 0;
}


/**
 * @brief Draw ellipses to the stereo images.
 * @param side Left/right side of stereo images.
 * @param masked Draw masked features.
 */
void StereoEllipses::Draw(int side, bool masked)
{
	SetColor(RGBColor::red);

	int nrEllipses = 0;
	if(side == LEFT) nrEllipses = NumEllipsesLeft2D();
	else nrEllipses = NumEllipsesRight2D();
	for(int i=0; i<nrEllipses; i++)
	{
		if (masked)
			vcore[side]->Gestalts(Gestalt::ELLIPSE, i)->Draw();	
		else
			if (vcore[side]->Gestalts(Gestalt::ELLIPSE, i)->IsUnmasked())
				vcore[side]->Gestalts(Gestalt::ELLIPSE, i)->Draw();	
	}
}


/**
 * @brief Draw matched ellipses.
 * @param side Left/right side of stereo images.
 */
void StereoEllipses::DrawMatched(int side)
{
	for(int i=0; i< ellMatches; i++)
		ellipses[side][i].surf.Draw(RGBColor::green);
}


/**
 * @brief Convert ellipse from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector's ellipse.
 * @return Return true for success
 */
bool StereoEllipses::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
// 	obj->model = new VisionData::GeometryModel;
// 	Closure3D closure = Closures(id);
// 
// 	// Recalculate pose of vertices (relative to the pose of the flap == cog)
// 	Pose3 pose;
// 	RecalculateCoordsystem(closure, pose);
// 
// 	// add center point to the model
// 	cogx::Math::Pose3 cogxPose;
// 	cogxPose.pos.x = pose.pos.x;
// 	cogxPose.pos.y = pose.pos.y;
// 	cogxPose.pos.z = pose.pos.z;
// 	obj->pose = cogxPose;
// 
// 	// create vertices (relative to the 3D center point)
// 	// How many vertices?
// 	for(unsigned i=0; i<closure.surf.vertices.Size(); i++)
// 	{
// 		VisionData::Vertex v;
// 		v.pos.x = closure.surf.vertices[i].p.x;
// 		v.pos.y = closure.surf.vertices[i].p.y;
// 		v.pos.z = closure.surf.vertices[i].p.z;
// 		obj->model->vertices.push_back(v);
// 	}
// 
// 	VisionData::Face f;
// 	for(unsigned j=0; j<closure.surf.vertices.Size(); j++)
// 		f.vertices.push_back(j);
// 	obj->model->faces.push_back(f);
// 	f.vertices.clear();
// 	obj->detectionConfidence = 1.0;						// TODO detection confidence is always 1

	return true;
}

/**
 * @brief Try to find a "natural" looking coordinate system for a ellipse.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity as position and set orientation to identity.
 * @param ellipse 3D ellipse
 * @param pose calculated pose
 */
void StereoEllipses::RecalculateCoordsystem(Ellipse3D &ellipse, Pose3 &pose)
{
/*  Vector3 c(0., 0., 0.);
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
	}*/
}


/**
 * @brief Find right best matching closure for given left closures, begining at position l of right closure array.
 * @param left_ell Tmp. ellipse of left stereo image.
 * @param right_ell Array of all ellipses from right stereo image.
 * @param l Begin at position l of right ellipse array
 * @return Returns position of best matching right ellipse from the right_ell array.
 */
unsigned StereoEllipses::FindMatchingEllipse(TmpEllipse &left_ell, Array<TmpEllipse> &right_ell, unsigned l)
{
// 	double match, best_match = HUGE;
// 	unsigned j, j_best = UNDEF_ID;
// 	unsigned off_0, off_0_best = 0;
// 
// // 	bool cross = false, cross_best = false;
// 	for(j = l; j < right_clos.Size(); j++)
// 	{
// // 		match = MatchingScore(left_rect, right_rects[j], off_0);
// // printf("    find matching score of right rect %u\n", j);
//     match = MatchingScoreSurf(left_clos.surf, right_clos[j].surf, off_0);
// // printf("      match = %6.5f\n", match);
// 		if(match < best_match)
// 		{
// 			best_match = match;
// 			j_best = j;
// // 			cross_best = cross;
// 			off_0_best = off_0;
// // 			off_1_best = off_1;
// 		}
// 	}
// 	if(j_best != UNDEF_ID)
// 	{
// 		right_clos[j_best].Fuddle(off_0_best);
// 	}
// 	return j_best;
}


/**
 * @brief Match left and right ellipses from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_ell Array of all ellipses from left stereo image (matching ellipses get sorted to the beginning of the array.)
 * @param right_ell Array of all ellipses from right stereo image.
 * @param matches Number of matched ellipses (sorted to the beginning of the arrays).
 */
void StereoEllipses::MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches)
{
//   unsigned j, l = 0, u = left_clos.Size();
//   for(; l < u && l < right_clos.Size();)
//   {
//     j = FindMatchingClosure(left_clos[l], right_clos, l);
// // printf("  left: %u - best right found: %u:", l, j);
//     // found a matching right, move it to same index position as left
//     if(j != UNDEF_ID)
//     {
// // printf(" MATCH!\n");
//       right_clos.Swap(l, j);				/// wechsle gefundenes right_rects[j] an selbe Stelle wie left_rects ==> l
//       l++;
//     }
//     // found no right, move left to end and decrease end
//     else
//     {
// // printf(" No match!\n");
//       left_clos.Swap(l, u-1);				/// wechsle left_rects[l] an die letzte Stelle u-1 und mach u um eins kleiner
//       u--;
//     }
//   }
//   u = min(u, right_clos.Size());
//   matches = u;
}


/**
 * @brief Calculate 3D ellipses from matched ellipses.
 * @param left_ell Array of all ellipses from left stereo image.
 * @param right_ell Array of all ellipses from right stereo image.
 * @param matches Number of matched ellipses.
 * @param ellipse3ds Array of calculated 3d ellipses.
 */
void StereoEllipses::Calculate3DEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches, Array<Ellipse3D> &ellipse3ds)
{
//   unsigned u = matches;
//   for(unsigned i = 0; i < u;)
//   {
//     Closure3D closure3d;
// 		if (closure3d.surf.Reconstruct(stereo_cam, left_clos[i].surf, right_clos[i].surf, false))
// 		{
// 			closure3ds.PushBack(closure3d);
// 			i++;
//     }
//     // move unacceptable closures to the end
//     else
//     {
//       left_clos.Swap(i, u-1);
//       right_clos.Swap(i, u-1);
//       u--;
//     }
//   }
//   matches = u;
}


/**
 * @brief Delete all arrays ...
 */
void StereoEllipses::ClearResults()
{
	ellipse3ds.Clear();
	ellipses[LEFT].Clear();
	ellipses[RIGHT].Clear();
	ellMatches = 0;
}


/**
 * @brief Match and calculate 3D closures from 2D closures.
 * @param side LEFT/RIGHT image of stereo.images.
 */
void StereoEllipses::Process()
{
printf("#### StereoEllipses::Process()\n");
  for(int side = LEFT; side <= RIGHT; side++)
  {
		for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::ELLIPSE); i++)
		{
			Ellipse *core_ellipse = (Ellipse*)vcore[side]->Gestalts(Gestalt::ELLIPSE, i);
			if(!vcore[side]->use_masking || !core_ellipse->IsMasked())
			{
				TmpEllipse ellipse(core_ellipse);
				if(ellipse.IsValid())
					ellipses[side].PushBack(ellipse);
			}
		}
// 		for(unsigned i = 0; i < closures[side].Size(); i++)
// 			closures[side][i].Rectify(stereo_cam, side);
// 		for(unsigned i = 0; i < closures[side].Size(); i++)
// 			closures[side][i].Refine();
	}
// 
//   // do stereo matching and depth calculation
// 	closMatches = 0;
// 	MatchClosures(closures[LEFT], closures[RIGHT], closMatches);
// 	Calculate3DClosures(closures[LEFT], closures[RIGHT], closMatches, closure3ds);
printf("#### StereoEllipses::Process() end\n");
}


}








