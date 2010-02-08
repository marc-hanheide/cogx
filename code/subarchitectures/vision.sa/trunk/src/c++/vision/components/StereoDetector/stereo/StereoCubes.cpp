/**
 * @file StereoCubes.cpp
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of cubes.
 */

#include "StereoCubes.h"

namespace Z
{

//-----------------------------------------------------------------//
//---------------------------- TmpCube ----------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpCube
 * @param cube VS3 Cube
 */
TmpCube::TmpCube(Cube *cube)
{
	surf[0].Init(cube, 0);
	surf[1].Init(cube, 1);
	surf[2].Init(cube, 2);
}

/**
 * @brief Rectify TmpCube
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpCube::Rectify(StereoCamera *stereo_cam, int side)
{
  surf[0].Rectify(stereo_cam, side);
  surf[1].Rectify(stereo_cam, side);
  surf[2].Rectify(stereo_cam, side);
}

/**
 * @brief Refine TmpFlap
 */
void TmpCube::Refine()
{
  surf[0].Refine();
  surf[1].Refine();
  surf[2].Refine();
}

/**
 * @brief Returns true, if cube is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if cube is at this position.
 */
bool TmpCube::IsAtPosition(int x, int y) const
{
  return surf[0].IsAtPosition(x, y) || surf[1].IsAtPosition(x, y) || surf[2].IsAtPosition(x, y);
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
void TmpCube::Fuddle(unsigned off0, unsigned off1, bool swap)
{
	printf("Fuddle the cube! Not yet implemented!\n");
//   surf[0].ShiftPointsLeft(off0);
//   surf[1].ShiftPointsLeft(off1);
//   if(swap)
//   {
//     TmpSurf t = surf[1];
//     surf[1] = surf[0];
//     surf[0] = t;
//   }
}


//----------------------------------------------------------------//
//---------------------------- Cube3D ----------------------------//
//----------------------------------------------------------------//

/**
 * @brief Reconstruct the cube in 3D
 * @param left
 * @param right
 * @param cam Stereo camera parameters and functions.
 * @return Return true for success.
 */
bool Cube3D::Reconstruct(StereoCamera *stereo_cam, TmpCube &left, TmpCube &right)
{
	// TODO Wie funktioniert das mit Reconstruct genau! => da passiert ein Fehler!
	printf("Cube3D::Reconstruct: Not yet implemented????\n");
//   bool ok0 = surf[0].Reconstruct(stereo_cam, left.surf[0], right.surf[0], true);
//   bool ok1 = surf[1].Reconstruct(stereo_cam, left.surf[1], right.surf[1], true);

//   bool ok2 = surf[2].Reconstruct(stereo_cam, left.surf[2], right.surf[2f], true);
//   return ok0 && ok1 && ok2;
}



//------------------------------------------------------------------//
//--------------------------- StereoCubes --------------------------//
//------------------------------------------------------------------//
/**
 * @brief Constructor of StereoCubes: Calculate stereo matching of cubes
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 * @param sc Stereo camera parameters
 */
StereoCubes::StereoCubes(VisionCore *vc[2], StereoCamera *sc) : StereoBase()
{
	vcore[LEFT] = vc[LEFT];
	vcore[RIGHT] = vc[RIGHT];
	stereo_cam = sc;
  cubeMatches = 0;
}


/**
 * @brief Number of cubes in 2D 
 * @param side LEFT/RIGHT side of stereo rig.
 */
int StereoCubes::NumCubes2D(int side)
{
  assert(side == LEFT || side == RIGHT);
  return cubes[side].Size();
}

/**
 * @brief Delivers 2D tmp. cubes
 * @param side LEFT/RIGHT side of stereo rig.
 * @param i Position of the flap in the array
 */
const TmpCube &StereoCubes::Cubes2D(int side, int i)
{
  assert(side == LEFT || side == RIGHT);
  return cubes[side][i];
}

/**
 * @brief Draw detected cubes as overlay.
 * @param side Left or right side of the stereo images.
 */
void StereoCubes::Draw(int side)
{
	SetColor(RGBColor::blue);
	int nrCubes = 0;
	if(side == LEFT) nrCubes = NumCubesLeft2D();
	else nrCubes = NumCubesRight2D();

	for(int i=0; i<nrCubes; i++)
	{
		if (vcore[side]->Gestalts(Gestalt::CUBE, i)->IsUnmasked())
				vcore[side]->Gestalts(Gestalt::CUBE, i)->Draw();	
		else
printf("StereoCubes::Draw: Cube is masked => no draw!\n");						/// TODO wieder weg
	}
}

/**
 * @brief Draw only matched cubes as overlay.
 * @param side Left or right side of the stereo images.
 */
void StereoCubes::DrawMatched(int side)
{
printf("StereoCubes::DrawMatched: %u\n", cubeMatches);								/// TODO wieder weg
	for(int i=0; i< cubeMatches; i++)
	{
		cubes[side][i].surf[0].Draw(RGBColor::red);
		cubes[side][i].surf[1].Draw(RGBColor::red);
		cubes[side][i].surf[2].Draw(RGBColor::red);
	}
}

/**
 * @brief Convert cube from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector cube.
 * @return Return true for success.
 */
bool StereoCubes::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
// 	obj->model = new VisionData::GeometryModel;
// 	Flap3D flap = Flaps(id);
// 
// 	// Recalculate pose of vertices (relative to the pose of the flap == COG)
// 	Pose3 pose;
// 	RecalculateCoordsystem(flap, pose);
// 
// 	// add center point to the model
// 	cogx::Math::Pose3 cogxPose;
// 	cogxPose.pos.x = pose.pos.x;
// 	cogxPose.pos.y = pose.pos.y;
// 	cogxPose.pos.z = pose.pos.z;
// 	obj->pose = cogxPose;
// 
// 	// create vertices (relative to the 3D center point)
// 	for(unsigned i=0; i<=1; i++)	// LEFT/RIGHT rectangle of flap
// 	{
// 		VisionData::Face f;
// 
// 		for(unsigned j=0; j<flap.surf[i].vertices.Size(); j++)
// 		{
// 			VisionData::Vertex v;
// 			v.pos.x = flap.surf[i].vertices[j].p.x;
// 			v.pos.y = flap.surf[i].vertices[j].p.y;
// 			v.pos.z = flap.surf[i].vertices[j].p.z;
// 			obj->model->vertices.push_back(v);
// 
// 			f.vertices.push_back(j+(i*4));
// 		}
// 
// 		obj->model->faces.push_back(f);
// 		f.vertices.clear();
// 	}
// 
// 	obj->detectionConfidence = 1.0;						// TODO detection confidence is always 1

	return true;
}

/**
 * TODO: 
 * Es wird der Schwerpunkt des Flaps als Zentrum des Flap-Koordinatensystem verwendet und die Pose zur Kamera errechnet.
 * @brief Try to find a "natural" looking coordinate system for a flap.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity of the corner points as position and set orientation to identity.
 * @param flap 3D Flap
 * @param pose pose
 */
void StereoCubes::RecalculateCoordsystem(Cube3D &cube, Pose3 &pose)
{
//   Vector3 c(0., 0., 0.);
//   int cnt = 0;
//   // find the center of gravity
//   for(int i = 0; i <= 1; i++)
//   {
//     for(unsigned j = 0; j < flap.surf[i].vertices.Size(); j++)
//     {
//       c += flap.surf[i].vertices[j].p;
//       cnt++;
//     }
//   }
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
//   for(int i = 0; i <= 1; i++)
//   {
//     for(unsigned j = 0; j < flap.surf[i].vertices.Size(); j++)
//     {
//       Vector3 p(flap.surf[i].vertices[j].p.x,
//                 flap.surf[i].vertices[j].p.y,
//                 flap.surf[i].vertices[j].p.z);
// 			flap.surf[i].vertices[j].p = inv.Transform(p);
//     }
// 	}
}


/**
 * @brief Calculate matching score for flaps
 * @param left_flap Left tmp. flap
 * @param right_flap Right tmp. flap
 * @param off_0 TODO Offset ???
 * @param off_1 TODO Offset ???
 * @param cross true, if match is "crossed" and not "straight"
 */
double StereoCubes::MatchingScore(TmpCube &left_cube, TmpCube &right_cube, unsigned &off_0, unsigned &off_1, bool &cross)
{
//   // _s .. straight (left flap surf 0 matches right flap surf 0)
//   // _x .. crossed (left flap surf 0 matches right flap surf 1)
//   unsigned off_s0, off_s1, off_x0, off_x1;
//   double sc_s = MatchingScoreSurf(left_flap.surf[0], right_flap.surf[0], off_s0) +
//                 MatchingScoreSurf(left_flap.surf[1], right_flap.surf[1], off_s1);
//   double sc_x = MatchingScoreSurf(left_flap.surf[0], right_flap.surf[1], off_x1) +
//                 MatchingScoreSurf(left_flap.surf[1], right_flap.surf[0], off_x0);
// 
// // printf("	Matching score: %4.2f - %4.2f\n", sc_s, sc_x);
//   // if flaps match "straight"
//   if(sc_s < sc_x)
//   {
//     cross = false;
//     off_0 = off_s0;
//     off_1 = off_s1;
//     return sc_s;
//   }
//   // else if flaps match "crossed"
//   else
//   {
//     cross = true;
//     off_0 = off_x0;
//     off_1 = off_x1;
//     return sc_x;
//   }
}

/**																																			/// TODO StereoFlaps verschieben
 * @brief Find right best matching flap for given left flaps, begining at position l of right flap array.
 * @param left_flap Tmp. flap of left stereo image.
 * @param right_flaps Array of all flaps from right stereo image.
 * @param l Begin at position l of right flap array
 * @return Returns position of best matching right flap from the right flap array.
 */
unsigned StereoCubes::FindMatchingCube(TmpCube &left_cube, Array<TmpCube> &right_cubes, unsigned l)
{
// // printf("    FindMatchingFlap:\n");
//   double match, best_match = HUGE;
//   unsigned j, j_best = UNDEF_ID;
//   unsigned off_0, off_1, off_0_best = 0, off_1_best = 0;
//   bool cross = false, cross_best = false;
//   for(j = l; j < right_flaps.Size(); j++)
//   {
//     match = MatchingScore(left_flap, right_flaps[j], off_0, off_1, cross);
//     if(match < best_match)
//     {
//       best_match = match;
//       j_best = j;
//       cross_best = cross;
//       off_0_best = off_0;
//       off_1_best = off_1;
//     }
//   }
//   if(j_best != UNDEF_ID)
//   {
//     right_flaps[j_best].Fuddle(off_0_best, off_1_best, cross_best);
//   }
//   return j_best;
}


/**
 * @brief Match left and right cubes from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_cubes Array of all cubes from left stereo image (matching cubes get sorted to the beginning of the array.)
 * @param right_cubes Array of all cubes from right stereo image.
 * @param matches Number of matched cubes (sorted to the beginning of the arrays).
 */
void StereoCubes::MatchCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &matches)
{
//   unsigned j, l = 0, u = left_flaps.Size();
//   for(; l < u && l < right_flaps.Size();)
//   {
//     j = FindMatchingFlap(left_flaps[l], right_flaps, l);
//     // found a matching right, move it to same index position as left
//     if(j != UNDEF_ID)
//     {
//       right_flaps.Swap(l, j);
//       l++;
//     }
//     // found no right, move left to end and decrease end
//     else
//     {
//       left_flaps.Swap(l, u-1);
//       u--;
//     }
//   }
//   u = min(u, right_flaps.Size());
//   matches = u;
}


/**
 * @brief Calculate 3D cubes from matched cubes.
 * @param left_cubes Array of all cubes from left stereo image.
 * @param right_cubes Array of all cubes from right stereo image.
 * @param matches Number of matched cubes.
 * @param cube3ds Array of calculated 3d cubes.
 */
void StereoCubes::Calculate3DCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &matches, Array<Cube3D> &cube3ds)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Cube3D cube3d;
    bool ok0 = cube3d.surf[0].Reconstruct(stereo_cam, left_cubes[i].surf[0], right_cubes[i].surf[0], true);
    bool ok1 = cube3d.surf[1].Reconstruct(stereo_cam, left_cubes[i].surf[1], right_cubes[i].surf[1], true);
    bool ok2 = cube3d.surf[2].Reconstruct(stereo_cam, left_cubes[i].surf[2], right_cubes[i].surf[2], true);
    if(ok0 && ok1 && ok2)
    {
      cube3ds.PushBack(cube3d);
      i++;
    }
    else		// move unacceptable flaps to the end
    {
      left_cubes.Swap(i, u-1);
      right_cubes.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}

/**
 * @brief Delete all arrays ...
 */
void StereoCubes::ClearResults()
{
	cubes[LEFT].Clear();
	cubes[RIGHT].Clear();
	cube3ds.Clear();

	cubeMatches = 0;
}

/**
 * @brief Match and calculate 3D flaps from 2D flaps.
 */
void StereoCubes::Process()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
		// note: the awkward Gestalt::FLAP thingy is necessary because the global		TODO ARI: Why?
		// NumFlaps() and Flaps() collide with StereoCores respective methods.
		for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::CUBE); i++)
		{
			Cube *core_cube = (Cube*)vcore[side]->Gestalts(Gestalt::CUBE, i);
			if(!vcore[side]->use_masking || !core_cube->IsMasked())
			{
				TmpCube cube(core_cube);
				if(cube.IsValid())
					cubes[side].PushBack(cube);
			}
		}
		for(unsigned i = 0; i < cubes[side].Size(); i++)
			cubes[side][i].Rectify(stereo_cam, side);
		for(unsigned i = 0; i < cubes[side].Size(); i++)
			cubes[side][i].Refine();
	}

  // do stereo matching and depth calculation
  cubeMatches = 0;
  MatchCubes(cubes[LEFT], cubes[RIGHT], cubeMatches);
//   Calculate3DCubes(cubes[LEFT], cubes[RIGHT], cubeMatches, cube3ds);
}


}








