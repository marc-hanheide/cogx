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
  printf("TmpCube::TmpCube: Surface init bad implemented!\n");
  surf[0].Init(cube, 0);
  surf[1].Init(cube, 1);
  surf[2].Init(cube, 2);
}

/**
 * @brief Recalculate all cube parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpCube::RePrune(int oX, int oY, int sc)
{
  surf[0].RePrune(oX, oY, sc);
  surf[1].RePrune(oX, oY, sc);
  surf[2].RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpCube
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpCube::Rectify(cast::StereoCamera *stereo_cam, int side)
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
 * @param off0 Shift points off first surface.
 * @param off0 Shift points off second surface.
 * @param off0 Shift points off third surface.
 * @param ass0 Assignment: Surface 0 is assigned to this surface => swap surfaces
 * @param ass1 Assignment: Surface 1 is assigned to this surface => swap surfaces
 * @param ass2 Assignment: Surface 2 is assigned to this surface => swap surfaces
 */
void TmpCube::Fuddle(unsigned off0, unsigned off1, unsigned off2, unsigned ass0, unsigned ass1, unsigned ass2)
{
// 	printf("TmpCube::Fuddle: fuddle the cube! Not correct implemented!\n");					/// TODO clean up
// 	printf("TmpCube::Fuddle: offset: %u - %u - %u\n", off0, off1, off2);
// 	printf("TmpCube::Fuddle: assignments: %u - %u - %u\n", ass0, ass1, ass2);
  surf[0].ShiftPointsLeft(off0);
  surf[1].ShiftPointsLeft(off1);
  surf[2].ShiftPointsLeft(off2);
	
  if(ass0 != 0 || ass1 != 1 || ass2 !=2)
  {
    Surf2D tmp_0 = surf[0];
    Surf2D tmp_1 = surf[1];
    Surf2D tmp_2 = surf[2];

		if(ass0 == 1) surf[0] = tmp_1;
		else if(ass0 == 2) surf[0] = tmp_2;
		if(ass1 == 0) surf[1] = tmp_0;
		else if(ass1 == 2) surf[1] = tmp_2;
		if(ass2 == 0) surf[2] = tmp_0;
		else if(ass2 == 1) surf[2] = tmp_1;
  }
}


//----------------------------------------------------------------//
//---------------------------- Cube3D ----------------------------//
//----------------------------------------------------------------//
/**
 * @brief Reconstruct the whole cube in 3D. Visible surfaces are given.
 * First calculate the missing hidden vertex point and then the three hidden surfaces.
 * @param stereo_cam Stereo camera parameters
 * @param left Left tmp. cube
 * @param right Right tmp. cube
 * @return Return true for success.
 * TODO Hier wird surf3D direkt angelegt => sollte eigentlich über Reconstruct() aus surf2D erstellt werden.
 */
bool Cube3D::Reconstruct(cast::StereoCamera *stereo_cam, TmpCube &left, TmpCube &right)
{
  bool ok0 = surf_vis[0].Reconstruct(stereo_cam, left.surf[0], right.surf[0], true);
  bool ok1 = surf_vis[1].Reconstruct(stereo_cam, left.surf[1], right.surf[1], true);
  bool ok2 = surf_vis[2].Reconstruct(stereo_cam, left.surf[2], right.surf[2], true);

	Vertex3D hidP[3];
	hidP[0].p = surf_vis[2].vertices[2].p + (surf_vis[0].vertices[2].p - surf_vis[0].vertices[1].p);

	surf_hid[0].vertices.PushBack(hidP[0]);
	surf_hid[0].vertices.PushBack(surf_vis[1].vertices[2]);
	surf_hid[0].vertices.PushBack(surf_vis[2].vertices[1]);
	surf_hid[0].vertices.PushBack(surf_vis[2].vertices[2]);

	surf_hid[1].vertices.PushBack(hidP[0]);
	surf_hid[1].vertices.PushBack(surf_vis[2].vertices[2]);
	surf_hid[1].vertices.PushBack(surf_vis[0].vertices[1]);
	surf_hid[1].vertices.PushBack(surf_vis[0].vertices[2]);

	surf_hid[2].vertices.PushBack(hidP[0]);
	surf_hid[2].vertices.PushBack(surf_vis[0].vertices[2]);
	surf_hid[2].vertices.PushBack(surf_vis[1].vertices[1]);
	surf_hid[2].vertices.PushBack(surf_vis[1].vertices[2]);

  return ok0 && ok1 && ok2;
}



//------------------------------------------------------------------//
//--------------------------- StereoCubes --------------------------//
//------------------------------------------------------------------//
/**
 * @brief Constructor of StereoCubes: Calculate stereo matching of cubes
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 * @param sc Stereo camera parameters
 */
StereoCubes::StereoCubes(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
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
 * @brief Draw only matched cubes as overlay.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoCubes::DrawMatched(int side, bool single, int id, int detail)
{
	if(single)
	{
		if(id < 0 || id >= cubeMatches)
		{
			printf("StereoClosures::DrawMatched: warning: id out of range!\n");
			return;
		}
		DrawSingleMatched(side, id, detail);
	}
	else
		for(int i=0; i< cubeMatches; i++)
			DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched cube.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoCubes::DrawSingleMatched(int side, int id, int detail)
{
	for(unsigned i=0; i<=2; i++)
	{	
		cubes[side][id].surf[i].Draw(detail);
		if(detail > 0)
		{
			char text[20];
			snprintf(text, 20, "%u", i);
			unsigned x = (cubes[side][id].surf[i].p[0].x + cubes[side][id].surf[i].p[1].x +
										cubes[side][id].surf[i].p[2].x + cubes[side][id].surf[i].p[3].x) / 4; 
			unsigned y = (cubes[side][id].surf[i].p[0].y + cubes[side][id].surf[i].p[1].y +
										cubes[side][id].surf[i].p[2].y + cubes[side][id].surf[i].p[3].y) / 4; 
			DrawText2D(text, x, y, RGBColor::red);
		}
	}
}

/**
 * @brief Convert cube from object detector to working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the object detector cube.
 * @return Return true for success.
 */
#ifdef HAVE_CAST
bool StereoCubes::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
	printf("StereoCubes::StereoGestalt2VisualObject: Not correct implemented: we need also the other side!\n");
	obj->model = new VisionData::GeometryModel;
	Cube3D cube = Cubes(id);

	// Recalculate pose of vertices (relative to the pose of the cube == COG)
	Pose3 pose;
	RecalculateCoordsystem(cube, pose);

	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = pose.pos.x;
	cogxPose.pos.y = pose.pos.y;
	cogxPose.pos.z = pose.pos.z;
	obj->pose = cogxPose;

	// create visible surfaces (we need it counter-clockwise for the TomGine)
	VisionData::Face f;
	for(unsigned i=0; i<=2; i++)
	{
		for(unsigned j=0; j<cube.surf_vis[i].vertices.Size(); j++)
		{
			VisionData::Vertex v;
			v.pos.x = cube.surf_vis[i].vertices[j].p.x;
			v.pos.y = cube.surf_vis[i].vertices[j].p.y;
			v.pos.z = cube.surf_vis[i].vertices[j].p.z;
			obj->model->vertices.push_back(v);

			f.vertices.push_back(/*j+*/(i*4)+3-j); // counter-clockwise
		}
		obj->model->faces.push_back(f);
		f.vertices.clear();
	}
	
	// create hidden surfaces (relative to the 3D center point)
	for(unsigned i=0; i<=2; i++)
	{
		for(unsigned j=0; j<cube.surf_hid[i].vertices.Size(); j++)
		{
			VisionData::Vertex v;
			v.pos.x = cube.surf_hid[i].vertices[j].p.x;
			v.pos.y = cube.surf_hid[i].vertices[j].p.y;
			v.pos.z = cube.surf_hid[i].vertices[j].p.z;
			obj->model->vertices.push_back(v);

			f.vertices.push_back(j+(i*4)+12);
		}
		obj->model->faces.push_back(f);
		f.vertices.clear();
	}

	obj->detectionConfidence = 1.0;						// TODO detection confidence is always 1

	// calculate the normal of all vertices and add it to the visual object
	/// => we calculate it in the TomGine!
// 	for(unsigned i=0; i<obj->model->vertices.size()/4; i++)
// 	{
// 		/// calculate the normal
// 		Vector3 v[4];
// 		for(unsigned j=0; j<4; j++)
// 		{
// 			v[j].x = obj->model->vertices[i*4+j].pos.x;
// 			v[j].y = obj->model->vertices[i*4+j].pos.y;
// 			v[j].z = obj->model->vertices[i*4+j].pos.z;
// 		}
// 		Vector3 normal = Cross(v[0]-v[1], v[2]-v[3]);
// 		if(normal.Normalise())
// 		{
// 			for(unsigned j=0; j<4; j++)
// 			{
// 				obj->model->vertices[i*4+j].normal.x = normal.x;
// 				obj->model->vertices[i*4+j].normal.y = normal.y;
// 				obj->model->vertices[i*4+j].normal.z = normal.z;
// 			}
// 		}
// 	}
	
	return true;
}
#endif

/**
 * TODO: 
 * Es wird der Schwerpunkt des Cubes als Zentrum des Cube-Koordinatensystem verwendet und die Pose zur Kamera errechnet.
 * @brief Try to find a "natural" looking coordinate system for a cube.
 * The coordinate system is really arbitrary, there is no proper implicitly defined coordinate system.
 * We take the (geometrical) center of gravity of the corner points as position and set orientation to identity.
 * @param cube 3D Cube
 * @param pose Estimated pose
 */
void StereoCubes::RecalculateCoordsystem(Cube3D &cube, Pose3 &pose)
{
  Vector3 c(0., 0., 0.);
  int cnt = 0;
  // find the center of gravity
  for(int i = 0; i <= 2; i++)
  {
    for(unsigned j = 0; j < cube.surf_vis[i].vertices.Size(); j++)
    {
      c += cube.surf_vis[i].vertices[j].p;
      cnt++;
    }
  }
  c /= (double)cnt;
  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;

	// set the orientation to identity, i.e. pStereoGestalt2VisualObjectarallel to world coordinate system
  pose.rot.x = 0.;
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

	// recalculate the vectors to the vertices from new center point
  for(int i = 0; i <= 2; i++)
  {
    for(unsigned j = 0; j < cube.surf_vis[i].vertices.Size(); j++)
    {
      Vector3 p(cube.surf_vis[i].vertices[j].p.x,
                cube.surf_vis[i].vertices[j].p.y,
                cube.surf_vis[i].vertices[j].p.z);
      cube.surf_vis[i].vertices[j].p = inv.Transform(p);
    }
  }
  for(int i = 0; i <= 2; i++)
  {
    for(unsigned j = 0; j < cube.surf_hid[i].vertices.Size(); j++)
    {
      Vector3 p(cube.surf_hid[i].vertices[j].p.x,
                cube.surf_hid[i].vertices[j].p.y,
                cube.surf_hid[i].vertices[j].p.z);
			cube.surf_hid[i].vertices[j].p = inv.Transform(p);
    }
	}
}

/**
 * @brief Calculate matching score for cubes.
 * @param left_cube Left tmp. cube with three surfaces
 * @param right_cube Right tmp. cube with three surfaces
 * @param off_0 Offset of first surface
 * @param off_1 Offset of second surface
 * @param off_2 Offset of third surface
 * @param assign1st Surface left[0] is assigned to this right surface
 * @param assign2nd Surface left[1] is assigned to this right surface
 * @param assign3rd Surface left[2] is assigned to this right surface
 * @return Returns the sum of the offsets.
 */
double StereoCubes::MatchingScore(TmpCube &left_cube, TmpCube &right_cube,
                                  unsigned &off_0, unsigned &off_1, unsigned &off_2,
                                  unsigned &assign_0, unsigned &assign_1, unsigned &assign_2)
{
  unsigned off_s0, off_s1, off_s2, off_x1, off_x2;
	double sc_s = HUGE;
	double sc_x = HUGE;

	// Get the best matching surface for first surface of left cube.
  double sc_0 = MatchingScoreSurf(left_cube.surf[0], right_cube.surf[0], off_s0);
  double sc_1 = MatchingScoreSurf(left_cube.surf[0], right_cube.surf[1], off_s1);
  double sc_2 = MatchingScoreSurf(left_cube.surf[0], right_cube.surf[2], off_s2);

	// which one is the best one?
	if(sc_0 < sc_1 && sc_0 < sc_2)						// 0-0 is the best matching surface
	{
// 		printf("StereoCubes::MatchingScore: 0-0 is best\n");
		off_0 = off_s0;
		assign_0 = 0;
		sc_s = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[1], off_s1) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[2], off_s2);
		sc_x = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[2], off_x2) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[1], off_x1);
		if(sc_s < sc_x)
		{
			assign_1 = 1;
			assign_2 = 2;
		} else 
		{
			assign_1 = 2;
			assign_2 = 1;
		}	
	}
	else if(sc_1 < sc_0 && sc_1 < sc_2)				// 0-1 is the best matching surface
	{
// 		printf("StereoCubes::MatchingScore: 0-1 is best\n");
		off_0 = off_s1;
		assign_0 = 1;
		sc_s = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[0], off_s1) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[2], off_s2);
		sc_x = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[2], off_x2) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[0], off_x1);
		if(sc_s < sc_x)
		{
			assign_1 = 0;
			assign_2 = 2;
		} else 
		{
			assign_1 = 2;
			assign_2 = 0;
		}	
	}
	else if(sc_2 < sc_0 && sc_2 < sc_1)				// 0-2 is the best matching surface
	{
// 		printf("StereoCubes::MatchingScore: 0-2 is best\n");
		off_0 = off_s2;
		assign_0 = 2;
		sc_s = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[0], off_s1) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[1], off_s2);
		sc_x = MatchingScoreSurf(left_cube.surf[1], right_cube.surf[1], off_x2) +
           MatchingScoreSurf(left_cube.surf[2], right_cube.surf[0], off_x1);
		if(sc_s < sc_x)
		{
			assign_1 = 0;
			assign_2 = 1;
		} else 
		{
			assign_1 = 1; off_1 = off_x2;
			assign_2 = 0; off_2 = off_x1;
		}	
	}
	else return HUGE;

  if(sc_s < sc_x)
  {
    off_1 = off_s1;
    off_2 = off_s2;
    return sc_s + off_0;
  }
  else
  {
    off_1 = off_x1;
    off_2 = off_x2;
    return sc_x + off_0;
  }
}

/**
 * @brief Find best matching right cube for given left cube, \n
 * begining at position l of right cube array.
 * @param left_cube Tmp. flap of left stereo image.
 * @param right_cubes Array of all flaps from right stereo image.
 * @param l Begin at position l of right flap array
 * @return Returns position of best matching right cube from the right cube array.
 */
unsigned StereoCubes::FindMatchingCube(TmpCube &left_cube, Array<TmpCube> &right_cubes, unsigned l)
{
// printf("    FindMatchingFlap:\n");
  double match, best_match = HUGE;
  unsigned j, j_best = UNDEF_ID;
  unsigned off_0, off_1, off_2, off_0_best = 0, off_1_best = 0, off_2_best = 0;
	unsigned ass_0, ass_1, ass_2;
  for(j = l; j < right_cubes.Size(); j++)
  {
    match = MatchingScore(left_cube, right_cubes[j], off_0, off_1, off_2, ass_0, ass_1, ass_2);
    if(match < best_match)
    {
      best_match = match;
      j_best = j;
      off_0_best = off_0;
      off_1_best = off_1;
      off_2_best = off_2;
    }
  }
  if(j_best != UNDEF_ID)
  {
    right_cubes[j_best].Fuddle(off_0_best, off_1_best, off_2_best, ass_0, ass_1, ass_2);
  }
  return j_best;
	return 0;
}


/**
 * @brief Match left and right cubes from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_cubes Array of all cubes from left stereo image (matching cubes get sorted to the beginning of the array.)
 * @param right_cubes Array of all cubes from right stereo image.
 * @param matches Number of matched cubes (sorted to the beginning of the arrays).
 */
void StereoCubes::MatchCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &matches)
{
  unsigned j, l = 0, u = left_cubes.Size();
  for(; l < u && l < right_cubes.Size();)
  {
    j = FindMatchingCube(left_cubes[l], right_cubes, l);
    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_cubes.Swap(l, j);
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_cubes.Swap(l, u-1);
      u--;
    }
  }
  u = min(u, right_cubes.Size());
  matches = u;
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
printf("StereoCubes::Calculate3DCubes: matches: %u\n", matches);
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Cube3D cube3d;
    if(cube3d.Reconstruct(stereo_cam, left_cubes[i], right_cubes[i]))
    {
      cube3ds.PushBack(cube3d);
      i++;
    }
    else		// move unacceptable cubes to the end
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
		if(pPara.pruning)
			for(unsigned i = 0; i < cubes[side].Size(); i++)
				cubes[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
		for(unsigned i = 0; i < cubes[side].Size(); i++)
			cubes[side][i].Rectify(stereo_cam, side);
		for(unsigned i = 0; i < cubes[side].Size(); i++)
			cubes[side][i].Refine();
	}

  // do stereo matching and depth calculation
  cubeMatches = 0;
  MatchCubes(cubes[LEFT], cubes[RIGHT], cubeMatches);
  Calculate3DCubes(cubes[LEFT], cubes[RIGHT], cubeMatches, cube3ds);
}


/**
 * @brief Match and calculate 3D flaps from 2D flaps.
 */
void StereoCubes::Process(int oX, int oY, int sc)
{
	pPara.pruning = true;
	pPara.offsetX = oX;
	pPara.offsetY = oY;
	pPara.scale = sc;
	Process();
	pPara.pruning = false;
}

}








