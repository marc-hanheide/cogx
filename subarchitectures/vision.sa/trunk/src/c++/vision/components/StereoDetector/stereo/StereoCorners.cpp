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
  id = corner->ID();
  
  isct2D.p.x = corner->isct.x;
  isct2D.p.y = corner->isct.y;
  
  if(corner->lines.Size() == 3)
  {
    armDir[0] = corner->lines[0]->dir;
    armDir[1] = corner->lines[1]->dir;
    armDir[2] = corner->lines[2]->dir;
    
    armPoint[0].p = isct2D.p + 10*armDir[0];			/// TODO Theoretische Punkte 10 px entfernt von der Intersection!
    armPoint[1].p = isct2D.p + 10*armDir[1];
    armPoint[2].p = isct2D.p + 10*armDir[2];

    isValid = true;
  }
  else
  {
    isValid = false;
    printf("TmpCorner::TmpCorner: Warning: More or less than 3 corner arms: %u arms\n", corner->lines.Size());
  }
}

/**
 * @brief Recalculate all rectangle parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpCorner::RePrune(int oX, int oY, int sc)
{
  isct2D.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpCorner
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpCorner::Rectify(StereoCamera *stereo_cam, int side)
{
  isct2D.Rectify(stereo_cam, side);
 
//   printf("TmpCorner::Rectify: %u before rectify: armDir[0]: %4.2f / %4.2f\n", id, armDir[0].x, armDir[0].y);
//   printf("TmpCorner::Rectify: %u before rectify: armDir[1]: %4.2f / %4.2f\n", id, armDir[1].x, armDir[1].y);
//   printf("TmpCorner::Rectify: %u before rectify: armDir[2]: %4.2f / %4.2f\n", id, armDir[2].x, armDir[2].y);
  
  armPoint[0].Rectify(stereo_cam, side);
  armPoint[1].Rectify(stereo_cam, side);
  armPoint[2].Rectify(stereo_cam, side);
  
  armDir[0] = Normalise(armPoint[0].p - isct2D.p);
  armDir[1] = Normalise(armPoint[1].p - isct2D.p);
  armDir[2] = Normalise(armPoint[2].p - isct2D.p); 
  
  // Recalculate arm points (10px away from center) after rectification
  armPoint[0].p = isct2D.p + 10 * armDir[0];
  armPoint[1].p = isct2D.p + 10 * armDir[1];
  armPoint[2].p = isct2D.p + 10 * armDir[2];
  
//   printf("TmpCorner::Rectify: %u => armPoint[0]-isct2D: %4.2f / %4.2f\n", id, armPoint[0].p.x - isct2D.p.x, armPoint[0].p.y - isct2D.p.y);
//   printf("TmpCorner::Rectify: %u => armPoint[1]-isct2D: %4.2f / %4.2f\n", id, armPoint[1].p.x - isct2D.p.x, armPoint[1].p.y - isct2D.p.y);
//   printf("TmpCorner::Rectify: %u => armPoint[2]-isct2D: %4.2f / %4.2f\n", id, armPoint[2].p.x - isct2D.p.x, armPoint[2].p.y - isct2D.p.y);

//   printf("TmpCorner::Rectify: %u after rectify: armDir[0]: %4.2f / %4.2f\n", id, armDir[0].x, armDir[0].y);
//   printf("TmpCorner::Rectify: %u after rectify: armDir[1]: %4.2f / %4.2f\n", id, armDir[1].x, armDir[1].y);
//   printf("TmpCorner::Rectify: %u after rectify: armDir[2]: %4.2f / %4.2f\n\n", id, armDir[2].x, armDir[2].y);
}


/**
 * @brief Refine TmpCorner
 */
void TmpCorner::Refine()
{
  isct2D.Refine();
}

/**
 * @brief Returns true, if corner junction is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if corner junction is at this position.
 */
bool TmpCorner::IsAtPosition(int x, int y) const
{
  return isct2D.IsAtPosition(x, y);
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
  corners[side][id].isct2D.Draw();
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
    match = MatchingScorePoint(left_corner.isct2D, right_corners[j].isct2D);

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

/**	TODO Besser und vor allem genauer formulieren!
 * @brief Calculate 3D points of the corner arms and calculate later the direction. 
 * @param left_corners Array of all corners from left stereo image.
 */
void StereoCorners::Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner)
{
//   printf("\nStereoCorners::Calculate3DCornerArms: armDir: %4.2f / %4.2f\n", left_corner.armDir[i], left_corner.armDir[i]);
//   printf("\nStart Calculate3DCornerArms\n");
  
  /// rausfinden, welche Arme zusammen gehören => In 2D!
  double angle[3][3];
  for(unsigned i=0; i<3; i++)
    for(unsigned j=0; j<3; j++)
//     {
      angle[i][j] = OpeningAngle(left_corner.armDir[i], right_corner.armDir[j]);
// printf("StereoCorners::Calculate3DCornerArms:  left armDir[%u]: %4.2f / %4.2f\n", i, left_corner.armDir[i].x, left_corner.armDir[i].y);
// printf("StereoCorners::Calculate3DCornerArms: right armDir[%u]: %4.2f / %4.2f\n", j, right_corner.armDir[j].x, right_corner.armDir[j].y);
//     }
  // kleinster Winkel
  double ref = HUGE;
  unsigned k=0, l=0;
  for(unsigned i=0; i<3; i++)
  {
    for(unsigned j=0; j<3; j++)
    {
// printf("StereoCorners::Calculate3DCornerArms: Winkel[%u][%u]: %4.3f!\n", i, j, angle[i][j]);
      if(angle[i][j] < ref)
      {
	ref = angle[i][j];
	k = i; l = j;
      }
    }
  }
  
// printf("StereoCorners::Calculate3DCornerArms: Kleinster Winkel: %4.3f von [%u][%u]!\n", ref, k, l);

  ref = HUGE;
  unsigned m=0, n=0;
  for(unsigned i=0; i<3; i++)
  {
    for(unsigned j=0; j<3; j++)
    {
      if(i != k && j != l)
      {
	if(angle[i][j] < ref)
	{
	  ref = angle[i][j];
	  m = i; n = j;
	}
      }
    }
  }

//  printf("StereoCorners::Calculate3DCornerArms: 2. Kleinst Winkel: %4.3f von [%u][%u]!\n", ref, m, n);

  ref = HUGE;
  unsigned o=0, p=0;
  for(unsigned i=0; i<3; i++)
  {
    for(unsigned j=0; j<3; j++)
    {
      if(i != k && j != l && i != m & j != n)
      {
	ref = angle[i][j];
	o = i; 
	p = j;
	
      }
    }
  }
  
// printf("StereoCorners::Calculate3DCornerArms: 3. Kleinst Winkel: %4.3f von [%u][%u]!\n", ref, o, p);

//   double sum = angle[k][l] + angle[m][n] + angle[o][p]; 
// printf("StereoCorners::Calculate3DCornerArms: Summe: %4.3f\n", sum);

  

  /// 3D Triangulieren der zusammengehörenden ArmPunkte
  corner->armPoints3D[0].Reconstruct(stereo_cam, left_corner.armPoint[k], right_corner.armPoint[l]);
  corner->armPoints3D[1].Reconstruct(stereo_cam, left_corner.armPoint[m], right_corner.armPoint[n]);
  corner->armPoints3D[2].Reconstruct(stereo_cam, left_corner.armPoint[o], right_corner.armPoint[p]);
//   corner->dir[0]; 

// printf("StereoCorners::Calculate3DCornerArms: isct3D: %4.2f / %4.2f / %4.2f\n", corner->isct3D.p.x, corner->isct3D.p.y, corner->isct3D.p.z);

// printf("StereoCorners::Calculate3DCornerArms: armPoints[0]: %4.2f / %4.2f / %4.2f\n", corner->armPoints3D[0].p.x, corner->armPoints3D[0].p.y, corner->armPoints3D[0].p.z);
// printf("StereoCorners::Calculate3DCornerArms: armPoints[1]: %4.2f / %4.2f / %4.2f\n", corner->armPoints3D[1].p.x, corner->armPoints3D[1].p.y, corner->armPoints3D[1].p.z);
// printf("StereoCorners::Calculate3DCornerArms: armPoints[2]: %4.2f / %4.2f / %4.2f\n", corner->armPoints3D[2].p.x, corner->armPoints3D[2].p.y, corner->armPoints3D[2].p.z);

  /// Berechnen der direction in 3D
  corner->armDir3D[0] = Normalise(corner->armPoints3D[0].p - corner->isct3D.p);
  corner->armDir3D[1] = Normalise(corner->armPoints3D[1].p - corner->isct3D.p);
  corner->armDir3D[2] = Normalise(corner->armPoints3D[2].p - corner->isct3D.p);

// printf("StereoCorners::Calculate3DCornerArms: armDir3D[0]: %4.2f / %4.2f / %4.2f\n", corner->armDir3D[0].x, corner->armDir3D[0].y, corner->armDir3D[0].z);
// printf("StereoCorners::Calculate3DCornerArms: armDir3D[1]: %4.2f / %4.2f / %4.2f\n", corner->armDir3D[1].x, corner->armDir3D[1].y, corner->armDir3D[1].z);
// printf("StereoCorners::Calculate3DCornerArms: armDir3D[2]: %4.2f / %4.2f / %4.2f\n", corner->armDir3D[2].x, corner->armDir3D[2].y, corner->armDir3D[2].z);
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
    if (corner3d->isct3D.Reconstruct(stereo_cam, left_corners[i].isct2D, right_corners[i].isct2D))
    {
      // calculate direction of arms if we have 3 arms per corner
      if(left_corners[i].IsValid() && right_corners[i].IsValid())
      {
	Calculate3DCornerArms(corner3d, left_corners[i], right_corners[i]);
        score->NewGestalt3D(corner3d);
        i++;
      }
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








