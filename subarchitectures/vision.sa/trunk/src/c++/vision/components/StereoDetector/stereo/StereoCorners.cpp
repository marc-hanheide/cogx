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
  obj->model = new VisionData::GeometryModel;
  Corner3D *corner = Corners3D(score, id);

  // Recalculate pose of vertices (relative to the pose of the flap == cog)
  Pose3 pose;
  Vector3 c(0., 0., 0.);
  c = corner->isct3D.p;

  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;
  pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

  // recalculate the vectors to the vertices from new center point
  Vector3 p(corner->isct3D.p.x, corner->isct3D.p.y, corner->isct3D.p.z);
  p = inv.Transform(p);
  
  // add center point to the model
  cogx::Math::Pose3 cogxPose;
  cogxPose.pos.x = pose.pos.x;
  cogxPose.pos.y = pose.pos.y;
  cogxPose.pos.z = pose.pos.z;
  obj->pose = cogxPose;

  // create vertices (relative to the 3D center point)
  // TODO Ich erzeuge hier eine Fläche aus 3 Punkten! (x-y-Ebene)
  VisionData::Vertex v;
  v.pos.x = p.x + 0.002;
  v.pos.y = p.y;
  v.pos.z = p.z;
  obj->model->vertices.push_back(v);
  VisionData::Vertex w;
  w.pos.x = p.x;
  w.pos.y = p.y + 0.002;
  w.pos.z = p.z;
  obj->model->vertices.push_back(w);
  VisionData::Vertex x;
  x.pos.x = p.x;
  x.pos.y = p.y;
  x.pos.z = p.z;
  obj->model->vertices.push_back(x);

  // add faces to the vision model
  VisionData::Face f;
  f.vertices.push_back(0);
  f.vertices.push_back(1);
  f.vertices.push_back(2);
  obj->model->faces.push_back(f);
  f.vertices.clear();

  obj->detectionConfidence = 1.0;					// TODO detection confidence is always 1

  return true;
}
#endif


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

    unsigned k=0,l=0,m=0,n=0,o=0,p=0;
    double angleSum = CalculateBestArmMatches(left_corner, right_corners[j], k, l, m, n, o, p);
    
    if(match < HUGE && SC_USE_CORNER_THRESHOLDS)					/// TODO TODO
    {
printf(" %u match = %6.5f\n", j, match);
// printf(" StereoCorners::FindMatchingCorner: %u/%u/%u bzw. %u/%u/%u\n", k, m, o, l, n, p);
// printf("  angles: %4.2f - %4.2f\n", PolarAngle(left_corner.armDir), PolarAngle(right_corners[j].dir[1]));

      // The angle difference is between 0 and 3*2*PI = 18,85 rad
      match = match * angleSum;
printf(" %u new match = %6.5f\n\n", j, match);

// printf("    min Angle: %4.3f\n", minAngle);
// printf(" new match = %6.5f\n", match);
    }

    if(match < best_match)
    {
      left_corner.armMatch[0] = k;
      right_corners[j].armMatch[0] = l;
      left_corner.armMatch[1] = m;
      right_corners[j].armMatch[1] = n;
      left_corner.armMatch[2] = o;
      right_corners[j].armMatch[2] = p;
      best_match = match;
      j_best = j;
    }
  }
  
  if(best_match > SC_CORNER_MATCH_LIMIT && SC_USE_CORNER_THRESHOLDS)
    return UNDEF_ID;
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
  unsigned j, l = 0, u = left_corners.Size();
  for(; l < u && l < right_corners.Size();)
  {
    j = FindMatchingCorner(left_corners[l], right_corners, l);

    // found a matching right, move it to same index position as left
    if(j != UNDEF_ID)
    {
      right_corners.Swap(l, j);           // change found right_ljcts[j] at same position than left_ljcts ==> l
      l++;
    }
    // found no right, move left to end and decrease end
    else
    {
      left_corners.Swap(l, u-1);          // change found left_ljcts[l] to last position
      u--;
    }
  }
  u = std::min(u, right_corners.Size());
  matches = u;
}


/**
 * @brief Calculate the best matching corner arms
 * @param corner 3D corner
 * @param left_corner Corner from the left image
 * @param right_corner Corner from the right image
 * @param k Best match to right corner with number k
 * @param l
 * @param m
 * @param n
 * @param o
 * @param p
 * @return Returns the sum of the deviation of the three angles
 */
double StereoCorners::CalculateBestArmMatches(TmpCorner &left_corner, TmpCorner &right_corner,
					    unsigned &k, unsigned &l, unsigned &m, unsigned &n, unsigned &o, unsigned &p)
{
  double angleSum = 0.;

  // Match corners in 2D
  double angle[3][3];
  for(unsigned i=0; i<3; i++)
    for(unsigned j=0; j<3; j++)
      angle[i][j] = OpeningAngle(left_corner.armDir[i], right_corner.armDir[j]);

  // find smallest angle
  double ref = HUGE;
  k=0; l=0;
  for(unsigned i=0; i<3; i++)
  {
    for(unsigned j=0; j<3; j++)
    {
      if(angle[i][j] < ref)
      {
	ref = angle[i][j];
	k = i; l = j;
      }
    }
  }
  angleSum = ref;
  left_corner.armMatchValue[0] = ref;
  
// printf(" StereoCorners::CalculateBestArmMatches: %u bzw. %u\n", k, l);

  ref = HUGE;
  m=0; n=0;
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
  angleSum += ref;
  left_corner.armMatchValue[1] = ref;

// printf(" StereoCorners::CalculateBestArmMatches: %u/%u bzw. %u/%u\n", k, m, l, n);

  ref = HUGE;
  o=0; p=0;
  for(unsigned i=0; i<3; i++)
  {
    for(unsigned j=0; j<3; j++)
    {
      if(i != k && j != l && i != m && j != n)
      {
	ref = angle[i][j];
	o = i; p = j;	
      }
    }
  }
  angleSum += ref;
  left_corner.armMatchValue[2] = ref;

// printf(" StereoCorners::CalculateBestArmMatches: %u/%u/%u bzw. %u/%u/%u\n", k, m, o, l, n, p);
  
  return angleSum;
}

/**	TODO Besser und vor allem genauer formulieren!
 *	TODO Das is eigentlich formal nicht richtig, denn die Punkte (10px vom intersection point) müssen ja nicht matchen!!!
 *      TODO Wie kann man Winkel in 2D auf Winkel in 3D berechnen (jeweils aus dem linken und rechten Bild)
 * @brief Calculate 3D points of the corner arms and calculate the direction. 
 * @param corner 3D corner
 * @param left_corner Corner from the left image
 * @param right_corner Corner from the right image
 */
void StereoCorners::Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner)
{
  // 3D Triangulieren der abgeschätzten ArmPunkte
  corner->armPoints3D[0].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[0]], right_corner.armPoint[right_corner.armMatch[0]]);
  corner->armPoints3D[1].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[1]], right_corner.armPoint[right_corner.armMatch[1]]);
  corner->armPoints3D[2].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[2]], right_corner.armPoint[right_corner.armMatch[2]]);

  // Calculate the direction in 3D
  corner->armDir3D[0] = Normalise(corner->armPoints3D[0].p - corner->isct3D.p);
  corner->armDir3D[1] = Normalise(corner->armPoints3D[1].p - corner->isct3D.p);
  corner->armDir3D[2] = Normalise(corner->armPoints3D[2].p - corner->isct3D.p);
}


/**
 * @brief Calculate 3D corners from matched 2D Gestalts.
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
    else    // move unacceptable points to the end
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








