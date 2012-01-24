/**
 * @file StereoCorners.h
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Stereo calculation of corners. Corners are intersections of 3 lines 
 * (logic: 2 l-junctions with one sharing arm)
 */

#include "StereoCorners.h"
#include "../utils/Color.hh"

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
  vs3ID = corner->ID();
  
  isct2D.p.x = corner->isct.x;
  isct2D.p.y = corner->isct.y;
  
  if(corner->lines.Size() == 3)						/// TODO TODO TODO TODO TODO TODO TODO Wie geht man mit allgmeinen corners mit mehr als nur 3 Richtungen um?
  {
    if(corner->near_points[0] == 0) armDir[0] = -corner->lines[0]->dir;
    else  armDir[0] = corner->lines[0]->dir;
    if(corner->near_points[1] == 0) armDir[1] = -corner->lines[1]->dir;
    else  armDir[1] = corner->lines[1]->dir;
    if(corner->near_points[2] == 0) armDir[2] = -corner->lines[2]->dir;
    else  armDir[2] = corner->lines[2]->dir;
    
    armPoint[0].p = isct2D.p - 10*armDir[0];			/// TODO Theoretische Punkte 10 px entfernt von der Intersection!
    armPoint[1].p = isct2D.p - 10*armDir[1];
    armPoint[2].p = isct2D.p - 10*armDir[2];

    armColor[0][0] = corner->lines[0]->MeanCol(0);
    armColor[0][1] = corner->lines[0]->MeanCol(1);
    armColor[1][0] = corner->lines[1]->MeanCol(0);
    armColor[1][1] = corner->lines[1]->MeanCol(1);
    armColor[2][0] = corner->lines[2]->MeanCol(0);
    armColor[2][1] = corner->lines[2]->MeanCol(1);
    
    isValid = true;
  }
  else if(corner->lines.Size() == 4)						/// TODO TODO TODO TODO TODO TODO TODO Wie kann man diese aufsplitten in 3er Corners? DAS IST HIER FALSCH: HACK!!!
  {
    if(corner->near_points[0] == 0) armDir[0] = -corner->lines[0]->dir;
    else  armDir[0] = corner->lines[0]->dir;
    if(corner->near_points[1] == 0) armDir[1] = -corner->lines[1]->dir;
    else  armDir[1] = corner->lines[1]->dir;
    if(corner->near_points[2] == 0) armDir[2] = -corner->lines[2]->dir;
    else  armDir[2] = corner->lines[2]->dir;
//     armDir[1] = corner->lines[1]->dir;
//     armDir[2] = corner->lines[2]->dir;
    
    armPoint[0].p = isct2D.p - 10*armDir[0];			/// TODO Theoretische Punkte 10 px entfernt von der Intersection!
    armPoint[1].p = isct2D.p - 10*armDir[1];
    armPoint[2].p = isct2D.p - 10*armDir[2];

    armColor[0][0] = corner->lines[0]->MeanCol(0);
    armColor[0][1] = corner->lines[0]->MeanCol(1);
    armColor[1][0] = corner->lines[1]->MeanCol(0);
    armColor[1][1] = corner->lines[1]->MeanCol(1);
    armColor[2][0] = corner->lines[2]->MeanCol(0);
    armColor[2][1] = corner->lines[2]->MeanCol(1);
    
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
void TmpCorner::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  isct2D.Rectify(stereo_cam, side);
  armPoint[0].Rectify(stereo_cam, side);
  armPoint[1].Rectify(stereo_cam, side);
  armPoint[2].Rectify(stereo_cam, side);

//   printf("TmpCorner::Rectify: %u before rectify: armDir[0]: %4.2f / %4.2f\n", id, armDir[0].x, armDir[0].y);
//   printf("TmpCorner::Rectify: %u before rectify: armDir[1]: %4.2f / %4.2f\n", id, armDir[1].x, armDir[1].y);
//   printf("TmpCorner::Rectify: %u before rectify: armDir[2]: %4.2f / %4.2f\n", id, armDir[2].x, armDir[2].y);
 
  armDir[0] = Normalise(armPoint[0].pr - isct2D.pr);
  armDir[1] = Normalise(armPoint[1].pr - isct2D.pr);
  armDir[2] = Normalise(armPoint[2].pr - isct2D.pr); 
  
// //   Recalculate arm points (10px away from center) after rectification
//   armPoint[0].pr = isct2D.pr + 10 * armDir[0];
//   armPoint[1].pr = isct2D.pr + 10 * armDir[1];
//   armPoint[2].pr = isct2D.pr + 10 * armDir[2];
  
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


//-------------------------------------------------------------------//
//-------------------------- TmpCorners3D ---------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of class TmpLJunction3D
 */
TmpCorner3D::TmpCorner3D()
{}

/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * @param stereo_cam Stereo camera parameters
 * @param left Tmp. corner from the left image
 * @param right Tmp. corner from the right image
 * @param significance2D Calculated 2D significance
 */
bool TmpCorner3D::Reconstruct(cast::StereoCamera *stereo_cam, TmpCorner &left, TmpCorner &right, double significance2D)
{
  vs3ID[LEFT] = left.GetVs3ID();
  vs3ID[RIGHT] = right.GetVs3ID();
  if(!isct3D.Reconstruct(stereo_cam, left.isct2D, right.isct2D)) return false;
  
  // now calculate significance values to get "good" (or correct) matches:
  
  // What can we calculate?
  // 
  sig = significance2D;
  return true;
}

//----------------------------------------------------------------//
//------------------------- StereoCorners ------------------------//
//----------------------------------------------------------------//
/**
 * @brief Constructor of StereoFlaps: Calculate stereo matching of flaps
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoCorners::StereoCorners(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
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
  if(detail == 0) corners[side][id].isct2D.Draw();
  
  if(detail == 1)
  {
    DrawLine2D(corners[side][id].isct2D.p.x, corners[side][id].isct2D.p.y, corners[side][id].armPoint[0].p.x, corners[side][id].armPoint[0].p.y, RGBColor::magenta);
    DrawLine2D(corners[side][id].isct2D.p.x, corners[side][id].isct2D.p.y, corners[side][id].armPoint[1].p.x, corners[side][id].armPoint[1].p.y, RGBColor::magenta);
    DrawLine2D(corners[side][id].isct2D.p.x, corners[side][id].isct2D.p.y, corners[side][id].armPoint[2].p.x, corners[side][id].armPoint[2].p.y, RGBColor::magenta);
    corners[side][id].isct2D.Draw();
  }
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
  c = corner->GetIsct3D().p;

  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;
  pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

  // recalculate the vectors to the vertices from new center point
  Vector3 p((corner->GetIsct3D()).p.x, (corner->GetIsct3D()).p.y, (corner->GetIsct3D()).p.z);
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
 * @brief Calculate the opening angles of the best matching corner arms.
 * @param corner 3D corner
 * @param left_corner Corner from the left image
 * @param right_corner Corner from the right image
 * @param k Best match left_corner[k] to right_corner[l]
 * @param l
 * @param m 2nd best match left_corner[m] to right_corner[n]
 * @param n
 * @param o 3rd best match left_corner[o] to right_corner[p]
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

/**
 * @brief Calculate a significance value for a matching corner pair.
 * @param match Match value on the epipolar line
 * @param left_ljct Left corner
 * @param right_ljct Right corner
 * @return Returns the significance value.
 */
double StereoCorners::Calculate2DSignificance(double match, TmpCorner left_corner, TmpCorner right_corner)
{
  double match_sig = 1 - (match/SC_MAX_DELTA_V_POINT);  // normalisation of the matching significance (y_dist)

  // use angles
  unsigned k=0,l=0,m=0,n=0,o=0,p=0;
  double angleSum = CalculateBestArmMatches(left_corner, right_corner, k, l, m, n, o, p);
  angleSum = 1-(angleSum/(3*M_PI));		// Maximum deviation is 3*PI

  /// TODO use line length???
  
  // Use line colors
  double d0 = Dist(left_corner.GetArmColor(k, 0), right_corner.GetArmColor(l, 0));
  double d1 = Dist(left_corner.GetArmColor(k, 1), right_corner.GetArmColor(l, 1));
  double d2 = Dist(left_corner.GetArmColor(m, 0), right_corner.GetArmColor(n, 0));
  double d3 = Dist(left_corner.GetArmColor(m, 1), right_corner.GetArmColor(n, 1));
  double d4 = Dist(left_corner.GetArmColor(o, 0), right_corner.GetArmColor(p, 0));
  double d5 = Dist(left_corner.GetArmColor(o, 1), right_corner.GetArmColor(p, 1));
  double col_dist = 1 - (d0 + d1 + d2 + d3 + d4 + d5)/(2650.02);	// max = 6* sqrt(x² + y² + z²) = 2650.02

  double sigsum = angleSum*col_dist;
  
//   printf("2DSig: match: %4.3f / angle: %4.3f / col: %4.3f  => sum: %4.3f of corners  %u - %u\n", match_sig, angleSum, col_dist, sigsum, left_corner.GetVs3ID(), right_corner.GetVs3ID());

  return sigsum;
}

/**
 * @brief Match left and right corner from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_corners Array of all corners from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_corners Array of all corners from right stereo image.
 * @param matches Number of matched corners (sorted to the beginning of the arrays).
 */
void StereoCorners::MatchCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, std::map<double, unsigned> *match_map)
{
  for(unsigned i=0; i < left_corners.Size(); i++)
  {
    for(unsigned j=0; j < right_corners.Size(); j++)
    {
      double match = MatchingScorePoint(left_corners[i].isct2D, right_corners[j].isct2D);
      
      if(match < HUGE)
      {
	
	double sig = Calculate2DSignificance(match, left_corners[i], right_corners[j]);
	if(sig > SC_MIN_2D_CORNER_SIGNIFICANCE  || !SC_USE_CORNER_THRESHOLDS)  // delete the really bad 2D results
	{
	  std::pair<double, unsigned> pair(sig, j /*right_corners[j].GetVs3ID()*/);
	  match_map[i].insert(pair);
	}
      }
    }    
  }
  
  /// print match map
//   for (unsigned i=0; i<left_corners.Size(); i++)
//   {
//     std::map<double, unsigned>::iterator it;
//     it = match_map[i].end();
//     unsigned nrMatches = match_map[i].size();
//     unsigned max=5;
//     if(nrMatches<5) max=nrMatches;
//     for(unsigned j=0; j<max; j++)
//     {
//       it--;
//       printf("   Corner: match: %4.3f of corner: %u-%u\n", (*it).first, corners[LEFT][i].GetVs3ID(), corners[RIGHT][(*it).second].GetVs3ID());
//     }
//   }
}


/** TODO Besser und vor allem genauer formulieren!
 * TODO Das is eigentlich formal nicht richtig, denn die Punkte (10px vom intersection point) müssen ja nicht matchen!!!
 * TODO Wie kann man Winkel in 2D auf Winkel in 3D berechnen (jeweils aus dem linken und rechten Bild)
 * @brief Calculate 3D points of the corner arms and calculate the direction. 
 * @param corner 3D corner
 * @param left_corner Corner from the left image
 * @param right_corner Corner from the right image
 */
void StereoCorners::Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner)								/// TODO TODO TODO Wird nicht verwendet!
{
  // 3D Triangulieren der abgeschätzten ArmPunkte
  corner->armPoints3D[0].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[0]], right_corner.armPoint[right_corner.armMatch[0]]);
  corner->armPoints3D[1].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[1]], right_corner.armPoint[right_corner.armMatch[1]]);
  corner->armPoints3D[2].Reconstruct(stereo_cam, left_corner.armPoint[left_corner.armMatch[2]], right_corner.armPoint[right_corner.armMatch[2]]);

  // Calculate the direction in 3D
  corner->armDir3D[0] = Normalise(corner->armPoints3D[0].p - corner->GetIsct3D().p);
  corner->armDir3D[1] = Normalise(corner->armPoints3D[1].p - corner->GetIsct3D().p);
  corner->armDir3D[2] = Normalise(corner->armPoints3D[2].p - corner->GetIsct3D().p);
}

/**
 * @brief Each right corner can have only one best matching left corner.
 * Delete double assigned ones.
 * @param match_map Match map
 * @param map_size Size of the match_map
 */
void StereoCorners::BackCheck(std::map<double, unsigned> *match_map, unsigned map_size)
{
  unsigned nrMatches[map_size];
  for(unsigned i=0; i< map_size; i++)
    nrMatches[i] = match_map[i].size();

  bool solved = false;
  while(!solved)
  {
    solved = true;
    std::map<double, unsigned>::iterator it_i;
    std::map<double, unsigned>::iterator it_j;

    for (unsigned i=0; i<map_size; i++)
    {    
      for(unsigned j=0; j<map_size; j++)
      {
	if(i != j && nrMatches[i] > 0 && nrMatches[j] > 0)
	{
	  it_i = match_map[i].end(); it_i--;
	  it_j = match_map[j].end(); it_j--;
	  
	  if((*it_i).second == (*it_j).second)
	  {
	    solved = false;
	    if((*it_i).first > (*it_j).first)  // delete smaller value
	    {
	      match_map[j].erase(it_j);
	      nrMatches[j]--;
	    }
	    else
	    {
	      match_map[i].erase(it_i);
	      nrMatches[i]--;
	    }
	  }
	}
      }
    }
  }
  
  /// PRINT match map
//   for(unsigned i=0; i<map_size; i++)
//   {
//     std::map<double, unsigned>::iterator it;
//     if(match_map[i].size() > 0)
//     {
//       it = match_map[i].end(); it--;
// //       printf("MatchMap after back check: %4.3f with %u / %u\n", (*it).first, corners[LEFT][i].GetVs3ID(), corners[RIGHT][(*it).second].GetVs3ID());
//       printf("MatchMap after back check: corners[i][sec]: %4.3f with %u / %u\n", (*it).first, i, (*it).second);
//     }
//   }
}

/**
 * @brief Calculate 3D corners from matched 2D Gestalts.
 * @param left_corners Array of all corners from left stereo image.
 * @param right_corners Array of all corners from right stereo image.
 * @param matches Number of matched points.
 * @param corner3ds Array of calculated 3d corners.
 */
unsigned StereoCorners::Calculate3DCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, std::map<double, unsigned> *match_map)
{
printf("Calculate3DCorners: start: xxx_corners.size(): %u - %u\n", left_corners.Size(), right_corners.Size());
  Array<TmpCorner> left, right;
  unsigned maxSize = 5;    // maximum size of match_map results
  unsigned nrMatches = 0;
  std::map<double, unsigned>::iterator it;
  TmpCorner3D tmpCorner3D[left_corners.Size()][maxSize];
  std::map<double, unsigned> new_match_map[left_corners.Size()]; 

  printf("Calculate3DCorners: 1\n");

  for(unsigned i=0; i<left_corners.Size(); i++)
  {
    unsigned nrResults = match_map[i].size();
    if(nrResults > maxSize) nrResults = maxSize;
    if(nrResults > 0)
    {
      it = match_map[i].end();
      for(unsigned j=0; j<nrResults; j++)
      {
	it--;
	bool reconstruct = tmpCorner3D[i][j].Reconstruct(stereo_cam, left_corners[i], right_corners[(*it).second], (*it).first);
	if(reconstruct)
	{ 
	  std::pair<double, unsigned> pair(tmpCorner3D[i][j].GetSignificance(), (*it).second);
	  new_match_map[i].insert(pair);
	  tmpCorner3D[i][j].SetTmpID(i, (*it).second);
	} 
	tmpCorner3D[i][j].SetValidation(reconstruct);
      }
    }
  }

printf("Calculate3DCorners: 2\n");

  // only one to one assignments between left and right l-junctions
  BackCheck(new_match_map, left_corners.Size());
  
printf("Calculate3DCorners: 3\n");

  // Create new stereo corners and store in the arrays
  for(unsigned i=0; i<left_corners.Size(); i++)
  {
    if(new_match_map[i].size() > 0)
    {
      it = new_match_map[i].end(); it--;
      for(unsigned k=0; k<maxSize; k++)
      {
        if(tmpCorner3D[i][k].IsValid())
        {
          if(tmpCorner3D[i][k].GetVs3ID(RIGHT) == right_corners[(*it).second].GetVs3ID())
          {
            corners3D.PushBack(tmpCorner3D[i][k]);
            Corner3D *corner3D = new Corner3D(tmpCorner3D[i][k].GetIsct3D());
              //tmpLJct3D[i][k].GetVs3ID(LEFT), (*it).second, tmpLJct3D[i][k].GetCenter(), 
          //			     tmpLJct3D[i][k].GetRadius(), tmpLJct3D[i][k].GetSignificance());
            score->NewGestalt3D(corner3D);    

          // printf("Calculate3DCorners: 3Dsig: %4.3f and ids: %u, %u\n", tmpCorner3D[i][k].GetSignificance(), tmpCorner3D[i][k].GetTmpID(LEFT), tmpCorner3D[i][k].GetTmpID(RIGHT));
            
            left.PushBack(left_corners[tmpCorner3D[i][k].GetTmpID(LEFT)]);
            right.PushBack(right_corners[tmpCorner3D[i][k].GetTmpID(RIGHT)]);
            nrMatches++;
          }
         }
      }
    }
  }

  left_corners = left;
  right_corners = right;
printf("Calculate3DCorners: end\n");
    
  return nrMatches;
}


/**
 * @brief Clear all arrays.
 */
void StereoCorners::ClearResults()
{
  corners[LEFT].Clear();
  corners[RIGHT].Clear();
  corners3D.Clear();
  cornerMatches = 0;
}


/**
 * @brief Match and calculate 3D corners from 2D corners.
 */
void StereoCorners::Process()
{
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

//   // do stereo matching and depth calculation
//   cornerMatches = 0;
// // printf("StereoLJunctions::Process: left: %u - right: %u\n", ljcts[LEFT].Size(), ljcts[RIGHT].Size());
//   MatchCorners(corners[LEFT], corners[RIGHT], cornerMatches);
// // printf("MatchedLJunctions: %u\n", ljctMatches);
//   Calculate3DCorners(corners[LEFT], corners[RIGHT], cornerMatches);
  
  
  
  struct timespec start, end;
clock_gettime(CLOCK_REALTIME, &start);
  
  // define match map with significance value as key and right ellipse id (== corners[RIGHT][id])
  std::map<double, unsigned> match_map[corners[LEFT].Size()];
  MatchCorners(corners[LEFT], corners[RIGHT], match_map);

clock_gettime(CLOCK_REALTIME, &end);
cout<<"StereoCorners::Process: Time to match [s]: " << timespec_diff(&end, &start) << endl;

struct timespec cstart, cend;
clock_gettime(CLOCK_REALTIME, &cstart);
  
    cornerMatches = Calculate3DCorners(corners[LEFT], corners[RIGHT], match_map);
  
clock_gettime(CLOCK_REALTIME, &cend);
cout<<"StereoLJunctions::Process: Time to calculate 3D corners [s]: " << timespec_diff(&cend, &cstart) << endl;


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








