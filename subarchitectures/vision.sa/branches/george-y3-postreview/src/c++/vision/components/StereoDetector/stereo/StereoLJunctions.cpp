/**
 * @file StereoLJunctions.cpp
 * @author Andreas Richtsfeld
 * @date Juni 2010
 * @version 0.1
 * @brief Stereo calculation of l-junctions.
 */

#include "StereoLJunctions.h"
#include "StereoTypes.h"

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
  isct2D.p.x = ljct->isct.x;
  isct2D.p.y = ljct->isct.y;
  
  dir[0] = ljct->dir[0];
  dir[1] = ljct->dir[1];
  
  vs3ID = ljct->ID();
  
  color[0][0] = ljct->line[0]->MeanCol(0);
  color[0][1] = ljct->line[0]->MeanCol(1);
  color[1][0] = ljct->line[1]->MeanCol(0);
  color[1][1] = ljct->line[1]->MeanCol(1);
}


/**
 * @brief Recalculate all rectangle parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpLJunction::RePrune(int oX, int oY, int sc)
{
  isct2D.RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpLJunction
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpLJunction::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  isct2D.Rectify(stereo_cam, side);
}


/**
 * @brief Refine TmpLJunction
 */
void TmpLJunction::Refine()
{
  isct2D.Refine();
}

/**
 * @brief Returns true, if junction is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if junction is at this position.
 */
bool TmpLJunction::IsAtPosition(int x, int y) const
{
  return isct2D.IsAtPosition(x, y);
}

//-------------------------------------------------------------------//
//-------------------------- TmpLjunctions3D ---------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of class TmpLJunction3D
 */
TmpLJunction3D::TmpLJunction3D()
{}

/**
 * @brief Fit a ellipse into some rectified points of the ellipse, \n
 * to get the rectified ellipse parameters.
 * @param left Tmp. ellipse from the left image
 * @param numPoints Defines the number of hull points for later 3D matching
 */
bool TmpLJunction3D::Reconstruct(cast::StereoCamera *stereo_cam, TmpLJunction &left, TmpLJunction &right, double significance2D)
{
  vs3ID[LEFT] = left.vs3ID;
  vs3ID[RIGHT] = right.vs3ID;
  if(!isct3D.Reconstruct(stereo_cam, left.isct2D, right.isct2D)) return false;
  
  // now calculate significance values to get "good" (or correct) matches:
  
  // What can we calculate?
  // 
  sig = significance2D;
  
  
  return true;
}


//-------------------------------------------------------------------//
//------------------------- StereoLJunctions ------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoLJunctions: Calculate stereo matching of L-junctions
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoLJunctions::StereoLJunctions(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
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
  ljcts[side][id].isct2D.Draw();
}

/**
 * @brief Convert the l-junction to a working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the l-junction.
 * @return Return true for success
 */
#ifdef HAVE_CAST
bool StereoLJunctions::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
  obj->model = new VisionData::GeometryModel;
  LJunction3D *ljct = LJunctions3D(score, id);

  // Recalculate pose of vertices (relative to the pose of the flap == cog)
  Pose3 pose;
  Vector3 c(0., 0., 0.);
  c = ljct->isct3D.p;

  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;
  pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

  // recalculate the vectors to the vertices from new center point
  Vector3 p(ljct->isct3D.p.x, ljct->isct3D.p.y, ljct->isct3D.p.z);
  p = inv.Transform(p);
  
  // add center point to the model
  cogx::Math::Pose3 cogxPose;
  cogxPose.pos.x = pose.pos.x;
  cogxPose.pos.y = pose.pos.y;
  cogxPose.pos.z = pose.pos.z;
  obj->pose = cogxPose;

  // create vertices (relative to the 3D center point)
  /// TODO Ich erzeuge hier eine Fläche aus 3 Punkten!
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
 * @brief Calculate a significance value of L-Junctions.
 * @param match Match value on the epipolar line
 * @param left_ljct Left L-junction
 * @param right_ljct Right L-junction
 * @return Returns the significance value.
 */
double StereoLJunctions::Calculate2DSignificance(double match, TmpLJunction left_ljct, TmpLJunction right_ljct)
{
  double match_sig = 1 - (match/SC_MAX_DELTA_V_POINT);  // normalisation of the matching significance (y_dist)
  
  // Winkelabweichung berechnen (ist im Bereich 0 bis 2*2*PI = 12,5
  double oa0 = OpeningAngle(left_ljct.dir[0], right_ljct.dir[0]) + OpeningAngle(left_ljct.dir[1], right_ljct.dir[1]);;
  double oa1 = OpeningAngle(left_ljct.dir[0], right_ljct.dir[1]) + OpeningAngle(left_ljct.dir[1], right_ljct.dir[0]);
  double angleDiff_sig = 1 - (min(oa0, oa1)/(2*M_PI)); // normalised

  double d0 = Dist(left_ljct.color[0][0], right_ljct.color[0][0]);
  double d1 = Dist(left_ljct.color[0][1], right_ljct.color[0][1]);
  double d2 = Dist(left_ljct.color[1][0], right_ljct.color[1][0]);
  double d3 = Dist(left_ljct.color[1][1], right_ljct.color[1][1]);
  double col_dist = 1 - (d0 + d1 + d2 + d3)/(1766.6918);	// max = 4* sqrt(x² + y² + z²) = 1766,69
  
  double sigsum = match_sig*angleDiff_sig*col_dist;
// printf("dist: %4.2f - %4.2f - %4.2f - %4.2f of lines: %u - %u\n", d0, d1, d2, d3, left_ljct.vs3ID, right_ljct.vs3ID);
  
// printf("2DSig: match: %4.3f / angle: %4.3f / col: %4.3f  => sum: %4.3f of ljcts  %u - %u\n", match_sig, angleDiff_sig, col_dist, sigsum, left_ljct.vs3ID, right_ljct.vs3ID);
// printf("                        : sig: %4.3f\n", match_sig*angleDiff_sig);

  /// TODO use line length???
  
  return col_dist; //match_sig * angleDiff_sig;		/// TODO TODO TODO significance, calculated only from color distance of the arms!!!
}

/**
 * @brief Match left and right l-junctions from an stereo image pair and get it sorted to the beginning of the array.
 * @param left_ljcts Array of all rectangles from left stereo image (matching flaps get sorted to the beginning of the array.)
 * @param right_ljcts Array of all rectangles from right stereo image.
 * @param matches Number of matched rectangles (sorted to the beginning of the arrays).
 */
void StereoLJunctions::MatchLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, std::map<double, unsigned> *match_map)
{
  for(unsigned i=0; i < left_ljcts.Size(); i++)
  {
    for(unsigned j=0; j < right_ljcts.Size(); j++)
    {
      double match = MatchingScorePoint(left_ljcts[i].isct2D, right_ljcts[j].isct2D);
      if(match != HUGE)
      {
	double sig = Calculate2DSignificance(match, left_ljcts[i], right_ljcts[j]);
// 	double size = (left_ell[i].a + right_ell[j].a)/2.;
// printf("MatchEllipses: size: %4.3f\n", size);
// 	if(sig > SC_MIN_2D_SIGNIFICANCE && size > SC_MIN_AXIS_SIZE)  // delete the really bad 2D results
	if(sig > SC_MIN_2D_LJCT_SIGNIFICANCE  || !SC_USE_LJCT_THRESHOLDS)  // delete the really bad 2D results
	{
	  std::pair<double, unsigned> pair(sig, /*right_ljcts[j].vs3ID*/j);
	  match_map[i].insert(pair);
	}
      }
    }    
  }
  
  /// print match map
//   for (unsigned i=0; i<left_ljcts.Size(); i++)
//   {
//     std::map<double, unsigned>::iterator it;
//     it = match_map[i].end();
//     unsigned nrMatches = match_map[i].size();
//     unsigned max=5;
//     if(nrMatches<5) max=nrMatches;
//     for(unsigned j=0; j<max; j++)
//     {
//       it--;
//       printf("   LJcts: match after: %4.5f of jcts: %u-%u\n", (*it).first, i, (*it).second);
//     }
//   }
}

/**
 * @brief Each right l-jct can have only one best matching left l-jct.
 * Delete double assigned ones.
 * @param match_map Match map for ellipses
 * @param map_size Size of the match_map
 */
void StereoLJunctions::BackCheck(std::map<double, unsigned> *match_map, unsigned map_size)
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
//       printf("MatchMap after: %4.3f with %u / %u\n", (*it).first, i, (*it).second);
//     }
//   }
}


/**
 * @brief Calculate 3D points from matched l-junctions.
 * @param left_ljcts Array of all l-junctions from left stereo image.
 * @param right_rects Array of all l-junctions from right stereo image.
 * @param matches Number of matched points.
 * @param ljct3ds Array of calculated 3d l-junctions.
 * @return Returns the number of matches.
 */
unsigned StereoLJunctions::Calculate3DLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, std::map<double, unsigned> *match_map)
{
printf("Calculate3DLJunctions: start\n");
  Array<TmpLJunction> left, right;
  unsigned maxSize = 5;    // maximum size of match_map results
  unsigned nrMatches = 0;
  std::map<double, unsigned>::iterator it;
  TmpLJunction3D tmpLJct3D[left_ljcts.Size()][maxSize];
  std::map<double, unsigned> new_match_map[left_ljcts.Size()]; 

  for(unsigned i=0; i<left_ljcts.Size(); i++)
  {
    unsigned nrResults = match_map[i].size();
    if(nrResults > maxSize) nrResults = maxSize;
    if(nrResults > 0)
    {
      it = match_map[i].end();
      for(unsigned j=0; j<nrResults; j++)
      {
	it--;
	unsigned rightID = (*it).second;
	bool reconstruct = tmpLJct3D[i][j].Reconstruct(stereo_cam, left_ljcts[i], right_ljcts[rightID], (*it).first);
	if(reconstruct)
	{ 
	  std::pair<double, unsigned> pair(tmpLJct3D[i][j].GetSignificance(), (*it).second);
	  new_match_map[i].insert(pair);
	  tmpLJct3D[i][j].SetTmpID(i, (*it).second);
	} 
	tmpLJct3D[i][j].SetValidation(reconstruct);
      }
    }
  }
  
  // only one to one assignments between left and right l-junctions
  BackCheck(new_match_map, left_ljcts.Size());
  
  // Create new stereo l-junctions and store in the arrays
  for(unsigned i=0; i<left_ljcts.Size(); i++)
  {
    if(new_match_map[i].size() > 0)
    {
      it = new_match_map[i].end(); it--;
      for(unsigned k=0; k<maxSize; k++)
      {
	if(tmpLJct3D[i][k].IsValid())
	{
	  if(tmpLJct3D[i][k].GetVs3ID(RIGHT) == (*it).second)
	  {
	    ljcts3D.PushBack(tmpLJct3D[i][k]);
	    LJunction3D *ljct3D = new LJunction3D(tmpLJct3D[i][k].GetIsct3D());
	      //tmpLJct3D[i][k].GetVs3ID(LEFT), (*it).second, tmpLJct3D[i][k].GetCenter(), 
		//			     tmpLJct3D[i][k].GetRadius(), tmpLJct3D[i][k].GetSignificance());
	    score->NewGestalt3D(ljct3D);    
	  
// printf("Calculate3DJcts: 3Dsig: %4.3f and ids: %u, %u\n", tmpEll3D[i][k].GetSignificance(), tmpEll3D[i][k].GetTmpEllID(LEFT), tmpEll3D[i][k].GetTmpEllID(RIGHT));
	    
	    left.PushBack(left_ljcts[tmpLJct3D[i][k].GetTmpID(LEFT)]);
	    right.PushBack(right_ljcts[tmpLJct3D[i][k].GetTmpID(RIGHT)]);
	    nrMatches++;
	  }
	}
      }
    }
  }

  left_ljcts = left;
  right_ljcts = right;
printf("Calculate3DLJunctions: end\n");
    
  return nrMatches;
}


/**
 * @brief Clear all arrays.
 */
void StereoLJunctions::ClearResults()
{
  ljcts[LEFT].Clear();
  ljcts[RIGHT].Clear();
  ljcts3D.Clear();
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
  
struct timespec start, end;
clock_gettime(CLOCK_REALTIME, &start);
  
  // define match map with significance value as key and right ellipse id
  std::map<double, unsigned> match_map[ljcts[LEFT].Size()];
//   MatchEllipses(ellipses[LEFT], ellipses[RIGHT], match_map);
  MatchLJunctions(ljcts[LEFT], ljcts[RIGHT], match_map);

clock_gettime(CLOCK_REALTIME, &end);
cout<<"StereoLJunctions::Process: Time to match [s]: " << timespec_diff(&end, &start) << endl;

struct timespec cstart, cend;
clock_gettime(CLOCK_REALTIME, &cstart);
  
   ljctMatches = Calculate3DLJunctions(ljcts[LEFT], ljcts[RIGHT], match_map);							/// TODO 
  
clock_gettime(CLOCK_REALTIME, &cend);
cout<<"StereoLJunctions::Process: Time to calculate 3D ljcts [s]: " << timespec_diff(&cend, &cstart) << endl;

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








