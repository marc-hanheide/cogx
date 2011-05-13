/**
 * @file StereoLines.cpp
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Stereo calculation of lines with MSLD-descriptor.
 */

#include <time.h>
#include "StereoLines.h"
#include "StereoTypes.h"

namespace Z
{

//-----------------------------------------------------------------//
//-------------------------- TmpLJunction -------------------------//
//-----------------------------------------------------------------//
/**
 * @brief Constructor TmpLine
 * @param line vs3 line
 */
TmpLine::TmpLine(Line *line)
{
  rectified = false;
  
  point2D[0].p = line->point[0];
  point2D[1].p = line->point[1];
  vs3ID = line->ID();
}

/**
 * @brief Draw the tmp. line
 */
void TmpLine::Draw(int detail)
{
  DrawLine2D(point2D[0].p.x, point2D[0].p.y, point2D[1].p.x, point2D[1].p.y, RGBColor::blue);
}

/**
 * @brief Recalculate all line parameters, when image was pruned from HR image.
 * @param oX Offset of x-coordinate
 * @param oY Offset of y-coordinate
 * @param sc Scale between original and pruned image
 */
void TmpLine::RePrune(int oX, int oY, int sc)
{
  point2D[0].RePrune(oX, oY, sc);
  point2D[1].RePrune(oX, oY, sc);
}

/**
 * @brief Rectify TmpLine
 * @param cam Stereo camera parameters and functions.
 * @param side LEFT / RIGHT side of stereo
 */
void TmpLine::Rectify(cast::StereoCamera *stereo_cam, int side)
{
  if(rectified)
  {
    printf("TmpLine::Rectify: Warning: Line is already rectified!\n");
    return;
  }
  
  point2D[0].Rectify(stereo_cam, side);
  point2D[1].Rectify(stereo_cam, side);
  rectified = true;
}


/**
 * @brief Refine TmpLine
 */
void TmpLine::Refine()
{
printf("TmpLine::Refine: Not yet implemented!\n");
//   point2D.Refine();
}

/**
 * @brief Returns true, if line is at the x/y-position in the image.
 * @param x X-coordinate in image pixel.
 * @param y Y-coordinate in image pixel.
 * @return Returns true, if line is at this position.
 * TODO We find only the points at the end!
 */
bool TmpLine::IsAtPosition(int x, int y) const
{
  printf("TmpLine::IsAtPosition: Not yet correctly implemented!\n");
  
  return false;//(Lines(vcore[side], vs3ID)->IsAtPosition(x,y));
}


//-------------------------------------------------------------------//
//------------------------- StereoLJunctions ------------------------//
//-------------------------------------------------------------------//
/**
 * @brief Constructor of StereoLines: Calculate stereo matching of Lines
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
StereoLines::StereoLines(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc) : StereoBase(sco)
{
  vcore[LEFT] = vc[LEFT];
  vcore[RIGHT] = vc[RIGHT];
  stereo_cam = sc;
  lineMatches = 0;
}


/**
 * @brief Draw matched Lines.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoLines::DrawMatched(int side, bool single, int id, int detail)
{
  if(single)
  {
    if(id < 0 || id >= lineMatches)
    {
      printf("StereoLines::DrawMatched: warning: id out of range!\n");
      return;
    }
    DrawSingleMatched(side, id, detail);
  }
  else
    for(int i=0; i< lineMatches; i++)
      DrawSingleMatched(side, i, detail);
}

/**
 * @brief Draw single matched line.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
void StereoLines::DrawSingleMatched(int side, int id, int detail)
{
  lines[side][id].Draw(detail);
}

/**
 * @brief Convert the lines to a working memory's visual object.
 * @param obj Visual object to create.
 * @param id ID of the l-junction.
 * @return Return true for success
 *                                                                TODO Also quick-hack for virtual scene: We produce small surfaces instead of lines.
 */
#ifdef HAVE_CAST
bool StereoLines::StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id)
{
  obj->model = new VisionData::GeometryModel;
  Line3D *line = Lines3D(score, id);

  // Recalculate pose of vertices (relative to the pose of the flap == cog)
  Pose3 pose;
  Vector3 c(0., 0., 0.);
  c = (line->isct3D[0].p + line->isct3D[1].p)/2.;

  pose.pos.x = c.x;
  pose.pos.y = c.y;
  pose.pos.z = c.z;
  pose.rot.x = 0.;   // set the orientation to identity, i.e. parallel to world coordinate system
  pose.rot.y = 0.;
  pose.rot.z = 0.;

  // invert to get pose of world w.r.t. flap
  Pose3 inv = pose.Inverse();

  // add center point to the model
  cogx::Math::Pose3 cogxPose;
  cogxPose.pos.x = pose.pos.x;
  cogxPose.pos.y = pose.pos.y;
  cogxPose.pos.z = pose.pos.z;
  obj->pose = cogxPose;

  // create vertices (relative to the 3D center point)
  Vector3 p(line->isct3D[0].p.x,
	    line->isct3D[0].p.y,
	    line->isct3D[0].p.z);
  p = inv.Transform(p);
  Vector3 p2(line->isct3D[1].p.x,
             line->isct3D[1].p.y,
             line->isct3D[1].p.z);
  p2 = inv.Transform(p2);
  
  // calculate normal direction of the line
  Vector3 tmpV;
  tmpV = Normalise(p2-p);

  VisionData::Vertex u;           /// Start point of line
  u.pos.x = p.x;
  u.pos.y = p.y;
  u.pos.z = p.z;
  u.normal.x = tmpV.x;
  u.normal.y = tmpV.y;
  u.normal.z = tmpV.z;
  obj->model->vertices.push_back(u);
  VisionData::Vertex v;           /// We create a surface: point near start point
  v.pos.x = p.x + 0.001;
  v.pos.y = p.y;
  v.pos.z = p.z;
  obj->model->vertices.push_back(v);  

  VisionData::Vertex w;           /// End point of line
  w.pos.x = p2.x;
  w.pos.y = p2.y;
  w.pos.z = p2.z;
  w.normal.x = tmpV.x;
  w.normal.y = tmpV.y;
  w.normal.z = tmpV.z;
  obj->model->vertices.push_back(w);
  VisionData::Vertex x;           /// We create a surface: point near end point
  x.pos.x = p2.x + 0.001;
  x.pos.y = p2.y;
  x.pos.z = p2.z;
  obj->model->vertices.push_back(x);
  
  // add faces to the vision model
  VisionData::Face f;
  f.vertices.push_back(0);
  f.vertices.push_back(1);
  f.vertices.push_back(2);
  f.vertices.push_back(3);
  obj->model->faces.push_back(f);
  f.vertices.clear();

  obj->detectionConfidence = line->GetSignificance();

  
  /// HACK HACK HACK: Add 2D information of left image to the visual object!
  double x1, y1 ,x2, y2;
  x1 = ((Z::Line*) vcore[LEFT]->Gestalts(Gestalt::LINE, line->GetVs3ID(LEFT)))->point[0].x;
  y1 = ((Z::Line*) vcore[LEFT]->Gestalts(Gestalt::LINE, line->GetVs3ID(LEFT)))->point[0].y;
  x2 = ((Z::Line*) vcore[LEFT]->Gestalts(Gestalt::LINE, line->GetVs3ID(LEFT)))->point[1].x;
  y2 = ((Z::Line*) vcore[LEFT]->Gestalts(Gestalt::LINE, line->GetVs3ID(LEFT)))->point[1].y;
  obj->points2D.push_back(x1);
  obj->points2D.push_back(y1);
  obj->points2D.push_back(x2);
  obj->points2D.push_back(y2);
  
  return true;
}
#endif


/**
 * @brief Check the results from the match_map for epipoolar sanity. The best "nrCompare" results, which are
 * not on the epipolar line will be deleted. 
 * The match_map has ordered entries with the best matching right lines. We delete every result which is not 
 * on the same epipolarline. As result we get the best matching results again in the match_map.
 * @param idx Index of the left line
 * @param match_map Best match pairs (distance, index) as map.
 * @param nrCompare Calculate nrCompare good results for every left line.
 * @return Returns the index of the best match
 *										TODO We should work with the rectified end points of the line: lines[LEFT][idx].point2D[0].pr ???
 */
unsigned StereoLines::EpipolarSanityCheck(unsigned idx, std::map<float, unsigned> &match_map, unsigned nrCompare)
{
  if(match_map.size() < nrCompare*3)
    nrCompare = match_map.size()/3;

  unsigned nrResults = 0;
  std::map<float, unsigned>::iterator it;
  it = match_map.begin();
  int count = 0;
  while(nrResults < 5 && count < (nrCompare*3))											/// TODO Austauschen mit define variablen!
  {
    Vertex2D startPointLeft, midPointLeft, endPointLeft;
    startPointLeft.p = lines[LEFT][idx].point2D[0].p;
    midPointLeft.p = (lines[LEFT][idx].point2D[0].p + lines[LEFT][idx].point2D[1].p)/2.;
    endPointLeft.p = lines[LEFT][idx].point2D[1].p;
    startPointLeft.Rectify(stereo_cam, LEFT);
    midPointLeft.Rectify(stereo_cam, LEFT);
    endPointLeft.Rectify(stereo_cam, LEFT);

    Vertex2D startPointRight, midPointRight, endPointRight;
    startPointRight.p = lines[RIGHT][it->second].point2D[0].p;
    midPointRight.p = (lines[RIGHT][it->second].point2D[0].p + lines[RIGHT][it->second].point2D[1].p)/2.;
    endPointRight.p = lines[RIGHT][it->second].point2D[1].p;
    startPointRight.Rectify(stereo_cam, RIGHT);
    midPointRight.Rectify(stereo_cam, RIGHT);
    endPointRight.Rectify(stereo_cam, RIGHT);
    
    // We looking for epipolar "good" lines: 2 of the 4 endpoints must intersect the line in the other image.
    unsigned goodPoints = 0;
    if((startPointRight.p.y > startPointLeft.p.y && endPointRight.p.y < startPointLeft.p.y) ||
       (startPointRight.p.y < startPointLeft.p.y && endPointRight.p.y > startPointLeft.p.y)) goodPoints++;
    if((startPointRight.p.y > endPointLeft.p.y && endPointRight.p.y < endPointLeft.p.y) ||
       (startPointRight.p.y < endPointLeft.p.y && endPointRight.p.y > endPointLeft.p.y)) goodPoints++;
    if((startPointLeft.p.y > startPointRight.p.y && endPointLeft.p.y < startPointRight.p.y) ||
       (startPointLeft.p.y < startPointRight.p.y && endPointLeft.p.y > startPointRight.p.y)) goodPoints++;
    if((startPointLeft.p.y > endPointRight.p.y && endPointLeft.p.y < endPointRight.p.y) ||
       (startPointLeft.p.y < endPointRight.p.y && endPointLeft.p.y > endPointRight.p.y)) goodPoints++;
    
    if(goodPoints > 1 )
    {
      nrResults++;
      it++; 
    }
    else match_map.erase(it++);       // delete the result, which are not on the epipolarline
    count++;
  }
  return nrResults; 
}


/**
 * @brief Extend (change) the desciptor distance in the match map.
 * @param idx Index of the LEFT line
 * @param match_map Match map: desciptor distance and index of right line for the best "nrMatches" matches.
 * @param nrMatches Number of matches for this left (idx) line.
 */
void StereoLines::ExtendDescriptors(unsigned idx, std::map<float, unsigned> &match_map, unsigned nrMatches)
{
  std::map<float, unsigned> new_map;
  std::map<float, unsigned>::iterator it;
  it = match_map.begin();
  if(match_map.size() < nrMatches) nrMatches = match_map.size();

  for(unsigned j=0; j<nrMatches; j++)
  {
    // Get the normalized angle (0-1) between the two lines
    double angleSig = SmallestAngle(lines[LEFT][idx].point2D[0].p - lines[LEFT][idx].point2D[1].p, 
                                    lines[RIGHT][it->second].point2D[0].p - lines[RIGHT][it->second].point2D[1].p) / (M_PI/2.);

    // normalization to a value between 0.5(good) and 0.7(worse)
    angleSig = angleSig/5. + 0.5;

    std::pair<float, unsigned> pair(it->first*angleSig, it->second);
    new_map.insert(pair);
    it++;
  }
  match_map = new_map;
}

/**
 * @brief Match left and right lines from an stereo image pair. We get a vector with matched line pair indices.
 * @param descr_left Vector with descriptor for every left image line
 * @param descr_right Vector with descriptor for every right image line
 * @param matches A vector of index pairs of matches between left and right image lines (tmpLines) 
 */
void StereoLines::MatchLines(std::vector< std::vector<float> > descr_left, 
                             std::vector< std::vector<float> > descr_right, 
                             std::vector< std::pair<unsigned, unsigned> > &matches)
{
  float dist;                                             // Calculted descriptor distance
  unsigned nrLinesLeft = descr_left.size();               // Number of left lines
  std::map<float, unsigned> best_matches[nrLinesLeft];    // map for a left line[i] with the descriptor distance (float) to a right tmpLine (unsigned)
  unsigned nrMatches[nrLinesLeft];                        // Number of matches for each left line
  

  // calculate the best_matches map for each line left (i)
  for (unsigned i=0; i<nrLinesLeft; i++)
  {
    if (descr_left[i].size() != 0)
    {
      for (unsigned j=0; j<descr_right.size(); j++)
      {
        if (descr_right[j].size() == descr_left[i].size())
        {
          dist = ::DistSqr(&descr_left[i][0], &descr_right[j][0], descr_left[i].size());
	  std::pair<float, unsigned> pair(dist, j);
	  if(best_matches[i].find(dist) != best_matches[i].end())
	    printf("StereoLines::MatchLines: Warning: same key found!\n");
	  best_matches[i].insert(pair);
        }
      }
    }
  }

  // Delete all matches from the best_matches map which are not on the epipolar line.
  for (unsigned i=0; i<nrLinesLeft; i++)
    nrMatches[i] = EpipolarSanityCheck(i, best_matches[i], 5);

  
  // We extend the descriptor distance with other geometric constraints!
  for (unsigned i=0; i<nrLinesLeft; i++)
    ExtendDescriptors(i, best_matches[i], nrMatches[i]);
  
  
  /// write results
//   for (unsigned i=0; i<nrLinesLeft; i++)
//   {
//     std::map<float, unsigned>::iterator it;
//     it = best_matches[i].begin();
//     unsigned max=5;
//     if(nrMatches[i]<5) max=nrMatches[i];
//     for(unsigned j=0; j<max; j++)
//     {
//       printf("   match: dist after extending: %4.5f of lines %u/%u\n", (*it).first, i, (*it).second);
//       it++;
//     }
//   }
  
  // Each line (left or right) can have only one match.										/// TODO TODO Eigene Funktion schreiben
  bool solved = false;
  while(!solved)
  {
    solved = true;
    std::map<float, unsigned>::iterator it_i;
    std::map<float, unsigned>::iterator it_j;

    for (unsigned i=0; i<nrLinesLeft; i++)
    {    
      for(unsigned j=0; j<nrLinesLeft; j++)
      {
        if(i != j && nrMatches[i] > 0 && nrMatches[j] > 0)
        {
          it_i = best_matches[i].begin();
          it_j = best_matches[j].begin();
          
          if((*it_i).second == (*it_j).second)
          {
            solved = false;
            if((*it_i).first < (*it_j).first)  // lösche den größeren Eintrag => es ist ja die Distanz
            {
              best_matches[j].erase(it_j++);
              nrMatches[j]--;
            }
            else
            {
              best_matches[i].erase(it_i++);
              nrMatches[i]--;
            }
          }
        }
      }
    }
  }

  /// TODO Write best matching pairs => Delete later
//   for (unsigned i=0; i<nrLinesLeft; i++)
//   {
//     std::map<float, unsigned>::iterator it;
//     it = best_matches[i].begin();
//     unsigned max=5;
//     if(nrMatches[i]<5) max=nrMatches[i];
//     for(unsigned j=0; j<max; j++)
//     {
//       printf("   match: dist after: %4.5f of lines %u/%u\n", (*it).first, i, (*it).second);
//       it++;
//     }
//   }


  // copy the best result for all lines into matches vector
  for (unsigned i=0; i<nrLinesLeft; i++)
  {    
    std::map<float, unsigned>::iterator it;
    it = best_matches[i].begin();
    if(nrMatches[i] > 0)
    {
      std::pair<unsigned, unsigned> pair(i, it->second);
      matches.push_back(pair);
    }
    best_matches[i].clear();
  }
}

/**
 * @brief Process the MSLD descriptor for each line for stereo matching.
 * @param desciptors MSLD descriptors for left and right lines.
 * @param matches A vector of matching line pairs 
 */
void StereoLines::ProcessMSLD(std::vector< std::vector<float> > *descriptors)
{
  cv::Mat image[2];
  std::vector< std::vector<cv::Vec2d> > vecLines[2];    // lines as openCV vectors
  cv::Vec2d v; 
  for(unsigned side = LEFT; side <= RIGHT; side++)
  {
    const IplImage *i = vcore[side]->GetImage();
    IplImage *grayImg = cvCreateImage(cvSize(i->width, i->height), IPL_DEPTH_8U, 1);
    cvCvtColor(i, grayImg, CV_RGB2GRAY);
    image[side] = grayImg;
    for(unsigned i=0; i<lines[side].Size(); i++)
    {
      std::vector<cv::Vec2d> vLines;
      v = cv::Vec2d(lines[side][i].point2D[0].p.x, lines[side][i].point2D[0].p.y);
      vLines.push_back(v);
      v = cv::Vec2d(lines[side][i].point2D[1].p.x, lines[side][i].point2D[1].p.y);
      vLines.push_back(v);
      vecLines[side].push_back(vLines);
    }
    createMSLD.Operate(image[side], vecLines[side], descriptors[side]);
  }
}

/**
 * @brief Sort the line arrays for displaying: left_lines[i] and right_lines[i] are always correct pairs.
 * @param left_lines Array of all lines from left stereo image.
 * @param right_lines Array of all lines from right stereo image.
 * @param matches Vector with matching line pairs (left/right).
 */
void StereoLines::SortLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, std::vector< std::pair<unsigned, unsigned> > &matches)
{
  Array<TmpLine> l_lines, r_lines;
  for(unsigned i=0; i<matches.size(); i++)
  {
    l_lines.PushBack(left_lines[matches[i].first]);
    r_lines.PushBack(right_lines[matches[i].second]);
  }
  left_lines = l_lines;
  right_lines = r_lines;
}


/**
 * @brief Calculate 3D Gestalt from matched line.
 * We have to calculate the end-points of the lines and theire intersection (from left/right) image. 
 * We use the rectified points from the TmpLines.
 * @param left_lines Array of all lines from left stereo image.
 * @param right_lines Array of all lines from right stereo image.
 * @param matches Number of matched points.
 * @param ljct3ds Array of calculated 3d lines.
 */
void StereoLines::Calculate3DLines(Array<TmpLine> &left_lines, Array<TmpLine> &right_lines, int &matches)
{
  unsigned u = matches;
  for(unsigned i = 0; i < u;)
  {
    Vertex2D isctPointLeft[2], isctPointRight[2];     // the four intersection points of the stereo line
    Vector2 ll[2], rl[2];                             // left/right line points RECTIFIED!
    unsigned leftBig, rightBig;                       // the point with higher y-value left/right 
    bool leftBiggest, leftSmallest;                   // Is the point with the highest y-value left (or right)
    double length;                                    // unused

    ll[0] = left_lines[i].point2D[0].pr;
    ll[1] = left_lines[i].point2D[1].pr;
    rl[0] = right_lines[i].point2D[0].pr;
    rl[1] = right_lines[i].point2D[1].pr;

    if(ll[0].y > ll[1].y) leftBig = 0;
    else leftBig = 1;
    if(rl[0].y > rl[1].y) rightBig = 0;
    else rightBig = 1;
    
    if(ll[leftBig].y > rl[rightBig].y) leftBiggest = true; 
    else leftBiggest = false;
    
    if(ll[Other(leftBig)].y < rl[Other(rightBig)].y) leftSmallest = true; 
    else leftSmallest = false;
    
    
    if(leftBiggest)              // biggest y-value is on the left image line
    {
      isctPointLeft[0].pr = ll[leftBig];
      Vector2 dir = Normalise(rl[1]-rl[0]);
      isctPointRight[0].pr = LineIntersectionEpipolarLine(rl[rightBig], dir, isctPointLeft[0].pr.y, &length);
    }
    else                         // biggest y-value is on the right image line
    {
      isctPointRight[0].pr = rl[rightBig];
      Vector2 dir = Normalise(ll[1]-ll[0]);
      isctPointLeft[0].pr = LineIntersectionEpipolarLine(ll[leftBig], dir, isctPointRight[0].pr.y, &length);
    }

    if(leftSmallest)             // smallest y-value is on the left image line
    {
      isctPointLeft[1].pr = ll[Other(leftBig)];
      Vector2 dir = Normalise(rl[1]-rl[0]);
      isctPointRight[1].pr = LineIntersectionEpipolarLine(rl[Other(rightBig)], dir, isctPointLeft[1].pr.y, &length);
    }
    else                         // smallest y-value is on the right image line
    {
      isctPointRight[1].pr = rl[Other(rightBig)];
      Vector2 dir = Normalise(ll[1]-ll[0]);
      isctPointLeft[1].pr = LineIntersectionEpipolarLine(ll[Other(leftBig)], dir, isctPointRight[1].pr.y, &length);
    }

    Line3D *line3d = new Line3D(left_lines[i].vs3ID, right_lines[i].vs3ID);
    if (line3d->isct3D[0].Reconstruct(stereo_cam, isctPointLeft[0], isctPointRight[0]) &&
        line3d->isct3D[1].Reconstruct(stereo_cam, isctPointLeft[1], isctPointRight[1]))
    {
      if(!Prune3DLines(line3d))
      {
        score->NewGestalt3D(line3d);
        i++;
      }
      else
      {
        left_lines.Swap(i, u-1);
        right_lines.Swap(i, u-1);
        u--;
      }
    }
    else    // move unacceptable points to the end
    {
      left_lines.Swap(i, u-1);
      right_lines.Swap(i, u-1);
      u--;
    }
  }
  matches = u;
}


/**
 * @brief Check the 3D line for plausibility. Prune if not ok.
 * @param line3d New 3D line to check.
 * @return Returns true or false
 */ 
bool StereoLines::Prune3DLines(Line3D *line3d)
{
  // Angle of the two 2D primitives
  Vector2 xDir(1., 0);
  double angle2DleftToX = SmallestAngle(Lines(vcore[LEFT], line3d->GetVs3ID(LEFT))->dir, xDir);
  double angle2DRightToX = SmallestAngle(Lines(vcore[RIGHT], line3d->GetVs3ID(RIGHT))->dir, xDir);
  
  // Angle between line and z-coordinate
  Vector3 zCoord(0., 0., 1.);
  Vector3 line((line3d->isct3D[0].p.x - line3d->isct3D[1].p.x),
	       (line3d->isct3D[0].p.y - line3d->isct3D[1].p.y),
	       (line3d->isct3D[0].p.z - line3d->isct3D[1].p.z));
  double angle3Dz = SmallestAngle(line, zCoord);
  line3d->CalculateSignificance(angle2DleftToX, angle2DRightToX, angle3Dz);

  // Relation of the length of z-component to length of x and y component
  double xL = Dot(line, Vector3(1., 0., 0.));
  double yL = Dot(line, Vector3(0., 1., 0.));
  double zL = Dot(line, Vector3(0., 0., 1.));
  double zRelation = fabs(zL/(fabs(xL)+fabs(yL)));
// printf("Verhältnis: %4.2f / %4.2f / %4.2f => %4.2f\n", xL, yL, zL, zRelation);
  
  // Wenn die Linie länger als 30cm in z-Richtung geht, dann wird sie aussortiert!
//  if(fabs(line3d->isct3D[0].p.z - line3d->isct3D[1].p.z) > 0.3) return true;

  if(SC_USE_LINE_THRESHOLDS)
  {
    if(fabs(zL/(xL+yL)) > SC_LINE_Z_TO_XY_LIMIT) return true;

    if(line3d->GetSignificance() > SC_LINE_SIG_LIMIT) return false;
    else return true;
  }
  else return false;
}


/**
 * @brief Get the unsplited lines from the vision cores.
 */
void StereoLines::GetUnsplitedLines()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    for(unsigned i = 0; i < vcore[side]->NumGestalts(Gestalt::LINE); i++)
    {
      Line *core_line = (Line*)vcore[side]->Gestalts(Gestalt::LINE, i);      
      if(!vcore[side]->use_masking || !core_line->IsMasked())
      {
	if(!core_line->HasDeferVote())     // line is end of a splitted line and has defer vote
	{
	  if(core_line->IsSplit())         // line is splitted
	  {
	    TmpLine line(core_line);
	    while(core_line->next != 0)    // get the line end point, if line is splited!
	      core_line = core_line->next;
	    line.point2D[1].p = core_line->point[1];
	    if(line.IsValid())
	    {
	      lines[side].PushBack(line);
	    }
	  }
	  else                             // line is not splitted
	  {
	    TmpLine line(core_line);
	    if(line.IsValid())
	    {
	      lines[side].PushBack(line);
	    }
	  }
	}
      }
    }
    if(pPara.pruning)
      for(unsigned i = 0; i < lines[side].Size(); i++)
	lines[side][i].RePrune(pPara.offsetX, pPara.offsetY, pPara.scale);
    for(unsigned i = 0; i < lines[side].Size(); i++)
      lines[side][i].Rectify(stereo_cam, side);
//     for(unsigned i = 0; i < lines[side].Size(); i++)
//       lines[side][i].Refine();								// TODO Refine lines???
      
  }
}


/**
 * @brief Clear all arrays.
 */
void StereoLines::ClearResults()
{
  lines[LEFT].Clear();
  lines[RIGHT].Clear();
  lineMatches = 0;
}


/**
 * @brief Match and calculate 3D lines from 2D lines.
 */
void StereoLines::Process()
{
  // Load the unsplitted lines instead of the split lines.
  GetUnsplitedLines();

  // match lines with MSLD descriptor
  std::vector< std::pair<unsigned, unsigned> > matches;  // line matches <LEFT/RIGHT>
  std::vector< std::vector<float> > descriptors[2];      // descriptor left/right for every line
  
  struct timespec start, endMSLD, end;
  clock_gettime(CLOCK_REALTIME, &start);
  
  // Calculate MSLD descriptor
  ProcessMSLD(descriptors);
  clock_gettime(CLOCK_REALTIME, &endMSLD);
  cout<<"StereoLines::Process: Time to create MSLD [s]: " << timespec_diff(&endMSLD, &start) << endl;

  // Match lines
  MatchLines(descriptors[0], descriptors[1], matches);
  lineMatches = matches.size();
  clock_gettime(CLOCK_REALTIME, &end);
  cout<<"StereoLines::Process: Time to create MSLD and match lines [s]: " << timespec_diff(&end, &start) << endl;

  // Sort results  
  SortLines(lines[LEFT], lines[RIGHT], matches);
  lineMatches = matches.size();
  
  // do stereo matching and depth calculation
  Calculate3DLines(lines[LEFT], lines[RIGHT], lineMatches);
  // printf("StereoLines::Process: MatchedLines after 3D calculation: %u\n", lineMatches);
}


/**
 * @brief Match and calculate 3D lines from 2D lines.
 * @param side LEFT/RIGHT image of stereo pair.
 */
void StereoLines::Process(int oX, int oY, int sc)
{
  pPara.pruning = true;
  pPara.offsetX = oX;
  pPara.offsetY = oY;
  pPara.scale = sc;
  Process();
  pPara.pruning = false;
}


}








