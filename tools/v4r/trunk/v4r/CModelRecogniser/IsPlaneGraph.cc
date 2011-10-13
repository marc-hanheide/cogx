/**
 * $Id$
 *
 * Similarity graph of planes for an image sequence 
 *
 */


#include "IsPlaneGraph.hh"

#define DEBUG


namespace P 
{



/********************** IsPlaneGraph ************************
 * Constructor/Destructor
 */
IsPlaneGraph::IsPlaneGraph(cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                         Parameter _param)
 : param(_param)
{
  matcher = descMatcher;
  selmat = new SelectMatchesMRF(SelectMatchesMRF::Parameter(param.width, param.height));
  hf = new MergePlanesHF(MergePlanesHF::Parameter(param.width, param.height, param.sigmaDistF, param.sigmaDistH));
  svm = new SVMPredictor(1, "./logGT/GTTracks-GRASPBox05.txt.model");
}

IsPlaneGraph::~IsPlaneGraph()
{
}




/************************** PRIVATE ************************/

void IsPlaneGraph::SetNNR(vector<vector<cv::DMatch> > &matches, vector<int> &selectedMatches)
{
  selectedMatches.resize(matches.size());

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (matches[i].size()>=2)
    {
      if (matches[i][0].distance/matches[i][1].distance < param.nnRatio && 
          matches[i][0].distance<param.thrDesc) 
        selectedMatches[i]=0;
      else selectedMatches[i]=-1;  
    }
    else if (matches[i].size()==1 && matches[i][0].distance<param.thrDesc) 
        selectedMatches[i]=0;
      else selectedMatches[i]=-1;
  }
}

/**
 * Get points
 */
void IsPlaneGraph::GetPoints(vector<cv::Ptr<PKeypoint> > &queryKeys, 
      vector<cv::Ptr<PKeypoint> > &trainKeys, vector<vector<cv::DMatch> > &matches, 
      vector<int> &selectedMatches, vector<cv::Point2f> &queryPts, vector<cv::Point2f> &trainPts)
{
  queryPts.clear();
  trainPts.clear();

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (selectedMatches[i]>=0)
    {
      cv::DMatch &ma = matches[i][selectedMatches[i]];

      trainPts.push_back(cv::Point2f(trainKeys[ma.trainIdx]->pt.x, trainKeys[ma.trainIdx]->pt.y));
      queryPts.push_back(cv::Point2f(queryKeys[ma.queryIdx]->pt.x, queryKeys[ma.queryIdx]->pt.y));
    }
  }
}


/**
 * MatchPlanes
 */
void IsPlaneGraph::MatchPlanes(Plane &trainPlane, Plane &queryPlane, int &numMatches)
{
  int inlIdx;
  double dist;
  cv::Point2d pt;
  cv::Mat_<double> H;
  vector<uchar> mask;
  vector<cv::Point2f> pts1, pts2;
  double sqrInlDist = 2*PMath::Sqr( param.inlDist);

  matches.clear();
  matcher->knnMatch(queryPlane.descriptors, trainPlane.descriptors, matches, param.kMatches);
  //selmat->Operate(queryPlane.keys, trainPlane.keys, matches, selectedMatches);
  SetNNR(matches,selectedMatches);

  GetPoints(queryPlane.keys, trainPlane.keys, matches, selectedMatches, pts2, pts1);

  numMatches=0;

  if (pts1.size()>4)
  {
    H = findHomography(cv::Mat(pts1), cv::Mat(pts2), mask, CV_RANSAC, param.inlDist);

    for (unsigned i=0; i<matches.size(); i++)
    {
      inlIdx = -1;
      for (unsigned j=0; j<matches[i].size(); j++)
      {
        cv::DMatch &ma = matches[i][j];
        PHom::MapPoint(&trainPlane.keys[ma.trainIdx]->pt.x, &H(0,0), &pt.x);
        if (PVec::DistanceSqr2(&pt.x, &queryPlane.keys[ma.queryIdx]->pt.x) < sqrInlDist)
        {
          inlIdx = j;
          break;
        }
      }
      if (inlIdx>=0)
      {
        //selectedMatches[i]=inlIdx;
        numMatches++;
      }
      //else selectedMatches[i]=-1;
    }
  }
}

/**
 * LinkPlanes
 */
void IsPlaneGraph::LinkPlanes(int idxSrcFrame, int idxDstFrame)
{
  int numMatches;

  GraphFrame &srcFrame = *graphPlanes[idxSrcFrame];
  GraphFrame &dstFrame = *graphPlanes[idxDstFrame];

  for (unsigned i=0; i<srcFrame.planes.size(); i++)
  {
    for (unsigned j=0; j<dstFrame.planes.size(); j++)
    {
      MatchPlanes(*srcFrame.planes[i], *dstFrame.planes[j], numMatches);

      srcFrame.planes[i]->fwTrack[dstFrame.planes[j]] = numMatches;
      dstFrame.planes[j]->bwTrack[srcFrame.planes[i]] = numMatches;
    }
  }
}







/************************** PUBLIC *************************/


/**
 * Track interest points and detect planes (homographies)
 * based on iterative model selection
 */
void IsPlaneGraph::Create(vector< cv::Ptr<Plane> > &planes, unsigned idFrame)
{
  graphPlanes.push_back(new GraphFrame(planes, idFrame, graphPlanes.size()));

  hf->dbg = dbg;
  hf->Operate(graphPlanes.back()->planes);

  if (graphPlanes.size() >= 2)
  {
    LinkPlanes(graphPlanes.size()-2, graphPlanes.size()-1);

    #ifdef DEBUG
    PrintInfo(*graphPlanes.back());
    cout<<"Number of frames in graph: "<<graphPlanes.size()<<endl;
    #endif
  }
}






/********************** STUFF for DISPLAYING *******************/

/**
 * SetPlaneColour
 */
void IsPlaneGraph::SetPlaneColour(GraphFrame &frame)
{
  if (frame.idx==0)
    return;

  double conf, max;
  unsigned idx;
  vector<bool> srcUsed(graphPlanes[frame.idx-1]->planes.size(),false);
  std::map<cv::Ptr<Plane>, unsigned>::iterator it;
  cv::Ptr<Plane> srcPlane, dstPlane;
  vector<double> vec(3);
  std::vector<double> prob;

  do{
    max=0;
    for (unsigned j=0; j<frame.planes.size(); j++)
    {
      if (frame.planes[j]->col==cv::Scalar(0))
      {
        for (it=frame.planes[j]->bwTrack.begin(); it!=frame.planes[j]->bwTrack.end(); it++)
        {
          if (!srcUsed[it->first->idx])
          {
            /*GetConf(it->second, it->first->keys.size(), frame.planes[j]->keys.size(), conf);
            if (conf > max)
            {
              max = conf;
              srcPlane = it->first;
              dstPlane = frame.planes[j]; 
            }*/

            GetConf(it->second, it->first->keys.size(), frame.planes[j]->keys.size(), vec[0]);
            GetPartOf(vec[0], it->second, it->first->keys.size(), frame.planes[j]->keys.size(), vec[1], vec[2]);
            if(svm->GetResult(1, vec, prob)==true)
            {
              frame.planes[j]->col = it->first->col;
              max = prob[0]; 
              srcPlane = it->first;
              dstPlane = frame.planes[j];
              #ifdef DEBUG
              cout<<srcPlane->id<<"->"<<dstPlane->id<<": conf="<<vec[0]<<", prob="<<prob[0]<<endl;
              #endif
            }
          }
        }
      }
    }
    if (max>0.000001)
    {
      //srcUsed[srcPlane->idx] = true;
      //dstPlane->col = srcPlane->col;
      #ifdef DEBUG
      //cout<<srcPlane->id<<"->"<<dstPlane->id<<": max="<<max<<endl;
      #endif
    }
  }while (max > 0.0000001);

  for (unsigned i=0; i<frame.planes.size(); i++)
  {
    Plane &plane = *frame.planes[i];

    if (plane.col == cv::Scalar(0))
    {
      plane.col = CV_RGB(rand()%180,rand()%180,rand()%180);
    }
  }
}

/**
 * Draw
 */
void IsPlaneGraph::Draw(cv::Mat &img, unsigned detail)
{
  if (graphPlanes.size()==0)
    return;

  string tag("id=");

  GraphFrame &frame = *graphPlanes.back();

  SetPlaneColour(frame);

  // show keypoints and motion vector
  for (unsigned i=0; i<frame.planes.size(); i++)
  {
    Plane &plane = *frame.planes[i];

    //plane.col = CV_RGB(rand()%180,rand()%180,rand()%180);
    
    for (unsigned j=0; j<plane.keys.size(); j++)
    {
      //PKeypoint::Draw(img,*plane.keys[j],CV_RGB(255,255,255));
      cv::circle(img, plane.keys[j]->pt, 2, plane.col, 2);
      if (detail>=2 && plane.lastKeys.size()==plane.keys.size())
        cv::line(img, plane.lastKeys[j]->pt, plane.keys[j]->pt, CV_RGB(255,255,255));
    }
  }

  // show text for plane tracks
  for (unsigned i=0; i<frame.planes.size(); i++)
  {
    Plane &plane = *frame.planes[i];

    if (detail>=1)
    {
      cv::putText(img, tag+toString(plane.id,0), plane.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255,255,255), 2);
      cv::putText(img, tag+toString(plane.id,0), plane.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, plane.col, 1);
    }
  }
}

/**
 * DrawText
 */
void IsPlaneGraph::DrawText(cv::Mat &img, unsigned idx)
{
  unsigned z=1;
  double conf, lPartofC, cPartofL;
  GraphFrame &frame = *graphPlanes.back();

  if (idx >= frame.planes.size())
    return;

  string txLink;
  Plane &plane = *frame.planes[idx];
  cv::Point2d pt = plane.center; 
  std::map<cv::Ptr<Plane>, unsigned>::iterator it;
  std::map<cv::Ptr<Plane>, double>::iterator itd;

  for (it=plane.bwTrack.begin(); it!=plane.bwTrack.end(); it++)
  {
    if (it->second > 0)
    {
      GetConf(it->second, it->first->keys.size(), plane.keys.size(), conf);
      GetPartOf(conf, it->second, it->first->keys.size(), plane.keys.size(), lPartofC, cPartofL);
      
      txLink = "link id="+toString(it->first->id,1)+", c="+toString(conf,2);
      txLink += ", LofC="+toString(lPartofC,1)+", CofL="+toString(cPartofL,1);
      pt.y += z*17;

      cv::putText(img, txLink, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 2);
      cv::putText(img, txLink, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, plane.col, 1);
    }
  }

  // display links of merged planes a frame
  cv::Point2d c;
  for (itd=plane.motLink.begin(); itd!=plane.motLink.end(); itd++)
  {
    cv::line(img, plane.center, itd->first->center, CV_RGB(155,155,155),1);
    PVec::MidPoint2(&plane.center.x, &itd->first->center.x, &c.x);
    cv::putText(img, toString(itd->second), c, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 2);
  }
}

/**
 * DrawText
 */
void IsPlaneGraph::GTSaveTracks(cv::Mat &img, unsigned idx)
{
  unsigned z=1;
  double conf, lPartofC, cPartofL;
  GraphFrame &frame = *graphPlanes.back();

  if (idx >= frame.planes.size())
    return;

  string txLink;
  Plane &plane = *frame.planes[idx];
  cv::Point2d pt = plane.center; 
  std::map<cv::Ptr<Plane>, unsigned>::iterator it;
  std::map<cv::Ptr<Plane>, double>::iterator itd;
  
  ofstream out("logGT/GTTracks.txt",ios_base::app);

  for (it=plane.bwTrack.begin(); it!=plane.bwTrack.end(); it++)
  {
    if (it->second > 0)
    {
      GetConf(it->second, it->first->keys.size(), plane.keys.size(), conf);
      GetPartOf(conf, it->second, it->first->keys.size(), plane.keys.size(), lPartofC, cPartofL);
      
      txLink = "link id="+toString(it->first->id,1)+", c="+toString(conf,2);
      txLink += ", LofC="+toString(lPartofC,1)+", CofL="+toString(cPartofL,1);
      pt.y += z*17;

      cv::putText(img, txLink, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,255,255), 2);
      cv::putText(img, txLink, pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, plane.col, 1);

      cv::imshow("Image", img);
      int key = cv::waitKey(0);

      if (((char)key) == '0')
        out<<"0 0:"<<conf<<" 1:"<<lPartofC<<" 2:"<<cPartofL<<'\n';
      else if (((char)key) == '1') out<<"1 0:"<<conf<<" 1:"<<lPartofC<<" 2:"<<cPartofL<<'\n';
    }
  }

  out.close();
}

/**
 * PrintInfo
 */
void IsPlaneGraph::PrintInfo(GraphFrame &frame)
{
  double conf, lPartofC, cPartofL;
  std::map<cv::Ptr<Plane>, unsigned>::iterator it;
  std::map<cv::Ptr<Plane>, double>::iterator itd;

  for (unsigned i=0; i<frame.planes.size(); i++)
  {
    Plane &plane = *frame.planes[i];

    cout<<"Plane id="<<plane.id<<":"<<endl;

    for (it=plane.bwTrack.begin(); it!=plane.bwTrack.end(); it++)
    {
      if (it->second > 0)
      {
        GetConf(it->second, it->first->keys.size(), plane.keys.size(), conf);
        GetPartOf(conf, it->second, it->first->keys.size(), plane.keys.size(), lPartofC, cPartofL);
        cout<<(" link id="+toString(it->first->id,1)+", c="+toString(conf,2)+
              ", LofC="+toString(lPartofC,1)+", CofL="+toString(cPartofL,1))<<endl;
      }
    }
  }
  cout<<"--"<<endl;
  for (unsigned i=0; i<frame.planes.size(); i++)
  {
    Plane &plane = *frame.planes[i];

    for (itd=plane.motLink.begin(); itd!=plane.motLink.end(); itd++)
    {
      cout<<itd->first->id<<"/"<<plane.id<<": pH="<<itd->second<<endl;
    }
  }
  cout<<"--"<<endl;
}

} //-- THE END --







