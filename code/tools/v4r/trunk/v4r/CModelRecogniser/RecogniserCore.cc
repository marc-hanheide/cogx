/**
 * $Id$
 */


#include "RecogniserCore.hh"

#define DEBUG


namespace P 
{

static const bool CmpCntIter(const pair<unsigned, map<int, vector<cv::DMatch> >::iterator> &a,
                             const pair<unsigned, map<int, vector<cv::DMatch> >::iterator> &b)
{
  return (a.first > b.first);
}

static const bool CmpConf(const pair<double, unsigned> &a, const pair<double, unsigned> &b)
{
  return (a.first > b.first);
}



/********************** RecogniserCore ************************
 * Constructor/Destructor
 */
RecogniserCore::RecogniserCore(cv::Ptr<KeypointDetector> &keyDetector,
                               cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                               cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                               Parameter p)
 : param(p)
{
  detector = keyDetector;
  extractor = descExtractor;
  matcher = descMatcher;

  SetCameraParameter(p.intrinsic,p.distCoeffs);

  //codebook = new P::MeanShiftCodebook(matcher,MeanShiftCodebook::Parameter(p.thrDesc, p.sigmaDesc, p.nnRatio));
  codebook = new P::MeanCodebook(matcher,MeanCodebook::Parameter(p.thrDesc, p.sigmaDesc, p.nnRatio));

}

RecogniserCore::~RecogniserCore()
{
}




/************************** PRIVATE ************************/



/**
 * Mean shift clustering of matches
 */
void RecogniserCore::ClusterMatches(unsigned oidx, vector<cv::DMatch> &matches, vector<vector< cv::DMatch > > &clusters)
{
  double angle, scale;
  vector<cv::Point2d> votes;
  vector<vector<unsigned> > cls;
  vector<cv::DMatch> tmpMatches;

  cv::Ptr<MeanShiftBase> mshift;
  if (param.useFams)
    mshift = new MeanShiftFAMS();
  else mshift = new MeanShift();

  #ifdef DEBUG
  srand(1);
  #endif

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    votes.push_back(cv::Point2d());
    PKeypoint::Vote(*recModels[oidx]->views[ma.imgIdx]->keys[ma.trainIdx],*keys[ma.queryIdx], 
                    recModels[oidx]->views[ma.imgIdx]->center, votes.back(), angle, scale);
  }

  cv::Mat_<double> samples(votes.size(),2,&votes[0].x);

  mshift->Cluster(samples,cls);

  clusters.clear();
  clusters.reserve(cls.size());
  for (unsigned i=0; i<cls.size(); i++)
  {
    if (cls[i].size()>=5)
    {
      clusters.push_back( vector< cv::DMatch >() );
      for (unsigned j=0; j<cls[i].size(); j++)
        clusters.back().push_back(matches[cls[i][j]]);
    }
  }

  #ifdef DEBUG
  if (!dbg.empty())
  {
    //cv::Mat tmp;
    //dbg.copyTo(tmp);
    for (unsigned j=0; j<cls.size(); j++)
    {
      cv::Scalar col = CV_RGB(rand()%255,rand()%255,rand()%255);
      for (unsigned k=0; k<cls[j].size(); k++)
      {
        cv::line(dbg, keys[matches[cls[j][k]].queryIdx]->pt, votes[cls[j][k]], CV_RGB(255,255,255));
        cv::circle(dbg, votes[cls[j][k]], 2, col, 2, 8, 0);
      }
    }
    //cv::imshow("Image",tmp);
    //cv::waitKey(0);
  }
  #endif
}

/**
 * GetRandIdx
 */
void RecogniserCore::GetRandIdx(unsigned size, unsigned num, vector<unsigned> &idx)
{
  unsigned temp;
  idx.clear();
  for (unsigned i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(Contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * Count inlier
 */
void RecogniserCore::CountInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, CModel &model,
      vector<vector<cv::DMatch> > &matches, Pose &pose, double &sig)
{
  sig=0; 
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];

  for (unsigned j=0; j<matches.size(); j++)
  {
    for (unsigned i=0; i<matches[j].size(); i++)
    {
      cv::DMatch &ma = matches[j][i];

      if (!model.views[ma.imgIdx]->keys[ma.trainIdx]->pos.empty())
      {
        PMat::MulAdd3(pose.R.ptr<double>(), &model.views[ma.imgIdx]->keys[ma.trainIdx]->pos->pt.x, 
                      pose.t.ptr<double>(), pos);

        if (pos[2]>0)
        {
          if (param.distCoeffs.empty())
            ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);
          else ProjectPoint2Image(pos,param.intrinsic.ptr<double>(), param.distCoeffs.ptr<double>(), pt);

          dist = PVec::DistanceSqr2(pt, &queryKeys[ma.queryIdx]->pt.x);

          if (dist < sqrInlDist)
            sig+=1.;
        }
        else
        {
          sig=0;
          return;
        }
      }
    }
  }
}

/**
 * Count inlier
 */
void RecogniserCore::MarkInlier(unsigned oidx, vector<vector<cv::DMatch> > &matches, ObjectLocation &obj, unsigned &numTotal, unsigned &numInlier)
{
  if (!obj.HavePose()){
    numTotal=5, numInlier=0;
    return;
  }

  numTotal = numInlier = 0;
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];
  CModel &model = *recModels[oidx];
  Point3dProjs::nbcnt++;
  vector<unsigned> cntInlPerView(matches.size(),0);

  for (unsigned j=0; j<matches.size(); j++)
  {
    for (unsigned i=0; i<matches[j].size(); i++)
    {
      cv::DMatch &ma = matches[j][i];
      PKeypoint &key = *model.views[ma.imgIdx]->keys[ma.trainIdx];

      if (!key.pos.empty())
      {
        PMat::MulAdd3(obj.pose.R.ptr<double>(), &key.pos->pt.x, obj.pose.t.ptr<double>(), pos);

        if (param.distCoeffs.empty())
          ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);
        else ProjectPoint2Image(pos,param.intrinsic.ptr<double>(), param.distCoeffs.ptr<double>(), pt);

        dist = PVec::DistanceSqr2(pt, &keys[ma.queryIdx]->pt.x);

        if (dist < sqrInlDist)
        {
          keys[ma.queryIdx]->nb = PKeypoint::nbcnt; 
          cntInlPerView[j]++;

          if (key.pos->nb != Point3dProjs::nbcnt)
            numInlier++;
        }
        //if (key.pos->nb != Point3dProjs::nbcnt)
        //{
        //  numTotal++;
        //  key.pos->nb=Point3dProjs::nbcnt;
        //}
      }
    }
  }

  // get max view...
  unsigned max=0, idx=UINT_MAX;
  for (unsigned i=0; i<cntInlPerView.size(); i++)
    if (cntInlPerView[i] > max)
    {
      max = cntInlPerView[i];
      idx = i;
    }

  if (idx!=UINT_MAX)
  {
    numTotal = model.views[matches[idx][0].imgIdx]->keys.size();
    if(numTotal < numInlier) numTotal=numInlier;
  }
  else numTotal = UINT_MAX;
}

/**
 * GetInlier
 */
void RecogniserCore::CountInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, 
      vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, double H[9], double &inl)
{
  inl = 0;
  cv::Point2d pt;
  double inlDistSqr = PMath::Sqr(param.inlDistAff);
 
  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D())
    {
      PHom::MapPoint(&trainKeys[ma.trainIdx]->pt.x, H, &pt.x);

      if (PVec::DistanceSqr2(&pt.x,&queryKeys[ma.queryIdx]->pt.x) < inlDistSqr)
        inl++;
    }
  }
}

/**
 * GetInlier
 */
void RecogniserCore::GetInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, 
      vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, double H[9], 
      vector<PKeypoint*> &querySelected, vector<PKeypoint*> &trainSelected)
{
  cv::Point2d pt;
  double inlDistSqr = PMath::Sqr(param.inlDistAff);
  Point3dProjs::nbcnt++;
  
  querySelected.clear();
  trainSelected.clear();
 
  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D() && Point3dProjs::nbcnt!=trainKeys[ma.trainIdx]->pos->nb)
    {
      PHom::MapPoint(&trainKeys[ma.trainIdx]->pt.x, H, &pt.x);

      if (PVec::DistanceSqr2(&pt.x,&queryKeys[ma.queryIdx]->pt.x) < inlDistSqr)
      {
        querySelected.push_back(&(*queryKeys[ma.queryIdx]));
        trainSelected.push_back(&(*trainKeys[ma.trainIdx]));
        trainKeys[ma.trainIdx]->pos->nb = Point3dProjs::nbcnt;
      }
    }
  }
}

/**
 * copy unexpained points
 */
/*int RecogniserCore::GetUnmarked(MatchPairs &mp, MatchPairs &unexp)
{
  int num=0;
  PKeypoint::nb2cnt++;
  unexp.ptsImage.clear();
  unexp.ptsModel.clear();
  unexp.idxViews.clear();
  unexp.idxObject = mp.idxObject;

  for (unsigned i=0; i<mp.ptsImage.size(); i++)
  {
    unexp.ptsImage.push_back(vector<PKeypoint*>());
    unexp.ptsModel.push_back(vector<PKeypoint*>());
    unexp.idxViews.push_back(mp.idxViews[i]);
    for (unsigned j=0; j<mp.ptsImage[i].size(); j++)
    {
      if (mp.ptsImage[i][j]->nb != PKeypoint::nbcnt)
      {
        unexp.ptsImage.back().push_back(mp.ptsImage[i][j]);
        unexp.ptsModel.back().push_back(mp.ptsModel[i][j]);
        if (mp.ptsImage[i][j]->nb2 != PKeypoint::nb2cnt)
        {
          num++;
          mp.ptsImage[i][j]->nb2 = PKeypoint::nb2cnt;
        }

      }
    }
    if (unexp.ptsImage.back().size()<3)  //aff thr
    {
      unexp.ptsImage.pop_back();
      unexp.ptsModel.pop_back();
      unexp.idxViews.pop_back();
    }
  }

  return num;
}*/

/**
 * Robust estimation of object poses
 */
bool RecogniserCore::LoRansac(unsigned oidx, vector<vector<cv::DMatch> > &matches, int numTotal, ObjectLocation &obj)
{
  if (numTotal < 5) return false; 

  // int ransac
  int k=0;
  double sig=3., svSig=0.;
  double eps = sig/(double)numTotal;
  vector<unsigned> idxKeys;
  Pose tmpPose(true);
  double *d, rod[3];
  CvMat matRod = cvMat(3,1,CV_64F, rod);
  CvMat matt = tmpPose.t;
  CvMat matR = tmpPose.R;
  CvMat matIntr = param.intrinsic;
  CvMat matDistCoeffs;
  if (param.distCoeffs.empty()){ 
    double dists[4];
    matDistCoeffs = cvMat(1,4,CV_64F,dists);
    cvSetZero(&matDistCoeffs);
  }else matDistCoeffs=param.distCoeffs;

  double pts3D[5*3];
  double pts2D[5*2];
  CvMat matPts3D = cvMat( 5, 3, CV_64F, pts3D );
  CvMat matPts2D = cvMat( 5, 2, CV_64F, pts2D );
  double H[9];
  cv::Mat matH = cv::Mat(3,3,CV_64F, H);
  obj.H = cv::Mat(3,3,CV_64F);
  bool ok;
  unsigned idxView;
  CModel &model = *recModels[oidx];
  vector<PKeypoint*> trainSelected, querySelected;

  //ransac pose
  while (pow(1. - pow(eps,3), k) >= param.etaRansac && k < param.maxRandTrials)
  {
    do{ 
      idxView = PMath::ExpSelect(matches.size()-1);
    }while(matches[idxView].size()<=5);

    vector<cv::DMatch> &mas = matches[idxView];
    vector<cv::Ptr<PKeypoint> > &mKeys = model.views[matches[idxView][0].imgIdx]->keys;

    GetRandIdx(mas.size(), 3, idxKeys);
    if (PHom::ComputeAff(&mKeys[mas[idxKeys[0]].trainIdx]->pt.x, &mKeys[mas[idxKeys[1]].trainIdx]->pt.x,
                         &mKeys[mas[idxKeys[2]].trainIdx]->pt.x,
                         &keys[mas[idxKeys[0]].queryIdx]->pt.x, &keys[mas[idxKeys[1]].queryIdx]->pt.x,
                         &keys[mas[idxKeys[2]].queryIdx]->pt.x, H))
    {
      CountInlier(keys, mKeys, mas, H, sig);

      if (sig > svSig && sig > 5)
      {
        svSig = sig;
        matH.copyTo(obj.H);

        GetInlier(keys, mKeys, mas, H, querySelected, trainSelected);
      
        if (querySelected.size()>5)  
        {
          for (unsigned i=0; i<param.numLoTrials; i++)
          {
            GetRandIdx(querySelected.size(), 5, idxKeys);

            for (unsigned j=0; j<5; j++)
            {
              d =  &trainSelected[ idxKeys[j] ]->pos->pt.x;

              pts3D[3*j+0] = d[0];
              pts3D[3*j+1] = d[1];
              pts3D[3*j+2] = d[2];

              d = &querySelected[ idxKeys[j] ]->pt.x;
              pts2D[2*j+0] = d[0];
              pts2D[2*j+1] = d[1];
            }

            cvFindExtrinsicCameraParams2(&matPts3D, &matPts2D, &matIntr, &matDistCoeffs, &matRod, &matt);
            cvRodrigues2(&matRod, &matR);
            CountInlier(keys, *recModels[oidx], matches, tmpPose, sig);

            if (sig > svSig)
            {
              svSig = sig;
              tmpPose.copyTo(obj.pose);
            }
          }
        }

        eps = svSig / (double)numTotal;
      }
    }
    k++;
  }

  #ifdef DEBUG
  cout<<"Number of lo-ransac trials: "<<k<<", inl="<<svSig<<"/"<<numTotal<<endl;
  #endif
  if (svSig >= 5)
    return true;
  return false;
}

/**
 * SortMatchesToViews
 */
bool RecogniserCore::SortMatchesToViews(vector<cv::DMatch> &matches, vector<vector<cv::DMatch> > &sorted,unsigned &cnt)
{
  cnt=0;
  vector< pair<unsigned, map<int, vector<cv::DMatch> >::iterator > > cntIter;
  map<int, vector<cv::DMatch> > viewMatches;
  map<int, vector<cv::DMatch> >::iterator it;

  // sort to views
  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (keys[ma.queryIdx]->nb != PKeypoint::nbcnt) 
    {
      viewMatches[ma.imgIdx].push_back(ma);
      cnt++;
    }
  }

  // sort depending on number of matches
  for (it=viewMatches.begin(); it!=viewMatches.end(); it++)
    cntIter.push_back( make_pair(it->second.size(), it) );

  std::sort(cntIter.begin(), cntIter.end(), CmpCntIter);

  // copy back
  sorted.resize(cntIter.size());
  for (unsigned i=0; i<cntIter.size(); i++)
  {
    sorted[i] = cntIter[i].second->second;
  }
  
  if (sorted.size()>0 && sorted[0].size() > 5)
    return true;
  return false;
}


/**
 * Detect objects
 */
bool RecogniserCore::DetectObjects(unsigned oidx, vector< vector<cv::DMatch> > &clusters, vector<ObjectLocation> &objects)
{
  unsigned numInl, numTotal;
  ObjectLocation obj;
  ProbModel probModel;
  vector<vector<cv::DMatch> > sortedMatches;

  bool ok = false;
  for (unsigned i=0; i<clusters.size(); i++)
  {
    while( SortMatchesToViews(clusters[i], sortedMatches, numTotal) )
    {
      if ( LoRansac(oidx, sortedMatches, numTotal, obj) )
      {
        MarkInlier(oidx, sortedMatches, obj, numTotal, numInl);

        obj.conf = probModel.ConfToProb( ((double)numInl)/(double)numTotal );

        if (obj.conf > 0.0000001) //0.001)
        {
          obj.idView = sortedMatches[0][0].imgIdx;
          obj.idObject = recModels[oidx]->id;
          objects.push_back(obj);
          ok=true;
        }
        else break;
      }
      else break;
    }
  }
  return ok;
}

/**
 * Project model points to current image and count inliers
 * num_supporting_points / num_points_of_recognised_view  (no matches used)
 */
/*double RecogniserCore::ComputeConfidence(vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned inl=0, cnt=0;
  double sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];
  double minDist, dist;

  for (unsigned i=0; i<model.size(); i++)
  {
    if (model[i]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &model[i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

      minDist = DBL_MAX;
      for (unsigned j=0; j<keys.size(); j++)
      {
        dist = PVec::DistanceSqr2(pt,&keys[j]->pt.x);
        if (dist  < minDist)
          minDist = dist;
      }

      if (minDist < sqrInlDist)
       inl++;
      cnt++;
    }
  }

  if (cnt>0)
    return ((double)inl)/(double)cnt;
  return 0.;
}*/

/**
 * Returns an iterator to the object with the most weighted matches
 */
map<unsigned, vector<cv::DMatch> >::iterator RecogniserCore::SelectMaxConf(map<unsigned, vector<cv::DMatch> > &ma)
{
  unsigned cnt, maxCnt;
  double weight, maxWeight=0;
  map<unsigned, vector<cv::DMatch> >::iterator it, svIt=ma.end();

  for (it=ma.begin(); it!=ma.end(); it++)
  {
    cnt=0;
    weight=0;
    for (unsigned i=0; i<it->second.size(); i++)
    {
      cv::DMatch &m = it->second[i];
      if (keys[m.queryIdx]->nb != PKeypoint::nbcnt) 
      {
        cnt++;
        weight += m.distance;
      }
    }
    if (weight>maxWeight && cnt>5 && weight>0)
    {
      maxWeight=weight;
      svIt = it;
      maxCnt=cnt;
    }
  }
  //cout<<"id="<<svIt->first<<": maxCnt="<<maxCnt<<", maxWeight="<<maxWeight<<endl;

  return svIt;
}




/************************** PUBLIC *************************/

/**
 * Delete the models and clear the vocabulary tree
 */
void RecogniserCore::Clear()
{
  codebook->clear();
  recModels.clear();
}

/**
 * Add model for recognition
 */ 
unsigned RecogniserCore::AddModel(cv::Ptr<CModel> &model)
{
  recModels.push_back(model);
  unsigned idx = recModels.size()-1;

  for (unsigned i=0; i<recModels.back()->views.size(); i++)
    codebook->InsertView(idx,i, recModels);

  /*for (unsigned i=0; i<codebook->cbEntries.size(); i++)
  if (fabs(sqrt(codebook->cbEntries[i]->sqrThr)-0.4) > 0.0001)
  cout<<sqrt(codebook->cbEntries[i]->sqrThr)<<" ";*/

  return idx;
}

/**
 * Load a vocabulary tree from file
 */
/*void RecogniserCore::LoadVocabularyTree(const string &filename)
{
  cout<<"That's the codebook version! You do not need a vocabulary tree!"<<endl;
}*/

/**
 * Recognise
 */
void RecogniserCore::Recognise(const cv::Mat &image, vector<ObjectLocation> &objects, cv::Mat mask)
{
  #ifdef DEBUG
  cout<<"------------------------- RECOGNITION ---------------------------"<<endl;
  struct timespec start1, end1;
  clock_gettime(CLOCK_REALTIME, &start1);
  #endif

  objects.clear();
  grayImage = image;
  if( image.type() != CV_8U ) cv::cvtColor( image, grayImage, CV_BGR2GRAY );

  // detect keypoints and descriptors of current frame
  detector->Detect(grayImage, keys, mask);
  PKeypoint::ConvertToCv(keys,cvKeys);
  extractor->compute(grayImage, cvKeys, descriptors);
  #ifdef DEBUG
  //if (!dbg.empty()) for(unsigned i=0;i<keys.size();i++) PKeypoint::Draw(dbg,*keys[i],CV_RGB(255,0,0));
  #endif

  // match with vocabulary tree
  unsigned idx;
  map<unsigned, vector<cv::DMatch> > matches;
  map<unsigned, vector<cv::DMatch> >::iterator it;
  vector< vector<cv::DMatch> > clusters;
  vector<ObjectLocation> tmpObjects;

  #ifdef DEBUG
  struct timespec start2, end2;
  clock_gettime(CLOCK_REALTIME, &start2);
  #endif

  codebook->QueryObjects(descriptors, matches);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end2);
  cout<<"Time matching [s]: "<<PMath::timespec_diff(&end2, &start2)<<endl;
  /*for (it=matches.begin(); it!=matches.end(); it++)
  {
    for (unsigned i=0; i<it->second.size(); i++)
    {
      cv::DMatch &ma = it->second[i];
      PKeypoint &trainKey = *recModels[it->first]->views[ma.imgIdx]->keys[ma.trainIdx];
      PKeypoint &queryKey = *keys[ma.queryIdx];
      cv::line(dbg, trainKey.pt, queryKey.pt, CV_RGB(255,255,255),1);
      cv::circle(dbg, queryKey.pt, 2, CV_RGB(0,0,255), 2);
    }
  }*/
  #endif

  // verify matches and compute pose
  PKeypoint::nbcnt++;
  while( (it=SelectMaxConf(matches)) != matches.end() )
  {
    #ifdef DEBUG
    struct timespec start5, end5;
    clock_gettime(CLOCK_REALTIME, &start5);
    #endif

    ClusterMatches(it->first, it->second, clusters);

    #ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &end5);
    cout<<"Time clustering [s]: "<<PMath::timespec_diff(&end5, &start5)<<endl;
    struct timespec start6, end6;
    clock_gettime(CLOCK_REALTIME, &start6);
    #endif

    if(!DetectObjects(it->first, clusters, tmpObjects))
      break;

    #ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &end6);
    cout<<"Time ransac [s]: "<<PMath::timespec_diff(&end6, &start6)<<endl;
    #endif
  };


  //sort objects and return
  vector<pair<double,unsigned> > confIdx(tmpObjects.size());

  for (unsigned i=0; i<tmpObjects.size(); i++)
   confIdx[i] = make_pair(tmpObjects[i].conf, i);

  std::sort(confIdx.begin(),confIdx.end(), CmpConf);

  for (unsigned i=0; i<confIdx.size(); i++)
    objects.push_back(tmpObjects[confIdx[i].second]);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time recogniser object [s]: "<<PMath::timespec_diff(&end1, &start1)<<endl;
  for (unsigned i=0; i<objects.size(); i++)
    cout<<"object id="<<objects[i].idObject<<", vid="<<objects[i].idView<<", conf="<<objects[i].conf<<endl;
  cout<<"objects.size()="<<objects.size()<<endl;
  #endif
}

/**
 * OptimizeCodebook
 */
void RecogniserCore::OptimizeCodebook()
{
  codebook->Optimize();
}


/**
 * set camera parameter 
 */
void RecogniserCore::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distCoeffs)
{
  param.intrinsic = _intrinsic;
  param.distCoeffs = cv::Mat();;

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(param.intrinsic, CV_64F);

  if (!param.distCoeffs.empty())
  {
    param.distCoeffs = cv::Mat::zeros(1,8,CV_64F);
    for (int i=0; i<_distCoeffs.cols; i++)
      param.distCoeffs.at<double>(1,i) = _distCoeffs.at<double>(1,i);
  }
}


/****************************** DEBUG **********************************/
void RecogniserCore::GetColor(map<unsigned, cv::Vec3b> &cols, unsigned id, cv::Vec3b &col)
{
  map<unsigned,cv::Vec3b>::iterator it = cols.find(id);
  if( it == cols.end() )
  {
    col[0] = rand()%255, col[1]=rand()%255, col[2]=rand()%255;
    cols[id] = col;
  }
  else
  {
    col = it->second;
  }
}




} //-- THE END --

