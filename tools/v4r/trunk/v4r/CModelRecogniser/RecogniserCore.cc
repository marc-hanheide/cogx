/**
 * $Id$
 * TODO:
 * - update confidence values  
 * - correct vocabulary tree 2
 * - return objects without pose (distinguish between aff and pose regarding confidence value)
 */


#include "RecogniserCore.hh"

#define DEBUG

#define CONST_NORM_VALUE 20 

namespace P 
{




/********************** RecogniserCore ************************
 * Constructor/Destructor
 */
RecogniserCore::RecogniserCore(cv::Ptr<KeypointDetector> &keyDetector,
                               cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                               cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                               Parameter _param)
 : param(_param)
{
  detector = keyDetector;
  extractor = descExtractor;
  matcher = descMatcher;

  SetCameraParameter(_param.intrinsic,_param.distortion);

  codebook = new P::MSCodebook(matcher,MSCodebook::Parameter(_param.thrDesc));
}

RecogniserCore::~RecogniserCore()
{
}




/************************** PRIVATE ************************/



/**
 * Mean shift clustering of matches
 */
void RecogniserCore::ClusterMatches(unsigned id, vector<cv::Ptr<OVMatches> > &matches, vector<cv::Ptr<MatchPairs> > &mps)
{
  unsigned z;
  double angle, scale;
  vector<cv::Point2d> votes;
  vector<vector<unsigned> > cls;
  cv::Ptr<MatchPairs> cluster;
  vector<cv::DMatch> tmpMatches;
  map<unsigned, vector<PKeypoint*> >::iterator it;
  map<unsigned, vector<PKeypoint*> > ptsModel;
  map<unsigned, vector<PKeypoint*> > ptsImage;
  mps.clear();

  cv::Ptr<MeanShiftBase> mshift;

  if (param.useFams)
    mshift = new MeanShiftFAMS();
  else
    mshift = new MeanShift();

  #ifdef DEBUG
  srand(1);
  #endif

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (matches[i]->oid == id)
    {
      OVMatches &ms = *matches[i];
      View &view = *recModels[ms.oid]->views[ms.vid];
      for (unsigned j=0; j<ms.matches.size(); j++)
      {
        tmpMatches.push_back(ms.matches[j]);
        votes.push_back(cv::Point2d());
        PKeypoint::Vote(*view.keys[ms.matches[j].trainIdx],*keys[ms.matches[j].queryIdx], 
                        view.center, votes.back(), angle, scale);
      }
    }
  }

  cv::Mat_<double> samples(votes.size(),2,&votes[0].x);
  mshift->Cluster(samples,cls);

  for (unsigned i=0; i<cls.size(); i++)
  {
    if (cls[i].size()>=5)
    {
      ptsModel.clear();
      ptsImage.clear();
      cluster = new MatchPairs(id);

      for (unsigned j=0; j<cls[i].size(); j++)
      {
        cv::DMatch &m = tmpMatches[cls[i][j]];
        ptsModel[m.imgIdx].push_back(&(*recModels[id]->views[m.imgIdx]->keys[m.trainIdx]));
        ptsImage[m.imgIdx].push_back(&(*keys[m.queryIdx]));
      }

      cluster->idxViews.resize(ptsModel.size());
      cluster->ptsModel.resize(ptsModel.size());
      cluster->ptsImage.resize(ptsImage.size());
      for (z=0,it=ptsModel.begin(); it!=ptsModel.end(); it++,z++)
      {
        cluster->idxViews[z] = it->first;
        cluster->ptsModel[z] = it->second;
      }
      for (z=0,it=ptsImage.begin(); it!=ptsImage.end(); it++,z++)
      {
        cluster->ptsImage[z] = it->second;
      }
      mps.push_back(cluster);
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
        cv::circle(dbg, votes[cls[j][k]], 2, col, 2, 8, 0);
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
void RecogniserCore::CountInlier(MatchPairs &mp, Pose &pose, double &sig)
{
  sig=0; 
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];
  PKeypoint::nb2cnt++;

  for (unsigned j=0; j<mp.ptsImage.size(); j++)
  {
    for (unsigned i=0; i<mp.ptsImage[j].size(); i++)
    {
      if (mp.ptsImage[j][i]->nb2 != PKeypoint::nb2cnt)
      {
        PMat::MulAdd3( pose.R.ptr<double>(), &mp.ptsModel[j][i]->pos.x, pose.t.ptr<double>(), pos);
        ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

        dist = PVec::DistanceSqr2(pt,&mp.ptsImage[j][i]->pt.x);
        if (dist < sqrInlDist)
        {
          if (pos[2]>0)
          {
            sig += 1; //param.inlDistRansac-sqrt(dist);
            mp.ptsImage[j][i]->nb2 = PKeypoint::nb2cnt;
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
}

/**
 * Count inlier
 */
int RecogniserCore::MarkInlier(MatchPairs &mp, ObjectLocation &obj)
{
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];
  int cntInl=0;
  unsigned inl, maxInl=0;
  obj.idView = UINT_MAX;
  PKeypoint::nb2cnt++;

  for (unsigned j=0; j<mp.ptsImage.size(); j++)
  {
    inl=0;
    for (unsigned i=0; i<mp.ptsImage[j].size(); i++)
    {
      PMat::MulAdd3( obj.pose.R.ptr<double>(), &mp.ptsModel[j][i]->pos.x, obj.pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&mp.ptsImage[j][i]->pt.x);
      if (dist < sqrInlDist)
      {
        if (pos[2]>0)
        {
          mp.ptsImage[j][i]->nb = PKeypoint::nbcnt;
          inl++;
          if (mp.ptsImage[j][i]->nb2 != PKeypoint::nb2cnt)
          {
            cntInl++;
            mp.ptsImage[j][i]->nb2 = PKeypoint::nb2cnt;
          }
        }
      }
    }
    if (inl>maxInl)  // inlier / view
    {
      maxInl = inl;
      obj.idView = mp.idxViews[j];
    }
  }

  return cntInl;       //to compute confidence: should we use all inl ..
  //return maxInl;         // ... or the max number per view
}

/**
 * GetInlier
 */
void RecogniserCore::GetInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], 
                               vector<unsigned> &idxInlier)
{
  cv::Point2d pt;
  idxInlier.clear();
  double inlDistSqr = PMath::Sqr(param.inlDistRansac*2); // HACK: weaker model => just double the inlier thr
 
  for (unsigned i=0; i<imKeys.size(); i++)
  {
    if (moKeys[i]->Have3D())
    {
      PHom::MapPoint(&moKeys[i]->pt.x, H, &pt.x);

      if (PVec::DistanceSqr2(&pt.x,&imKeys[i]->pt.x) < inlDistSqr)
        idxInlier.push_back(i);
    }
  }
}

/**
 * CountInlier
 */
void RecogniserCore::CountInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], double &inl)
{
  inl=0;
  cv::Point2d pt;
  double inlDistSqr = PMath::Sqr(param.inlDistRansac*2); // HACK: weaker model => just double the inlier thr

  for (unsigned i=0; i<moKeys.size(); i++)
  {
    PHom::MapPoint(&moKeys[i]->pt.x, H, &pt.x);

    if (PVec::DistanceSqr2(&pt.x,&imKeys[i]->pt.x) < inlDistSqr)
      inl++;
  }
}

/**
 * copy unexpained points
 */
int RecogniserCore::GetUnmarked(MatchPairs &mp, MatchPairs &unexp)
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
}

/**
 * Robust estimation of object poses
 */
bool RecogniserCore::LoRansac(MatchPairs &mp, ObjectLocation &obj, int num)
{
  if (num < 5)
    return false; 

  // int ransac
  int k=0;
  double sigAff, svSigAff=0., sig=3, sigPose, svSigPose=0;
  double eps = sig/(double)num;
  double *d, rod[3], H[9];
  vector<unsigned> idxKeys, idxInlier, idsModel;
  Pose tmpPose(true);
  CvMat matRod = cvMat(3,1,CV_64F, rod);
  CvMat matt = tmpPose.t;
  CvMat matR = tmpPose.R;
  CvMat matIntr = param.intrinsic;
  CvMat matDistCoeffs = param.distortion;
  double pts3D[5*3];
  double pts2D[5*2];
  CvMat matPts3D = cvMat( 5, 3, CV_64F, pts3D );
  CvMat matPts2D = cvMat( 5, 2, CV_64F, pts2D );
  obj.H = cv::Mat(3,3,CV_64F);
  bool ok;
  unsigned idxView=0;

  while(idxView<mp.ptsModel.size() && mp.ptsModel[idxView].size() < 5) { idxView++; }
  if (idxView==mp.ptsModel.size())
    return false;

  //ransac pose
  while (pow(1. - pow(eps,5), k) >= param.etaRansac && k < param.maxRandTrials)
  {
    vector<PKeypoint*> &moKeys = mp.ptsModel[idxView];
    vector<PKeypoint*> &imKeys = mp.ptsImage[idxView];    

    GetRandIdx(moKeys.size(), 3, idxKeys);

    if (PHom::ComputeAff(&moKeys[idxKeys[0]]->pt.x, &moKeys[idxKeys[1]]->pt.x, &moKeys[idxKeys[2]]->pt.x,
                  &imKeys[idxKeys[0]]->pt.x, &imKeys[idxKeys[1]]->pt.x, &imKeys[idxKeys[2]]->pt.x, H))
    {
      CountInlier(moKeys,imKeys, H, sigAff);

      if (sigAff > (unsigned)svSigAff && sigAff >= 5)
      {
        svSigAff = sigAff;
        //if (sigAff > sig) sig = sigAff;
        d = obj.H.ptr<double>(0);
        for (unsigned k=0; k<9; k++) d[k] = H[k];

        GetInlier(moKeys,imKeys, H, idxInlier);
        
        if (idxInlier.size()>=5)  
        for (unsigned i=0; i<param.numLoTrials; i++)
        {
          GetRandIdx(idxInlier.size(), 5, idxKeys);
          idsModel.clear();
          ok=true;
          for (unsigned j=0; j<5; j++)
          {
            if (!Contains(idsModel,moKeys[idxInlier[idxKeys[j]]]->id))
            {
              idsModel.push_back(moKeys[idxInlier[idxKeys[j]]]->id);
              d =  &moKeys[idxInlier[idxKeys[j]]]->pos.x;

              pts3D[3*j+0] = d[0];
              pts3D[3*j+1] = d[1];
              pts3D[3*j+2] = d[2];

              d = &imKeys[idxInlier[idxKeys[j]]]->pt.x;
              pts2D[2*j+0] = d[0];
              pts2D[2*j+1] = d[1];
            }
            else
            {
              ok=false;
              break;
            }
          }

          if (ok)
          {
            cvFindExtrinsicCameraParams2(&matPts3D, &matPts2D, &matIntr, &matDistCoeffs, &matRod, &matt);
            cvRodrigues2(&matRod, &matR);
            CountInlier(mp, tmpPose, sigPose);
            if (sigPose > svSigPose)
            {
              svSigPose = sigPose;
              if (sigPose > sig) sig = sigPose;
              tmpPose.copyTo(obj.pose);
            }
          }
        }

        eps = sig / (double)num;
      }
    }
    k++;

    do
    {
      idxView++;
      if (idxView==mp.ptsModel.size()) idxView=0;
    }
    while(mp.ptsModel[idxView].size() < 5);
  }

  #ifdef DEBUG
  cout<<"Number of lo-ransac trials: "<<k<<", inl="<<sig<<"/"<<num<<endl;
  #endif
  if (sig >= 5)
    return true;
  return false;
}

/**
 * Robust estimation of object poses
 * TODO: distinguish between aff and pose regarding confidence value
 */
void RecogniserCore::RansacObjects(vector<cv::Ptr<MatchPairs> > &mps, vector<ObjectLocation> &objects)
{
  int numTotal, numInl;
  MatchPairs mp;
  ObjectLocation obj;
  bool ok;
  ConfValues getConf(param.intrinsic, param.distortion, param.inlDistRansac);

  PKeypoint::nbcnt++;
  for (unsigned i=0; i<mps.size(); i++)
  {
    ok=true;
    obj.idObject = recModels[mps[i]->idxObject]->id;
    numTotal = GetUnmarked(*mps[i], mp);

    while( LoRansac(mp, obj, numTotal) && ok )
    {
      numInl = MarkInlier(*mps[i], obj);

      if(obj.idView != UINT_MAX)
      {
        //obj.conf = getConf.ConfToProb( getConf.SupportingPointsPerView(keys, recModels[mps[i]->idxObject]->views[obj.idView]->keys, obj.pose) );
        //obj.conf = getConf.ConfToProb( getConf.SupportingPointsPerXX(keys, recModels[mps[i]->idxObject]->views[obj.idView]->keys, obj.pose) );
        obj.conf = getConf.ConfToProb( ((double)numInl)/(double)recModels[mps[i]->idxObject]->views[obj.idView]->keys.size() );
        //obj.conf = getConf.ConfToProb( ((double)numInl)/(double)CONST_NORM_VALUE );
        //obj.conf = getConf.ConfToProb( getConf.WeightedMatchedPointsPerView(*mps[i], recModels[mps[i]->idxObject]->views[obj.idView]->keys.size(), obj.pose));
        //obj.conf = getConf.ConfToProb( getConf.WeightedMatchedPointsPerXX(*mps[i], obj.pose) );

        if (obj.conf > 0.0000001) //0.001)
          objects.push_back(obj);
        else
          ok=false;
      }
      else ok = false;

      numTotal = GetUnmarked(*mps[i],mp);
    }
  }
}

/**
 * Project model points to current image and count inliers
 * num_supporting_points / num_points_of_recognised_view  (no matches used)
 */
double RecogniserCore::ComputeConfidence(vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, Pose &pose)
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


  /////////////////////////////////// HACK //////////////////////////
  /*unsigned cnt=0;
  cv::Point3d center(0.,0.,0.);
  for (unsigned i=0; i<model->views.size(); i++)
  {
    vector< cv::Ptr<PKeypoint> > &ks = model->views[i]->keys;
    for (unsigned j=0; j<ks.size(); j++)
      if (ks[j]->Have3D())
      {
        cnt++;
        center += ks[j]->pos;
      }
  }
  center.x /= (double)cnt;
  center.y /= (double)cnt;
  center.z /= (double)cnt;
  model->center = center;

  double pos[3];
  for (unsigned i=0; i<model->views.size(); i++)
  {
    View &view = *model->views[i];
    PMat::MulAdd3( view.pose.R.ptr<double>(), &center.x, view.pose.t.ptr<double>(), pos);
    ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), &view.center.x);
  }*/
  //////////////////////////////////////////////////////////////////

  for (unsigned i=0; i<model->views.size(); i++)
    codebook->InsertView(idx, i, model->views[i]);

  /*for (unsigned i=0; i<codebook->cbEntries.size(); i++)
  if (fabs(sqrt(codebook->cbEntries[i]->sqrThr)-0.4) > 0.0001)
  cout<<sqrt(codebook->cbEntries[i]->sqrThr)<<" ";*/

  return idx;
}

/**
 * Load a vocabulary tree from file
 */
void RecogniserCore::LoadVocabularyTree(const string &filename)
{
  cout<<"That's the codebook version! You do not need a vocabulary tree!"<<endl;
}

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
  if (!dbg.empty()) for (unsigned i=0; i<keys.size(); i++) PKeypoint::Draw(dbg, *keys[i], CV_RGB(255,0,0));
  #endif

  // match with vocabulary tree
  vector<cv::Ptr<OVMatches> > ovMatches;
  vector<pair<unsigned,vector<cv::DMatch> > > matches;
  #ifdef DEBUG
  struct timespec start2, end2;
  clock_gettime(CLOCK_REALTIME, &start2);
  #endif

  codebook->QueryObjects(descriptors, ovMatches);

  #ifdef DEBUG
  clock_gettime(CLOCK_REALTIME, &end2);
  cout<<"Time matching [s]: "<<PMath::timespec_diff(&end2, &start2)<<endl;
  #endif

  // verify matches and compute pose
  unsigned id;
  vector<cv::Ptr<MatchPairs> > mps;
  set<unsigned> used;


  do
  {
    id=UINT_MAX;
    mps.clear();

    for (unsigned i=0; i<ovMatches.size(); i++)
      if (used.find(ovMatches[i]->oid) == used.end())
      {
        id = ovMatches[i]->oid;
        used.insert(id);
        break;
      }
    #ifdef DEBUG
    struct timespec start5, end5;
    clock_gettime(CLOCK_REALTIME, &start5);
    #endif

    ClusterMatches(id, ovMatches, mps);
    #ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &end5);
    cout<<"Time clustering [s]: "<<PMath::timespec_diff(&end5, &start5)<<endl;
    struct timespec start6, end6;
    clock_gettime(CLOCK_REALTIME, &start6);
    #endif

    RansacObjects(mps, objects);

    #ifdef DEBUG
    clock_gettime(CLOCK_REALTIME, &end6);
    cout<<"Time ransac [s]: "<<PMath::timespec_diff(&end6, &start6)<<endl;
    #endif

  }
  while(used.size()<param.numObjectsToRecognise && id!=UINT_MAX);

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
void RecogniserCore::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion)
{
  param.intrinsic = _intrinsic;
  param.distortion = _distortion;

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(param.intrinsic, CV_64F);

  if (_distortion.type() != CV_64F)
    _distortion.convertTo(param.distortion, CV_64F);
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

