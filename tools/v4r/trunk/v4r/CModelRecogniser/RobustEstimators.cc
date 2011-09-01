/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */


#include "RobustEstimators.hh"

#define DEBUG


namespace P 
{

int RobustEstimators::NUM_RANSAC_POINTS = 5; 


RobustEstimators::RobustEstimators()
{
}

RobustEstimators::RobustEstimators(Parameter _param)
 : param(_param)
{
}

RobustEstimators::~RobustEstimators()
{
}




/************************************** PRIVATE ************************************/

/**
 * Get matches
 */
bool RobustEstimators::GetPoints(vector< cv::Ptr<PKeypoint> > &keys, 
       vector< cv::Ptr<PKeypoint> > &model, vector<cv::DMatch> &matches, 
       vector<PKeypoint*> &ptsImage, vector<PKeypoint*> &ptsModel)
{
  ptsImage.clear();
  ptsModel.clear();

  for (unsigned i=0; i<model.size(); i++)
    model[i]->id=i;

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (model[matches[i].trainIdx]->Have3D())
    {
      ptsImage.push_back( &(*keys[matches[i].queryIdx]) );
      ptsModel.push_back( &(*model[matches[i].trainIdx]) );
    }
  }

  if (ptsImage.size() > NUM_RANSAC_POINTS)
    return true;
  return false;
}

/**
 * GetRandIdx
 */
void RobustEstimators::GetRandIdx(unsigned size, unsigned num, vector<unsigned> &idx)
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
void RobustEstimators::CountInlier(vector<PKeypoint*> &ptsImage, vector<PKeypoint*> &ptsModel, Pose &pose, double &cnt)
{
  cnt=0;
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];
  
  for (unsigned i=0; i<ptsImage.size(); i++)
  {
    PMat::MulAdd3( pose.R.ptr<double>(), &ptsModel[i]->pos.x, pose.t.ptr<double>(), pos);
    ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

    dist = PVec::DistanceSqr2(pt,&ptsImage[i]->pt.x);
    if (dist < sqrInlDist)
    {
      if (pos[2]>0)
      {
        cnt += param.inlDistRansac-sqrt(dist);
      }
      else
      {
        cnt=0;
        return;
      }
    }
  }  
}

/**
 * Count inlier
 */
void RobustEstimators::CountInlier3D(vector<PKeypoint*> &ptsImage, vector<PKeypoint*> &ptsModel, Pose &pose, double &cnt)
{
  cnt=0;
  double cnt3D=0;
  double dist, sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double sqrInlThr3D = PMath::Sqr(param.inlThr3D);
  double pt[2], pos[3];
  
  for (unsigned i=0; i<ptsImage.size(); i++)
  {
    PMat::MulAdd3( pose.R.ptr<double>(), &ptsModel[i]->pos.x, pose.t.ptr<double>(), pos);
    ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

    dist = PVec::DistanceSqr2(pt,&ptsImage[i]->pt.x);
    if (dist < sqrInlDist)
    {
      if (pos[2]>0)
      {
        cnt += param.inlDistRansac-sqrt(dist);
        if (ptsImage[i]->Have3D() && PVec::DistanceSqr3(pos, &ptsImage[i]->pos.x) < sqrInlThr3D)
          cnt3D+=param.inlDistRansac;
      }
      else
      {
        cnt=0;
        return;
      }
    }
  }  
  
  if (cnt3D < cnt/5)   // TODO: and dubious threshold :-)
    cnt=0;
}

/**
 * RANSAC pose
 */
void RobustEstimators::FitPoseRANSAC(vector<PKeypoint*> &ptsImage, vector<PKeypoint*> &ptsModel, Pose &pose, double &conf, bool check3D)
{
  int k=0;
  int numPoints = ptsModel.size();
  double eps = ((double)NUM_RANSAC_POINTS)/(double)numPoints;
  double inl, inls = 0;
  vector<unsigned> idx;
  double pts3D[NUM_RANSAC_POINTS*3];
  double pts2D[NUM_RANSAC_POINTS*2];
  double rod[3], t[3];
  CvMat matPts3D = cvMat( NUM_RANSAC_POINTS, 3, CV_64F, pts3D );
  CvMat matPts2D = cvMat( NUM_RANSAC_POINTS, 2, CV_64F, pts2D );

  CvMat matRod = cvMat(3,1,CV_64F, rod);
  Pose tmpPose(true);
  CvMat matt = tmpPose.t;
  CvMat matR = tmpPose.R;
  CvMat intr = param.intrinsic;
  CvMat disto = param.distortion;

  srand(1);
  double *d;
  vector<unsigned> ids;
  bool ok;

  #ifdef DEBUG
  //if (!dbgWin.empty()) dbgWin->Clear();
  //vector<cv::Ptr<Pose> > acPose;
  //vector<int> acInl;
  #endif

  while (pow(1. - pow(eps,NUM_RANSAC_POINTS), k) >= param.etaRansac && k < param.maxRandTrials)
  {
    GetRandIdx(numPoints, NUM_RANSAC_POINTS, idx);

    ids.clear();
    ok=true;
    for (unsigned i=0; i<NUM_RANSAC_POINTS; i++)
    {
      if (!Contains(ids,ptsModel[idx[i]]->id))
      {
        ids.push_back(ptsModel[idx[i]]->id);
        d =  &ptsModel[idx[i]]->pos.x;

        pts3D[3*i+0] = d[0];
        pts3D[3*i+1] = d[1];
        pts3D[3*i+2] = d[2];

        d = &ptsImage[idx[i]]->pt.x;
        pts2D[2*i+0] = d[0];
        pts2D[2*i+1] = d[1]; 
      }
      else
      {
        ok=false;
        break;
      }
    }

    if (ok)
    {
      cvFindExtrinsicCameraParams2(&matPts3D, &matPts2D, &intr, &disto, &matRod, &matt);
      cvRodrigues2(&matRod, &matR);

      if(check3D) CountInlier3D(ptsImage, ptsModel, tmpPose, inl);
      else CountInlier(ptsImage, ptsModel, tmpPose, inl);

      #ifdef DEBUG
      /*if (inl>0)
      {
        acPose.push_back(new Pose());
        *acPose.back() = tmpPose;
        acInl.push_back(inl);
      }*/
      #endif

      if (inl > inls)
      {
        inls = inl;
        eps = (double)inls / (param.inlDistRansac*(double)numPoints);
        pose = tmpPose;
      }

    }
    k++;
  }

  conf = ((double)inls)/(param.inlDistRansac*(double)numPoints);

  #ifdef DEBUG
  /*double pt3[3];
  Pose invPose;
  InvPose(pose,invPose);
  for (unsigned i=0; i<acPose.size(); i++)
  {
    PMat::MulAdd3(invPose.R.ptr<double>(),acPose[i]->t.ptr<double>(),invPose.t.ptr<double>(), pt3);
    int col = 170 - 170*acInl[i]/(ptsModel.size()*.7);
    if (!dbgWin.empty()) dbgWin->AddPoint3D(pt3[0],pt3[1],pt3[2], col,col,col,20-(int)((double)col)/170.*20);
  }*/
  cout<<"Iter="<<k<<", inl="<<inls<<"/"<<numPoints<<endl;
  #endif
}

/**
 * CountInlier
 */
void RobustEstimators::CountInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], double &inl)
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
 * GetInlier
 */
void RobustEstimators::GetInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9],
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
 * Local optimized ransac for pose estimation
 */
void RobustEstimators::FitPoseLoRANSAC(vector<PKeypoint*> &ptsImage, vector<PKeypoint*> &ptsModel, Pose &pose, double &sig)
{
  // int ransac
  sig=3;
  unsigned num = ptsImage.size();
  int k=0;
  double sigAff, svSigAff=0., sigPose, svSigPose=0;
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
  cv::Mat svH = cv::Mat(3,3,CV_64F);
  bool ok;

  //ransac pose
  while (pow(1. - pow(eps,5), k) >= param.etaRansac && k < param.maxRandTrials)
  {
    GetRandIdx(ptsModel.size(), 3, idxKeys);

    if (PHom::ComputeAff(&ptsModel[idxKeys[0]]->pt.x, &ptsModel[idxKeys[1]]->pt.x, &ptsModel[idxKeys[2]]->pt.x,
                  &ptsImage[idxKeys[0]]->pt.x, &ptsImage[idxKeys[1]]->pt.x, &ptsImage[idxKeys[2]]->pt.x, H))
    {
      CountInlier(ptsModel,ptsImage, H, sigAff);

      if (sigAff > (unsigned)svSigAff && sigAff >= 5)
      {
        svSigAff = sigAff;
        d = svH.ptr<double>(0);
        for (unsigned k=0; k<9; k++) d[k] = H[k];

        GetInlier(ptsModel,ptsImage, H, idxInlier);

        if (idxInlier.size()>=5)
        for (unsigned i=0; i<param.numLoTrials; i++)
        {
          GetRandIdx(idxInlier.size(), 5, idxKeys);
          idsModel.clear();
          ok=true;
          for (unsigned j=0; j<5; j++)
          {
            if (!Contains(idsModel,ptsModel[idxInlier[idxKeys[j]]]->id))
            {
              idsModel.push_back(ptsModel[idxInlier[idxKeys[j]]]->id);
              d =  &ptsModel[idxInlier[idxKeys[j]]]->pos.x;

              pts3D[3*j+0] = d[0];
              pts3D[3*j+1] = d[1];
              pts3D[3*j+2] = d[2];

              d = &ptsImage[idxInlier[idxKeys[j]]]->pt.x;
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
            CountInlier(ptsImage,ptsModel, tmpPose, sigPose);
            if (sigPose > svSigPose)
            {
              svSigPose = sigPose;
              if (sigPose > sig) sig = sigPose;
              tmpPose.copyTo(pose);
            }
          }
        }

        eps = sig / (double)num;
      }
    }
    k++;
  }

  #ifdef DEBUG
  cout<<"Number of lo-ransac trials: "<<k<<", inl="<<sig<<"/"<<num<<endl;
  #endif
}



/************************************** PUBLIC ************************************/

/**
 * Robust estimation of the pose using RANSAC
 */
double RobustEstimators::RansacPose(vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, vector<cv::DMatch> &matches, Pose &pose, bool check3D)
{
  if (param.intrinsic.empty()) throw runtime_error("RobustEstimators::RansacPose Need camera parameter!");

  double conf=0.;
  vector<PKeypoint*> ptsModel;
  vector<PKeypoint*> ptsImage;

  if (GetPoints(keys,model,matches,ptsImage,ptsModel))
  {
    FitPoseRANSAC(ptsImage, ptsModel, pose, conf, check3D);

    #ifdef DEBUG
    if (!dbg.empty()) DrawInlier(dbg, keys, model, matches, pose);
    #endif

    return conf;
  }
 
  return 0.;
}

/**
 * Local optimized ransac for pose estimation
 * a significance value is returned (number of inlier weighted with error of the inlier)
 */
double RobustEstimators::LoRansacPose( vector< cv::Ptr<PKeypoint> > &keys, 
         vector< cv::Ptr<PKeypoint> > &model, vector<cv::DMatch> &matches, Pose &pose)
{
  if (param.intrinsic.empty()) throw runtime_error("RobustEstimators::RansacPose need camera parameter!");

  double sig=0.;
  vector<PKeypoint*> ptsModel;
  vector<PKeypoint*> ptsImage;

  if (GetPoints(keys,model,matches,ptsImage,ptsModel))
  {
    FitPoseLoRANSAC(ptsImage, ptsModel, pose, sig);

    #ifdef DEBUG
    if (!dbg.empty()) DrawInlier(dbg, keys, model, matches, pose);
    #endif

    return (sig/param.inlDistRansac);
  }
 
  return 0.;
}


/**
 * SetCameraParameter
 */
void RobustEstimators::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion)
{
  param.intrinsic = _intrinsic;
  param.distortion = _distortion;
}


/**
 * Draw inlier
 */
void RobustEstimators::DrawInlier(cv::Mat &img, vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, vector<cv::DMatch> &matches, Pose &pose)
{
  if (pose.empty())
    return;

  double sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double pt[2], pos[3];

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (model[matches[i].trainIdx]->Have3D())
    {
      double *pt3 = &model[matches[i].trainIdx]->pos.x; 
      PMat::MulAdd3( pose.R.ptr<double>(), pt3, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), pt);

      PKeypoint::Draw(dbg, *keys[matches[i].queryIdx], CV_RGB(255,0,0));

      if (PVec::DistanceSqr2(pt,&keys[matches[i].queryIdx]->pt.x) < sqrInlDist)
      {
        PKeypoint::Draw(dbg, *keys[matches[i].queryIdx], CV_RGB(0,255,0));
      }
    }
  }  
}


}

