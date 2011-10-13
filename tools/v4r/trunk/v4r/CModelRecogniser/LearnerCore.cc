/**
 * $Id$
 */


#include "LearnerCore.hh"

#define DEBUG
#define VIEW_HIST_SUB_DIV 3 

#define LOG_POINT_CLOUD
#define POINT_CLOUD_CONTOUR


namespace P 
{


/********************** LearnerCore ************************
 * Constructor/Destructor
 */
LearnerCore::LearnerCore(cv::Ptr<KeypointDetector> &keyDetector,
                               cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                               cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                               Parameter p)
 : param(p)
{
  detector = keyDetector;
  extractor = descExtractor;
  matcher = descMatcher;

  //estimator = new RobustEstimators(RobustEstimators::Parameter(p.maxRandTrials, 
  //                  p.numLoTrials, p.etaRansac, p.inlDistRansac));

  SetCameraParameter(p.intrinsic, p.distCoeffs);
  model = new CModel(VIEW_HIST_SUB_DIV);

  codebook = new MeanShiftCodebook(matcher, MeanShiftCodebook::Parameter(p.thrDesc,p.sigmaDesc));
  //codebook = new MeanCodebook(matcher, MeanCodebook::Parameter(p.thrDesc,p.sigmaDesc));
}

LearnerCore::~LearnerCore()
{
}



/************************** PRIVATE ************************/

/**
 * Setup
 */
int LearnerCore::Setup(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat &R, cv::Mat &T, const string &oid)
{
  // set images
  if (param.intrinsic.empty())
    throw runtime_error("LearnerCore::Setup : Camera parameter are not set!");
  if (image.cols != param.width || image.rows != param.height)
    throw runtime_error("LearnerCore::Setup : Wrong image size!");
  if (cloud.empty())
    throw runtime_error("LearnerCore::Setup : No point cloud available!");
  if (!model->id.empty() && model->id.compare(oid)!=0)
    throw runtime_error("LearnerCore::Setup : Invalide object id!");


  // set the object pose
  Pose pose(R,T);

  view = new View();

  if ( !pose.empty() )
  {
    if ( invRefGlobal.empty() ) InvPose(pose,invRefGlobal);
    if ( refObject.empty() ) refObject = pose;
    MulPose(invRefGlobal, refObject, view->pose);
    MulPose(pose, view->pose, view->pose);

    model->pose = view->pose;

    // check pose
    for (unsigned i=0; i<model->views.size(); i++)
    {
      double angle = AngleBetween(view->pose,model->views[i]->pose);
      #ifdef DEBUG
      cout<<"Angle between views: "<<angle*180/M_PI<<endl;
      #endif
      if (angle < param.minAngleBetweenViews*M_PI/180.)
        return 0;
    }

    imgGray = image;
    if( image.type() != CV_8U ) cv::cvtColor( image, imgGray, CV_BGR2GRAY );

    // set object name
    if (model->id.empty()) model->id = oid;

    return 1;
  }

  return 0;
}


/**
 * Set 3d location of interest points
 */
void LearnerCore::AlignKeypoints3D(const cv::Mat_<cv::Vec4f> &cloud, const cv::Mat &mask, Pose &pose, const vector< cv::Ptr<PKeypoint> > &keys, std::vector<cv::Point3f> &pts)
{
  pts.clear();
  if (pose.empty())
    return;

  cv::Point3f pos;
  cv::Point2f pt;
  Pose invPose;
  InvPose(pose,invPose);

  // transform to current view
  cloud3f.clear();
  projCloud2f.clear();
  for (int v=0; v<cloud.rows; v++)
  {
    for (int u=0; u<cloud.cols; u++)
    {
      const cv::Vec4f &pt3f = cloud(v,u);
      if (pt3f[0]==pt3f[0] && pt3f[1]==pt3f[1] && pt3f[2]==pt3f[2]) 
      {
        if (param.distCoeffs.empty())
          ProjectPoint2Image(&pt3f[0], param.intrinsic.ptr<double>(), &pt.x);
        else
          ProjectPoint2Image(&pt3f[0], param.intrinsic.ptr<double>(), param.distCoeffs.ptr<double>(), &pt.x);

        if (mask.empty() || mask.at<uchar>((int)(pt.y+.5),(int)(pt.x+.5))>0)
        {
          projCloud2f.push_back(pt);
          cloud3f.push_back(cv::Point3f(pt3f[0],pt3f[1],pt3f[2]));
        }
      }
    }
  }
  
  if (cloud3f.size()<5)
  {
    pts.clear();
    return;
  }

  // find corresponding 3d points for keypoints
  vector<cv::Point2f> keys2f(keys.size());
  for (unsigned i=0; i<keys.size(); i++)
    keys2f[i] = keys[i]->pt;

  cv::Mat matPts2f(projCloud2f.size(),2,CV_32F,&projCloud2f[0]);
  cv::Mat matKeys2f(keys2f.size(),2,CV_32F, &keys2f[0]);
  cv::Mat indices(keys.size(), 1, CV_32S);
  cv::Mat dists(keys.size(), 1, CV_32F);

  cv::flann::Index flann_index(matPts2f, cv::flann::KDTreeIndexParams(4));
  flann_index.knnSearch(matKeys2f, indices, dists, 1, cv::flann::SearchParams(32) );
  
  // return results
  float sqrDist = PMath::Sqr(param.alignDist);
  pts.resize(keys.size());
  float noPoint = std::numeric_limits<float>::quiet_NaN ();
  for (unsigned i=0; i<pts.size(); i++)
  {
    if (dists.at<float>(i,0) < sqrDist)
    {
      PMat::MulAdd3( invPose.R.ptr<double>(), &cloud3f[indices.at<int>(i,0)].x, invPose.t.ptr<double>(), &pts[i].x);
    }
    else
    {
      pts[i] = cv::Point3f(noPoint,noPoint,noPoint);
    }
  }

}

/**
 * AngleBetween viewpoints
 */
double LearnerCore::AngleBetween(Pose &pose1, Pose &pose2)
{
  if (pose1.empty() || pose2.empty())  // there is nothing to test, so just ignore...
    return 100;


  cv::Point3d pos1, pos2;

  ComputeViewRay(pose1, model->center, pos1);
  ComputeViewRay(pose2, model->center, pos2);

  return PVec::AngleBetween3(&pos1.x,&pos2.x);
}

/**
 * AddViewToModel and link the points
 */
int LearnerCore::AddViewToModel(std::vector<cv::Point3f> &keys3f)
{
  if (keys3f.size() != view->keys.size())
    return 0;

  int status=0, cnt=0;
  cv::Point2f pt;
  cv::Point3f pos;
  map<unsigned, vector<cv::DMatch> > matches;
  map<unsigned, vector<cv::DMatch> >::iterator it;
  double sqrInlDist = PMath::Sqr(param.inlDistRansac);
  double sqrDistPoint3d = PMath::Sqr(param.maxDistPoint3d);
  vector<bool> okLink;
  vector<int> cntLinksPerView;

  // query matches
  codebook->QueryObjects(view->descriptors, matches);

  // link 3d points
  view->id = model->views.size();

  if (matches.size()==1)
  {
    it = matches.begin();
    okLink.resize(it->second.size(),false);
    cntLinksPerView.resize(model->views.size(),0);

    //cout<<"matches.size()="<<it->second.size()<<endl;
    for (unsigned i=0; i<it->second.size(); i++)
    {
      cv::DMatch &ma = it->second[i];
      PKeypoint &key = *model->views[ma.imgIdx]->keys[ma.trainIdx];
      cv::Point3f &pt3f = keys3f[ma.queryIdx];
      //cv::line(dbg, key.pt, view->keys[ma.queryIdx]->pt, CV_RGB(255,0,0),1);
      if (pt3f.x==pt3f.x && !key.pos.empty())
      {
        PMat::MulAdd3( view->pose.R.ptr<double>(), &key.pos->pt.x, view->pose.t.ptr<double>(), &pos.x);
        ProjectPoint(pos, param.intrinsic, param.distCoeffs, pt);
        if (PVec::DistanceSqr2(&pt.x, &view->keys[ma.queryIdx]->pt.x) < sqrInlDist)
        {
          if (PVec::DistanceSqr3(&key.pos->pt.x, &pt3f.x) < sqrDistPoint3d)
          {
            if (key.pos->TestInsert(view->id, ma.queryIdx))
            {
              cnt++;
              okLink[i] = true;
              cntLinksPerView[ma.imgIdx]++;
            }
          }
        }
      }
    }
  }

  int idx = GetIndexMax(cntLinksPerView);
  double conf = (cnt>3 ? ((double)cnt)/(double)model->views[idx]->keys.size() : DBL_MAX);

  // add new 3d points and add view
  if (conf<param.maxConfToLearn || param.forceLearning || model->views.size()==0)
  {
    if (matches.size()==1)
    {
      for (unsigned i=0; i<okLink.size(); i++)
      {
        if (okLink[i])
        {
          cv::DMatch &ma = it->second[i];
          PKeypoint &key = *model->views[ma.imgIdx]->keys[ma.trainIdx];
          view->keys[ma.queryIdx]->pos = key.pos;
          key.pos->Insert(view->id, ma.queryIdx, keys3f[ma.queryIdx]);
        }
      }
    }

    status = (cnt==0?1:2);
    model->views.push_back(view);

    for (int i=0; i<keys3f.size(); i++)
    {
      cv::Point3f &pt3f = keys3f[i];
      if (pt3f.x==pt3f.x && view->keys[i]->pos.empty())
      {
        model->points.push_back(new Point3dProjs(model->points.size(), pt3f));
        model->points.back()->projs.push_back(std::pair<unsigned,unsigned>(view->id, i));
        view->keys[i]->pos = model->points.back();
      }
    }

    // insert to codebook
    std::vector<cv::Ptr<CModel> > models(1,model);
    codebook->InsertView(0,view->id, models);

    // compute object center
    if (model->views.size()==1 && param.computeObjectCenter)
      SetObjectCenter();

    // add to view indexing sphere
    AddViewToViewSphere();
  }

  #ifdef DEBUG
  cout<<"Test conf for learning: conf="<<conf<<endl;
  cout<<"Number of linked points: "<<cnt<<" -> "<<(status==2?"learned view!":"no view learned")<<endl;
  #endif

  return status;
}

/**
 * SetObjectCenter
 */
void LearnerCore::SetObjectCenter()
{
  // shift 3d points and set origin of the object
  cv::Point3d center = GetMean3D(model->views);
  Translate3D(model->views, -center);
  model->center = cv::Point3d(0.,0.,0.);

  // renew pose
  cv::Mat_<double> pt;
  model->views.back()->pose.t.copyTo(pt);
  double *t = model->views.back()->pose.t.ptr<double>(0);
  double *R = model->views.back()->pose.R.ptr<double>(0);
  PMat::MulAdd3(R,&center.x,&pt(0,0), t);
  refObject.t.copyTo(pt);
  PMat::MulAdd3(refObject.R.ptr<double>(0),&center.x, &pt(0,0), refObject.t.ptr<double>(0));
}


/**
 * AddViewToViewSphere
 */
void LearnerCore::AddViewToViewSphere()
{
  //set view center in px
  double pos[3];
  View &view = *model->views.back();
  PMat::MulAdd3( view.pose.R.ptr<double>(), &model->center.x, view.pose.t.ptr<double>(), pos);
  if (param.distCoeffs.empty())
    ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), &view.center.x);
  else
    ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), param.distCoeffs.ptr<double>(), &view.center.x);

  // add to completeness representation (viewpoint historgram)
  if (!model->viewHist.empty())
  {
    cv::Point3d vr;
    ProbModel predProb;
    
    // compute view ray..
    ComputeViewRay(view.pose, model->center, vr);
    model->viewHist->InsertMax(model->views.size()-1,vr, predProb);
  }
}

/**
 * Get mean of 3d points
 */
cv::Point3d LearnerCore::GetMean3D(vector<cv::Ptr<View> > &views)
{
  unsigned cnt=0;
  cv::Point3d mean(0.,0.,0.);

  for (unsigned i=0; i<views.size(); i++)
  {
    vector<cv::Ptr<PKeypoint> > &keys = views[i]->keys;

    for (unsigned j=0; j<keys.size(); j++)
      if (keys[j]->Have3D())
      {
        mean += keys[j]->pos->pt;
        cnt++;
      }
  }
  
  mean.x /= (double)cnt;
  mean.y /= (double)cnt;
  mean.z /= (double)cnt;

  return mean;
}

/**
 * Translate3D 3d location of keypoints
 */
void LearnerCore::Translate3D(vector<cv::Ptr<View> > &views, cv::Point3d T )
{
  for (unsigned i=0; i<views.size(); i++)
  {
    vector<cv::Ptr<PKeypoint> > &keys = views[i]->keys;

    for (unsigned j=0; j<keys.size(); j++)
      if (keys[j]->Have3D())
        keys[j]->pos->pt += T;
  }
}

/**
 * Compute view ray
 */
void LearnerCore::ComputeViewRay(Pose &pose, cv::Point3d &objCenter, cv::Point3d &vr)
{
  double pos[3];
  Pose invPose;
  InvPose(pose,invPose);

  pos[0]=0., pos[1]=0., pos[2]=0.;
  PMat::MulAdd3( invPose.R.ptr<double>(), pos, invPose.t.ptr<double>(), &vr.x);
  vr -= objCenter;
  if (!PMath::IsZero(vr.x) && !PMath::IsZero(vr.y) && !PMath::IsZero(vr.z))
    PVec::Normalise3(&vr.x,&vr.x);
}

/**
 * Transform a point cloud and copy it to a dense vector 
 */
void LearnerCore::SetPointCloud(const cv::Mat_<cv::Vec4f> &cloud, const cv::Mat_<uchar> &mask, Pose &pose, vector<cv::Vec4f> &vecCloud)
{
  Pose invPose;
  InvPose(pose,invPose);
  vecCloud.clear();

  for (int v=0; v<cloud.rows; v++)
  {
    for (int u=0; u<cloud.cols; u++)
    {
      if (mask(v,u) > 0)
      {
        const cv::Vec4f &pt4f = cloud(v,u);
        if (pt4f[0]==pt4f[0] && pt4f[1]==pt4f[1] && pt4f[2]==pt4f[2])
        {
          vecCloud.push_back( cv::Vec4f() );
          PMat::MulAdd3( invPose.R.ptr<double>(), &pt4f[0], invPose.t.ptr<double>(), &vecCloud.back()[0]);
          #ifdef POINT_CLOUD_CONTOUR
          vecCloud.back()[3] = 0;
          #else
          vecCloud.back()[3] = pt4f[3];
          #endif
        }
      }
    }
  }
}




/************************** PUBLIC *************************/
/**
 * Learning of objects needs an orderd sequence of images
 * @param image rgb or gray scale image
 * @param cloud  grid of 3d points aligned with image (up to a scale factor) in camara coordinates
 * @param R T global coordinates of the camera assuming the object is static
 * @param mask mask to learn only a part of the image (optional)
 * @return status of learning (0..not_learned, 1..learned, 2..learned_and_linked)
 */
int LearnerCore::Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask)
{
  #ifdef DEBUG
  cout<<"************************* PROCESS A NEW FRAME **********************************"<<endl;
  //if (!dbgWin.empty()) dbgWin->SetImage(image);
  //if (!dbgWin.empty()) dbgWin->SetPointCloud(cloud);
  #endif

  // init ...
  double maxConf=0;
  vector<cv::Point3f> keys3f;

  int status = Setup(image, cloud, R, T, oid);

  if (status >= 1)
  {
    // detect keypoints and descriptors of current frame
    detector->Detect(imgGray, view->keys, mask);
    PKeypoint::ConvertToCv(view->keys,cvKeys);
    extractor->compute(imgGray, cvKeys, view->descriptors);

   // align keypoints
    AlignKeypoints3D(cloud, mask, view->pose, view->keys, keys3f);

    //add view to model 
    status = AddViewToModel(keys3f);

    #ifdef LOG_POINT_CLOUD
    if (status==2)
    {
      #ifdef POINT_CLOUD_CONTOUR
      cv::Mat contour;
      cv::Canny(mask, contour, 50, 100, 3, false);
      cv::Mat element= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1*2+1,1*2+1), cv::Point(1,1));
      dilate(contour, contour, element);
      SetPointCloud(cloud,contour, model->views.back()->pose, model->views.back()->pointcloud);
      #else
      SetPointCloud(cloud,mask, model->views.back()->pose, model->views.back()->pointcloud);
      #endif
    }
    #endif

    #ifdef DEBUG
    cout<<"model id="<<oid<<": views.size()="<<model->views.size()<<endl;
    #endif
  }
  cout<<"model id="<<oid<<": views.size()="<<model->views.size()<<endl;


  return status;
}


/**
 * Clear the model for learnModeling,
 * i.e. delete the views and clear the view indexing histogram of the model for learnModeling
 */
void LearnerCore::Clear()
{
  model->clear();
  codebook->clear();
  invRefGlobal.release();
  refObject.release();
}

/**
 * Set a model which should be improved
 */
void LearnerCore::SetModel(cv::Ptr<CModel> &_model)
{
  Clear();

  model = _model;
  std::vector<cv::Ptr<CModel> > models(1,model);

  for (unsigned i=0; i<model->views.size(); i++)
    codebook->InsertView(0,i, models);
}

/**
 * Access the learnModeled model
 */
cv::Ptr<CModel> &LearnerCore::GetModel()
{
  return model;
}

/**
 * Set the reference object pose from recognition to continue learning
 */
void LearnerCore::SetReferenceObjectPose(const Pose &pose)
{
  refObject = pose;
}

/**
 * GetViewRays
 */
void LearnerCore::GetViewRays(vector<cv::Point3d> &vr)
{
  cout<<"TODO: LearnerCore::GetViewRays!"<<endl;
//  if (!model->viewHist.empty())
//    model->viewHist->GetViewRays(vr, param.minProbLearn, param.maxProbLearn);
}

/**
 * set camera parameter 
 */
void LearnerCore::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distCoeffs)
{
  param.intrinsic = _intrinsic;
  param.distCoeffs = cv::Mat();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(param.intrinsic, CV_64F);


  if (!param.distCoeffs.empty())
  {
    param.distCoeffs = cv::Mat::zeros(1,8,CV_64F);
    for (int i=0; i<_distCoeffs.cols; i++)
      param.distCoeffs.at<double>(1,i) = _distCoeffs.at<double>(1,i);
  }

  //estimator->SetCameraParameter(param.intrinsic, param.distCoeffs);
}









} //-- THE END --

