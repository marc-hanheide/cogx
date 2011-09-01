/**
 * $Id$
 * TODO
 * - add prob function (ConfValues)
 * ------
 * - voc tree 2                         // Mi.
 * - optionally bundle (mot, struct)
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
                               Parameter _param)
 : scaleWidth(1.), scaleHeight(1.), param(_param)
{
  detector = keyDetector;
  extractor = descExtractor;
  matcher = descMatcher;

  estimator = new RobustEstimators(RobustEstimators::Parameter(_param.maxRandTrials, 
      _param.numLoTrials, _param.etaRansac, _param.inlDistRansac));

  SetCameraParameter(_param.intrinsic,_param.distortion);
  model = new CModel(VIEW_HIST_SUB_DIV);

  codebook = new MSCodebook(matcher, MSCodebook::Parameter(_param.thrDesc));
}

LearnerCore::~LearnerCore()
{
}



/************************** PRIVATE ************************/

/**
 * Setup
 */
void LearnerCore::Setup(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat &R, cv::Mat &T, const string &oid)
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

  imgGray = image;
  if( image.type() != CV_8U ) cv::cvtColor( image, imgGray, CV_BGR2GRAY );


  // set the object pose
  Pose pose(R,T);

  view = new View();

  if ( !pose.empty() )
  {
    if ( invRefGlobal.empty() ) InvPose(pose,invRefGlobal);
    if ( refObject.empty() ) refObject = pose;
    MulPose(invRefGlobal, refObject, view->pose);
    MulPose(pose, view->pose, view->pose);
  }

  // scaling point cloud / image
  scaleWidth = (double)cloud.cols / (double)image.cols;
  scaleHeight = (double)cloud.rows / (double)image.rows;

  // set object name
  if (model->id.empty()) model->id = oid;
}


/**
 * Set 3d location of interest points
 */
void LearnerCore::AlignKeypoints3D(const cv::Mat_<cv::Vec4f> &cloud, vector< cv::Ptr<PKeypoint> > &keys, Pose &pose)
{
  if (pose.empty())
    return;

  double pos[3];
  Pose invPose;
  InvPose(pose,invPose);

  for (unsigned i=0; i<keys.size(); i++)
  {
    const cv::Vec4f &pt3f = cloud(toCloudY(keys[i]->pt.y), toCloudX(keys[i]->pt.x));
    if (pt3f[0]==pt3f[0] && pt3f[1]==pt3f[1] && pt3f[2]==pt3f[2])
    {
      PMat::MulAdd3( invPose.R.ptr<double>(), &pt3f[0], invPose.t.ptr<double>(), pos);
      keys[i]->Set3D( &pos[0] );
    }
  }
}

/**
 * LinkViews
 * Note: 
 * Actually, becaus of 3d data from point cloud 
 * there is no reason to forward 3d keypoint locations except if we want to do sfm
 */
void LearnerCore::LinkViews(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches)
{
  for (unsigned i=0; i<matches.size(); i++)
  {
    queryKeys[matches[i].queryIdx]->InsertLink(*trainKeys[matches[i].trainIdx]);
    if (!queryKeys[matches[i].queryIdx]->Have3D()) 
      queryKeys[matches[i].queryIdx]->pos = trainKeys[matches[i].trainIdx]->pos;
  }
}

/**
 * Recognise
 */
void LearnerCore::Recognise(cv::Mat_<float> &queryDescs, vector<cv::Ptr<PKeypoint> > &queryKeys, 
       vector<Pose> &poses, vector<double> &probs, vector<unsigned> &idxViews, 
       vector<vector<cv::DMatch> > &matches)
{
  double conf, sig;
  vector<vector<cv::DMatch> > tmpMatches;
  vector<cv::Ptr<OVMatches> > queryMatches;
  MatchFilter filter;
  ConfValues getConf(param.intrinsic, param.distortion, param.inlDistRansac);

  probs.clear();
  idxViews.clear(); 
  poses.clear();
  matches.clear();

  // query view
  codebook->QueryObjects(queryDescs, queryMatches);

  // test views
  poses.reserve(param.maxLinkViews);
  matches.reserve(param.maxLinkViews);

  for (unsigned i=0; i<queryMatches.size() && i<param.maxLinkViews; i++)
  {
    poses.push_back(Pose());
    View &mview = *model->views[queryMatches[i]->vid];

    filter.Voting(queryKeys, mview.keys, queryMatches[i]->matches, mview.center, tmpMatches);
   
    if (tmpMatches.size()>0)
    {

      sig = estimator->LoRansacPose(queryKeys, mview.keys, tmpMatches[0], poses.back());
      //probs.push_back(sig/(double)mview.keys.size());
      conf = getConf.SupportingPointsPerView(queryKeys, mview.keys, poses.back());
      probs.push_back( getConf.ConfToProb(conf) );

      idxViews.push_back(queryMatches[i]->vid);
      matches.push_back(tmpMatches[0]);
    }
    else
    {
      poses.pop_back();
    }
  }

}

/**
 * ComparePose
 * up to now just compare the view points
 */
bool LearnerCore::ComparePose(Pose &pose1, Pose &pose2)
{
  if (pose1.empty() || pose2.empty())  // there is nothing to test, so just ignore...
    return true;

  cv::Point3d vr(0.,0.,1.), pos1, pos2;
  
  PMat::Mul3( pose1.R.ptr<double>(), &vr.x, &pos1.x);
  PMat::Mul3( pose2.R.ptr<double>(), &vr.x, &pos2.x);

  if (PVec::AngleBetween3(&pos1.x,&pos2.x) < param.cmpViewAngle*M_PI/180.)
    return true;
  return false;
}


/**
 * Recognise objects and link keypoints
 */
int LearnerCore::RecogniseAndLink(double &maxConf)
{
  maxConf=0.;
  unsigned idxBestPose=UINT_MAX;
  int status=0;
  vector<cv::DMatch> inlMatches;
  vector<Pose> poses;
  vector<double> probs;
  vector<unsigned> idxViews;
  vector<vector<cv::DMatch> > matches;

  //recognise
  Recognise(view->descriptors, view->keys, poses, probs, idxViews, matches);

  //check status
  for (unsigned i=0; i<probs.size(); i++)
  {
    if (probs[i] > maxConf)                      // test higher learning threshold
    {
      maxConf = probs[i];
      idxBestPose = i;
      if (maxConf > param.maxProbLearn)
      {
        if (view->pose.empty())
          model->pose = poses[i];
        else
          model->pose = view->pose;
        return 1;
      }
    }
  }

  // link keypoints of views
  MatchFilter filter;
  for (unsigned i=0; i<poses.size(); i++)
  {                                             // test lower learning threshold compare poses
    if (probs[i] >= param.minProbLearn && ComparePose(poses[i], view->pose))
    {
      filter.ThrProjDist(view->keys, model->views[idxViews[i]]->keys, matches[i], poses[i], 
                         param.intrinsic, inlMatches, param.inlDistRansac);
      LinkViews(view->keys, model->views[idxViews[i]]->keys, inlMatches);
      status = 2;
    }
  }

  // set best pose if we did not get a pose
  if (idxBestPose!=UINT_MAX)
  {
    if (view->pose.empty() || param.useRecognisedPose)
    {
      view->pose = poses[idxBestPose];
      model->pose = poses[idxBestPose];
    }
  }

  if (view->pose.empty()) InitializePose(view->pose);

  if (model->views.size()==0)
    status = 2;

  return status;
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
        mean += keys[j]->pos;
        cnt++;
      }
  }
  
  mean.x /= (double)cnt;
  mean.y /= (double)cnt;
  mean.z /= (double)cnt;

  return mean;
}

/**
 * Get mean of 3d points
 */
cv::Point3d LearnerCore::GetMean3D(View& view)
{
  unsigned cnt=0;
  cv::Point3d mean(0.,0.,0.);

  for (unsigned j=0; j<view.keys.size(); j++)
    if (view.keys[j]->Have3D())
    {
      mean += view.keys[j]->pos;
      cnt++;
    }
  
  mean.x /= (double)cnt;
  mean.y /= (double)cnt;
  mean.z /= (double)cnt;

  return mean;
}

/**
 * Translate3D 3d location of keypoints
 */
bool LearnerCore::Translate3D(vector<cv::Ptr<View> > &views, cv::Point3d T )
{
  for (unsigned i=0; i<views.size(); i++)
  {
    vector<cv::Ptr<PKeypoint> > &keys = views[i]->keys;

    for (unsigned j=0; j<keys.size(); j++)
      if (keys[j]->Have3D())
        keys[j]->pos += T;
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
 * AddViewToModel
 */
int LearnerCore::AddViewToModel()
{
  if (view->pose.empty())
    return 0;

  view->id = model->views.size();
  model->views.push_back(view);
  codebook->InsertView(0,model->views.size()-1,model->views.back());

  if (model->views.size()==1 && param.computeObjectCenter)
  {
    // shift 3d points and set origin of the object
    cv::Point3d center = GetMean3D(model->views);
    if (param.computeObjectCenter)
    {
      Translate3D(model->views, -center);
      model->center = cv::Point3d(0.,0.,0.);
    }
    else model->center=center;

    // renew pose
    cv::Mat_<double> pt;
    model->views.back()->pose.t.copyTo(pt);
    double *t = model->views.back()->pose.t.ptr<double>(0);
    double *R = model->views.back()->pose.R.ptr<double>(0);
    PMat::MulAdd3(R,&center.x,&pt(0,0), t);
    refObject.t.copyTo(pt);
    PMat::MulAdd3(refObject.R.ptr<double>(0),&center.x, &pt(0,0), refObject.t.ptr<double>(0));
  }

  //set view center in px
  double pos[3];
  View &view = *model->views.back();
  PMat::MulAdd3( view.pose.R.ptr<double>(), &model->center.x, view.pose.t.ptr<double>(), pos);
  ProjectPoint2Image(pos, param.intrinsic.ptr<double>(), &view.center.x);

  // add to completeness representation (viewpoint historgram)
  if (!model->viewHist.empty())
  {
    cv::Point3d vr;
    ConfValues predProb(param.intrinsic, param.distortion);
    
    // compute view ray..
    //cv::Point3d objCenter = GetMean3D(view);
    ComputeViewRay(view.pose, model->center, vr);

    model->viewHist->InsertMax(model->views.size()-1,vr, predProb);
  }

  return 2;
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
 * @return status of learning (0..not_recognised, 1..recognised_not_learned, 2..learned)
 */
int LearnerCore::Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask)
{
  #ifdef DEBUG
  cout<<"************************* PROCESS A NEW FRAME **********************************"<<endl;
  //if (!dbgWin.empty()) dbgWin->SetImage(image);
  //if (!dbgWin.empty()) dbgWin->SetPointCloud(cloud);
  #endif

  // init ...
  int status;
  double maxConf=0;
  Setup(image, cloud, R, T, oid);

  // detect keypoints and descriptors of current frame
  detector->Detect(imgGray, view->keys, mask);
  PKeypoint::ConvertToCv(view->keys,cvKeys);
  extractor->compute(imgGray, cvKeys, view->descriptors);

  //try to recognise the object and link keypoints
  status = RecogniseAndLink(maxConf);

  // set 3d coordinates
  if (status == 2 || param.forceLearning)
  {
    if (refObject.empty())
    {
      if (!R.empty() && !T.empty())
      {
        Pose pose(R,T);
        InvPose(pose, invRefGlobal);
      }
      refObject = view->pose;
    }

    // align keypoints with 3d model using the point cloud
    AlignKeypoints3D(cloud, view->keys, view->pose);

    // add view
    status = AddViewToModel();

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
  }

  #ifdef DEBUG
  cout<<"model id="<<oid<<", conf="<<maxConf<<": views.size()="<<model->views.size()<<endl;
  #endif

  return status;
}

/**
 * Detect keypoints in image
 * (backward compatibility to BLORT)
 */
void LearnerCore::DetectKeypoints(const cv::Mat &image, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  // init ...
  if (image.cols != param.width || image.rows != param.height)
    throw runtime_error("LearnerCore::DetectKeypoints : Wrong image size!");

  imgGray = image;
  if( image.type() != CV_8U ) cv::cvtColor( image, imgGray, CV_BGR2GRAY );

  // detect keypoints
  detector->Detect(imgGray, keys, mask);
}

/**
 * Add keypoints to object model
 * Attention: DetectKeypoint has to be called before (to store the image)!
 * (backward compatibility to BLORT)
 */ 
int LearnerCore::InsertToModel(const vector<cv::Ptr<PKeypoint> > &keys,cv::Mat R, cv::Mat T, const string &oid)
{
  // init ...
  if (param.intrinsic.empty())
    throw runtime_error("LearnerCore::DetectKeypoints : Camera parameter are not set!");
  if (!model->id.empty() && model->id.compare(oid)!=0)
    throw runtime_error("LearnerCore::DetectKeypoints : Invalide object id!");

  // set the object pose
  Pose pose(R,T);
  view = new View();

  if ( !pose.empty() )
  {
    if ( invRefGlobal.empty() ) InvPose(pose,invRefGlobal);
    if ( refObject.empty() ) refObject = pose;
    MulPose(invRefGlobal, refObject, view->pose);
    MulPose(pose, view->pose, view->pose);
  }

  if (model->id.empty()) model->id = oid;

  for (unsigned i=0; i<keys.size(); i++)
    if (keys[i]->Have3D())
      view->keys.push_back(keys[i]);

  PKeypoint::ConvertToCv(view->keys,cvKeys);
  extractor->compute(imgGray, cvKeys, view->descriptors);

  //try to recognise the object and link keypoints
  int status;
  double maxConf=0;
  status = RecogniseAndLink(maxConf);

  // add view
  if (status == 2 || param.forceLearning)
  {
    if (refObject.empty())
    {
      if (!R.empty() && !T.empty())
      {
        Pose pose(R,T);
        InvPose(pose, invRefGlobal);
      }
      refObject = view->pose;
    }

    status = AddViewToModel();
  }

  #ifdef DEBUG
  cout<<"model id="<<oid<<", conf="<<maxConf<<": views.size()="<<model->views.size()<<endl;
  #endif

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

  for (unsigned i=0; i<model->views.size(); i++)
    codebook->InsertView(0,i,model->views[i]);
}

/**
 * Access the learnModeled model
 */
cv::Ptr<CModel> &LearnerCore::GetModel()
{
  return model;
}

/**
 * GetViewRays
 */
void LearnerCore::GetViewRays(vector<cv::Point3d> &vr)
{
  if (!model->viewHist.empty())
    model->viewHist->GetViewRays(vr, param.minProbLearn, param.maxProbLearn);
}

/**
 * set camera parameter 
 */
void LearnerCore::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion)
{
  param.intrinsic = _intrinsic;
  param.distortion = _distortion;

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(param.intrinsic, CV_64F);

  if (_distortion.type() != CV_64F)
    _distortion.convertTo(param.distortion, CV_64F);

  estimator->SetCameraParameter(param.intrinsic, param.distortion);
}









} //-- THE END --

