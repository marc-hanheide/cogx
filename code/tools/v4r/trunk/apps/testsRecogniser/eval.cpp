/**
 * $Id$
 */
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/ModelCoefficients.h>

#include <v4r/PCore/Point3fCol.hh>
#include <v4r/PCore/toString.hpp>
#include <v4r/TomGine/tgTomGineThread.h>
#include <v4r/PCLAddOns/PlanePopout.hh>
#include <v4r/PCLAddOns/utils/PCLUtils.h>
#include <v4r/PCLAddOns/functions/PCLFunctions.h>
#include <v4r/CModelRecogniser/RecogniserThread.hh>
#include <v4r/CModelRecogniser/CModelHandler.hh>
#include <v4r/CModelRecogniser/ObjectLocation.hh>
#include <v4r/EllCalib/arDetectGTPose.hh>
#include <v4r/PMath/PMath.hh>
#include <v4r/PGeometry/Pose.hh>
#include "TgModelProbSphere.hh"


using namespace std;

#define THR_OK_POSE_DISTANCE .04
#define THR_OK_POSE_ANGLE 15.

#define M_IDLE_TIME 500          //[ms]
#define MAX_QUEUE_SIZE 1 

#define PC_FILE_NAME "log/pointcloud_%04d.pcd"
#define IM_FILE_NAME "log/image_%04d.jpg"

//#define USE_SURF_CC       // use cpu-surf and cpu-matcher or ...
#define USE_SIFT_GC     // use gpu-sift and cpu-matcher or ...
//#define USE_SIFT_GG     // use gpu-sift and gpu-matcher    


double GetAngleBetweenViews(P::Pose &refPose, P::Pose &pose);
void ComputeViewRay(P::Pose &pose, cv::Point3d &center, cv::Point3d &vr);
bool GetImagePointFromCloud(cv::Point2f &ptImg, cv::Mat_<cv::Vec4f> &matCloud, cv::Point3f &pt);
void DoStatistics(vector<double> &dists, double &mean, double &sigma);
void DoMeanStatistics(vector<double> &means, vector<double> &sigmas);

void SelectPopout(const cv::Mat &cam, P::Pose &pose, const cv::Mat_<ushort> &labels, 
      const std::vector<unsigned> &sizeClusters, cv::Point3d boxCenter, double boxSize, 
      cv::Mat_<uchar> &mask);
void DrawFillPoly(cv::Mat &img, vector<cv::Point> &vs);





class RecEvalData
{
public:
  bool okResult;                // correct recognition result
  double conf1;                 // num_supporting_points / num_points_of_recognised_view  (no matches used)
  double angle;                 // angle between current view and nearest learned view
  double scale;                 // scale of current view and nearest learnded view
  unsigned numModelKeys;        // number of keypoints of nearest model view
  cv::Point3d err;              // error in x,y,z (z = camera view ray)
  double errAngle;
  double time;
  RecEvalData() : okResult(0), conf1(0.), angle(0.), scale(1.), time(0.), numModelKeys(0), errAngle(0) {};
  ~RecEvalData(){};
};
ostream& operator<<(ostream &os, const RecEvalData &e)
{
  os << e.okResult<<' '<<e.conf1<<' '<<e.angle<<' '<<e.scale<<' '<<e.numModelKeys<<' '<<time<<' '<<'['<<' '<<e.err.x<<' '<<e.err.y<<' '<<e.err.z<<' '<<"] "<<e.errAngle<<' ';
  return os;
}



/**
 * The openni grabber and the main loop
 */
class OpenNIProcessor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool eval;
  bool stop;
  int mode;              // 0..nothing, 1..recognise, 2..learn
  unsigned cfg;              

  static const unsigned LOAD_POINTCLOUDS = 1;
  static const unsigned LOG_POINTCLOUDS = 2;

  cv::Ptr<TomGine::tgTomGineThread> dbgWin;
  unsigned uIdleTime;
  unsigned cntImagesEval;

  pthread_mutex_t mutData;
  string filename;

  cv::Mat_<cv::Vec4f> drwCloud, matCloud;
  cv::Mat_<cv::Vec3b> dbg, image;
  cv::Mat_<ushort> labels;
  cv::Mat_<uchar> mask;
  vector<unsigned> sizeClusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloudFiltered;
  queue< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > shClouds;
  pcl::ModelCoefficients::Ptr tableCoefficients;
  pcl::PointIndices popouts;

  cv::Ptr<pcl::Grabber> interface;
  cv::Ptr<pclA::PlanePopout> planePopout;
  cv::Ptr<P::RecogniserThread> recogniser;
  P::CModelHandler cmhandler;
  cv::Ptr<P::CModel> model;
  vector<P::ObjectLocation> objects;

  cv::Mat cam, distCoeffs;

  //container for drawing
  TomGine::TgModelProbSphere probSphere;

  // ell pattern detector
  P::arDetectGTPose::Parameter param1, param2;
  P::arDetectGTPose detector;
  P::EllPattern pattern;
  P::Pose pose;

  // eval stuff
  vector< RecEvalData > evalData;


  OpenNIProcessor () 
   : eval(true), stop(false), mode(0), cfg(0), uIdleTime(M_IDLE_TIME*1000), cntImagesEval(0)
  {
    #ifdef USE_SURF_CC
    P::RecogniserCore::Parameter paramRecogniser = 
      P::RecogniserCore::Parameter(640,480, 5000,100,0.01,2.,5., false, .3, .15, .8);
    P::LearnerCore::Parameter paramLearner = 
      P::LearnerCore::Parameter(640,480,1000,50,0.01,2, 5., .3, .15, 2., false, false, .01, .1);
    recogniser = new P::RecogniserThread(P::RecogniserThread::SURF_CC,paramRecogniser,paramLearner);
    #else
    P::RecogniserCore::Parameter paramRecogniser = 
      P::RecogniserCore::Parameter(640,480,5000,100,0.01,2.,5., false, .35, .15, .8);
    P::LearnerCore::Parameter paramLearner = 
      P::LearnerCore::Parameter(640,480,1000,50,0.01,2, 5., .2, .1, 2., false, false, .01, .1);
    #ifdef USE_SIFT_GC
    recogniser = new P::RecogniserThread(P::RecogniserThread::SIFT_GC,paramRecogniser,paramLearner);
    #else //USE_SIFT_GG
    recogniser = new P::RecogniserThread(P::RecogniserThread::SIFT_GG,paramRecogniser,paramLearner);
    #endif
    #endif
    

    pthread_mutex_init(&mutData,NULL);
    cloudFiltered.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
    tableCoefficients.reset (new pcl::ModelCoefficients());
    SetMagicKinectCamera(*recogniser, cam, distCoeffs);
    //param1 = P::arDetectGTPose::Parameter(9, 28.92, 28.8, -5, 0, -1, 11, -4, 4, 5.);
    //param2 = P::arDetectGTPose::Parameter(9, 28.92, 28.8, 4 ,0, -10, 2, -4, 4, 5.);
    param1 = P::arDetectGTPose::Parameter(9, 28.75, 28.75, -5, 0, -1, 11, -4, 4, 5.); //kth
    param2 = P::arDetectGTPose::Parameter(9, 28.75, 28.75, 4 ,0, -10, 2, -4, 4, 5.);

    detector = P::arDetectGTPose("../v4r/EllCalib/pattern/1.pat",
                                 "../v4r/EllCalib/pattern/2.pat", param1,param2);
    double sacThr = 0.08;	// SAC threshold
    double eucThr = 0.01;	// Euclidean clustering threshold (cclabeling)
    planePopout = new pclA::PlanePopout(pclA::PlanePopout::Parameter(0.0, 1.0, 
                        0.02, 0.01, 10, sacThr, 0.1, 0.005, 0.7, eucThr, 100));

  }
  ~OpenNIProcessor ()
  {
    pthread_mutex_destroy(&mutData);
  }

  void SetMagicKinectCamera(P::RecogniserThread &recogniser, cv::Mat &cam, cv::Mat &distCoeffs);
  void DrawLearning(P::CModel &model, cv::Ptr<TomGine::tgTomGineThread> &win, unsigned idView=UINT_MAX);
  void ParseOptions(int argc, char **argv);
  void TryLoadObject(const string &file);
  void CallbackCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &_cloud);
  void SaveResults(const string &matfile, vector<RecEvalData> &dat); 
  void Eval(P::CModel &model, vector<P::ObjectLocation> &res, P::Pose &gtPose);
  void run ();


};

/***************************** SOME METHODES ***********************************/
/**
 * Parse command line options, namely given images.
 */
void OpenNIProcessor::ParseOptions(int argc, char **argv)
{
  int c;
  while(1)
  {
    c = getopt(argc, argv, "lf");
    if(c == -1)
      break;
    switch(c)
    {
      case 'l':
        cfg |= LOG_POINTCLOUDS;
        break;
      case 'f':
        cfg |= LOAD_POINTCLOUDS; 
        break;
    }
  }
  // remaining argument is image filename to load
  if (optind < argc)
  {
    TryLoadObject(argv[optind++]);
  }
  else
  {
    printf("%s [-f] [-l] object-file.cm\n"
        "   -f load pointclouds from file\n"
        "   -l log pointclouds and images\n", argv[0]);
    exit(1);
  }
}

/**
 * Load object model if exist
 */
void OpenNIProcessor::TryLoadObject(const string &file)
{
  filename = file;
  if (cmhandler.Load(filename, model))
  {
    cmhandler.RenewProbSphere(cam,distCoeffs,*model);
    recogniser->SetModelLearn(model);
    recogniser->AddModelRecogniser(model);
  }
}

/**
 * Kinect is calibrated...
 * ...here are the magic calibration coefficients
 */
void OpenNIProcessor::SetMagicKinectCamera(P::RecogniserThread &recogniser, cv::Mat &cam, cv::Mat &distCoeffs)
{
  cam = cv::Mat::zeros(3,3,CV_64F);
  //distCoeffs = cv::Mat::zeros(5,1,CV_64F);

  cam.at<double>(0,0) = cam.at<double>(1,1) = 525;
  cam.at<double>(0,2) = 320;
  cam.at<double>(1,2) = 240;
  cam.at<double>(2,2) = 1.;

  recogniser.SetCameraParameter(cam,distCoeffs);
}


/**
 * Draw results
 */
void OpenNIProcessor::DrawLearning(P::CModel &model, cv::Ptr<TomGine::tgTomGineThread> &win, unsigned idView)
{
  if (win.empty())
  { 
    win = new TomGine::tgTomGineThread(cloud->width,cloud->height);
    win->SetCamera(cam);
    //win->AddPointCloud( cv::Mat() );
    win->AddModel(&probSphere);
  }
  else
  {
    if (idView < UINT_MAX && model.views.size() && model.views[idView]->pointcloud.size()>0)
    {
      win->ClearPoints3D();
      pclA::RGBValue col;
      for (unsigned i=0; i<model.views[idView]->pointcloud.size(); i++)
      {
        float *d = &model.views[idView]->pointcloud[i][0];
        col.float_value = d[3];
        win->AddPoint3D(d[0],d[1],d[2], 0,0,255);
      }

      win->SetPointCloud(0, cv::Mat(model.views[idView]->pointcloud) );
    }
  }
 
  win->SetImage(image);

  //>>>> set camera pose
  if (!model.pose.empty()){ 
    P::Pose invPose;
    P::InvPose(model.pose,invPose);
    win->SetCamera(invPose.R, invPose.t);
  }

  //>>>> draw prob sphere
  if (!model.viewHist.empty())
  {
    probSphere.Lock();
    model.viewHist->copyTo(probSphere);  
    probSphere.Unlock();
  }

  //>>> draw model points
  //win->ClearPoints3D();
  for (unsigned i=0; i<model.views.size(); i++)
  {
    P::View &view = *model.views[i];
    for (unsigned j=0; j<view.keys.size(); j++)
    {
      if (!view.keys[j]->pos.empty())
      {
        double *d = &view.keys[j]->pos->pt.x;
        win->AddPoint3D(d[0],d[1],d[2], 255,255,255, 2);
      }
    }
  }
  win->Update();
}


/**
 * The main loop..
 */
void OpenNIProcessor::CallbackCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &_cloud)
{
  //cout<<"Have a new image: "<<_cloud->width<<"x"<<_cloud->height<<endl;
  pthread_mutex_lock(&mutData);
  if (shClouds.size()<MAX_QUEUE_SIZE){
    shClouds.push( pcl::PointCloud<pcl::PointXYZRGB>::Ptr() );
    shClouds.back().reset (new pcl::PointCloud<pcl::PointXYZRGB>());
    pclA::CopyPointCloud(*_cloud, *shClouds.back());
  } else cout<<"Drop frame ..."<<endl;
  pthread_mutex_unlock(&mutData);

  usleep(uIdleTime);
}

/**
 * save results
 */
void OpenNIProcessor::SaveResults(const string &matfile, vector<RecEvalData> &dat)
{
  cout<<"Save to file: "<<matfile<<"..."; // write tp/tn matlab file
  vector<RecEvalData> tp, tn;
  for (unsigned i=0; i<dat.size(); i++)
  {
    if(dat[i].okResult) tp.push_back(dat[i]);
    else tn.push_back(dat[i]);
  }
  ofstream ofMat(matfile.c_str());
  time_t t = time(NULL);
  ofMat<<"# recognizer evaluation file generated "<<ctime(&t);
  ofMat<<"# name: tp_conf1"<<'\n';       //write conf1
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tp.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tp.size(); i++)
    ofMat<<tp[i].conf1<<'\n';
  ofMat<<"# name: tn_conf1"<<'\n';
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tn.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tn.size(); i++)
    ofMat<<tn[i].conf1<<'\n';
  ofMat<<"# name: tp_angle"<<'\n';      // write angle 
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tp.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tp.size(); i++)
    ofMat<<tp[i].angle<<'\n';
  ofMat<<"# name: tn_angle"<<'\n';
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tn.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tn.size(); i++)
    ofMat<<tn[i].angle<<'\n';
  ofMat<<"# name: tp_scale"<<'\n';       // write scale 
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tp.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tp.size(); i++)
    ofMat<<tp[i].scale<<'\n';
  ofMat<<"# name: tn_scale"<<'\n';
  ofMat<<"# type: matrix"<<'\n';
  ofMat<<"# rows: "<<tn.size()<<'\n';
  ofMat<<"# columns: 1"<<'\n';
  for (unsigned i=0; i<tn.size(); i++)
    ofMat<<tn[i].scale<<'\n';
  ofMat.close();

  cout<<"ok"<<endl;
}


/**
 * Eval...
  bool okResult;                // correct recognition result
  double conf1;                 // num_supporting_points / num_points_of_recognised_view  (no matches used)
  double angle;                 // angle between current view and nearest learned view
  double scale;                 // scale of current view and nearest learnded view
  unsigned numModelKeys;        // number of keypoints of nearest model view
  cv::Point3d err;              // error in x,y,z (z = camera view ray)
  double time;
  RecEvalData() : okResult(0), conf1(0.), angle(0.), scale(1.), time(0.), numModelKeys(0) {};
 */
void OpenNIProcessor::Eval(P::CModel &model, vector<P::ObjectLocation> &res, P::Pose &gtPose)
{
  cv::Point3d dErr;
  double angErr;

  for (unsigned i=0; i<res.size(); i++)
  {
    evalData.push_back(RecEvalData());

    PVec::Sub3(gtPose.t.ptr<double>(),res[i].pose.t.ptr<double>(),&dErr.x);
    evalData.back().err = dErr;
    evalData.back().errAngle = angErr;
    angErr = GetAngleBetweenViews(res[i].pose,gtPose);
    if (PVec::Norm3(&dErr.x)<THR_OK_POSE_DISTANCE && angErr*180./M_PI<THR_OK_POSE_ANGLE)
      evalData.back().okResult = true;
    else evalData.back().okResult = false;

    evalData.back().numModelKeys = model.views[res[i].idView]->keys.size();
    double l1 = PVec::Norm3(model.views[res[i].idView]->pose.t.ptr<double>());
    double l2 = PVec::Norm3(res[i].pose.t.ptr<double>());
    evalData.back().scale = l1/l2; //(l2>l1?l1/l2:l2/l1);
    evalData.back().angle = GetAngleBetweenViews(model.views[res[i].idView]->pose,res[i].pose);

    evalData.back().conf1 = res[i].conf;

    cout<<evalData.back()<<endl;
  }
  
}



/**
 * just start the grabber and init the bind callback
 */
//static vector<double> means1, sigmas1, means2, sigmas2;
void OpenNIProcessor::run ()
{
  if (cfg&LOAD_POINTCLOUDS)
  {
    cv::waitKey(0);
    cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  }
  else
  {
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
      boost::bind (&OpenNIProcessor::CallbackCloud, this, _1);
    interface->registerCallback (f);
    cv::waitKey(0);          // just wait till the user hits a key...
    interface->start ();
  }

  while (!stop)
  {      
    if (cfg&LOAD_POINTCLOUDS)
    {
      char imFilename[PATH_MAX];
      snprintf(imFilename,1024, PC_FILE_NAME, cntImagesEval);
      cout<<imFilename<<endl;
      if(pcl::io::loadPCDFile(imFilename, *cloud)==-1 && eval){
        eval = !eval;
        SaveResults(filename+".mat",evalData);
        evalData.clear();
        mode=0;
        cout<<"-- END of image sequence ------------------------------------------ "<<imFilename<<endl;
      }
      cntImagesEval++;
    }
    else
    {
      pthread_mutex_lock(&mutData);
      if (!shClouds.empty()){
        cloud = shClouds.front();
        shClouds.pop();
      } else cloud.reset();
      cout<<"Size of the point cloud queue: "<<shClouds.size()<<endl;
      pthread_mutex_unlock(&mutData);
    }

    if(cloud.get()!=0)
    {
      pclA::ConvertPCLCloud2Image(cloud, image);
      image.copyTo(dbg);

      if (cfg&LOG_POINTCLOUDS)
      {
        char imFilename[PATH_MAX];
        snprintf(imFilename,1024, PC_FILE_NAME, cntImagesEval);
        pcl::io::savePCDFileBinary(imFilename, *cloud);
        snprintf(imFilename,1024, IM_FILE_NAME, cntImagesEval);
        cv::imwrite(imFilename,image);
        cntImagesEval++;
      }

      switch (mode)
      {
        case 1:
          recogniser->SetDebugImage(dbg);
          recogniser->Recognise(image,objects);        // recognise !!

          if (eval!=eval)            ///HACK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          {
            pose = P::Pose();
            detector.dbg = cv::Mat();//dbg;
            if(detector.GetPose(image, cam, distCoeffs, pose.R, pose.t, pattern))
            {
              pose.t *= 0.001;
              Eval(*model, objects, pose); 
            }
          }
          if (objects.size()>0)
          {
            model->pose=objects[0].pose;
            DrawLearning(*model, dbgWin, objects[0].idView);
          }
          break;
        case 2:
          pose = P::Pose();
          detector.dbg = dbg;
          mask = cv::Mat(image.rows,image.cols,CV_8U);
          
          if(detector.GetPose(image, cam, distCoeffs, pose.R, pose.t, pattern) && !pose.empty())
          {
            pose.t *= 0.001;

            planePopout->FilterZ(cloud, *cloudFiltered);
            planePopout->DetectPopout(cloudFiltered, popouts);
            planePopout->ConvertPopout2Mat(*cloud, popouts, matCloud);
            planePopout->LabelClusters(matCloud, labels, sizeClusters);
            SelectPopout(cam, pose, labels, sizeClusters, cv::Point3d(0.,0.,0.), 0.2, mask);
            //planePopout->CreateMaskAll(labels, sizeClusters, mask);

            recogniser->SetDebugImage(dbg);
            int status = recogniser->Learn(image, matCloud, pose.R, pose.t, filename, mask);
          }

          recogniser->GetModelLearn(model);

          DrawLearning(*model, dbgWin, model->views.size()-1);

          cv::imshow("Mask",mask);
          break;
      }

      cv::imshow("Image",dbg);

      char key = cv::waitKey(100);
      switch (key)
      {
        case 27:
          stop=true;
          break; 
        case 'p':
            recogniser->SetReferenceObjectPose(objects[0].pose);
            cout<<"Set reference object pose!!!!!"<<endl;
            cv::waitKey(1000);
          break;
        case 'r':
          mode=1;
          cntImagesEval=0;
          recogniser->GetModelLearn(model);
          recogniser->ClearRecogniser();
          recogniser->AddModelRecogniser(model);
          recogniser->OptimizeCodebook();
          break;
        case 'l':
          mode=2;
          break;
        case 'i':
          mode=0;
          break;
        case 'w':
          recogniser->GetModelLearn(model);
          cmhandler.Save(filename, model);
          break;
        case 'e':
          eval = !eval;
          SaveResults(filename+".mat",evalData);
          evalData.clear();
          break;
        case '+':
          uIdleTime+=100000;
          break;
        case '-':
          uIdleTime-=100000;
          break;
      }
      if (!dbgWin.empty() && dbgWin->Stopped()) {stop = true;}
    }
    else 
    {
      usleep(uIdleTime/5);
    }
  }

  if (!cfg&LOAD_POINTCLOUDS)
  {
    sleep(1);
    interface->stop ();
    sleep(1);
  }
}



/**********************************************************************************
 * main
 */
int main (int argc, char** argv)
{
  printf("\n Demo CModelRecogniser\n\n");

  printf(" CModelRecogniser control\n");
  printf(" -------------------------------------------\n");
  printf(" [l] Learning mode\n");
  printf(" [r] Recognition mode\n");
  printf(" [i] Idle\n");
  printf(" [w] Write model to file\n\n");
  printf(" Press a key to start...\n\n");


  cv::namedWindow("Image", 1);
  cv::namedWindow("Mask", 1);

  OpenNIProcessor v;
  v.ParseOptions(argc, argv);
  v.run ();

  cv::destroyWindow("Image");
  cv::destroyWindow("Mask");

  return 0;
}








/****************************** SOME HELPER FUNCTIONS *****************************/
/**
 * Compute the angle between two views
 */
double GetAngleBetweenViews(P::Pose &refPose, P::Pose &pose)
{
  cv::Point3d vr1, vr2;
  cv::Point3d center = cv::Point3d(0.,0.,0.);

  ComputeViewRay(refPose, center, vr1);
  ComputeViewRay(pose, center, vr2);

  return acos(PVec::Dot3(&vr1.x,&vr2.x));
}


/**
 * Get angle between views
 */
void ComputeViewRay(P::Pose &pose, cv::Point3d &centerObject, cv::Point3d &vr)
{
  double pos[3];
  P::Pose invPose;
  P::InvPose(pose,invPose);

  pos[0]=0., pos[1]=0., pos[2]=0.;
  PMat::MulAdd3( invPose.R.ptr<double>(), pos, invPose.t.ptr<double>(), &vr.x);
  vr -= centerObject;
  PVec::Normalise3(&vr.x,&vr.x);
}

/**
 * interpolate the 3d point
 */
bool GetImagePointFromCloud(cv::Point2f &ptImg, cv::Mat_<cv::Vec4f> &matCloud, cv::Point3f &pt)
{
  int x = (int)ptImg.x;
  int y = (int)ptImg.y;
  float dx = ptImg.x-x;
  float dy = ptImg.y-y;

  cv::Vec4f &pt11 = matCloud(y,x);
  cv::Vec4f &pt12 = matCloud(y,x+1);
  cv::Vec4f &pt21 = matCloud(y+1,x);
  cv::Vec4f &pt22 = matCloud(y+1,x+1);

  if (pt11[0]==pt11[0] && pt11[1]==pt11[1] && pt11[2]==pt11[2] && pt11[3]==pt11[3] &&
      fabs(pt12[2]-pt11[2]) < 0.01 && fabs(pt21[2]-pt11[2]) < 0.01 && fabs(pt12[2]-pt11[2]) < 0.01)
  {
    pt.x = ( (1-dx)*(1-dy)*pt11[0] + dx*(1-dy)*pt12[0] + (1-dx)*dy*pt21[0] + dx*dy*pt22[0] );
    pt.y = ( (1-dx)*(1-dy)*pt11[1] + dx*(1-dy)*pt12[1] + (1-dx)*dy*pt21[1] + dx*dy*pt22[1] );
    pt.z = ( (1-dx)*(1-dy)*pt11[2] + dx*(1-dy)*pt12[2] + (1-dx)*dy*pt21[2] + dx*dy*pt22[2] );
    return true;
  }
  return false;
}

void DoStatistics(vector<double> &dists, double &mean, double &sigma)
{
  if (dists.size()>1) 
  {
    mean=0, sigma=0;
    for (unsigned i=0; i<dists.size(); i++)
      mean+=dists[i];
    mean/=(double)dists.size();

    for (unsigned i=0; i<dists.size(); i++)
      sigma+=PMath::Sqr(dists[i]-mean);
    sigma = sqrt(1./(dists.size()-1) * sigma);

    cout<<"-> mean="<<mean<<", sigma="<<sigma<<endl;
  }
}

void DoMeanStatistics(vector<double> &means, vector<double> &sigmas)
{
  if (means.size()>0 && sigmas.size()>0)
  {
    double mean=0, sigma=0;
    for (unsigned i=0; i<means.size(); i++)
    {
      mean+=means[i];
      sigma+=sigmas[i];
    }

    mean/=(double)means.size();
    sigma/=(double)means.size();
    cout<<"mean-mean="<<mean<<", mean-sigma="<<sigma<<endl;
  }
}

/**
 * Select popout within roi
 */
void SelectPopout(const cv::Mat &cam,  P::Pose &pose, const cv::Mat_<ushort> &labels, 
      const std::vector<unsigned> &sizeClusters, cv::Point3d boxCenter, double boxSize, 
      cv::Mat_<uchar> &mask)
{
  mask = cv::Mat::zeros(labels.rows,labels.cols,CV_8U);
  
  if (sizeClusters.size()==0) return;

  cv::Mat_<uchar> roi(labels.rows,labels.cols);
  cv::Point2f pt;
  cv::Point3f objPos;
  vector<cv::Point> pts;
  vector<cv::Point> hull;

  // project roi to image...
  for (int x=-1; x<=1; x+=2)
    for (int y=-1; y<=1; y+=2)
      for (int z=0; z<=1; z++)
      {
        cv::Point3f pt3(boxCenter.x+boxSize*x, boxCenter.y+boxSize*y,boxCenter.z+boxSize*z);
        PMat::MulAdd3(pose.R.ptr<double>(), &pt3.x, pose.t.ptr<double>(), &objPos.x);
        P::ProjectPoint2Image(&objPos.x, cam.ptr<double>(), &pt.x);
        pts.push_back( cv::Point((int)(pt.x+.5),(int)(pt.y+.5)) );
      }

  convexHull(pts, hull, false, true);
  DrawFillPoly(roi, hull);

  //count labels in roi...
  vector<unsigned> cntLabels(sizeClusters.size(),0);
  for (int v=0; v<labels.rows; v++)
    for (int u=0; u<labels.cols; u++)
      if (roi(v,u)>0)
        cntLabels[labels(v,u)]++;

  // get max...
  double max=DBL_MIN;
  short label;
  for (unsigned i=1; i<cntLabels.size(); i++)
    if (cntLabels[i]>max)
    {
      max = cntLabels[i];
      label = i;
    }

  // set mask..
   for (int v=0; v<labels.rows; v++)
    for (int u=0; u<labels.cols; u++)
      if (labels(v,u)==label)
        mask(v,u) = 255;
}

void DrawFillPoly(cv::Mat &img, vector<cv::Point> &vs)
{
  CvPoint *pts[1];
  int npts[1];
  pts[0] = (CvPoint*)cvAlloc(vs.size()*sizeof(pts[0][0]));
  npts[0]=vs.size();

  for (unsigned i=0; i<vs.size(); i++){
    pts[0][i].x=vs[i].x;
    pts[0][i].y=vs[i].y;
  }

  IplImage iplImg(img);
  cvFillPoly(&iplImg, pts, npts, 1, CV_RGB(255,255,255) );

  cvFree(&pts[0]);
}



/*cv::Point3f ptCloud, pos;
pclA::ConvertPCLCloud2CvMat(*cloud, matCloud);
vector<double> dists1, dists2;
for (unsigned i=0; i<pattern.objPoints.size(); i++)
{
  cv::Point3f &ptObj = pattern.objPoints[i];
  cv::Point2f &ptImg = pattern.imgPoints[i];
  cv::Vec4f ptMat;
  if (GetImagePointFromCloud(ptImg, matCloud, ptCloud)) 
  {
    ptMat = matCloud(PMath::Round(ptImg.y),PMath::Round(ptImg.x));
    ptObj.x/=1000.;ptObj.y/=1000.;ptObj.z/=1000.;
    PMat::MulAdd3( pose.R.ptr<double>(), &ptObj.x, pose.t.ptr<double>(), &pos.x);
    if (ptMat[0]==ptMat[0] && ptCloud.x==ptCloud.x)
    {
      dists1.push_back(PVec::Distance3(&pos.x, &ptMat[0])*1000);
      dists2.push_back(PVec::Distance3(&pos.x, &ptCloud.x)*1000);
    }
  }
}

cout<<"dist="<<PVec::Norm3(pose.t.ptr<double>())<<endl;
double sigma, mean;
DoStatistics(dists1, mean, sigma);
means1.push_back(mean);
sigmas1.push_back(sigma);
DoStatistics(dists2, mean, sigma);
means2.push_back(mean);
sigmas2.push_back(sigma);
DoMeanStatistics(means1, sigmas1);
DoMeanStatistics(means2, sigmas2);*/

