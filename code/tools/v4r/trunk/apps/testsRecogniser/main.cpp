/**
 * $Id$
 */
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/ModelCoefficients.h>

#include <v4r/PCore/Point3fCol.hh>
#include <v4r/TomGine/tgTomGineThread.h>
#include <v4r/PCLAddOns/PlanePopout.hh>
#include <v4r/PCLAddOns/utils/PCLUtils.h>
#include <v4r/PCLAddOns/functions/PCLFunctions.h>
#include <v4r/CModelRecogniser/RecogniserThread.hh>
#include <v4r/CModelRecogniser/CModelHandler.hh>
#include <v4r/CModelRecogniser/ObjectLocation.hh>
#include <v4r/EllCalib/arDetectGTPose.hh>
#include "TgModelProbSphere.hh"


using namespace std;




/**
 * The openni grabber and the main loop
 */
class OpenNIProcessor
{
public:
  bool stop;
  int mode;              // 0..nothing, 1..recognise, 2..learn
  cv::Ptr<TomGine::tgTomGineThread> dbgWin;

  pthread_mutex_t mutData;
  string filename;

  cv::Mat_<cv::Vec4f> matCloud;
  cv::Mat_<cv::Vec3b> dbg, image, shImage;
  cv::Mat_<ushort> labels;
  cv::Mat_<uchar> mask;
  vector<unsigned> sizeClusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, shCloud, cloudFiltered;
  pcl::ModelCoefficients::Ptr tableCoefficients;
  pcl::PointIndices popouts;

  cv::Ptr<pcl::Grabber> interface;

  pclA::PlanePopout planePopout;
  P::RecogniserThread recogniser;
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


  OpenNIProcessor () : stop(false), mode(0)
  {
    pthread_mutex_init(&mutData,NULL);
    cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
    shCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloudFiltered.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
    tableCoefficients.reset (new pcl::ModelCoefficients());
    SetMagicKinectCamera(recogniser, cam, distCoeffs);
    //param1 = P::arDetectGTPose::Parameter(9, 19.4,19.4, -5, 0, -1, 7, -4, 4, 5.);
    //param2 = P::arDetectGTPose::Parameter(9, 19.4,19.4, 4 ,0, -6, 2, -4, 4, 5.);
    param1 = P::arDetectGTPose::Parameter(9, 28.92, 28.8, -5, 0, -1, 11, -4, 4, 5.);
    param2 = P::arDetectGTPose::Parameter(9, 28.92, 28.8, 4 ,0, -10, 2, -4, 4, 5.);
    detector = P::arDetectGTPose("/home/hannes/XWorks/v4r/v4r/EllCalib/pattern/1.pat",
                                 "/home/hannes/XWorks/v4r/v4r/EllCalib/pattern/2.pat", param1,param2);
  }
  ~OpenNIProcessor ()
  {
    pthread_mutex_destroy(&mutData);
  }

  void SetMagicKinectCamera(P::RecogniserThread &recogniser, cv::Mat &cam, cv::Mat &distCoeffs);
  void DrawLearning(P::CModel &model, cv::Ptr<TomGine::tgTomGineThread> &win);
  void TryLoadObject(const string &file);
  void CallbackCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &_cloud);
  void run ();


};

/***************************** SOME METHODES ***********************************/

/**
 * Load object model if exist
 */
void OpenNIProcessor::TryLoadObject(const string &file)
{
  filename = file;
  if (cmhandler.Load(filename, model))
  {
    cmhandler.RenewProbSphere(cam,distCoeffs,*model);
    recogniser.SetModelLearn(model);
    recogniser.AddModelRecogniser(model);
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
void OpenNIProcessor::DrawLearning(P::CModel &model, cv::Ptr<TomGine::tgTomGineThread> &win)
{
  if (win.empty())
  { 
    win = new TomGine::tgTomGineThread(cloud->width,cloud->height);
    win->SetCamera(cam);
    //win->AddPointCloud(matCloud);
    win->AddModel(&probSphere);
  }
  //else win->SetPointCloud(0,matCloud);
 
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
  win->ClearPoints3D();
  for (unsigned i=0; i<model.views.size(); i++)
  {
    P::View &view = *model.views[i];
    for (unsigned j=0; j<view.keys.size(); j++)
    {
      double *d = &view.keys[j]->pos->pt.x;
      win->AddPoint3D(d[0],d[1],d[2], 255,255,255, 2);
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
  pclA::ConvertPCLCloud2Image(_cloud, shImage);
  pclA::CopyPointCloud(*_cloud, *shCloud);
  pthread_mutex_unlock(&mutData);
  usleep(10000);
}

/**
 * just start the grabber and init the bind callback
 */
void OpenNIProcessor::run ()
{

  interface = new pcl::OpenNIGrabber();

  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&)> f =
    boost::bind (&OpenNIProcessor::CallbackCloud, this, _1);

  interface->registerCallback (f);

  cv::waitKey(0);          // just wait till the user hits a key...

  interface->start ();

  while (!stop)
  {      
    pthread_mutex_lock(&mutData);
    shImage.copyTo(image);
    pclA::CopyPointCloud(*shCloud, *cloud);
    pthread_mutex_unlock(&mutData);

    if(!image.empty())
    {
      image.copyTo(dbg);
      switch (mode)
      {
        case 1:
          recogniser.Recognise(image,objects);
          if (objects.size()>0)
          {
            model->pose=objects[0].pose;
            DrawLearning(*model, dbgWin);
          }
          break;
        case 2:
          planePopout.FilterZ(cloud, *cloudFiltered);
          planePopout.DetectPopout(cloudFiltered, popouts);
          planePopout.ConvertPopout2Mat(*cloud, popouts, matCloud);
          planePopout.LabelClusters(matCloud, labels, sizeClusters);
          planePopout.CreateMaskLargest(labels, sizeClusters, mask);
          //planePopout.CreateMaskAll(labels, sizeClusters, mask);
          //planePopout.ConvertPointCloud2Mat(*cloud, matCloud);

          pose = P::Pose();
          detector.dbg = dbg;
          if(detector.GetPose(image, cam, distCoeffs, pose.R, pose.t, pattern))
            pose.t *= 0.001;

          int status = recogniser.Learn(image, matCloud, pose.R, pose.t, filename, mask);

          recogniser.GetModelLearn(model);

          DrawLearning(*model, dbgWin);
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
        case 'r':
          mode=1;
          recogniser.GetModelLearn(model);
          recogniser.ClearRecogniser();
          recogniser.AddModelRecogniser(model);
          recogniser.OptimizeCodebook();
          break;
        case 'l':
          mode=2;
          break;
        case 'i':
          mode=0;
          break;
        case 'w':
          recogniser.GetModelLearn(model);
          cmhandler.Save(filename, model);
          break;
      }
      if (!dbgWin.empty() && dbgWin->Stopped()) {stop = true;}
    }
  }

  sleep(1);
  interface->stop ();
  sleep(1);
}



/**********************************************************************************
 * main
 */
int main (int argc, char** argv)
{
  if (argc != 2)
  {
    printf("%s object-file.cm\n",argv[0]);
    return 0;
  }
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
  v.TryLoadObject(argv[1]);
  v.run ();

  cv::destroyWindow("Image");
  cv::destroyWindow("Mask");

  return 0;
}






