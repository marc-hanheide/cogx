
#include <map>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PMath/PMath.hh"
#include "v4r/CModelRecogniser/CModelHandler.hh"
#include "v4r/CModelRecogniser/KeypointDetectorSURF.hh"
#include "v4r/CModelRecogniser/KeypointDetector.hh"
#include "v4r/CModelRecogniser/ObjectLocation.hh"
#include "v4r/CModelRecogniser/RecogniserCore.hh"
//#include "v4r/CModelRecogniser/RecogniserThread.hh"



using namespace std;


int main(int argc, char *argv[] )
{
  struct timespec start1, end1;
  int key;
  cv::Mat imgLoad, imgCol, imgGray, imgDraw;
  char filename[PATH_MAX];
  vector<cv::KeyPoint> keys;
  cv::Mat_<float> descriptors;
  vector<P::ObjectLocation> objects;
  cv::Mat intrinsic, distortion(cv::Mat::zeros(4,1,CV_64F));
  cv::FileStorage fs("/home/hannes/images/IROS2011/Resources/intrinsicKinect.xml", cv::FileStorage::READ);
  fs["intrinsic"] >> intrinsic;

  //P::RecogniserThread rec;
  //rec.ClearLearn();
  //rec.ClearLearn();
  //rec.ClearLearn();

  cv::Ptr<P::KeypointDetector> detector = new P::KeypointDetectorSURF();          // SURF
  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor(3,4,true);
  cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::L2<float> >();

  cv::Ptr<P::RecogniserCore> recogniser = new P::RecogniserCore(detector,extractor,matcher,P::RecogniserCore::Parameter());
  
  recogniser->SetCameraParameter(intrinsic,distortion);

  P::CModelHandler cmhandler;
  vector<cv::Ptr<P::CModel> > models(16);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/3DVision.cm",models[0]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Burti.cm",models[1]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/DenkMit.cm",models[2]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/DigestiveBraun.cm",models[3]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Digestive.cm",models[4]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/DigestiveDark.cm",models[5]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Dressing.cm",models[6]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Koala.cm",models[7]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Lindt.cm",models[8]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/MozartHerzen.cm",models[9]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/MunterMacher.cm",models[10]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Peanuts.cm",models[11]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Ricola.cm",models[12]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/SchokoKnusper.cm",models[13]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/WickBlau.cm",models[14]);
  cmhandler.Load("/home/hannes/images/IROS2011/Resources/Xerox.cm",models[15]);

  for (unsigned i=0; i<models.size(); i++)
    recogniser->AddModel(models[i]);


  cv::namedWindow("Image", 1);

  //imgLoad = cv::imread("/home/hannes/images/IROS2011/Resources/Peanuts_0000.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/Resources/Koala_0000.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/Resources/MunterMacher_0000.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/Resources/MunterMacher-test.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-05-dbg_0010.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-02_0005.jpg", 1); //2
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-08_0002.jpg", 1);
  //imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-09_0003.jpg", 1); //!!!!!!!!!1
  imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-10_0002.jpg", 1);


  imgLoad.copyTo(imgCol);
  imgLoad.copyTo(imgDraw);
  imgGray = imgCol;
  if( imgCol.type() != CV_8U ) cv::cvtColor( imgCol, imgGray, CV_BGR2GRAY);


  cv::imshow("Image",imgDraw);

  // ***************************************
  clock_gettime(CLOCK_REALTIME, &start1);
  
  recogniser->dbg = imgDraw;
  recogniser->Recognise(imgGray, objects);

  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time recogniser object [s]: "<<PMath::timespec_diff(&end1, &start1)<<endl;
  // ****************************************
  cv::imshow("Image",imgDraw);

  cv::waitKey(0);

  cv::destroyWindow("Image");	
	
	return 0;
}




