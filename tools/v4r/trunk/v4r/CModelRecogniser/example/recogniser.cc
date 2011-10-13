
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
#include "v4r/CModelRecogniser/LearnerCore.hh"
//#include "v4r/CModelRecogniser/RecogniserCore.hh"
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
  cv::Mat intrinsic(3,3,CV_64F), distortion(cv::Mat::zeros(4,1,CV_64F));

  cv::Ptr<P::KeypointDetector> detector = new P::KeypointDetectorSURF();          // SURF
  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor(3,4,true);
  cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::L2<float> >();

  //cv::Ptr<P::RecogniserCore> recogniser = new P::RecogniserCore(detector,extractor,matcher,P::RecogniserCore::Parameter());
  //recogniser->SetCameraParameter(intrinsic,distortion);

  cv::Ptr<P::LearnerCore> learner = new P::LearnerCore(detector,extractor,matcher,P::LearnerCore::Parameter());
  learner->SetCameraParameter(intrinsic,distortion);


  cv::namedWindow("Image", 1);

  imgLoad = cv::imread("/home/hannes/images/IROS2011/images/sceneTable-10_0002.jpg", 1);


  imgLoad.copyTo(imgCol);
  imgLoad.copyTo(imgDraw);
  imgGray = imgCol;
  if( imgCol.type() != CV_8U ) cv::cvtColor( imgCol, imgGray, CV_BGR2GRAY);


  cv::imshow("Image",imgDraw);

  // ***************************************
  clock_gettime(CLOCK_REALTIME, &start1);
  
  //recogniser->dbg = imgDraw;
  //recogniser->Recognise(imgGray, objects);

  cv::Mat_<cv::Vec4f> cloud(3,3);
  cv::Mat R, T;
  string name = "name";
  learner->Learn(imgGray, cloud, R, T, name);

  clock_gettime(CLOCK_REALTIME, &end1);
  cout<<"Time recogniser object [s]: "<<PMath::timespec_diff(&end1, &start1)<<endl;
  // ****************************************
  cv::imshow("Image",imgDraw);

  cv::waitKey(0);

  cv::destroyWindow("Image");	
	
	return 0;
}




