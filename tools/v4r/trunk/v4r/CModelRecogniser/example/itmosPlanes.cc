
#include <map>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PMath/PMath.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/CModelRecogniser/CModelHandler.hh"
#include "v4r/CModelRecogniser/KeypointDetectorFAST.hh"
#include "v4r/CModelRecogniser/KeypointDetectorSURF.hh"
#include "v4r/CModelRecogniser/KeypointDetector.hh"
#include "v4r/CModelRecogniser/ItMoSPlanes.hh"
#include "v4r/CModelRecogniser/Plane.hh"
#include "v4r/CModelRecogniser/PSiftGPU.hh"
#include "v4r/CModelRecogniser/MergePlanesHF.hh"

#include "external/SiftGPU/src/SiftGPU/SiftGPU.h"

//#define UNDISTORT

/*#define FILENAME "/home/hannes/images/ICRA2011/Box2-02/img-000000%04d.jpg"
#define START 315  //515
#define END 1000
#define STEP 1
#define INTRINSIC "/home/hannes/images/ICRA2011/cal/intrinsic.yml"
#define DIST_COEFFS "/home/hannes/images/ICRA2011/cal/distCoeffs.yml"*/

/*#define FILENAME "/home/hannes/temp/img/MoveCamBox02/img-000000%04d_640.jpg"
#define START 10
#define END 1000
#define STEP 1
#define INTRINSIC "/home/hannes/temp/img/MoveCamBox02/cal/intrinsic.yml"
#define DIST_COEFFS "/home/hannes/temp/img/MoveCamBox02/cal/distCoeffs.yml"*/

/*#define FILENAME "/home/hannes/images/BMVC2010/GuteLaune/tex-%04d.jpg"
#define START 0
#define END 4
#define STEP 1*/

#define FILENAME "/home/hannes/images/GRASPBox05/GRASPBox05-000000%04d.png"
#define START 50
#define END 3000
#define STEP 1 


using namespace std;



void Draw(cv::Mat &img, vector<cv::Ptr<P::Plane> > planes);



int main(int argc, char *argv[] )
{
  int key;
  char filename[PATH_MAX];
  cv::Mat imLoad, imCol, imGray, imDraw;
  cv::Mat intrinsic, distCoeffs, map1, map2;

  vector< cv::Ptr<P::PKeypoint> > keys;
  cv::Mat_<float> descriptors;
  vector<cv::KeyPoint> cvKeys;
  vector<cv::Ptr<P::Plane> > planes;
  struct timespec start1, end1;

  cv::namedWindow("Image", 1);

  cv::Ptr<P::KeypointDetector> detector = new P::KeypointDetectorSURF();
  //cv::Ptr<P::KeypointDetector> detector = new P::KeypointDetectorFAST();
  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor(3,4,true);
  cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::L2<float> >();

  /*P::PSiftGPU *sift = new P::PSiftGPU(P::PSiftGPU::Parameter(.6,.8,1), cv::Ptr<SiftGPU>(), cv::Ptr<SiftMatchGPU>(), 4096);
  cv::Ptr<P::KeypointDetector> detector = &(*sift);  detector.addref();
  cv::Ptr<cv::DescriptorExtractor> extractor = &(*sift); extractor.addref();
  cv::Ptr<cv::DescriptorMatcher> matcher = &(*sift);   matcher.addref();*/

  P::ItMoSPlanes splane(matcher, P::ItMoSPlanes::Parameter(640, 480));
  P::MergePlanesHF mergeF;

  #ifdef UNDISTORT
  cv::FileStorage fs(INTRINSIC, cv::FileStorage::READ);
  fs["intrinsic"]>>intrinsic;
  fs.release();
  fs.open(DIST_COEFFS,cv::FileStorage::READ);
  fs["distCoeffs"]>>distCoeffs;
  #endif


  for (unsigned cnt=START; cnt<END; cnt+=STEP)
  {
    snprintf(filename,PATH_MAX,FILENAME, cnt);
    cout<<filename<<endl;
    imLoad = cv::imread(filename, 1);
    
    #ifdef UNDISTORT
    if (map1.empty() || map2.empty())
    {
      cv::initUndistortRectifyMap(intrinsic, distCoeffs, cv::Mat(), intrinsic, 
                                  cv::Size(imLoad.cols,imLoad.rows), CV_32FC1, map1, map2);
    }
    cv::remap(imLoad,imCol, map1, map2, cv::INTER_LINEAR);
    imCol.copyTo(imDraw);
    #else
    imLoad.copyTo(imDraw);
    imLoad.copyTo(imCol);
    #endif
    if( imCol.type() != CV_8U ) cv::cvtColor( imCol, imGray, CV_BGR2GRAY);
    else  imGray = imCol;

    // detect keypoints
    detector->Detect(imGray, keys);
    P::PKeypoint::ConvertToCv(keys,cvKeys);
   
    extractor->compute(imGray, cvKeys, descriptors);

    /**************************************************/
    clock_gettime(CLOCK_REALTIME, &start1);

    splane.dbg = imDraw;
    bool selected = splane.Operate(keys,descriptors, planes);
    splane.Draw(imDraw,planes);

    if (selected)
    {
      Draw(imDraw,planes);     // debug drawing

      mergeF.dbg = imDraw;
      mergeF.Operate(planes);
    }


    clock_gettime(CLOCK_REALTIME, &end1);
    cout<<"Time for plane detection (inkl. matching) [s]: "<<PMath::timespec_diff(&end1,&start1)<<endl;
    /**************************************************/

    cv::imshow("Image",imDraw);
    key = cv::waitKey(0);

    if ( ((int)key) == 27 ) 
      break;
  }

  cv::destroyWindow("Image");

	return 0;
}



/*********************************** SOME HELPER METHODES **********************************/
void MeanKeys(vector< cv::Ptr<P::PKeypoint> > &keys, cv::Point2d &ptMean)
{
  ptMean = cv::Point2d(0.,0.);
  for (unsigned i=0; i<keys.size(); i++)
  {
    ptMean += keys[i]->pt;
  }
  ptMean.x /= (double)keys.size();
  ptMean.y /= (double)keys.size();
}

void Draw(cv::Mat &img, vector<cv::Ptr<P::Plane> > planes)
{
  cv::Point2d pt1,pt2;

  for (unsigned i=0; i<planes.size(); i++)
  {
    MeanKeys(planes[i]->keys, pt1);
    P::PHom::MapPoint(&pt1.x, &planes[i]->H(0,0), &pt2.x);
    cv::line(img, pt1,pt2, CV_RGB(255,255,0));
    cv::circle(img, pt1, 2, CV_RGB(255,255,255), 2);
if (PVec::Distance2(&pt1.x, &pt2.x)>2)
  planes[i]->haveMotion=true;
else planes[i]->haveMotion=false;
  }

}




