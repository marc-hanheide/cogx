/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SIFTGPU_WRAPPER_HH
#define P_SIFTGPU_WRAPPER_HH

#include <limits.h>
#include <GL/glut.h>
#include <dlfcn.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "external/SiftGPU/src/SiftGPU/SiftGPU.h"
#include "KeypointDetector.hh"
#include "PKeypoint.hh"


namespace P
{


class PSiftGPU : public KeypointDetector, public cv::DescriptorExtractor, public cv::DescriptorMatcher
{
public:
  class Parameter  // ATTENTION: Do not touch these parameters! (CModelRecogniser handels them)
  {
  public:
    float distmax;         // absolute descriptor distance (e.g. = 0.6)
    float ratiomax;        // compare best match with second best (e.g. =0.8)
    int mutual_best_match; // compare forward/backward matches (1) 
    Parameter(float d=FLT_MAX, float r=1., int m=0) 
      : distmax(d), ratiomax(r), mutual_best_match(m) {}
  };

private:
  cv::Ptr<SiftGPU> sift;
  cv::Ptr<SiftMatchGPU> matcher;
  int gpuMemSize;

//  vector<cv::Mat> trainDescriptors;

  inline float Distance128(float d1[128], float d2[128]);

protected:
  virtual void computeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, 
             cv::Mat& descriptors) const;
  virtual void knnMatchImpl( const cv::Mat& queryDescriptors, vector<vector<cv::DMatch> >& matches,
             int k, const vector<cv::Mat>& masks=vector<cv::Mat>(), bool compactResult=false );
  virtual void radiusMatchImpl( const cv::Mat& queryDescriptors, vector<vector<cv::DMatch> >& matches, 
             float maxDistance, const vector<cv::Mat>& masks=vector<cv::Mat>(), bool compactResult=false );

  
public:
  Parameter param;

  PSiftGPU(Parameter p=Parameter(), cv::Ptr<SiftGPU> _sift=cv::Ptr<SiftGPU>(), cv::Ptr<SiftMatchGPU> _matcher=cv::Ptr<SiftMatchGPU>(), int memSize = 4096);
  ~PSiftGPU();

  virtual void Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
  virtual void DetectDescriptor(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat &descriptors, cv::Mat mask=cv::Mat());

  void compute(const cv::Mat& image, vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors );
  virtual int descriptorSize() const {return 128;}
  virtual int descriptorType() const {return CV_32F;}


  //void match( const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors, vector<cv::DMatch>& matches, const cv::Mat& mask=cv::Mat() );
//  virtual void add( const vector<cv::Mat>& descriptors );
//  virtual void clear();
  virtual bool isMaskSupported() const {return false;}
  virtual cv::Ptr<cv::DescriptorMatcher> clone( bool emptyTrainData=false ) const; 
};


/************************** INLINE METHODES ******************************/

inline float PSiftGPU::Distance128(float d1[128], float d2[128])
{
  float sqrDist=0.;

  for (unsigned i=0; i<128; i++)
    sqrDist += PMath::Sqr(d1[i]-d2[i]);

  return sqrt(sqrDist);
}


}

#endif

