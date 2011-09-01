/**
 * $Id$
 */

#ifndef P_RECOGNISER_THREAD_HH
#define P_RECOGNISER_THREAD_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <pthread.h>
#include <semaphore.h>
#include "CModel.hh"
#include "RecogniserCore.hh"
#include "LearnerCore.hh"
#include "KeypointDetectorSURF.hh"
#include "KeypointDetector.hh"
#include "ObjectLocation.hh"
#include "PSiftGPU.hh"



namespace P
{

class RecogniserThread
{
public:
  enum KeypointType{
    GPUSIFT,
    SURF,
    SIFT_GG,              // SiftGPU and GPU-Matcher
    SIFT_GC,              // SiftGPU and BruteForce-Matcher
    SURF_CC,              // CPU-SURF and BruteForce-Matcher
    SURF_CG,              // CPU-SURF and GPU-Matcher
  };
  KeypointType kt; 
  
private:
  enum Command{
    LEARN,                   // learning commands
    CLEAR_LEARN,
    SET_MODEL_LEARN,
    GET_MODEL_LEARN,
    GET_VIEW_RAYS_LEARN,
    DETECT_KEYPOINTS,
    INSERT_TO_MODEL,
    RECOGNISE,               // recognise
    CLEAR_RECOGNISER,
    ADD_MODEL_RECOGNISER,
    OPTIMIZE_CODEBOOK,
    SET_CAMERA,
    STOP,
    IDLE,
  };
  Command cmd;

  bool stopRecogniserThread;                     // stop the recogniser thread

  pthread_t thread;                              // recogniser thread
  pthread_mutex_t mutShare;                     // mutex for data

  sem_t operated, startOperate;

  // temp data...
  cv::Ptr<CModel> tmodel;
  int tstatus;
  cv::Mat timage, tcloud, tR, tT, tmask;
  string toid;
  vector<cv::Point3d> tvr;
  vector<cv::Ptr<PKeypoint> > tkeys;
  cv::Mat tcam, tdistCoeffs;
  vector<ObjectLocation> tobjects;

  friend void* ThreadRecognise(void* c);

  void Stop();


public:
  P::RecogniserCore::Parameter paramRecogniser;
  P::LearnerCore::Parameter paramLearner;
  RecogniserThread(KeypointType keypointType=SURF_CC, 
                   P::RecogniserCore::Parameter _paramRecogniser = P::RecogniserCore::Parameter(), 
                   P::LearnerCore::Parameter _paramLearner = P::LearnerCore::Parameter());
  ~RecogniserThread();

  // learning commands
  void ClearLearn();
  void SetModelLearn(const cv::Ptr<CModel> &_model);
  void GetModelLearn(cv::Ptr<CModel> &_model);
  int Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, 
        cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask=cv::Mat());
  void GetViewRays(vector<cv::Point3d> &vr);
  void DetectKeypoints(const cv::Mat &image,vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
  int InsertToModel(const vector<cv::Ptr<PKeypoint> > &keys,cv::Mat R,cv::Mat T, const string &oid);

  // recogniser commands
  void LoadVocabularyTree(const string &filename);
  void ClearRecogniser();
  unsigned AddModelRecogniser(cv::Ptr<CModel> &model);
  void Recognise(const cv::Mat &image, vector<ObjectLocation> &objects, cv::Mat mask=cv::Mat());
  void OptimizeCodebook();

  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion);

  inline bool Stopped(){ return this->stopRecogniserThread; }
};

}

#endif

