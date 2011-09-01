/**
 * $Id$
 */


#include "RecogniserThread.hh"

namespace P 
{

/**
 * ThreadRecogniser
 */
void* ThreadRecognise(void* c)
{
  RecogniserThread *tt = (RecogniserThread*)c;

  bool end=false;

  // create recogniser and learner
  cv::Ptr<P::PSiftGPU> sift;
  cv::Ptr<P::KeypointDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  switch(tt->kt)
  {
    case RecogniserThread::SURF_CC:
      detector = new P::KeypointDetectorSURF();
      extractor = new cv::SurfDescriptorExtractor(3,4,true);
      matcher = new cv::BruteForceMatcher<cv::L2<float> >();
      break;
    case RecogniserThread::SURF_CG:
      sift = new P::PSiftGPU();
      detector = new P::KeypointDetectorSURF();
      extractor = new cv::SurfDescriptorExtractor(3,4,true);
      matcher = &(*sift);   matcher.addref();
      break;
    case RecogniserThread::SIFT_GG:
      sift = new P::PSiftGPU();
      detector = &(*sift);  detector.addref();
      extractor = &(*sift); extractor.addref();
      matcher = &(*sift);   matcher.addref();
      break;
    case RecogniserThread::SIFT_GC:
      sift = new P::PSiftGPU();
      detector = &(*sift);  detector.addref();
      extractor = &(*sift); extractor.addref();
      matcher = new cv::BruteForceMatcher<cv::L2<float> >();
      break;
  }

  cv::Ptr<P::RecogniserCore> recogniser = new P::RecogniserCore(detector,extractor,matcher, tt->paramRecogniser);  
  cv::Ptr<P::LearnerCore> learner = new P::LearnerCore(detector,extractor,matcher, tt->paramLearner);  

  // main loop
  while (!end && !tt->stopRecogniserThread)
  {
    sem_wait(&tt->startOperate);

    switch(tt->cmd)
    {
      case RecogniserThread::STOP:          // stop thread 
        pthread_mutex_lock(&tt->mutShare);
        end=true;
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::CLEAR_LEARN:          // learner commands
        pthread_mutex_lock(&tt->mutShare);
        learner->Clear();
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::GET_MODEL_LEARN:
        pthread_mutex_lock(&tt->mutShare);
        tt->tmodel = learner->GetModel();
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::SET_MODEL_LEARN:
        pthread_mutex_lock(&tt->mutShare);
        learner->SetModel(tt->tmodel);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::LEARN:
        pthread_mutex_lock(&tt->mutShare);
        tt->tstatus = learner->Learn(tt->timage, tt->tcloud, tt->tR, tt->tT, tt->toid, tt->tmask);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::GET_VIEW_RAYS_LEARN:
        pthread_mutex_lock(&tt->mutShare);
        learner->GetViewRays(tt->tvr);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::DETECT_KEYPOINTS:
        pthread_mutex_lock(&tt->mutShare);
        learner->DetectKeypoints(tt->timage, tt->tkeys, tt->tmask);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::INSERT_TO_MODEL:
        pthread_mutex_lock(&tt->mutShare);
        learner->InsertToModel(tt->tkeys, tt->tR, tt->tT, tt->toid);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::SET_CAMERA:
        pthread_mutex_lock(&tt->mutShare);
        learner->SetCameraParameter(tt->tcam, tt->tdistCoeffs);
        recogniser->SetCameraParameter(tt->tcam, tt->tdistCoeffs);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::CLEAR_RECOGNISER:          // recogniser commands
        pthread_mutex_lock(&tt->mutShare);
        recogniser->Clear();
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::ADD_MODEL_RECOGNISER:
        pthread_mutex_lock(&tt->mutShare);
        recogniser->AddModel(tt->tmodel);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::RECOGNISE:
        pthread_mutex_lock(&tt->mutShare);
        recogniser->Recognise(tt->timage, tt->tobjects, tt->tmask);
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::OPTIMIZE_CODEBOOK:
        pthread_mutex_lock(&tt->mutShare);
        recogniser->OptimizeCodebook();
        pthread_mutex_unlock(&tt->mutShare);
        break;

      case RecogniserThread::IDLE:
      default:
        break;
    }
    
    sem_post(&tt->operated);
  }

  tt->stopRecogniserThread = true;
  return((void *)0);
}



/********************** RecogniserThread ************************/
/**
 * Constructor
 */
RecogniserThread::RecogniserThread(KeypointType keypointType,
                                   P::RecogniserCore::Parameter _paramRecogniser, 
                                   P::LearnerCore::Parameter _paramLearner)
 : kt(keypointType), paramRecogniser(_paramRecogniser), paramLearner(_paramLearner)
{
  pthread_mutex_init(&mutShare,NULL);

  sem_init(&operated, 0, 0);
  sem_init(&startOperate, 0, 0);

  pthread_create(&thread, NULL, ThreadRecognise, this);
}

/**
 * Destructor
 */
RecogniserThread::~RecogniserThread()
{
  Stop();
  pthread_join(thread,NULL);

  (void)sem_destroy(&operated);
  (void)sem_destroy(&startOperate);

  pthread_mutex_destroy(&mutShare);
}

/**
 * stop the thread
 */
void RecogniserThread::Stop()
{
  pthread_mutex_lock(&mutShare);
  cmd = STOP;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}


/***************************** PUBLIC *****************************/

/**
 * Clear learning
 * and delete the object model
 */
void RecogniserThread::ClearLearn()
{
  pthread_mutex_lock(&mutShare);
  cmd = CLEAR_LEARN;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}

/**
 * Set model learn
 */
void RecogniserThread::SetModelLearn(const cv::Ptr<CModel> &_model)
{
  pthread_mutex_lock(&mutShare);
  tmodel = _model;
  cmd = SET_MODEL_LEARN;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}

/**
 * Get model learn
 */
void RecogniserThread::GetModelLearn(cv::Ptr<CModel> &_model)
{
  pthread_mutex_lock(&mutShare);
  cmd = GET_MODEL_LEARN;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  pthread_mutex_lock(&mutShare);
  _model = tmodel;
  pthread_mutex_unlock(&mutShare);
}

/**
 * learn an object model
 */
int RecogniserThread::Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, 
                            cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask)
{
  pthread_mutex_lock(&mutShare);
  cmd = LEARN;
  timage = image, tcloud = cloud, tR = R, tT = T, toid = oid, tmask = mask;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  return tstatus;
}

/**
 * Get next view rays to learn
 */
void RecogniserThread::GetViewRays(vector<cv::Point3d> &vr)
{
  pthread_mutex_lock(&mutShare);
  cmd = GET_VIEW_RAYS_LEARN;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  pthread_mutex_lock(&mutShare);
  vr = tvr;
  pthread_mutex_unlock(&mutShare);
}

/**
 * DetectKeypoints
 */
void RecogniserThread::DetectKeypoints(const cv::Mat &image,vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  pthread_mutex_lock(&mutShare);
  cmd = DETECT_KEYPOINTS;
  timage = image, tmask = mask;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  pthread_mutex_lock(&mutShare);
  keys = tkeys;
  pthread_mutex_unlock(&mutShare);
}

/**
 * InsertToModel
 */
int RecogniserThread::InsertToModel(const vector<cv::Ptr<PKeypoint> > &keys,cv::Mat R,cv::Mat T, const string &oid)
{
  pthread_mutex_lock(&mutShare);
  cmd = INSERT_TO_MODEL;
  tkeys = keys, tR = R, tT = T, toid = oid;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  return tstatus;
}

/**
 * SetCameraParameter
 */
void RecogniserThread::SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion)
{
  pthread_mutex_lock(&mutShare);
  cmd = SET_CAMERA;
  tcam = _intrinsic, tdistCoeffs = _distortion;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}

/**
 * LoadVocabularyTree
 */
void RecogniserThread::LoadVocabularyTree(const string &filename)
{
  cout<<"No vocabulary tree used in this version!"<<endl;
}

/**
 * ClearRecogniser
 */
void RecogniserThread::ClearRecogniser()
{
  pthread_mutex_lock(&mutShare);
  cmd = CLEAR_RECOGNISER;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}

/**
 * AddModelRecogniser
 */
unsigned RecogniserThread::AddModelRecogniser(cv::Ptr<CModel> &model)
{
  pthread_mutex_lock(&mutShare);
  cmd = ADD_MODEL_RECOGNISER;
  tmodel = model;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}

/**
 * Recognise
 */
void RecogniserThread::Recognise(const cv::Mat &image, vector<ObjectLocation> &objects, cv::Mat mask)
{
  pthread_mutex_lock(&mutShare);
  cmd = RECOGNISE;
  timage = image, tmask = mask;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);

  pthread_mutex_lock(&mutShare);
  objects = tobjects;
  pthread_mutex_unlock(&mutShare);
}

/**
 * OptimizeCodebook
 */
void RecogniserThread::OptimizeCodebook()
{
  pthread_mutex_lock(&mutShare);
  cmd = OPTIMIZE_CODEBOOK;
  pthread_mutex_unlock(&mutShare);

  sem_post(&startOperate);
  sem_wait(&operated);
}


}

