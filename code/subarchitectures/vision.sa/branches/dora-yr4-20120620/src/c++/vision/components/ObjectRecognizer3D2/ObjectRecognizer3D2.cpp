
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include "ObjectRecognizer3D2.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectRecognizer3D2();
  }
}

using namespace VisionData;
using namespace ObjectRecognizerIce;
using namespace cast;
using namespace cogx;
using namespace Math;
using namespace std;

// whether we should overwrite a recognized visual object in working memory
// in addition to returning the recognition result in the recogition task
static const bool OVERWRITE_VISUAL_OBJECT = false;

// use segmentation mask inside ROI
static const bool USE_SEGMENTATION_MASK = false;

// recognizer confidences are typically really low (like around 0.1)
// so scale it up to make it into a "nice" probability.
// note: probabilities are normalized to 1
static const double FREAKY_CONFIDENCE_SCALE_FACTOR = 20.;

ObjectRecognizer3D2::ObjectRecognizer3D2(){
  m_detect = 0;
	m_min_confidence = 0.01; // thef original authors choice: 0.08;
	m_haveCameraParameters = false;
}

ObjectRecognizer3D2::~ObjectRecognizer3D2(){
	if(m_detect)
		delete(m_detect);
}

void ObjectRecognizer3D2::configure(const map<string,string> & _config){
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureStereoCommunication(_config);

  istringstream plyiss;
  istringstream siftiss;
  istringstream labeliss;

  if((it = _config.find("--labels")) != _config.end()){
		labeliss.str(it->second);
	}else{
		throw runtime_error(exceptionMessage(__HERE__, "No labels given"));
	}

	if((it = _config.find("--siftfiles")) != _config.end()){
		siftiss.str(it->second);
	}

	if((it = _config.find("--plyfiles")) != _config.end()){
		plyiss.str(it->second);
	}

	std::string label, plystr, siftstr;

	while(labeliss >> label && plyiss >> plystr && siftiss >> siftstr){
		m_recEntries[label].plyfile = plystr;
		m_recEntries[label].siftfile = siftstr;
	}

  loadModels();

#ifdef FEAT_VISUALIZATION	
  m_display.configureDisplayClient(_config);
#endif
}

void ObjectRecognizer3D2::start(){

  startStereoCommunication(*this);

  addChangeFilter(createLocalTypeFilter<ObjectRecognitionTask>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D2>(this,
        &ObjectRecognizer3D2::receiveRecognitionTask));

#ifdef FEAT_VISUALIZATION        
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
#endif
}


void ObjectRecognizer3D2::runComponent(){

  P::DetectGPUSIFT sift;

  initRecognizer();

  // Running Loop
  while(isRunning()) {
    if(!m_recTaskId.empty())
    {
      string objId = m_recTask->visualObjectAddr.id;
      VisualObjectPtr obj = getMemoryEntry<VisualObject>(objId);
      recognizeSiftModel(sift, obj, m_recTask);
      if(OVERWRITE_VISUAL_OBJECT || obj->overwriteVisualObject)
        overwriteWorkingMemory(objId, obj);
      overwriteWorkingMemory(m_recTaskId, m_recTask);
      m_recTaskId.clear();
    }
    // HACK: yes the sleep is ugly ...
    sleepComponent(500);
  }
}

void ObjectRecognizer3D2::destroy(){

}

void ObjectRecognizer3D2::receiveImages(const std::vector<Video::Image>& images){
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

void ObjectRecognizer3D2::receiveRecognitionTask(const cdl::WorkingMemoryChange & _wmc){
  log("Receiving ObjectRecognitionTask");
  m_recTaskId = _wmc.address.id;
  m_recTask = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
}

/**
 * allocate recognizer and load models
 * NOTE: MUST be called within runComponent()
 */
void ObjectRecognizer3D2::initRecognizer() {
	m_detect = new(P::ODetect3D);
}

/**
 * load models
 */
void ObjectRecognizer3D2::loadModels() {

  log("loading models ...");
	std::map<std::string,RecEntry>::iterator it;
	for(it = m_recEntries.begin(); it!=m_recEntries.end(); it++){
		log("Loading Sift Model '%s'", (*it).second.siftfile.c_str());
		(*it).second.object = new(P::Object3D);
		if(!sift_model_learner.LoadModel((*it).second.siftfile.c_str(), (*(*it).second.object)))
      throw runtime_error(exceptionMessage(__HERE__, "failed to load %s", (*it).second.siftfile.c_str()));
	}
  log("... done loading models");
}

/**
 * Set camera parameters with given image size.
 * Will get an image from the stereo server (with given image width), get
 * camera parameters from the image and set parameters for the recognizer.
 * Will do that only once.
 */
void ObjectRecognizer3D2::setCameraParameters(int imgWidth, int igHeight) {

  if(!m_haveCameraParameters)
  {
    log("setting camera parameters");

    Video::Image image;
    StereoClient::getRectImage(LEFT, imgWidth, image);

    double fx, fy, cx, cy;
    fx = image.camPars.fx;
    fy = image.camPars.fy;
    cx = image.camPars.cx;
    cy = image.camPars.cy;

    CvMat *C = cvCreateMat(3,3, CV_32F);
    cvmSet(C, 0, 0, fx);
    cvmSet(C, 0, 1, 0.);
    cvmSet(C, 0, 2, cx);
    cvmSet(C, 1, 0, 0.);
    cvmSet(C, 1, 1, fy);
    cvmSet(C, 1, 2, cy);
    cvmSet(C, 2, 0, 0.);
    cvmSet(C, 2, 1, 0.);
    cvmSet(C, 2, 2, 1.);

    m_detect->SetCameraParameter(C);

    cvReleaseMat(&C);

    m_camPose = image.camPars.pose;

    m_haveCameraParameters = true;
  }
}

void ObjectRecognizer3D2::fillImages(ProtoObjectPtr pobj, IplImage **img, IplImage **grey)
{
  setCameraParameters(pobj->sourceImageSize.x, pobj->sourceImageSize.y);
  
  *img = cvCreateImage(cvSize(pobj->sourceImageSize.x, pobj->sourceImageSize.y),
    IPL_DEPTH_8U, 3);
  *grey = cvCreateImage(cvSize(pobj->sourceImageSize.x, pobj->sourceImageSize.y),
    IPL_DEPTH_8U, 1);

  // fill the segmented image patch from the proto object into full image
  cvSet(*img, cvScalar(0));
  Video::AccessRgbImage imgAcc(*img);
  int xoffs = pobj->imageOrigin.x;
  int yoffs = pobj->imageOrigin.y;
  for(int y = 0; y < pobj->image.height; y++)
    for(int x = 0; x < pobj->image.width; x++)
    {
      // foreground object is labelled 1
      if(!USE_SEGMENTATION_MASK || pobj->mask.data[pobj->image.width*y + x] == 1)
      {
        int yy = y + yoffs;
        int xx = x + xoffs;
        if(xx >= 0 && xx < (*img)->width && yy >= 0 && yy < (*img)->height)
        {
          Video::AccessRgbPixel &p = imgAcc[yy][xx];
          p.r = pobj->image.data[3*(y*pobj->image.width + x) + 0];
          p.g = pobj->image.data[3*(y*pobj->image.width + x) + 1];
          p.b = pobj->image.data[3*(y*pobj->image.width + x) + 2];
        }
      }
    }  
  cvConvertImage(*img, *grey);
}

void ObjectRecognizer3D2::releaseImages(IplImage **img, IplImage **grey)
{
	cvReleaseImage(img);
	cvReleaseImage(grey);
}

/**
 * Recognizes all known models and stores a prob dist over labels in the provided visual model as
 * as well as in the recTask.
 */
void ObjectRecognizer3D2::recognizeAllObjects(P::DetectGPUSIFT &sift, IplImage *img, IplImage *grey,
  VisualObjectPtr obj, ObjectRecognitionTaskPtr recTask)
{
  float best_conf = 0;
  Pose3 best_pose;

  setIdentity(best_pose);
  
  sift.Operate(grey, m_image_keys);

  m_detect->SetDebugImage(img);

  // make sure these are empty, could be that someone has already put an "unknown" in
  obj->identLabels.clear();
  obj->identDistrib.clear();

	std::map<std::string,RecEntry>::iterator it;
	for(it = m_recEntries.begin(); it != m_recEntries.end(); it++)
	{
	  float conf;
    Pose3 pose;
    if(!m_detect->Detect(m_image_keys, *(*it).second.object))
    {
      conf = 0.;
      setIdentity(pose);
    }
    else
    {
      if((*it).second.object->conf < m_min_confidence)
      {
        // set confidence to 0 to indicate that we consider the object not detected
        conf = 0.;
        setIdentity(pose);

        P::SDraw::DrawPoly(img, (*it).second.object->contour.v, CV_RGB(255,0,0), 2);
        m_detect->DrawInlier(img, CV_RGB(255,0,0));
      }
      else
      {
        // Transform pose from Camera to world coordinates
        Pose3 localPose;
        convertPoseCv2MathPose((*it).second.object->pose, localPose);
        Math::transform(m_camPose, localPose, pose);
        
        conf = (*it).second.object->conf*FREAKY_CONFIDENCE_SCALE_FACTOR;
        if(conf > best_conf)
        {
          best_conf = (*it).second.object->conf;
          best_pose = pose;
        }

        P::SDraw::DrawPoly(img, (*it).second.object->contour.v, CV_RGB(0,255,0), 2);
        m_detect->DrawInlier(img, CV_RGB(255,0,0));
      }
    }

    log("recognized %s with confidence %lf", (*it).first.c_str(), conf);

    // fill the visual object strcuture
    obj->identLabels.push_back((*it).first);
    obj->identDistrib.push_back(conf);

    // fill the recognition command results structure
    RecognitionResult res;
    res.label = (*it).first;
    res.probability = conf;
    // note: we only have one pose as output
    // note further: this expects the wrong pose format
    //res.poses.push_back(pose);
    //res.posePd.push_back(1.);
    recTask->matches.push_back(res);
	}

  obj->pose = best_pose;

  RecognitionResult res;
  res.label = "unknown";
  // distribution must of course sum to 1
  // NOTE: this is a crude and clearly non proper way of determining
  // the likelihood of the unknown class.
  float sum = 0.;
  for(size_t i = 0; i < recTask->matches.size(); i++)
    sum += recTask->matches[i].probability;
  if(sum < 1.)
  {
    res.probability = 1. - sum;
  }
  else
  {
    for(size_t i = 0; i < recTask->matches.size(); i++)
      recTask->matches[i].probability /= sum;
    res.probability = 0.;
  }
  recTask->matches.push_back(res);
  
  for(unsigned i = 0; i < m_image_keys.Size(); i++)
	  delete(m_image_keys[i]);
	m_image_keys.Clear();
}

void ObjectRecognizer3D2::finalizeObject(VisualObjectPtr obj)
{
  obj->identLabels.push_back("unknown");
  // distribution must of course sum to 1
  // NOTE: this is a crude and clearly non proper way of determining
  // the likelihood of the unknown class.
  float sum = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    sum += obj->identDistrib[i];
  if(sum < 1.)
  {
    obj->identDistrib.push_back(1. - sum);
  }
  else
  {
    for(size_t i = 0; i < obj->identDistrib.size(); i++)
      obj->identDistrib[i] /= sum;
    obj->identDistrib.push_back(0.);
  }
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);
  obj->componentID = getComponentID();
}

/**
 * Recognize which of the learned models matches the image stored in the proto object
 * referred to by given visual object.
 */
void ObjectRecognizer3D2::recognizeSiftModel(P::DetectGPUSIFT &sift, VisualObjectPtr obj,
  ObjectRecognitionTaskPtr recTask){

  log("recognizing object ...");

  ProtoObjectPtr pobj = getMemoryEntry<ProtoObject>(obj->protoObjectID);
  IplImage *img = 0, *grey = 0;

  fillImages(pobj, &img, &grey);
  recognizeAllObjects(sift, img, grey, obj, recTask);
  finalizeObject(obj);

#ifdef FEAT_VISUALIZATION
  m_display.setImage("ObjectRecognizer3D2", img);
#endif
  
  releaseImages(&img, &grey);
  
  log("... done");
}
