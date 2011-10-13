/**
 * @author Michael Zillich
 * @date August 2011
 * @brief Recognise trained 3D models
 * - HACK hard coded image size
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include <VisionUtils.h>
#include "ObjectRecognizer3D.h"


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectRecognizer3D();
  }
}

using namespace VisionData;
using namespace cast;
using namespace cogx;
using namespace Math;
using namespace Video;
using namespace std;

ObjectRecognizer3D::ObjectRecognizer3D()
{
  camId = 0;
  // HACK hard coded image size!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  P::RecogniserCore::Parameter paramRecogniser = P::RecogniserCore::Parameter(640,480,5000,100,0.01,2.,5., false, .35, .15, .8);
  P::LearnerCore::Parameter paramLearner = P::LearnerCore::Parameter(640,480,1000,50,0.01,2, 15., .35, .15, 2., false, false, .01, .05);
  recogniser = new P::RecogniserThread(P::RecogniserThread::SIFT_GC,paramRecogniser,paramLearner);
}

ObjectRecognizer3D::~ObjectRecognizer3D()
{
}

void ObjectRecognizer3D::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
  istringstream labeliss;
  istringstream modeliss;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  if((it = _config.find("--labels")) != _config.end())
    labeliss.str(it->second);
  else
		throw runtime_error(exceptionMessage(__HERE__, "No labels given"));

  if((it = _config.find("--models")) != _config.end())
    modeliss.str(it->second);
  else
		throw runtime_error(exceptionMessage(__HERE__, "No models given"));

  std::string label, model;
  models.clear();
  P::CModelHandler cmhandler;

  while(labeliss >> label && modeliss >> model)
  {
    modelFiles[label] = model;
    objectWMIds[label] = "";
    models.push_back(new P::CModel() );
    cmhandler.Load(model, models.back());
  }

  recogniser->ClearRecogniser();

  for(size_t i = 0; i < models.size(); i++)
    recogniser->AddModelRecogniser(models[i]);

  recogniser->OptimizeCodebook();

#ifdef FEAT_VISUALIZATION	
  m_display.configureDisplayClient(_config);
#endif
}

void ObjectRecognizer3D::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  addChangeFilter(createLocalTypeFilter<VisionData::DetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveDetectionCommand));

  addChangeFilter(createLocalTypeFilter<VisionData::RecognitionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
        &ObjectRecognizer3D::receiveRecognitionCommand));

//  initRecognizer();

#ifdef FEAT_VISUALIZATION        
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(getComponentID(), string("button.det.all"),
      string("Detect all"));
  for(map<string,string>::iterator it = objectWMIds.begin(); it != objectWMIds.end(); it++)
  {
    m_display.addButton(getComponentID(), string("button.det.") + it->first,
        string("Detect ") + it->first);
    m_display.addButton(getComponentID(), string("button.rec.") + it->first,
        string("Recognize ") + it->first);
  }
#endif
}


#ifdef FEAT_VISUALIZATION
void ObjectRecognizer3D::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if(event.type == Visualization::evButtonClick)
  {
    if(event.sourceId == "button.det.all")
    {
      DetectionCommandPtr det_cmd = new DetectionCommand;
      det_cmd->labels.clear();
      pRec->addToWorkingMemory(pRec->newDataID(), det_cmd);
    }
    else
    {
      for(map<string,string>::iterator it = pRec->objectWMIds.begin(); it !=
          pRec->objectWMIds.end(); it++)
      {
        ostringstream id;
        id << "button.det." << it->first;
        if(event.sourceId == "button.det." + it->first)
        {
          DetectionCommandPtr det_cmd = new DetectionCommand;
          det_cmd->labels.push_back(it->first);
          pRec->addToWorkingMemory(pRec->newDataID(), det_cmd);
        }
        else if(event.sourceId == "button.rec." + it->first)
        {
          RecognitionCommandPtr rec_cmd = new RecognitionCommand;
          rec_cmd->labels.push_back(it->first);
          cast::cdl::WorkingMemoryAddress addr;
          // HACK
          addr.subarchitecture = "vision.sa";
          addr.id = "1:6";
          rec_cmd->visualObject =
            createWmPointer<VisionData::VisualObject>(addr);
          pRec->addToWorkingMemory(pRec->newDataID(), rec_cmd);
        }
      }
    }
  }
}
#endif

// /*void ObjectRecognizer3D::initRecognizer()
// {
// printf("ObjectRecognizer3D::initRecognizer: start!\n");
// 
// //  cv::Ptr<P::KeypointDetector> detector = new P::KeypointDetectorSURF();          // SURF
// //  cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor(3, 4, true);
// //  cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::L2<float> >();
//   // note that recogniser is a smart pointer that takes care of freeing memory
// 
//   // for Review 2011 with Sift
//   cv::Ptr<P::PSiftGPU> sift;
//   sift = new P::PSiftGPU(P::PSiftGPU::Parameter(0.6,0.8,1));
//   cv::Ptr<P::KeypointDetector> detector = &(*sift);  detector.addref();
//   cv::Ptr<cv::DescriptorExtractor> extractor = &(*sift); extractor.addref();
//   cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BruteForceMatcher<cv::L2<float> >();
// 
// printf("ObjectRecognizer3D::initRecognizer: start 2!\n");
//   
//   int imageWidth = 0;
//   int imageHeight = 0;
//   videoServer->getImageSize(imageWidth, imageHeight);
// 
// //  P::RecogniserCore::Parameter paramRecogniser = 
// //        P::RecogniserCore::Parameter(imageWidth,imageHeight,5000,50,0.01,2.,10,false, 0.3);
// 
//   // for Review 2011 with SIFT
//   P::RecogniserCore::Parameter paramRecogniser = 
//       P::RecogniserCore::Parameter(640,480,5000,100,0.01,2.,5., false, .35, .15, .8);
// 
// printf("ObjectRecognizer3D::initRecognizer: start 3!\n");
// 
//   recogniser = new P::RecogniserCore(detector, extractor, matcher, paramRecogniser/*P::RecogniserCore::Parameter()*/);
// 
// printf("ObjectRecognizer3D::initRecognizer: end!\n");
// }*/



void ObjectRecognizer3D::recognize(vector<string> &labels, cv::Mat &colImg,
    Video::CameraParameters &camPars, cv::Mat mask,
    vector<P::ObjectLocation> &objects)
{
  cv::Mat grayImg;
  cv::cvtColor(colImg, grayImg, CV_RGB2GRAY);
  IplImage displayImg = colImg; // cheap: no copy


  // get camera parameters into recogniser
  // TODO; set actual distortion
  cv::Mat intrinsic(cv::Mat::zeros(3,3,CV_64F)), distortion(cv::Mat::zeros(4,1,CV_64F));
  intrinsic.at<double>(0, 0) = camPars.fx;
  intrinsic.at<double>(0, 2) = camPars.cx;
  intrinsic.at<double>(1, 1) = camPars.fy;
  intrinsic.at<double>(1, 2) = camPars.cy;
  intrinsic.at<double>(2, 2) = 1.;
  recogniser->SetCameraParameter(intrinsic, distortion);

  // the actual recognition
  // set the debug image to draw stuff into
  recogniser->SetDebugImage(colImg);
  recogniser->Recognise(grayImg, objects);

// #ifdef FEAT_VISUALIZATION
//   m_display.setImage(getComponentID(), &displayImg);
// #endif

}

/**
 * Tries to detect all objects given in labels in the entire image.
 * Writes all actually detected objects into working memory, where objects that
 * have already been detected in a previous detetion call are overwritten.
 * Objects not found are not written to working memory.
 * NOTE that this is a different behaviour from the previous ObjectRecognizer3D
 * component, which would write an object with 0 confidence. This old behaviour
 * carries a slightly strange semantics. So for the time being the new
 * recognizer will not replicate that behaviour.
 */
void ObjectRecognizer3D::receiveDetectionCommand(const cdl::WorkingMemoryChange & _wmc)
{
//   initRecognizer();

  sleepComponent(1000);

  log("Receiving DetectionCommand");
  DetectionCommandPtr det_cmd = getMemoryEntry<DetectionCommand>(_wmc.address);

  // get image
  Image image;
  videoServer->getImage(camId, image);
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat col = cv::cvarrToMat(iplImage);

  vector<P::ObjectLocation> objects;
  vector<string> labels;
  // if no labels are specified we recognise all our known labels
  if(!det_cmd->labels.empty())
  {
    labels = det_cmd->labels;
  }
  else
  {
    for(map<string,string>::iterator it = objectWMIds.begin(); it != objectWMIds.end(); it++)
      labels.push_back(it->first);
  }
  recognize(labels, col, image.camPars, cv::Mat(), objects);

  // reading off results
  for(size_t i = 0; i < objects.size(); i++)
  {
    log("recognized model '%s' with confidence %f", objects[i].idObject.c_str(),
        objects[i].conf);

    VisionData::VisualObjectPtr wmObj;
    // if have recognised that object previously, overwrite WM entry
    if(!objectWMIds[objects[i].idObject].empty())
    {
      log("overwriting WM entry with address '%s'",
          objectWMIds[objects[i].idObject].c_str());
      wmObj = getMemoryEntry<VisualObject>(objectWMIds[objects[i].idObject]);
      objectLoationToVisualObject(objects[i], wmObj);
      overwriteWorkingMemory(objectWMIds[objects[i].idObject], wmObj);
   }
    // otherwise create new WM entry
    else
    {
      wmObj = createVisualObject();
      log("adding WM entry with address '%s'",
          objectWMIds[objects[i].idObject].c_str());
      objectLoationToVisualObject(objects[i], wmObj);
      objectWMIds[objects[i].idObject] = newDataID();
printf("The new visual object with id: %s\n", objects[i].idObject.c_str()); 
      addToWorkingMemory(objectWMIds[objects[i].idObject], wmObj);
    }
  }
  log("done DetectionCommand");
}

/**
 * Recognizes a given object, as being one of the given labels.
 * Will fetch that object from WM and use its bounding sphere to produce a mask
 * image for the recognizer. Then ask recognizer if it recognizes any of the
 * given labels.
 */
void ObjectRecognizer3D::receiveRecognitionCommand(const cdl::WorkingMemoryChange & _wmc)
{
  log("Receiving RecognitionCommand");
  RecognitionCommandPtr rec_cmd = getMemoryEntry<RecognitionCommand>(_wmc.address);
  // TODO: catch exception and return with error condition if object is not
  // actually in WM
  VisualObjectPtr wmObj =
    getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);

  // get image
  Image image;
  videoServer->getImage(camId, image);
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat col = cv::cvarrToMat(iplImage);

  // get mask image from bounding sphere of visual object
  cv::Mat mask(col.size(), CV_8UC1, cv::Scalar(0));
  Rect2 roi = projectSphere(image.camPars, wmObj->boundingSphere);
  rectangle(mask,
      cv::Point(roi.pos.x - roi.width/2, roi.pos.y - roi.height/2),
      cv::Point(roi.pos.x + roi.width/2, roi.pos.y + roi.height/2),
      cv::Scalar(255), CV_FILLED);
  cv::imshow("mask", mask);
  cv::waitKey(100);

  vector<P::ObjectLocation> objects;
  vector<string> labels;
  // if no labels are specified we recognise all our known labels
  if(!rec_cmd->labels.empty())
  {
    labels = rec_cmd->labels;
  }
  else
  {
    for(map<string,string>::iterator it = objectWMIds.begin(); it != objectWMIds.end(); it++)
      labels.push_back(it->first);
  }
  recognize(labels, col, image.camPars, cv::Mat(), objects);

  if(objects.size() > 0)
  {
    size_t best = 0;
    double best_conf = objects[0].conf;
    for(size_t i = 1; i < objects.size(); i++)
    {
      if(objects[i].conf > best_conf)
      {
        best_conf = objects[i].conf;
        best = i;
      }
    }
    objectLoationToVisualObject(objects[best], wmObj);
    objectWMIds[objects[best].idObject] = rec_cmd->visualObject->address.id;
    log("overwriting WM entry with address '%s'",
        objectWMIds[objects[best].idObject].c_str());
    overwriteWorkingMemory(objectWMIds[objects[best].idObject], wmObj);
  }
  log("done RecognitionCommand");
}

void ObjectRecognizer3D::receiveImages(const std::vector<Video::Image>& images)
{
  if(images.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "image list is empty"));
}

/**
 * Copy data from a recogniser ObjectLocation to a VisionData::VisualObject
 */
void ObjectRecognizer3D::objectLoationToVisualObject(P::ObjectLocation &objLoc,
    VisualObjectPtr &visObj)
{
  // create a very simple distribution: label and unknown
  visObj->identLabels.push_back(objLoc.idObject);
  visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
  // note: distribution must of course sum to 1
  visObj->identDistrib.push_back(objLoc.conf);
  visObj->identDistrib.push_back(1. - objLoc.conf);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  visObj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  visObj->identAmbiguity = 0.;
  for(size_t i = 0; i < visObj->identDistrib.size(); i++)
    if(fpclassify(visObj->identDistrib[i]) != FP_ZERO)
      visObj->identAmbiguity -= visObj->identDistrib[i]*::log(visObj->identDistrib[i]);
  
  visObj->pose.pos.x = objLoc.pose.t.at<double>(0, 0);
  visObj->pose.pos.y = objLoc.pose.t.at<double>(1, 0);
  visObj->pose.pos.z = objLoc.pose.t.at<double>(2, 0);
  visObj->pose.rot.m00 = objLoc.pose.R.at<double>(0, 0);
  visObj->pose.rot.m01 = objLoc.pose.R.at<double>(0, 1);
  visObj->pose.rot.m02 = objLoc.pose.R.at<double>(0, 2);
  visObj->pose.rot.m10 = objLoc.pose.R.at<double>(1, 0);
  visObj->pose.rot.m11 = objLoc.pose.R.at<double>(1, 1);
  visObj->pose.rot.m12 = objLoc.pose.R.at<double>(1, 2);
  visObj->pose.rot.m20 = objLoc.pose.R.at<double>(2, 0);
  visObj->pose.rot.m21 = objLoc.pose.R.at<double>(2, 1);
  visObj->pose.rot.m22 = objLoc.pose.R.at<double>(2, 2);

  visObj->boundingSphere.pos = visObj->pose.pos;
  visObj->boundingSphere.rad = 0.1;  // HACK

  visObj->componentID = getComponentID();
}

/**
 * Load an empty (i.e. actually not detected) visual model to WM. It has a label,
 * identity pose and confidence 0.
 * @return the WM ID of the newly created object
 */
/*std::string ObjectRecognizer3D::loadEmptyVisualModelToWM(std::string &label)
{
  log("creating empty object %s", label.c_str());
  VisionData::VisualObjectPtr obj = new VisionData::VisualObject();

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(0.);
  obj->identDistrib.push_back(1.);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);
  setIdentity(obj->pose);
  obj->componentID = getComponentID();

  std::string newObjID = newDataID();
  addToWorkingMemory(newObjID, obj);
  // do not add a command to track it (as would be the case for a properly detected
  // visual object)
  log("Add model to working memory: '%s' id: %s", obj->identLabels[0].c_str(), newObjID.c_str());
  return newObjID;
}*/

