/**
 * @author Michael Zillich
 * @date August 2011
 * @brief Recognise trained 3D models
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <v4r/PCLAddOns/PCLUtils.h>
#include <v4r/PCLAddOns/PCLFunctions.h>
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

using namespace std;
using namespace cast;
using namespace cogx;
using namespace Math;
using namespace Video;
using namespace VisionData;

/**
 * Convert points from CAST format to PCL format.
 * NOTE: this should go to VisionUtils. But that introduces a PCL dependency in VisionUtils,
 * which affects all source files including VisionUtils.h. So leave it here for now.
 */
/*static inline void ConvertSurfacePoints2OrganisedPCLCloud(const vector<PointCloud::SurfacePoint> &points,
	pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
	int width, int height)
{
    if((int)points.size() < width*height)
	throw runtime_error(cast::exceptionMessage(__HERE__,
		    "need %d x %d points, have %d", width, height, (int)points.size()));

    pcl_cloud.width = width;
    pcl_cloud.height = height;
    pcl_cloud.points.resize(width*height);
    for(size_t i = 0; i < pcl_cloud.points.size(); i++)
    {
	RGBValue color;
	if(PointIsSane(points[i]))
	{
	    pcl_cloud.points[i].x = (float) points[i].p.x;
	    pcl_cloud.points[i].y = (float) points[i].p.y;
	    pcl_cloud.points[i].z = (float) points[i].p.z;
	    color.r = points[i].c.r;
	    color.g = points[i].c.g;
	    color.b = points[i].c.b;
	    pcl_cloud.points[i].rgb = color.float_value;
	}
	else
	{
	    pcl_cloud.points[i].x = 0.;
	    pcl_cloud.points[i].y = 0.;
	    pcl_cloud.points[i].z = 0.;
	    color.r = 0;
	    color.g = 0;
	    color.b = 0;
	    pcl_cloud.points[i].rgb = color.float_value;
	}
    }
}*/

static inline void ConvertSurfacePoints2PCLCloud(const vector<PointCloud::SurfacePoint> &points,
    pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud)
{
  pcl_cloud.width = points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  pcl_cloud.points.resize(points.size());
  for(size_t i = 0; i < pcl_cloud.points.size(); i++)
  {
    RGBValue color;
    pcl_cloud.points[i].x = (float) points[i].p.x;
    pcl_cloud.points[i].y = (float) points[i].p.y;
    pcl_cloud.points[i].z = (float) points[i].p.z;
    color.r = points[i].c.r;
    color.g = points[i].c.g;
    color.b = points[i].c.b;
    pcl_cloud.points[i].rgb = color.float_value;
  }
}

/**
 * Returns the best (highest probability) label that is not "unknmown".
 */
static inline string GetBestLabel(VisualObject &obj)
{
  double maxProb = 0.;
  string label = "";
  for(size_t i = 0; i < obj.identLabels.size(); i++)
  {
    if(obj.identDistrib[i] > maxProb && obj.identLabels[i] != VisionData::IDENTITYxUNKNOWN)
    {
      maxProb = obj.identDistrib[i];
      label = obj.identLabels[i];
    }
  }
  return label;
}

/**
 * Given a distribution (identDistrib) of identity labels, calculate the ambiguity.
 * Gain is always set to 1 (as I have no idea what else to do really).
 */
static inline void recalculateIdentGainAmbiguity(VisualObjectPtr &visObj)
{
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  visObj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  visObj->identAmbiguity = 0.;
  for(size_t i = 0; i < visObj->identDistrib.size(); i++)
    if(fpclassify(visObj->identDistrib[i]) != FP_ZERO)
      visObj->identAmbiguity -= visObj->identDistrib[i]*::log(visObj->identDistrib[i]);
}

ObjectRecognizer3D::ObjectRecognizer3D()
{
  camId = 0;
  modelNameCnt = 0;
  P::Im3dRecogniserCore::Parameter paramRecogniser;
  P::LearnerCore::Parameter paramLearner;
  recogniser = new P::CModelThread(paramRecogniser, paramLearner);
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

  // Load all given models. Note that further models can be learend at
  // run time.
  std::string label, modelfile;
  models.clear();
  P::CModelHandler cmhandler;
  while(labeliss >> label && modeliss >> modelfile)
  {
    //objectWMIds[label] = "";
    P::ObjectModel::Ptr model(new P::ObjectModel());
    cmhandler.Load(modelfile, model);
    models[label] = model;
  }
  recogniser->clearRecogniser();
  for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
    recogniser->addModelRecogniser(it->second);
  if(models.size() > 0)
    recogniser->initFlannRecogniser();

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

  addChangeFilter(createLocalTypeFilter<VisionData::LearnObjectViewCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
      &ObjectRecognizer3D::receiveLearnObjectViewCommand));

  // get an image to obtain camera parameters and provide these for the recogniser
  Image image;
  videoServer->getImage(camId, image);
  // TODO; set actual distortion
  cv::Mat intrinsic(cv::Mat::zeros(3,3,CV_64F)), distortion(cv::Mat::zeros(4,1,CV_64F));
  intrinsic.at<double>(0, 0) = image.camPars.fx;
  intrinsic.at<double>(0, 2) = image.camPars.cx;
  intrinsic.at<double>(1, 1) = image.camPars.fy;
  intrinsic.at<double>(1, 2) = image.camPars.cy;
  intrinsic.at<double>(2, 2) = 1.;
  recogniser->setCameraParameter(intrinsic, distortion);

#ifdef FEAT_VISUALIZATION        
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(getComponentID(), string("button.det.all"),
      string("Detect all"));
  for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
  {
    m_display.addButton(getComponentID(), string("button.det.") + it->first,
        string("Detect ") + it->first);
  }
  m_display.addButton(getComponentID(), string("button.rec"),
      string("Recognize Obj in WM"));
  m_display.addButton(getComponentID(), string("button.learn"),
      string("Learn Obj in WM"));
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
    else if(event.sourceId == "button.rec")
    {
      std::vector < boost::shared_ptr< CASTData<VisualObject> > > visObjs;
      pRec->getWorkingMemoryEntries(1, visObjs);
      if(visObjs.size() >= 1)
      {
        log(string(__FILE__) + ":" + string(__FUNCTION__) + ": recognizing vis obj "
          + visObjs[0]->getID());
        RecognitionCommandPtr rec_cmd = new RecognitionCommand;
        cast::cdl::WorkingMemoryAddress addr;
        addr.subarchitecture = "vision.sa";
        addr.id = visObjs[0]->getID();
        rec_cmd->visualObject =
          createWmPointer<VisionData::VisualObject>(addr);
        pRec->addToWorkingMemory(pRec->newDataID(), rec_cmd);
      }
      else
      {
        log(string(__FILE__) + ":" + string(__FUNCTION__) + ": no vis obj to recognize in WM");
      }
    }
    else if(event.sourceId == "button.learn")
    {
      std::vector < boost::shared_ptr< CASTData<VisualObject> > > visObjs;
      pRec->getWorkingMemoryEntries(1, visObjs);
      if(visObjs.size() >= 1)
      {
        log(string(__FILE__) + ":" + string(__FUNCTION__) + ": learning vis obj "
          + visObjs[0]->getID());
        LearnObjectViewCommandPtr learn_cmd = new LearnObjectViewCommand;
        cast::cdl::WorkingMemoryAddress addr;
        addr.subarchitecture = "vision.sa";
        addr.id = visObjs[0]->getID();
        learn_cmd->visualObject =
          createWmPointer<VisionData::VisualObject>(addr);
        pRec->addToWorkingMemory(pRec->newDataID(), learn_cmd);
      }
      else
      {
        log(string(__FILE__) + ":" + string(__FUNCTION__) + ": no vis obj to learn in WM");
      }
    }
    else
    {
      for(map<string, P::ObjectModel::Ptr>::iterator it = pRec->models.begin(); it !=
          pRec->models.end(); it++)
      {
        ostringstream id;
        id << "button.det." << it->first;
        if(event.sourceId == "button.det." + it->first)
        {
          DetectionCommandPtr det_cmd = new DetectionCommand;
          det_cmd->labels.push_back(it->first);
          pRec->addToWorkingMemory(pRec->newDataID(), det_cmd);
        }
      }
    }
  }
}
#endif

void ObjectRecognizer3D::recognize(const Image &image,
    const vector<PointCloud::SurfacePoint> &points,
    vector<P::ObjectLocation> &objects)
{
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat cvImage = cv::cvarrToMat(iplImage);

  // note that points are given in robot ego, need to be transformed into camera
  vector<PointCloud::SurfacePoint> camPoints(points.size());
  for(size_t i = 0; i < points.size(); i++)
    camPoints[i].p = cogx::Math::transform(image.camPars.pose, points[i].p);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ConvertSurfacePoints2PCLCloud(camPoints, *pcl_cloud);

  recogniser->recognise(cvImage, pcl_cloud, pcl::PointCloud<pcl::Normal>::Ptr(),
                        pcl::PointIndices::Ptr(), objects);

  cvReleaseImage(&iplImage);
}

void ObjectRecognizer3D::learn(const string &label, const Image &image,
    const vector<PointCloud::SurfacePoint> &points)
{
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat cvImage = cv::cvarrToMat(iplImage);

  // note that points are given in robot ego, need to be transformed into camera
  vector<PointCloud::SurfacePoint> camPoints(points.size());
  for(size_t i = 0; i < points.size(); i++)
    camPoints[i].p = cogx::Math::transform(image.camPars.pose, points[i].p);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ConvertSurfacePoints2PCLCloud(camPoints, *pcl_cloud);

  // set model for learner
  // see if we already know the model with given label
  P::ObjectModel::Ptr model;
  map<string, P::ObjectModel::Ptr>::iterator it;
  if((it = models.find(label)) != models.end())
  {
    model = it->second;
  }
  else
  {
    model.reset(new P::ObjectModel());
    models[label] = model;
  }
  recogniser->setModelLearner(model);

  // set pose for learner:
  // a reasonable pose for the view: axis aligned and centroid
  Eigen::Matrix4f pose;
  Eigen::Vector4f t;
  pcl::compute3DCentroid(*pcl_cloud, t);
  // Note: learner requires inverse pose (pose of camera w.r.t. object)
  pose.setIdentity();
  pose.block<4,1> (0, 3) = -t;
  recogniser->setPoseLearner(pose);

  // now we are ready to learn
  log("learn model with id '%s' with label '%s'", model->id.c_str(), label.c_str());
  int status = recogniser->learn(cvImage, pcl_cloud, pcl::PointIndices::Ptr(), label, pose);
  log("learned object '%s': status %d", label.c_str(), status);

  // the recogniser needs to be cleared and all currentl known models added again
  recogniser->clearRecogniser();
  for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
    recogniser->addModelRecogniser(it->second);
  if(models.size() > 0)
    recogniser->initFlannRecogniser();

  cvReleaseImage(&iplImage);
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
  log("Receiving DetectionCommand - TODO: implement");
/*  DetectionCommandPtr det_cmd = getMemoryEntry<DetectionCommand>(_wmc.address);

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
  recognize(labels, col, cv::Mat(), objects);

  // reading off results
  for(size_t i = 0; i < objects.size(); i++)
  {
    log("recognized model '%s' with confidence %f", objects[i].idObject.c_str(),
        objects[i].conf);

    VisionData::VisualObjectPtr visObj;
    // if have recognised that object previously, overwrite WM entry
    if(!objectWMIds[objects[i].idObject].empty())
    {
      log("overwriting WM entry with address '%s'",
          objectWMIds[objects[i].idObject].c_str());
      visObj = getMemoryEntry<VisualObject>(objectWMIds[objects[i].idObject]);
      objectLoationToVisualObject(objects[i], visObj);
      overwriteWorkingMemory(objectWMIds[objects[i].idObject], visObj);
    }
    // otherwise create new WM entry
    else
    {
      visObj = createVisualObject();
      log("adding WM entry with address '%s'",
          objectWMIds[objects[i].idObject].c_str());
      objectLoationToVisualObject(objects[i], visObj);
      objectWMIds[objects[i].idObject] = newDataID();
      addToWorkingMemory(objectWMIds[objects[i].idObject], visObj);
    }
  }*/
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
  VisualObjectPtr visObj =
    getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);

  // get image
  Image image;
  videoServer->getImage(camId, image);
  // get proto object with points
  ProtoObjectPtr protoObj =
      getMemoryEntry<ProtoObject>(visObj->protoObject->address.id);

  vector<P::ObjectLocation> objects;
  recognize(image, protoObj->points, objects);

  if(objects.size() > 0)
  {
    vector<string> labels;
    // if no labels are specified we recognise all our known labels
    if(!rec_cmd->labels.empty())
    {
      labels = rec_cmd->labels;
    }
    else
    {
      for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
        labels.push_back(it->first);
    }

    size_t best = 0;
    double best_conf = objects[0].conf;
    for(size_t i = 1; i < objects.size(); i++)
    {
      // only consider returned objects with a label we are interested in
      if(find(labels.begin(), labels.end(), objects[i].idObject) != labels.end())
      {
        if(objects[i].conf > best_conf)
        {
          best_conf = objects[i].conf;
          best = i;
        }
      }
    }
    // read agagin to decrease likelihood of inconsitency
    visObj = getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);
    objectLoationToVisualObject(objects[best], visObj);
  }
  else
  {
    // read agagin to decrease likelihood of inconsitency
    visObj = getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);
    visObj->identLabels.clear();
    visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
    visObj->identDistrib.clear();
    visObj->identDistrib.push_back(1.);
    recalculateIdentGainAmbiguity(visObj);
  }
  log("overwriting WM entry with address.id '%s'",
      rec_cmd->visualObject->address.id.c_str());
  overwriteWorkingMemory(rec_cmd->visualObject->address, visObj);

  log("done RecognitionCommand");
}

/**
 * Learn a view of a given visual object.
 */
void ObjectRecognizer3D::receiveLearnObjectViewCommand(const cdl::WorkingMemoryChange & _wmc)
{
  log("Receiving LearnObjectViewCommand");

  LearnObjectViewCommandPtr learn_cmd = getMemoryEntry<LearnObjectViewCommand>(_wmc.address);
  // TODO: catch exception and return with error condition if object is not
  // actually in WM
  VisualObjectPtr visObj =
    getMemoryEntry<VisualObject>(learn_cmd->visualObject->address);

  // get image
  Image image;
  videoServer->getImage(camId, image);
  // get proto object with points
  ProtoObjectPtr protoObj =
      getMemoryEntry<ProtoObject>(visObj->protoObject->address.id);

  // get the most likely label of the visual object
  string label = GetBestLabel(*visObj);
  // if it does not have any yet, invent one
  if(label.empty())
  {
    stringstream ss;
    ss << "object" << modelNameCnt++;
    label = ss.str();
  }

  log("learning new view for object '%s'", label.c_str());
  learn(label, image, protoObj->points);

  // read agagin to decrease likelihood of inconsitency
  visObj = getMemoryEntry<VisualObject>(learn_cmd->visualObject->address);
  visObj->identLabels.clear();
  visObj->identDistrib.clear();
  visObj->identLabels.push_back(label);
  visObj->identDistrib.push_back(1.);
  visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
  visObj->identDistrib.push_back(0.);
  log("overwriting WM entry with address.id '%s'",
      learn_cmd->visualObject->address.id.c_str());
  overwriteWorkingMemory(learn_cmd->visualObject->address, visObj);


  log("done LearnObjectViewCommand");
}

/**
 * Copy data from a recogniser ObjectLocation to a VisionData::VisualObject
 */
void ObjectRecognizer3D::objectLoationToVisualObject(P::ObjectLocation &objLoc,
    VisualObjectPtr &visObj)
{
  // create a very simple distribution: label and unknown
  visObj->identLabels.clear();
  visObj->identLabels.push_back(objLoc.idObject);
  visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
  // note: distribution must of course sum to 1
  visObj->identDistrib.clear();
  visObj->identDistrib.push_back(objLoc.conf);
  visObj->identDistrib.push_back(1. - objLoc.conf);
  recalculateIdentGainAmbiguity(visObj);
  
  /* TODO: overwrite
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
  visObj->pose.rot.m22 = objLoc.pose.R.at<double>(2, 2);*/

  visObj->boundingSphere.pos = visObj->pose.pos;
  visObj->boundingSphere.rad = 0.1;  // HACK

  visObj->componentID = getComponentID();
}

cv::Mat ObjectRecognizer3D::generateMaskImage(VisualObjectPtr visObj,
    const CameraParameters &camPars, bool boundingBoxOnly)
{
  cv::Mat mask(cvSize(camPars.width, camPars.height), CV_8UC1, cv::Scalar(0));
  if(boundingBoxOnly)
  {
    Rect2 roi = projectSphere(camPars, visObj->boundingSphere);
    rectangle(mask,
        cv::Point(roi.pos.x - roi.width/2, roi.pos.y - roi.height/2),
        cv::Point(roi.pos.x + roi.width/2, roi.pos.y + roi.height/2),
        cv::Scalar(255), CV_FILLED);
  }
  else
  {
    ProtoObjectPtr protoObj =
      getMemoryEntry<ProtoObject>(visObj->protoObject->address.id);
    for(size_t i = 0; i < protoObj->points.size(); i++)
    {
      Vector2 p = projectPoint(camPars, protoObj->points[i].p);
      rectangle(mask, cv::Point(p.x, p.y), cv::Point(p.x, p.y),
        cv::Scalar(255), CV_FILLED);
    }
    dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  }
  return mask;
}
