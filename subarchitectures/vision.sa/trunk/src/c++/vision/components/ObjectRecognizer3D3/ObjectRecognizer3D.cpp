/**
 * @author Michael Zillich
 * @date August 2011
 * @brief Recognise trained 3D models
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <opencv2/opencv.hpp>
#include <v4r/PCLAddOns/PCLUtils.h>
#include <v4r/PCLAddOns/PCLFunctions.h>
#include <cogxmath.h>
#include <VideoUtils.h>
#include <VisionUtils.h>
#include "ObjectRecognizer3D.h"

// if defined, the identDistrib/identLabels of a VisualObject will contain the
// unknown label (VisionData::IDENTITYxUNKNOWN = "not-learned" as first entry) as well
//#define USE_UNKOWN_LABEL

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
  double maxProb = -1.;
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
#ifdef USE_UNKOWN_LABEL
  // ambiguity in the distribution: we use the distribution's entropy
  visObj->identAmbiguity = 0.;
  for(size_t i = 0; i < visObj->identDistrib.size(); i++)
    if(fpclassify(visObj->identDistrib[i]) != FP_ZERO)
      visObj->identAmbiguity -= visObj->identDistrib[i]*::log(visObj->identDistrib[i]);
#else
  // ambiguity in the distribution: we use the distribution's entropy
  visObj->identAmbiguity = 0.;
  double sumProb = 0.;
  for(size_t i = 0; i < visObj->identDistrib.size(); i++)
    sumProb += visObj->identDistrib[i];
  // the remaining probability is assigned to unknown
  double unknownProb = 1. - sumProb;
  for(size_t i = 0; i < visObj->identDistrib.size(); i++)
    if(fpclassify(visObj->identDistrib[i]) != FP_ZERO)
      visObj->identAmbiguity -= visObj->identDistrib[i]*::log(visObj->identDistrib[i]);
  if(fpclassify(unknownProb) != FP_ZERO)
    visObj->identAmbiguity -= unknownProb*::log(unknownProb);
#endif
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

  if((it = _config.find("--videoname")) != _config.end())
    videoServerName = it->second;

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  if((it = _config.find("--databasepath")) != _config.end())
    databasePath = it->second;

  if((it = _config.find("--labels")) != _config.end())
    labeliss.str(it->second);

  // make sure database path has a trailing '/'
  if(!databasePath.empty())
  {
    if(*databasePath.rbegin() != '/')
      databasePath += '/';
  }

  // Load all given models. Note that further models can be learend at
  // run time.
  std::string label, modelfile;
  models.clear();
  P::CModelHandler cmhandler;
  while(labeliss >> label)
  {
    P::ObjectModel::Ptr model(new P::ObjectModel());
    modelfile = databasePath + label + ".cm";
    log("loading model '%s' ...", modelfile.c_str());
    if(cmhandler.Load(modelfile, model))
    {
      models[label] = model;
      log("... done");
    }
    else
    {
      log("... failed");
    }
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

  addChangeFilter(createGlobalTypeFilter<VisionData::VisualLearningTask>(cdl::ADD),
    new MemberFunctionChangeReceiver<ObjectRecognizer3D>(this,
      &ObjectRecognizer3D::receiveVisualLearningTask));

#ifdef FEAT_VISUALIZATION        
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(getComponentID(), string("button.save"), string("Save Database"));
  //m_display.addButton(getComponentID(), string("button.load"), string("Load Database"));
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
  ostringstream ss;
  ss <<  "function render()\nend\n";
  m_display.setLuaGlObject("Recognizer3D.Models", guiid("Models"), ss.str());
#endif
}

#ifdef FEAT_VISUALIZATION
void ObjectRecognizer3D::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if(event.type == Visualization::evButtonClick)
  {
    if(event.sourceId == "button.save")
    {
      pRec->saveModelDatabase();
    }
    /*else if(event.sourceId == "button.load.all")
    {
      printf("NOTE: Loading at runtime not implemented! Specify objects in the cast command line.");
    }*/
    else if(event.sourceId == "button.det.all")
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

void ObjectRecognizer3D::saveModelDatabase()
{
  P::CModelHandler cmhandler;
  for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
  {
    string modelfile = databasePath + it->first + ".cm";
    log("saving model '%s'", modelfile.c_str());
    cmhandler.Save(modelfile, it->second);
  }
}

/**
 * Calculate actual recognition probability from confidence.
 * Using learned observation functions from a small set of example objects.
 * Note: actually these observation functions should be learned for each object
 * or actually each view. But in approximation a more general obs.func. is OK.
 */
double ObjectRecognizer3D::recognitionConfidenceToProbability(double conf)
{
  // trained from dino
  //const double p_mu = 0.142046, p_sig = 0.077907;
  //const double n_mu = 0.030988, n_sig = 0.022120;
  // trained from digestive
  const double p_mu = 0.108980, p_sig = 0.064258;
  const double n_mu = 0.016250, n_sig = 0.010887;
  // object prior probability
  const double p_o = 1.;
  double prob = normpdf(conf, p_mu, p_sig) * p_o /
         (normpdf(conf, p_mu, p_sig) + normpdf(conf, n_mu, n_sig));
  log("conf %f to prob %f\n", conf, prob);
  return prob;
}

void ObjectRecognizer3D::visualizeLearnedObject(cv::Mat &img, 
    const CameraParameters &cam, P::ObjectModel::Ptr obj)
{
  double cx = cam.cx/2.;
  double cy = cam.cy/2.;
  cv::Mat intrinsic(cv::Mat::zeros(3,3,CV_64F));
  intrinsic.at<double>(0, 0) = 2.*cam.fx;
  intrinsic.at<double>(1, 1) = 2.*cam.fy;
  intrinsic.at<double>(2, 2) = 1.;
  for(size_t i = 0; i < obj->views.size(); i++)
  {
    int row = i / 3;
    int col = i % 3;
    intrinsic.at<double>(0, 2) = cx*(1. + (double)col);
    intrinsic.at<double>(1, 2) = cy*(1. + (double)row);
    // last params: brighness and whether to draw keypoints
    P::View::draw(*obj->views[i], intrinsic, img, Eigen::Matrix4f::Identity(),
        1., true);
  }
}

void ObjectRecognizer3D::visualizeRecognizedObject(cv::Mat &img,
    const CameraParameters &cam, P::ObjectModel::Ptr obj, P::ObjectLocation &loc)
{
  if(loc.idxView < obj->views.size())
  {
    cv::Mat intrinsic(cv::Mat::zeros(3,3,CV_64F));
    intrinsic.at<double>(0, 0) = cam.fx;
    intrinsic.at<double>(0, 2) = cam.cx;
    intrinsic.at<double>(1, 1) = cam.fy;
    intrinsic.at<double>(1, 2) = cam.cy;
    intrinsic.at<double>(2, 2) = 1.;
    // last params: brighness and whether to draw keypoints
    P::View::draw(*obj->views[loc.idxView], intrinsic, img, loc.pose, 2., false);
  }
}

void ObjectRecognizer3D::setRecogniserCamereParameters(Image &image)
{
  // TODO; set actual distortion
  cv::Mat intrinsic(cv::Mat::zeros(3,3,CV_64F)), distortion(cv::Mat::zeros(4,1,CV_64F));
  intrinsic.at<double>(0, 0) = image.camPars.fx;
  intrinsic.at<double>(0, 2) = image.camPars.cx;
  intrinsic.at<double>(1, 1) = image.camPars.fy;
  intrinsic.at<double>(1, 2) = image.camPars.cy;
  intrinsic.at<double>(2, 2) = 1.;
  recogniser->setCameraParameter(intrinsic, distortion);
}

void ObjectRecognizer3D::recognize(const Image &image,
    const vector<PointCloud::SurfacePoint> &points,
    vector<P::ObjectLocation> &objects)
{
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat cvImage = cv::cvarrToMat(iplImage);
  objects.clear();

  try
  {
    // note that points are given in robot ego, need to be transformed into camera
    vector<PointCloud::SurfacePoint> camPoints(points.size());
    for(size_t i = 0; i < points.size(); i++)
      camPoints[i].p = cogx::Math::transformInverse(image.camPars.pose, points[i].p);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ConvertSurfacePoints2PCLCloud(camPoints, *pcl_cloud);

    if(pcl_cloud.get() != 0 && pcl_cloud->points.size() >= 3)
    {
      recogniser->recognise(cvImage, pcl_cloud, pcl::PointCloud<pcl::Normal>::Ptr(),
                            pcl::PointIndices::Ptr(), objects);
#ifdef FEAT_VISUALIZATION
      cvImage *= 0.5;
      // visualize the most confident result, i.e. is the first in the sorted
      // list of results. Note that we do not pay attention to the label here as
      // in receiveRecognitionCommand.
      if(objects.size() > 0)
        visualizeRecognizedObject(cvImage, image.camPars, models[objects[0].idObject], objects[0]);
      Image dispImage;
      convertImageFromIpl(iplImage, dispImage);
      m_display.setImage(getComponentID(), dispImage);
#endif
    }
    else
    {
      if(pcl_cloud.get() != 0)
        log("***** point cloud is 0 ***");
      if(pcl_cloud->points.size() >= 3)
        log("***** point cloud has fewer than 3 points ***");
    }
  }
  catch(runtime_error &e)
  {
    log("caught exception: " + string(e.what()));
  }

  cvReleaseImage(&iplImage);
}

bool ObjectRecognizer3D::learn(const string &label, const Image &image,
    const vector<PointCloud::SurfacePoint> &points)
{
  bool succeed = false;
  IplImage *iplImage = convertImageToIpl(image);
  cv::Mat cvImage = cv::cvarrToMat(iplImage);

  try
  {
    // note that points are given in robot ego, need to be transformed into camera
    vector<PointCloud::SurfacePoint> camPoints(points.size());
    for(size_t i = 0; i < points.size(); i++)
      camPoints[i].p = cogx::Math::transformInverse(image.camPars.pose, points[i].p);
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
    // (0..not_learned, 1..tracked, 2..recognised, 3..learned)
    if(status == 3)
    {
      succeed = true;

      // the recogniser needs to be cleared and all currentl known models added again
      recogniser->clearRecogniser();
      for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it != models.end(); it++)
        recogniser->addModelRecogniser(it->second);
      if(models.size() > 0)
        recogniser->initFlannRecogniser();

#ifdef FEAT_VISUALIZATION
      /*cvImage = cvScalar(0);
      // visualize the learned views
      visualizeLearnedObject(cvImage, image.camPars, model);
      Image dispImage;
      convertImageFromIpl(iplImage, dispImage);
      m_display.setImage(getComponentID(), dispImage);*/
      drawModels();
#endif
    }
  }
  catch(runtime_error &e)
  {
    log("caught exception: " + string(e.what()));
  }

  cvReleaseImage(&iplImage);

  return succeed;
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
  setRecogniserCamereParameters(image);
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
  setRecogniserCamereParameters(image);
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
    // read agagin to at least decrease likelihood of WM access inconsitency
    visObj = getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);
    objectLoationToVisualObject(objects[best], visObj);
  }
  else
  {
    // read agagin to at least decrease likelihood of WM access inconsitency
    visObj = getMemoryEntry<VisualObject>(rec_cmd->visualObject->address);
    visObj->identLabels.clear();
    visObj->identDistrib.clear();
#ifdef USE_UNKOWN_LABEL
    visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
    visObj->identDistrib.push_back(1.0);
#endif
    recalculateIdentGainAmbiguity(visObj);
  }
  log("overwriting WM entry with address.id '%s'",
      rec_cmd->visualObject->address.id.c_str());
  overwriteWorkingMemory(rec_cmd->visualObject->address, visObj);

  log("done RecognitionCommand");
}

/**
 * Perform a learning step, i.e. learn a new view of a given visual object.
 */
void ObjectRecognizer3D::receiveVisualLearningTask(const cdl::WorkingMemoryChange & _wmc)
{
  log("Receiving VisualLearningTask");
  VisualLearningTaskPtr learn_task = getMemoryEntry<VisualLearningTask>(_wmc.address);
  if(learn_task->concept == "objecttype")
  {
    bool succeed = false;
    log("this is an objecttype task");
    if(learn_task->labels.size() == 1)
    {
      log("learning new view for object '%s'", learn_task->labels[0].c_str());
      // NOTE: we ignore the label weight
      if(learnObjectView(learn_task->visualObjectAddr->address, learn_task->labels[0]))
        succeed = true;
    }
    else
    {
      log("more than one label given!");
    }
    if(succeed)
      learn_task->status = VisionData::VCSUCCEEDED;
    else
      learn_task->status = VisionData::VCFAILED;
    overwriteWorkingMemory(_wmc.address, learn_task);
  }
  log("done VisualLearningTask");
}

/**
 * Learn a view of a given visual object.
 */
void ObjectRecognizer3D::receiveLearnObjectViewCommand(const cdl::WorkingMemoryChange & _wmc)
{
  log("Receiving LearnObjectViewCommand");
  LearnObjectViewCommandPtr learn_cmd = getMemoryEntry<LearnObjectViewCommand>(_wmc.address);
  learnObjectView(learn_cmd->visualObject->address, string(""));
  log("done LearnObjectViewCommand");
}

/**
 * Learn a view of a given visual object.
 * NOTE: label is passed by value on purpose.
 */
bool ObjectRecognizer3D::learnObjectView(cast::cdl::WorkingMemoryAddress
    &visObjAddr, string label)
{
  // TODO: catch exception and return with error condition if object is not
  // actually in WM
  VisualObjectPtr visObj =
    getMemoryEntry<VisualObject>(visObjAddr);
  // get image
  Image image;
  videoServer->getImage(camId, image);
  setRecogniserCamereParameters(image);
  // get proto object with points
  ProtoObjectPtr protoObj =
      getMemoryEntry<ProtoObject>(visObj->protoObject->address.id);

  // if no label given, get the currently most likely label of the visual object
  if(label.empty())
    label = GetBestLabel(*visObj);
  // if we still have no label, invent one
  if(label.empty())
  {
    stringstream ss;
    ss << "object" << modelNameCnt++;
    label = ss.str();
  }

  log("learning new view for object '%s'", label.c_str());
  bool succeed = learn(label, image, protoObj->points);
  if(succeed)
  {
    log("OK, learned a new view");
    // read again to at least decrease likelihood of WM access inconsitency
    visObj = getMemoryEntry<VisualObject>(visObjAddr);
    visObj->identLabels.clear();
    visObj->identDistrib.clear();
#ifdef USE_UNKOWN_LABEL
    visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
    visObj->identDistrib.push_back(0.0);
#endif
    visObj->identLabels.push_back(label);
    visObj->identDistrib.push_back(1.0);
    recalculateIdentGainAmbiguity(visObj);
    log("overwriting WM entry with address.id '%s'", visObjAddr.id.c_str());
    overwriteWorkingMemory(visObjAddr, visObj);
  }
  else
  {
    log("failed to learn a new view");
  }

  return succeed;
}

/**
 * Copy data from a recogniser ObjectLocation to a VisionData::VisualObject
 */
void ObjectRecognizer3D::objectLoationToVisualObject(P::ObjectLocation &objLoc,
    VisualObjectPtr &visObj)
{
  double objProb = recognitionConfidenceToProbability(objLoc.conf);
  visObj->identLabels.clear();
  visObj->identDistrib.clear();
#ifdef USE_UNKOWN_LABEL
  visObj->identLabels.push_back(VisionData::IDENTITYxUNKNOWN);
  visObj->identDistrib.push_back(1.0 - objProb);
#endif
  visObj->identLabels.push_back(objLoc.idObject);
  visObj->identDistrib.push_back(objProb);
  recalculateIdentGainAmbiguity(visObj);
  
  /* TODO: transform from camera to ego pose
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

#ifdef FEAT_VISUALIZATION
static void drawCube(std::ostringstream &str, double size)
{
  double s = size/2.;
  str << "glBegin(GL_LINE_LOOP)\n";
  str << "v(" << -s << "," << -s << "," << s << ")\n";
  str << "v(" << s << "," << -s << "," << s << ")\n";
  str << "v(" << s << "," << s << "," << s << ")\n";
  str << "v(" << -s << "," << s << "," << s << ")\n";
  str << "glEnd()\n";
  str << "glBegin(GL_LINE_LOOP)\n";
  str << "v(" << -s << "," << -s << "," << -s << ")\n";
  str << "v(" << s << "," << -s << "," << -s << ")\n";
  str << "v(" << s << "," << s << "," << -s << ")\n";
  str << "v(" << -s << "," << s << "," << -s << ")\n";
  str << "glEnd()\n";
}

void ObjectRecognizer3D::drawModels()
{
#define FCHN(x) (float)x/255.0
  double modelSeparation = 1.0;
  double viewSeparation = 0.5;
  std::ostringstream str;

  str.unsetf(ios::floatfield); // unset floatfield
  str.precision(3); // set the _maximum_ precision
  str << "function render()\n";
  str << "glPointSize(2)\n";
  str << "v=glVertex\nc=glColor\n";
  
  // coordinate cross
  str << "glBegin(GL_LINES)\n";
  str << "c(255,0,0)\n";
  str << "v(" << 0 << "," << 0 << "," << 0 << ")\n";
  str << "v(" << 0.2 << "," << 0 << "," << 0 << ")\n";
  str << "c(0,255,0)\n";
  str << "v(" << 0 << "," << 0 << "," << 0 << ")\n";
  str << "v(" << 0 << "," << 0.2 << "," << 0 << ")\n";
  str << "c(0,0,255)\n";
  str << "v(" << 0 << "," << 0 << "," << 0 << ")\n";
  str << "v(" << 0 << "," << 0 << "," << 0.2 << ")\n";
  str << "glEnd()\n";

  // models
  int modelCnt = 0;
  for(map<string, P::ObjectModel::Ptr>::iterator it = models.begin(); it !=
      models.end(); it++, modelCnt++)
  {
    double dy = modelSeparation*(double)modelCnt;
    str << "showLabel(0, " << dy << ", " << viewSeparation*0.75 <<", '" << it->second->id << "', 12);\n";
    for(size_t i = 0; i < it->second->views.size(); i++)
    {
      double dx = viewSeparation*(double)i;

      str << "glPushMatrix()\n";
      str << "glTranslate(" << dx << "," << dy << "," << 0 << ")\n";

      // draw a green cube around each view
      str << "c(0,255,0)\n";
      drawCube(str, viewSeparation);
      str << "glBegin(GL_POINTS)\n";

      pcl::PointCloud<pcl::PointXYZRGB> &cloud = *it->second->views[i]->cloud;
      for(size_t j = 0; j < cloud.points.size(); j++)
      {
        pcl::PointXYZRGB &p = cloud.points[j];
        str << "c(" << FCHN(p.r) << "," << FCHN(p.g) << "," << FCHN(p.b) << ")\n";
        str << "v(" << p.x << "," << p.y << "," << p.z << ")\n";
      }
      str << "glEnd()\n";
      str << "glPopMatrix()\n";
    }
  }
  str << "end\n";
  m_display.setLuaGlObject("Recognizer3D.Models", guiid("Models"), str.str());
#undef FCHN
}
#endif

void ObjectRecognizer3D::runComponent()
{
	while(isRunning())
  {
    drawModels();
    sleep(2);
  }
}

