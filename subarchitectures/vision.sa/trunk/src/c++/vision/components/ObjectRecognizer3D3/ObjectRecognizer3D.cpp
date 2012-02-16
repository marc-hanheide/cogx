/**
 * @author Michael Zillich
 * @date August 2011
 * @brief Recognise trained 3D models
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

/**
 * Convert points from CAST format to PCL format.
 * NOTE: this should go to VisionUtils. But that introduces a PCL dependency in VisionUtils,
 * which affects all source files including VisionUtils.h. So leave it here for now.
 */
static inline void ConvertSurfacePoints2PCLCloud(const vector<PointCloud::SurfacePoint> &points,
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
}

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
  recogniser->SetCameraParameter(intrinsic, distortion);

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
	  log("%s: s: hardcode the address of the proto object you want to recognise here",
	      __FILE__, __FUNCTION__);
          /*RecognitionCommandPtr rec_cmd = new RecognitionCommand;
          rec_cmd->labels.push_back(it->first);
          cast::cdl::WorkingMemoryAddress addr;
          // HACK: add the address of the proto object you want to recognise here:
          addr.subarchitecture = "vision.sa";
          addr.id = "1:6";
          rec_cmd->visualObject =
            createWmPointer<VisionData::VisualObject>(addr);
          pRec->addToWorkingMemory(pRec->newDataID(), rec_cmd);*/
        }
      }
    }
  }
}
#endif

void ObjectRecognizer3D::recognize(vector<string> &labels, cv::Mat &colImg,
    cv::Mat &mask, vector<P::ObjectLocation> &objects)
{
  cv::Mat grayImg;
  cv::cvtColor(colImg, grayImg, CV_RGB2GRAY);
  IplImage displayImg = colImg; // cheap: no copy

  // the actual recognition
  // set the debug image to draw stuff into
  recogniser->SetDebugImage(colImg);
  recogniser->Recognise(grayImg, objects);

#ifdef FEAT_VISUALIZATION
  m_display.setImage(getComponentID(), &displayImg);
#endif
}

void ObjectRecognizer3D::learn(PointCloud::SurfacePointSeq &points,
    cv::Mat &mask, Pose3 &pose)
{
/*  // first convert to PCL format
  // TODO get from cast-file!
  int pointCloudWidth = 640;
  int pointCloudHeight = 480;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ConvertSurfacePoints2PCLCloud(points, *pcl_cloud, pointCloudWidth, pointCloudHeight);

  P::Pose tmp_pose;
  // TODO: convert from CAST pose to v4r pose
  cv::Mat_<cv::Vec3b> image;
  pclA::ConvertPCLCloud2Image(pcl_cloud, image);
  // recogniser->SetDebugImage(image);
  // TODO: convert pcl cloud to mat cloud
  int status = recogniser->Learn(image, matCloud, pose.R, pose.t, filename, mask);

  //recogniser->GetModelLearn(model);
  //DrawLearning(*model, dbgWin, model->views.size() - 1);
  //cv::imshow("Mask", mask);*/
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
  recognize(labels, col, cv::Mat(), objects);

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

  // HACK
  log("object '%s' has proto object '%s', last proto object: '%s'",
      _wmc.address.id, wmObj->protoObject.address.id, wmObj->lastProtoObject.address.id);

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
  recognize(labels, col, cv::Mat(), objects);

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
  }
}
