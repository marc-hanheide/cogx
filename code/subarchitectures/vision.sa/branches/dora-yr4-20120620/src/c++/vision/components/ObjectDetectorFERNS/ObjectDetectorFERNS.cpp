/**
 * @author Michael Zillich
 * @date February 2009
 *
 * This component just wraps the FERNS tracker by
 * Authors: Vincent Lepetit (http://cvlab.epfl.ch/~lepetit)
 *          Mustafa Ozuysal (http://cvlab.epfl.ch/~oezuysal)
 *          Julien  Pilet   (http://cvlab.epfl.ch/~jpilet)
 */

#include <cctype>
#include <cassert>
#include <sstream>
#include <highgui.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VideoUtils.h>
#include "ObjectDetectorFERNS.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectDetectorFERNS();
  }
}

namespace cast
{

using namespace std;
using namespace cogx::Math;
using namespace VisionData;

// font for drawing text into OpenCV windows
static CvFont font;

string tolower(const string &upper)
{
  string lower(upper);
  for(size_t i = 0; i < lower.size(); i++)
    lower[i] = (char)::tolower((int)upper[i]);
  return lower;
}

Rect2 calculateObjectBoundingBox(planar_pattern_detector * d)
{
  Rect2 bbox;
  double xmin = min(min(d->detected_u_corner[0], d->detected_u_corner[1]),
                    min(d->detected_u_corner[2], d->detected_u_corner[3]));
  double xmax = max(max(d->detected_u_corner[0], d->detected_u_corner[1]),
                    max(d->detected_u_corner[2], d->detected_u_corner[3]));
  double ymin = min(min(d->detected_v_corner[0], d->detected_v_corner[1]),
                    min(d->detected_v_corner[2], d->detected_v_corner[3]));
  double ymax = max(max(d->detected_v_corner[0], d->detected_v_corner[1]),
                    max(d->detected_v_corner[2], d->detected_v_corner[3]));
  bbox.pos.x = (xmin + xmax)/2.;
  bbox.pos.y = (ymin + ymax)/2.;
  bbox.width = (xmax - xmin) + 1.;
  bbox.height = (ymax - ymin) + 1.;
  return bbox;
}

Rect2 calculateObjectBoundingBox(
    template_matching_based_tracker * t)
{
  Rect2 bbox;
  double xmin = min(min(t->u[0], t->u[2]),
                    min(t->u[4], t->u[6]));
  double xmax = max(max(t->u[0], t->u[2]),
                    max(t->u[4], t->u[6]));
  double ymin = min(min(t->u[1], t->u[3]),
                    min(t->u[5], t->u[7]));
  double ymax = max(max(t->u[1], t->u[3]),
                    max(t->u[5], t->u[7]));
  bbox.pos.x = (xmin + xmax)/2.;
  bbox.pos.y = (ymin + ymax)/2.;
  bbox.width = (xmax - xmin) + 1.;
  bbox.height = (ymax - ymin) + 1.;
  return bbox;
}

void draw_quadrangle(IplImage * frame,
		     int u0, int v0,
		     int u1, int v1,
		     int u2, int v2,
		     int u3, int v3,
		     CvScalar color, int thickness = 1)
{
  cvLine(frame, cvPoint(u0, v0), cvPoint(u1, v1), color, thickness);
  cvLine(frame, cvPoint(u1, v1), cvPoint(u2, v2), color, thickness);
  cvLine(frame, cvPoint(u2, v2), cvPoint(u3, v3), color, thickness);
  cvLine(frame, cvPoint(u3, v3), cvPoint(u0, v0), color, thickness);
}

void draw_detected_position(IplImage * frame,
    planar_pattern_detector * detector)
{
  draw_quadrangle(frame,
		  detector->detected_u_corner[0], detector->detected_v_corner[0],
		  detector->detected_u_corner[1], detector->detected_v_corner[1],
		  detector->detected_u_corner[2], detector->detected_v_corner[2],
		  detector->detected_u_corner[3], detector->detected_v_corner[3],
		  cvScalar(255), 3);
}

void draw_initial_rectangle(IplImage * frame,
    template_matching_based_tracker * tracker)
{
  draw_quadrangle(frame,
		  tracker->u0[0], tracker->u0[1],
		  tracker->u0[2], tracker->u0[3],
		  tracker->u0[4], tracker->u0[5],
		  tracker->u0[6], tracker->u0[7],
		  cvScalar(128), 3);
}

void draw_tracked_position(IplImage * frame,
    template_matching_based_tracker * tracker)
{
  draw_quadrangle(frame,
		  tracker->u[0], tracker->u[1],
		  tracker->u[2], tracker->u[3],
		  tracker->u[4], tracker->u[5],
		  tracker->u[6], tracker->u[7],
		  cvScalar(255), 3);
}

void draw_tracked_locations(IplImage * frame,
    template_matching_based_tracker * tracker)
{
  for(int i = 0; i < tracker->nx * tracker->ny; i++) {
    int x1, y1;
    tracker->f.transform_point(tracker->m[2 * i], tracker->m[2 * i + 1], x1, y1);
    cvCircle(frame, cvPoint(x1, y1), 3, cvScalar(255, 255, 255), 1);
  }
}

void draw_detected_keypoints(IplImage * frame,
    planar_pattern_detector * detector)
{
  for(int i = 0; i < detector->number_of_detected_points; i++)
    cvCircle(frame,
	     cvPoint(detector->detected_points[i].fr_u(),
		     detector->detected_points[i].fr_v()),
	     16 * (1 << int(detector->detected_points[i].scale)),
	     cvScalar(100), 1);
}

void draw_recognized_keypoints(IplImage * frame,
    planar_pattern_detector * detector)
{
  for(int i = 0; i < detector->number_of_model_points; i++)
    if (detector->model_points[i].class_score > 0)
      cvCircle(frame,
	       cvPoint(detector->model_points[i].potential_correspondent->fr_u(),
		       detector->model_points[i].potential_correspondent->fr_v()),
	       16 * (1 << int(detector->detected_points[i].scale)),
	       cvScalar(255, 255, 255), 1);
}

void draw_detected_label(IplImage * frame, planar_pattern_detector * detector,
    string & label)
{
  double x = (detector->detected_u_corner[0] + detector->detected_u_corner[1] +
      detector->detected_u_corner[2] + detector->detected_u_corner[3])/4;
  double y = (detector->detected_v_corner[0] + detector->detected_v_corner[1] +
      detector->detected_v_corner[2] + detector->detected_v_corner[3])/4;
  cvPutText(frame, label.c_str(), cvPoint(x, y), &font, cvScalar(255));
};

void draw_tracked_label(IplImage * frame,
    template_matching_based_tracker * tracker, string & label)
{
  double x = (tracker->u[0] + tracker->u[2] + tracker->u[4] + tracker->u[6])/4;
  double y = (tracker->u[1] + tracker->u[3] + tracker->u[5] + tracker->u[7])/4;
  cvPutText(frame, label.c_str(), cvPoint(x, y), &font, cvScalar(255));
};

void draw_detected_bounding_box(IplImage * frame,
    planar_pattern_detector * detector)
{
  Rect2 bbox = calculateObjectBoundingBox(detector);
  draw_quadrangle(frame,
     bbox.pos.x - bbox.width/2., bbox.pos.y - bbox.height/2.,
     bbox.pos.x + bbox.width/2., bbox.pos.y - bbox.height/2.,
     bbox.pos.x + bbox.width/2., bbox.pos.y + bbox.height/2.,
     bbox.pos.x - bbox.width/2., bbox.pos.y + bbox.height/2.,
     cvScalar(255), 5);
}

void draw_tracked_bounding_box(IplImage * frame,
    template_matching_based_tracker * tracker)
{
  Rect2 bbox = calculateObjectBoundingBox(tracker);
  draw_quadrangle(frame,
     bbox.pos.x - bbox.width/2., bbox.pos.y - bbox.height/2.,
     bbox.pos.x + bbox.width/2., bbox.pos.y - bbox.height/2.,
     bbox.pos.x + bbox.width/2., bbox.pos.y + bbox.height/2.,
     bbox.pos.x - bbox.width/2., bbox.pos.y + bbox.height/2.,
     cvScalar(255), 5);
}


ObjectDetectorFERNS::ObjectDetectorFERNS()
{
  mode = DETECT_AND_TRACK;
  camId = 0;
  doDisplay = false;
  show_tracked_locations = false;
  show_keypoints = false;
}

ObjectDetectorFERNS::~ObjectDetectorFERNS()
{
  for(size_t i = 0; i < model_images.size(); i++)
  {
    delete detectors[i];
    delete trackers[i];
  }

#ifndef FEAT_VISUALIZATION
  if(doDisplay)
    cvDestroyWindow("ObjectDetectorFERNS");
#endif
}

void ObjectDetectorFERNS::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

  if((it = _config.find("--models")) != _config.end())
  {
    istringstream istr(it->second);
    string model;
    while(istr >> model)
    {
      string label;
      // remove pathname: find last of '/' (Unix) or '\' (DOS)
      size_t start = model.find_last_of("/\\");
      if(start == string::npos)
        start = 0;
      else
        start++;
      // remove suffix .jpg etc: find last '.'
      size_t end = model.find_last_of(".");
      if(end == string::npos)
        end = model.size();
      label = model.substr(start, end - start);
      model_images.push_back(model);
      model_labels.push_back(label);
    }

    ostringstream ostr;
    for(size_t i = 0; i < model_images.size(); i++)
      ostr << " '" << model_images[i] << "'";
    log("using models: %s", ostr.str().c_str());
  }

  if((it = _config.find("--mode")) != _config.end())
  {
    if(it->second == "DETECT_AND_TRACK")
      mode = DETECT_AND_TRACK;
    else if(it->second == "DETECT_ONLY")
      mode = DETECT_ONLY;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

  if((it = _config.find("--displaylevel")) != _config.end())
  {
    istringstream istr(it->second);
    int level;
    istr >> level;
    if(level > 0)
      doDisplay = true;
    if(level > 1)
      show_tracked_locations = true;
    if(level > 2)
      show_keypoints = true;
  }

  if(doDisplay)
  {
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,
         1.0, 1.0, 0.0,
         3, 8);

#ifdef FEAT_VISUALIZATION
    m_display.configureDisplayClient(_config);
#else
    cvNamedWindow("ObjectDetectorFERNS", 1);
#endif
  }

  setupFERNS();
}

void ObjectDetectorFERNS::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // we want to receive DetectionCommands
  addChangeFilter(createLocalTypeFilter<DetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectDetectorFERNS>(this,
        &ObjectDetectorFERNS::receiveDetectionCommand));
}

void ObjectDetectorFERNS::runComponent()
{
#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
#endif
}

void ObjectDetectorFERNS::receiveDetectionCommand(
    const cdl::WorkingMemoryChange & _wmc)
{
  DetectionCommandPtr cmd =
    getMemoryEntry<DetectionCommand>(_wmc.address);

  ostringstream ostr;
  for(size_t i = 0; i < cmd->labels.size(); i++)
    ostr << " '" << cmd->labels[i] << "'";
  log("FERNS detecting: %s", ostr.str().c_str());

  Video::Image image;
  videoServer->getImage(camId, image);
  IplImage *grayImage = convertImageToIplGray(image);
  detectObjects(grayImage, cmd->labels);
  postObjectsToWM(cmd->labels, image);
  if(doDisplay)
  {
    drawResults(grayImage);
#ifdef FEAT_VISUALIZATION
    m_display.setImage("ObjectDetectorFERNS", grayImage);
#else
    cvShowImage("ObjectDetectorFERNS", grayImage);
    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    cvWaitKey(10);
#endif
  }
  cvReleaseImage(&grayImage);

  // executed the command, results (if any) are on working memory,
  // now delete command as not needed anymore
  deleteFromWorkingMemory(_wmc.address);
}

void ObjectDetectorFERNS::setupFERNS() throw(runtime_error)
{
  affine_transformation_range range;

  detectors.resize(model_images.size());
  trackers.resize(model_images.size());
  last_frame_ok.resize(model_images.size());
  objWMIds.resize(model_images.size());

  for(size_t i = 0; i < detectors.size(); i++)
  {
    detectors[i] = planar_pattern_detector_builder::build_with_cache(
                     model_images[i].c_str(),
							       &range,
							       40,  // 400
							       500, // 5000
							       0.0,
							       32, 7, 4,
							       30, 12,
							       10000, 200);

    if (!detectors[i])
      throw runtime_error(exceptionMessage(__HERE__, "Unable to build detector."));

    detectors[i]->set_maximum_number_of_points_to_detect(1000);

    trackers[i] = new template_matching_based_tracker();
    string trackerfn = model_images[i] + string(".tracker_data");
    if (!trackers[i]->load(trackerfn.c_str()))
    {
      log("Training template matching...\n");
      trackers[i]->learn(detectors[i]->model_image,
         5, // number of used matrices (coarse-to-fine)
         40, // max motion in pixel used to train to coarser matrix
         20, 20, // defines a grid. Each cell will have one tracked point.
         detectors[i]->u_corner[0], detectors[i]->v_corner[1],
         detectors[i]->u_corner[2], detectors[i]->v_corner[2],
         40, 40, // neighbordhood for local maxima selection
         10000 // number of training samples
         );
      trackers[i]->save(trackerfn.c_str());
    }
    trackers[i]->initialize();

    last_frame_ok[i] = false;
  }
}

void ObjectDetectorFERNS::detectObject(IplImage * frame, const string & label)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    if(model_labels[i] == label)
      detectObject_Internal(frame, i);
}

void ObjectDetectorFERNS::detectObjects(IplImage * frame,
    const vector<string> & labels)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    if(find(labels.begin(), labels.end(), model_labels[i]) != labels.end())
      detectObject_Internal(frame, i);
}

void ObjectDetectorFERNS::detectAllObjects(IplImage * frame)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    detectObject_Internal(frame, i);
}

void ObjectDetectorFERNS::detectObject_Internal(IplImage * frame, size_t i)
{
  if(mode == DETECT_AND_TRACK)
  {
    if(last_frame_ok[i])
      last_frame_ok[i] = trackers[i]->track(frame);

    if(!last_frame_ok[i])
    {
      detectors[i]->detect(frame);

      if(detectors[i]->pattern_is_detected)
      {
        last_frame_ok[i] = true;

        trackers[i]->initialize(
            detectors[i]->detected_u_corner[0], detectors[i]->detected_v_corner[0],
            detectors[i]->detected_u_corner[1], detectors[i]->detected_v_corner[1],
            detectors[i]->detected_u_corner[2], detectors[i]->detected_v_corner[2],
            detectors[i]->detected_u_corner[3], detectors[i]->detected_v_corner[3]);
      }
      // else last_frame_ok remains false
    }
  }
  else // mode == DETECT_ONLY
  {
    detectors[i]->detect(frame);
    last_frame_ok[i] = detectors[i]->pattern_is_detected;
  }
}

void ObjectDetectorFERNS::drawResults(IplImage * frame)
{
  for(size_t i = 0; i < last_frame_ok.size(); i++)
  {
    if(last_frame_ok[i])
    {
      if(mode == DETECT_AND_TRACK)
      {
        draw_tracked_label(frame, trackers[i], model_labels[i]);
        draw_tracked_bounding_box(frame, trackers[i]);
        if(show_tracked_locations)
          draw_tracked_position(frame, trackers[i]);
        if(show_keypoints)
          draw_tracked_locations(frame, trackers[i]);
      }
      else // mode == DETECT_ONLY
      {
        draw_detected_label(frame, detectors[i], model_labels[i]);
        draw_detected_bounding_box(frame, detectors[i]);
        if(show_tracked_locations)
          draw_detected_position(frame, detectors[i]);
        if(show_keypoints)
        {
          draw_detected_keypoints(frame, detectors[i]);
          draw_recognized_keypoints(frame, detectors[i]);
        }
      }
    }
  }
}

void ObjectDetectorFERNS::postObjectToWM(const string & label,
    const Video::Image &image)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    if(model_labels[i] == label)
      postObjectToWM_Internal(i, image);
}

void ObjectDetectorFERNS::postObjectsToWM(const vector<string> & labels,
    const Video::Image &image)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    if(find(labels.begin(), labels.end(), model_labels[i]) != labels.end())
      postObjectToWM_Internal(i, image);
}

void ObjectDetectorFERNS::postAllObjectsToWM(const Video::Image &image)
{
  for(size_t i = 0; i < model_images.size(); i++)
    postObjectToWM_Internal(i, image);
}

void ObjectDetectorFERNS::postObjectToWM_Internal(size_t i,
    const Video::Image &image)
{
  VisualObjectPtr obj = createVisualObject(i, image);

  // if no WM ID yet for that object
  if(objWMIds[i] == "")
  {
    objWMIds[i] = newDataID();
    addToWorkingMemory(objWMIds[i], obj);
  }
  else
  {
    overwriteWorkingMemory(objWMIds[i], obj);
  }
}

VisualObjectPtr ObjectDetectorFERNS::createVisualObject(size_t i,
    const Video::Image &image)
{
  VisualObjectPtr obj =  new VisualObject;

  Rect2 bbox;
  if(mode == DETECT_AND_TRACK)
    bbox = calculateObjectBoundingBox(trackers[i]);
  else // mode == DETECT_ONLY
    bbox = calculateObjectBoundingBox(detectors[i]);

  double detectionConfidence;
  if(detectors[i]->pattern_is_detected || last_frame_ok[i])
    detectionConfidence = 1.;
  else
    detectionConfidence = 0.;

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(model_labels[i]);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(detectionConfidence);
  obj->identDistrib.push_back(1. - detectionConfidence);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);

  obj->time = image.time;

  // don't have 3D information at this point
  setIdentity(obj->pose);
  setZero(obj->boundingSphere.pos);
  obj->boundingSphere.rad = 0.;

  obj->views.resize(1);
  obj->views[0].boundingBox = bbox;
  obj->views[0].camId = image.camId;
  obj->views[0].detectionConfidence = obj->detectionConfidence;

  return obj;
}

}

