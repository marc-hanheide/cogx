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
#include <set>
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
  numDetectionAttempts = 1;
  doDisplay = false;
  show_tracked_locations = false;
  show_keypoints = false;
  outputToNav = false;
}

ObjectDetectorFERNS::~ObjectDetectorFERNS()
{
  for(size_t i = 0; i < model_images.size(); i++)
  {
    delete detectors[i];
    delete trackers[i];
  }
  if(doDisplay)
    cvDestroyWindow("ObjectDetectorFERNS");
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
      // find label name, either given explicitely as
      // some/path/foo-front.jpg:foolabel
      // or if no ':' is found by implicitly assuming stripped filename as
      // label: foo-front
      size_t label_pos = model.find_last_of(":");
      if(label_pos != string::npos && label_pos < model.size() - 1)
      {
        label = model.substr(label_pos + 1, string::npos);
        model = model.substr(0, label_pos);
      }
      else
      {
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
      }
      model_images.push_back(model);
      model_labels.push_back(label);
    }

    ostringstream ostr;
    for(size_t i = 0; i < model_images.size(); i++)
      ostr << " " << model_images[i] << ":" << model_labels[i];
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

  if((it = _config.find("--numattempts")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> numDetectionAttempts;
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

  if((it = _config.find("--output_to_nav")) != _config.end())
  {
    if(tolower(it->second) == "true")
      outputToNav = true;
  }

  if(doDisplay)
  {
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,
         1.0, 1.0, 0.0,
         3, 8);
    cvNamedWindow("ObjectDetectorFERNS", 1);
  }

  setupFERNS();
  imgcnt = 0;
}

void ObjectDetectorFERNS::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // we want to receive DetectionCommands
  addChangeFilter(createGlobalTypeFilter<DetectionCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<ObjectDetectorFERNS>(this,
        &ObjectDetectorFERNS::receiveDetectionCommand));
}

size_t ObjectDetectorFERNS::indexOf(const string &label) throw(runtime_error)
{
  for(size_t i = 0; i < model_labels.size(); i++)
    if(label == model_labels[i])
      return i;
  throw runtime_error(exceptionMessage(__HERE__, "unknown label '%s'", label.c_str()));
  return 0;
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

  // set holding all objects that were detected over a series of images
  std::set<string> detectedObjects;

  // try detection in a series of images
  for(int i = 0; i < numDetectionAttempts; i++)
  {
    videoServer->getImage(camId, image);
    IplImage *grayImage = convertImageToIplGray(image);
    detectObjects(grayImage, cmd->labels);
    // remember what objects were detected
    for(size_t i = 0; i < cmd->labels.size(); i++)
      if(last_frame_ok[indexOf(cmd->labels[i])])
        detectedObjects.insert(cmd->labels[i]);
    if(doDisplay)
    {   
      drawResults(grayImage);
      cvShowImage("ObjectDetectorFERNS", grayImage);

      // needed to make the window appear
      // (an odd behaviour of OpenCV windows!)
      cvWaitKey(10);
    }
    cvReleaseImage(&grayImage);
  }
  // a bit HACKy: say that we detected an object with a given label
  // in the last frame, when actually we only know that we detected it at
  // least in one of the last numDetectionAttempts frames
  // This is required because postObjectsToWM() looks at last_frame_ok when
  // setting detection confidence.
  for(std::set<string>::iterator it = detectedObjects.begin();
      it != detectedObjects.end(); it++)
  {
    last_frame_ok[indexOf(*it)] = true;
  }
  postObjectsToWM(cmd->labels, image);

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
  navWMIds.resize(model_images.size());

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

    if (mode==DETECT_AND_TRACK) {
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
    }
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
      
            
      char filename[1024];
      ImgNeed2BeAddintoWM.clear();
      ImgNeed2BeAddintoWM.push_back(imgcnt);
      snprintf(filename, 1024, "img%03d.jpg", imgcnt++);
      cvSaveImage(filename, frame, 0);
      
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
  for(size_t i = 0; i < model_labels.size(); i++)
    postObjectToWM_Internal(i, image);
}

namespace Unlikely_Reused{
  std::set<std::string> previous_postings;
}

void ObjectDetectorFERNS::postObjectToWM_Internal(size_t i,
    const Video::Image &image)
{
  VisualObjectPtr obj = createVisualObject(i, image);

  if(obj->detectionConfidence < 0.1) return;

  if( Unlikely_Reused::previous_postings.find(obj->label)
      != Unlikely_Reused::previous_postings.end()) return;

  Unlikely_Reused::previous_postings.insert(obj->label);

  // if no WM ID yet for that object
  if(objWMIds[i] == "")
  {
    objWMIds[i] = newDataID();
    addToWorkingMemory(objWMIds[i], obj);
    
    string WMIdCurrImg= "WM Id: " + objWMIds[i] +" @vision.sa";
    for (unsigned int i = 0; i<ImgNeed2BeAddintoWM.size(); i++)
    {
	char filename[1024];
	snprintf(filename, 1024, "img%03d.jpg", ImgNeed2BeAddintoWM.at(i));
	IplImage* tmpimg = cvLoadImage(filename);
	double x, y;
	x = tmpimg->width/4;
	y = tmpimg->height-5;
	cvPutText(tmpimg, WMIdCurrImg.c_str(), cvPoint(x, y), &font, cvScalar(255));
	cvSaveImage(filename, tmpimg, 0);
	cvReleaseImage(&tmpimg);
    }
  }
  else
  {
    overwriteWorkingMemory(objWMIds[i], obj);
  }

  if(outputToNav)
  {
    // now put an ObjObs into the nav.sa in order to display our object in the
    // robot visualisation
    NavData::ObjObsPtr obs = createObjObs(obj, image);

    // if no nav.sa WM ID yet for that object
    if(navWMIds[i] == "")
    {
      navWMIds[i] = newDataID();
      addToWorkingMemory(navWMIds[i], "nav.sa", obs);
    }
    else
    {
      overwriteWorkingMemory(navWMIds[i], "nav.sa", obs);
    }
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

  obj->label = model_labels[i];
  obj->time = image.time;
  if(last_frame_ok[i])
    obj->detectionConfidence = 1.;
  else
    obj->detectionConfidence = 0.;

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

NavData::ObjObsPtr ObjectDetectorFERNS::createObjObs(VisualObjectPtr obj,
    const Video::Image &image)
{
  NavData::ObjObsPtr obs = new NavData::ObjObs;

  obs->category = obj->label;
  obs->time = obj->time;

  // we don't know distance (no 3D information at this point) but view vector
  // (if the camera is calibrated)
  assert(obj->views.size() > 0);
  Vector3 ray = localRay(image.camPars, obj->views[0].boundingBox.pos);

  // assume that camera is mounted so that image x coordinate is horizontal and
  // points to the right
  obs->angles.resize(1);
  obs->angles[0] = -atan2(ray.x, ray.z);

  return obs;
}

}

