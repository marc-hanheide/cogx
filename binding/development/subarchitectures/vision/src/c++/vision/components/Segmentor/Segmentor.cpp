
#include <iostream>
#include <sstream>

#include <opencv/highgui.h>

#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "Segmentor.h"


extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Segmentor(_id);
  }
}


Segmentor::Segmentor(const string &_id) :
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id),
  SceneObjectWriter(dynamic_cast<ManagedProcess&>(*this)) {

  // defaults from svn
  m_shadowThreshold = 0.5;
  m_overlapThreshold = 0.5;
  m_minContourArea = 1000;
  m_trainImgCount = 30;
  m_normImgCount = 30;
  m_camera = 0;

  ptrBackModel = new _back_model();
}


Segmentor::~Segmentor() {
  delete ptrBackModel;
}


/**
 * Options:
 * -r .. ROI, e.g. "150 240 620 480"
 * -a .. minimum contour area, e.g. 1000
 * -s .. shadow sensitivity, e.g. 0.5
 * -o .. overlap threshold, e.g. 0.5
 * -t .. number of training backround images, e.g. 30
 * -n .. number of normalization backround images, e.g. 30
 * -c .. inoput camera to use, default 0
 * -w .. show segmentation window
 * -nd .. segmentor does not delete rois
 */
void Segmentor::configure(map<string,string> & _config) {
  VideoClientProcess::configure(_config);

  ostringstream outStream;

  if(_config["-r"] != "") {
    string in_config = _config["-r"];
    istringstream myStream(in_config);

    myStream >> (m_box.tLeft_x) >>  (m_box.tLeft_y)
	     >> (m_box.bRight_x) >>  (m_box.bRight_y);

    m_bBox_is_set = true;

    outStream.str("");
    outStream<<"setting ROI: "<<in_config;
  }
  else {
    m_bBox_is_set = false;
  }

  if(_config["-s"] != "") {
    istringstream configStream(_config["-s"]);
    configStream >> m_shadowThreshold;

    outStream.str("");
    outStream<<"setting shadow sensitivity: "<<m_shadowThreshold;
    log(outStream.str());
  }

  if(_config["-a"] != "") {
    istringstream configStream(_config["-a"]);
    configStream >> m_minContourArea;

    outStream.str("");
    outStream<<"setting minimum contour area: "<<m_minContourArea;
    log(outStream.str());
  }

  if(_config["-o"] != "") {
    istringstream configStream(_config["-o"]);
    configStream >> m_overlapThreshold;

    outStream.str("");
    outStream<<"setting overlap threshold: "<<m_overlapThreshold;
    log(outStream.str());
  }

  if(_config["-t"] != "") {
    istringstream configStream(_config["-t"]);
    configStream >> m_trainImgCount;

    outStream.str("");
    outStream<<"setting the number of training background images: "<<m_trainImgCount;
    log(outStream.str());
  }

  if(_config["-n"] != "") {
    istringstream configStream(_config["-n"]);
    configStream >> m_normImgCount;

    outStream.str("");
    outStream<<"setting the number of normalization background images: "<<m_normImgCount;
    log(outStream.str());
  }

  if(_config["-c"] != "") {
    istringstream configStream(_config["-c"]);
    configStream >> m_camera;

    outStream.str("");
    outStream<<"setting camera: "<<m_camera;
    log(outStream.str());
  }

  if(_config["-w"] != "") {
    m_segWindow = true;
  }
  else
    m_segWindow = false;

  if(_config["-nd"] != "") {
    m_noROIDelete = true;
  }
  else
    m_noROIDelete= false;

}



bool Segmentor::is_interesting(int x, int y) {

  if (m_bBox_is_set == false) // every pixel is interesting
    return true;
  else {
    if (x>m_box.tLeft_x && x<m_box.bRight_x &&
	y>m_box.tLeft_y && y<m_box.bRight_y)
      return true;
    else
      return false;
  }
}



void Segmentor::start() {
  ManagedProcess::start();

  addChangeFilter(createLocalTypeFilter<SceneChanged>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<Segmentor>(this,
							      &Segmentor::HandleSceneChanged));
}



void Segmentor::HandleSceneChanged(const cdl::WorkingMemoryChange &change) {

  shared_ptr<const CASTData<SceneChanged> > pSceneChangedData
    = getWorkingMemoryEntry<SceneChanged>(change.m_address);
  shared_ptr<const SceneChanged> sceneChanged = pSceneChangedData->getData();

  //added to work with HandPoineter component
  vector<shared_ptr<const CASTData<HandPointingResults> > > hpr;

  getWorkingMemoryEntries<HandPointingResults>(0, hpr);

  vector<shared_ptr<const CASTData<HandPointingResults> > >::const_iterator zac = hpr.begin();

  bool handInScene = false;
  bool hpStructureInWM = false;

  if(hpr.size() > 0) {
    handInScene = (*zac)->getData().get()->m_handInScene;
    hpStructureInWM = true;
  } 

  // Get the ID for the next task.
  string taskID = newTaskID();

  if(sceneChanged->m_sceneChanged && !sceneChanged->m_sceneProcessed
     && (!handInScene || !hpStructureInWM)) { //added to work with Handpointer component
    // propose a task to detect ROIs.
    m_tasks[taskID] = VisionGoals::SEGMENT_ROI_TASK;

    proposeInformationProcessingTask(taskID, VisionGoals::SEGMENT_ROI_TASK);
    
    // remember the SceneChanged memory ID
    m_sceneStatusID = string(change.m_address.m_id);
  }
}



void Segmentor::taskAdopted(const string &_taskID) {
  // First, see what the task ID is about.
  std::map<string, string>::iterator tit =
    m_tasks.find(_taskID);

  if (tit != m_tasks.end()) {
    // Get the task name and remove it from the map.
    string task = m_tasks[_taskID];
    m_tasks.erase(tit);

    // Now, decide what to do with this task.
    if (task == VisionGoals::SEGMENT_ROI_TASK) {
      AdoptSegmentROI(_taskID);
    } // if - else
  } // if

  // and now we're finished, tell the goal manager that the task is
  // over successfully (assuming it is... naughty!)
  taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);
}



void Segmentor::AdoptSegmentROI(const string &_taskID) {
  IplImage* frameImage = getIplImage(m_camera);
  pushROIsToWorkingMemory(frameImage);
  log( "SGMT: done adding ROIs -- releasing image and leaving..." );
  cvReleaseImage(&frameImage);
  log( "SGMT: done releasing image -- exiting now..." );
}



/**
 * Get current camera image and convert to IplImage, our internal format.
 * Note the caller is responsible to delete the returned IplImage.
 */
IplImage* Segmentor::getIplImage(int camNum) {
  Vision::ImageFrame Image;
  getImage(camNum, Image);
  IplImage* image = buffer2image(&Image);
  return image;
}



/**
 * Get an sequence of background images from the camera to learn the
 * background pixels' distributions.
 */
void Segmentor::learnBackground() {
  IplImage* image;

  println("Learning background...");

  for( size_t i = 0; i < m_trainImgCount; i++ ){
      image = getIplImage(m_camera);

      unsigned char* inputImg = (unsigned char*)image->imageData;

      ptrBackModel->insertImage(inputImg, image->width, image->height, 0);

      cvReleaseImage(&image);
      sleepProcess(50);
  }

  for( size_t i = 0; i < m_normImgCount; i++ ){
      image = getIplImage(m_camera);

      unsigned char* inputImg = (unsigned char*)image->imageData;

      ptrBackModel->addImageToNorms(inputImg);

      cvReleaseImage(&image);
      sleepProcess(50);
  }

  int calculate_thresholds = 1;
  ptrBackModel->generateConstants( calculate_thresholds );

  println("Background learned.");

  ptrBackModel->shadowIntensitySensitivity(m_shadowThreshold);

  println("Shadow sensitivity set.");

}


void Segmentor::pushROIsToWorkingMemory(IplImage* _image) {
  // First, get the regions of interest.
  std::vector<Vision::ROI*> regions;
  getSalientRegions(_image, regions);
  // Get the list of current ROIs in the working memory.
  std::vector<shared_ptr<const CASTData<Vision::ROI> > > roisInWM;
  getWorkingMemoryEntries(roisInWM);
  // IJC: These ROIs in memory need to be locked? Iterate & lock.
  for( std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
	     wit = roisInWM.begin(), wit_e = roisInWM.end();
	   wit != wit_e; wit++) {
     lockEntry(CORBA::string_dup( (*wit)->getID().c_str()), cast::cdl::LOCKED_ODR);

  };

  // We have to check whether all ROI in WM are also present in the scene now.
  // If not, we must remove them from WM. To this end, initialise an array of
  // boolean values.
  std::vector<bool> keepInWM(false);
  keepInWM.resize(roisInWM.size());
  std::vector<bool> addToWM(false);
  addToWM.resize(regions.size());

  println("Number of input ROIs: %d", regions.size());

  // nah: changing the order of actions to minimise the contents of
  // visual working memory and hence the load on other systems
  // now we check for overlap, then remove, then add... rather than check, add,
  // remove

  // pointer to hold roi being added
  Vision::ROI *roi = NULL;

  std::vector<bool>::iterator add = addToWM.begin();

  for( std::vector<Vision::ROI*>::const_iterator
	 rit = regions.begin(), rit_e = regions.end();
       rit != rit_e; rit++, add++ ) {
    bool matchFound = false;
    std::vector<bool>::iterator fit = keepInWM.begin();

    log("Segmented obj data: x= %f, y = %f, xsize= %f, ysize=%f", 
            (*rit)->m_bbox.m_center.m_x, (*rit)->m_bbox.m_center.m_y,
            (*rit)->m_bbox.m_size.m_x, (*rit)->m_bbox.m_size.m_y);

    for( std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
	   wit = roisInWM.begin(), wit_e = roisInWM.end();
	 wit != wit_e; wit++, fit++ ) {
      float O = overlap((*rit), (*wit)->getData().get());
      
      // If overlap is large enough, mark ROI as already in the WM and leave it
      // there.
      if( O > m_overlapThreshold ) {
        (*fit) = true;
	matchFound = true;
        break;
      } // if
    } // for

    // Add ROI to WM only if there was no match.
    if( !matchFound ) {
        (*add) = true;
    } // if
  } // for

  // Now remove ROIs from WM if they are no longer present in the scene.
  std::vector<bool>::iterator fit = keepInWM.begin();
  for( std::vector<shared_ptr<const CASTData<Vision::ROI> > >::const_iterator
	 wit = roisInWM.begin(), wit_e = roisInWM.end();
       wit != wit_e; wit++, fit++ ) {
   
      // Somboon: replace m_noROIDelete with okToDel
      //          Now, -nd is no longer use.
      ////////////////////////////////////////
      string soMemID = CORBA::string_dup((*wit)->getData()->m_objId);
      if (soMemID.length() <= 0)
	  throw BALTException(__HERE__, "No matched SceneObject found");
      shared_ptr<const CASTTypedData<Vision::SceneObject> > spCastSceneObj =
	  getWorkingMemoryEntry<Vision::SceneObject>(soMemID);
      string manager 
	  = CORBA::string_dup(spCastSceneObj->getData()->m_managingProcess);
      bool okToDel = (manager=="") || (manager==getProcessIdentifier());
      ////////////////////////////////////////


    if( !(*fit) && okToDel) {
      println("Trying to removing WM ROI entry");
      bool deleted=false;
      while (!deleted){
        try{
            deleteFromWorkingMemory((*wit)->getID());
            deleted=true;
            //also check for a scene objects
            //
            //get the id of the scene object
            string soID = CORBA::string_dup((*wit)->getData()->m_objId);
            if(soID.length() > 0) {
                println("Removing WM SceneObject entry");
                deleteFromWorkingMemory(soID);
            }
        }//try
        catch (PermissionException){
            println("Tried to remove ROI, but locked. Sleeping and trying again");
            sleep(50);
        };
      }// while
    } // if
    // IJC: If the entry HASN'T been deleted, then unlock it.
    else{
      unlockEntry(CORBA::string_dup( (*wit)->getID().c_str()));
    }//else
  } // for

  // Now add ROIs to WM if they have not been matched
  add = addToWM.begin();

  int numAdded = 0;

  for( std::vector<Vision::ROI*>::const_iterator
	 rit = regions.begin(), rit_e = regions.end();
       rit != rit_e; rit++, add++ ) {
    
    if( (*add) ) {
      roi = (*rit);
      
      //----------------------------------------------
      // generate sceneObject for the detected roi
      //----------------------------------------------
      Camera cam;
      GetCamera(cam, m_camera);

      //create new so
      newSceneObject();

      //update position
      ProjectRoiToGroundplane(GROUND_HEIGHT, cam, *roi, currentObject());
      objectPositionUpdated();

      //       cout << "SGMT: sceneObj: pose: "
// 	   <<  pSceneObj->m_pose.m_position.m_x << "," 
// 	   <<  pSceneObj->m_pose.m_position.m_y << ","
// 	   <<  pSceneObj->m_pose.m_position.m_z << ")" << endl;

//       cout << "SGMT: sceneObj: bbox: " 
// 	   <<  pSceneObj->m_bbox.m_centroid.m_x << "," 
// 	   <<  pSceneObj->m_bbox.m_centroid.m_y << ","
// 	   <<  pSceneObj->m_bbox.m_centroid.m_z << ")" << endl;



      //sire roi id
      string strROIId = newDataID();
      addObjectROI(strROIId);

      //write object to wm and keep the id
      string strSceneObjId(writeObject());

      roi->m_address =  CORBA::string_dup( strROIId.c_str());
      roi->m_objId = CORBA::string_dup( strSceneObjId.c_str());

      numAdded += 1;
      println("Adding WM ROI entry ID: " + strROIId);
      addToWorkingMemory<ROI>( strROIId, roi  ,cdl::BLOCKING);
    } // if
  } // for

 
  println("Number of newly added ROIs: %i", numAdded); 
/*
  // Signal that the scene was processed
  Vision::SceneChanged* sc = new Vision::SceneChanged();

  sc->m_sceneChanging = false;
  sc->m_sceneChanged = false;
  sc->m_sceneProcessed = true;
  sc->m_camNum = m_camera;

  if(m_sceneStatusID != "") {
      this->overwriteWorkingMemory<SceneChanged>(m_sceneStatusID, sc);
    log("SGNT: The scene has been processed.");
  }
  else {
    log("SGMT: Cannot get sceneStatusID.");
  }

  //TODO nah: Memory leak? what happens to ROIs not added to working
  //memory???
*/
} // Segmentor::pushROIsToWorkingMemory




/**
 * Function determines the salient regions in the input image...
 */
void Segmentor::getSalientRegions(IplImage* _image,
				  std::vector<Vision::ROI*>& _regions) {
  // Prepare mask image.
  IplImage* segmentedImage = cvCreateImage(cvSize(_image->width,
     _image->height), IPL_DEPTH_8U, 1);
  cvSetZero(segmentedImage);

  // Get the size of the image and set the pointers to start the background
  // estimation phase.
  unsigned size = _image->width * _image->height;
  unsigned char* imgData = (unsigned char*) _image->imageData;
  unsigned char* segmentedImageData = (unsigned char*) segmentedImage->imageData;

  // get the segmentation mask
  ptrBackModel->getMaskFunction(imgData, segmentedImageData);

  // which segmented areas are we interested in?
  for (unsigned i = 0; i < size; i++, segmentedImageData++) {
    int coord_x = i%_image->width;
    int coord_y = i/_image->width;

    if (!is_interesting(coord_x, coord_y))
        *segmentedImageData = 0;
  }

  // Now use the dilation and erosion to fill empty spaces.
  IplImage* smoothedImage = cvCreateImage(
      cvSize(segmentedImage->width, segmentedImage->height),
      IPL_DEPTH_8U, 1);

  cvDilate(segmentedImage, smoothedImage, NULL, 3);
  cvErode(smoothedImage, smoothedImage, NULL, 2);

  IplImage* segmentation = cvCreateImage(
      cvSize(smoothedImage->width, smoothedImage->height), 8, 3);
  cvCopyImage(_image, segmentation);


  // Prepare storage for contours.
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* contour;

  // Find connected regions. The result of this is a list of contours that
  // represent the segmented shapes.
  cvFindContours( smoothedImage, storage, &contour,
		  sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

  // Now, focus on large contours and add them to the list of salient regions.
  for(; contour != 0; contour = contour->h_next ) {
    double area = fabs(cvContourArea(contour));
    if( area > m_minContourArea ) {
      // Get the bounding box of the region.
      CvRect bbox = cvBoundingRect(contour, 1);

      // Prepare the ROI object.
      Vision::ROI* roi = new Vision::ROI();

      // Set the bounding box.
      roi->m_bbox.m_center.m_x = bbox.x + bbox.width / 2;
      roi->m_bbox.m_center.m_y = bbox.y + bbox.height / 2;
      roi->m_bbox.m_size.m_x = bbox.width;
      roi->m_bbox.m_size.m_y = bbox.height;
      roi->m_camNum = m_camera;
      roi->m_time = BALTTimer::getBALTTime();

      // Time to analyze the contours and store the points for later
      // analysis...
//       cout << "SGMT: properties of contour: (" << area << ", "
// 	   << contour->flags << ", " << contour->header_size << ", " 
// 	   << contour->total << ", " << contour->elem_size << ", " 
// 	   << contour->delta_elems << ")\n";
      
      roi->m_contourPoints.length( contour->total );
      for( int c = 0; c < contour->total; ++c ) {
	CvPoint *point = (CvPoint*)cvGetSeqElem( contour, c );
	roi->m_contourPoints[c].m_x = point->x;
	roi->m_contourPoints[c].m_y = point->y;
      }


      // Get the ROI region.
      cvSetImageROI(_image, bbox);
      IplImage* regionImg = cvCreateImage(
          cvSize(bbox.width, bbox.height), 8, 3);
      cvCopy(_image, regionImg);
      image2buffer(regionImg, roi->m_region);

      // Prepare the mask image.
      IplImage* maskImg = cvCreateImage(cvSize(bbox.width, bbox.height),
          IPL_DEPTH_8U, 1);
      cvSetZero(maskImg);
      CvScalar C = cvScalar(255, 0, 0, 0);
      cvDrawContours(maskImg, contour, C, C, -1, CV_FILLED, 8,
          cvPoint(-bbox.x, -bbox.y));

      image2buffer(maskImg, roi->m_mask);

      // Init feature vector to empty
      roi->m_features.dimensions_.length(0);
      roi->m_features.data_.length(0);

      // TODO What to do with the remainig attributes of ROI struct.
      

      // Push the region.
      _regions.push_back(roi);

      //----------------------------------------------------------------------
      // This is used only for displaying results.
      //----------------------------------------------------------------------
      if (m_segWindow)
      {
        CvScalar color = CV_RGB(rand()&255, rand()&255, rand()&255);
        /* replace CV_FILLED with 1 to see the outlines */
        cvDrawContours(segmentation, contour, color, color, -1, 1, 8 ); //CV_FILLED
        cvDrawContours(_image, contour, color, color, -1, 1, 8 );

        // Draw a rectangle.
        CvPoint topleft = cvPoint(bbox.x, bbox.y);
        CvPoint bottomright = cvPoint(bbox.x + bbox.width, bbox.y + bbox.height);
        cvRectangle(segmentation, topleft, bottomright, color);
        cvRectangle(_image, topleft, bottomright, color);
      }

      cvReleaseImage(&regionImg);
      cvReleaseImage(&maskImg);
    } // if
  } // for
  
  if (m_segWindow)
  {
    cvNamedWindow("segmentation", 1);
    cvShowImage("segmentation", segmentation);
    cvWaitKey(100);
  }
  else {
    println("Segmentation window disabled");
  }

  // Clear up the allocated memory...
  cvReleaseImage(&segmentation);
  cvReleaseImage(&segmentedImage);
  cvReleaseMemStorage( &storage );

} // Segmentor::getSalientRegions

float Segmentor::overlap( const Vision::ROI* _roi1,
			  const Vision::ROI* _roi2 ) const
{
  log("ROI Matching: x1= %f, y1 = %f, x2= %f, y2 = %f", 
            _roi1->m_bbox.m_center.m_x, _roi1->m_bbox.m_center.m_y,
            _roi2->m_bbox.m_center.m_x, _roi2->m_bbox.m_center.m_y);

  log("ROI Matching: x1size= %f, y1size = %f, x2size= %f, y2size = %f", 
            _roi1->m_bbox.m_size.m_x, _roi1->m_bbox.m_size.m_y,
            _roi2->m_bbox.m_size.m_x, _roi2->m_bbox.m_size.m_y);


  float centerMatch =
    min(1 - fabs(_roi1->m_bbox.m_center.m_x - _roi2->m_bbox.m_center.m_x)/_roi1->m_bbox.m_size.m_x/2,
        1 - fabs(_roi1->m_bbox.m_center.m_y - _roi2->m_bbox.m_center.m_y)/_roi1->m_bbox.m_size.m_y/2);

  
  log("ROI Matching: %f percent match", centerMatch);

  return centerMatch;
} // Segmentor::overlap

/*
float Segmentor::overlap( const Vision::ROI* _roi1,
			  const Vision::ROI* _roi2 ) const
{
  println("x1= %f, y1 = %f, x2= %f, y2 = %f", 
            _roi1->m_bbox.m_center.m_x, _roi1->m_bbox.m_center.m_y,
            _roi2->m_bbox.m_center.m_x, _roi2->m_bbox.m_center.m_y);

 println("x1size= %f, y1size = %f, x2size= %f, y2size = %f", 
            _roi1->m_bbox.m_size.m_x, _roi1->m_bbox.m_size.m_y,
            _roi2->m_bbox.m_size.m_x, _roi2->m_bbox.m_size.m_y);

  float intersectionWidth =
    min(_roi1->m_bbox.m_center.m_x + _roi1->m_bbox.m_size.m_x/2,
        _roi2->m_bbox.m_center.m_x + _roi2->m_bbox.m_size.m_x/2) -
    max(_roi1->m_bbox.m_center.m_x - _roi1->m_bbox.m_size.m_x/2,
        _roi2->m_bbox.m_center.m_x - _roi2->m_bbox.m_size.m_x/2);

  float intersectionHeight =
    min(_roi1->m_bbox.m_center.m_y + _roi1->m_bbox.m_size.m_y/2,
        _roi2->m_bbox.m_center.m_y + _roi2->m_bbox.m_size.m_y/2) -
    max(_roi1->m_bbox.m_center.m_y - _roi1->m_bbox.m_size.m_y/2,
        _roi2->m_bbox.m_center.m_y - _roi2->m_bbox.m_size.m_y/2);

  float intersectionArea = intersectionWidth * intersectionHeight;
  float unionArea =
    _roi1->m_bbox.m_size.m_x * _roi1->m_bbox.m_size.m_y +
    _roi2->m_bbox.m_size.m_x * _roi2->m_bbox.m_size.m_y -
    intersectionArea;

  println("intersection= %f, union = %f, ratio = %f", 
            intersectionArea, unionArea,
            intersectionArea / unionArea);

  return intersectionArea / unionArea;
} // Segmentor::overlap
*/


void Segmentor::GetCamera(Camera &cam, int camId) {
  vector<shared_ptr<const CASTData<Camera> > > cams;
  getWorkingMemoryEntries(cams);

  for(unsigned i = 0; i < cams.size(); i++)
  {
    shared_ptr<const Camera> pcam = cams[i]->getData();
    if(pcam->m_num == camId) {
      cam = *pcam;
      return;
    }
  }
  println("failed to get camera #%d", camId);
}



void Segmentor::saveImage(IplImage* _image, const char* _prefix /* = "test" */,
			  bool _showImage /* = false */) {
  static std::map<std::string, unsigned>  imgCountMap;
  imgCountMap[_prefix]++;

  char filename[256];
  snprintf(filename, 256, "%s_%04d.jpg", _prefix, imgCountMap[_prefix]);

  cvSaveImage(filename, _image);

  if (_showImage)
  {
    cvvNamedWindow(_prefix, 1);
    cvvShowImage(_prefix, _image);
    cvWaitKey(1);
  } // if
}



void Segmentor::taskRejected(const string &_taskID)
{
  // so what?
}


void Segmentor::runComponent()
{
  //this is ugly, but we don't know when the camera has frames!!!
  sleepProcess(1000);
  learnBackground();
}

