/** @file ObjectTracker.cpp
 *  @brief An 3D object tracker.
 *
 *   Tracks objects in 3D. 
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */

#include <vision/VisionGoals.h>
#include <vision/utils/VisionUtils.h>
#include "ObjectTracker.h"
#include "BTComponents/BlobTracker.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <vision/components/common/SystemUtils/Common.h>

#define ACTIVELY_MANAGE_ROI

using namespace Common;
using namespace Vision;

extern "C" {
    FrameworkProcess* newComponent(const string &_id) {
	return new ObjectTracker(_id);
    }
}

ObjectTracker::ObjectTracker(const string &_id) : 
    WorkingMemoryAttachedComponent(_id),
    VisionProxy(_id),
    TrackerAbstract(),
    SceneObjectWriter(dynamic_cast<ManagedProcess&>(*this)),
    m_tracker(0), 
    m_attendedListMemoryID(""), 
    m_stereoMode(false),
    m_framerate_ms(50)
{ 
} // ObjectTracker::ObjectTracker


ObjectTracker::~ObjectTracker() 
{
    m_cameraDataToProcess.clear();
    m_roiDataToProcess.clear();
    m_tasks.clear();

    if (m_outfile.is_open())
	m_outfile.close();
    if (m_tracker)
	delete m_tracker;
} // ObjectTracker::~ObjectTracker


/**
 * Options:
 * -c [filename] .. tracker configuration file for a blob tracker.
 * -s [0,1] .. option for stereo mode (currently not supported).
 * -t [num] .. time interval between two frames in ms.
 * -out [filename] .. ascii file to output tracking results
 * -v [0,1] .. an option for displaying a tracking window
 * -d [0,1,2,3] .. an option for saving images. 
 *               0 (or any number other than 1,2,3): don't save (by default).
 *               1: save input images.
 *               2: save images with tracking results overlaid.
 *               3: save both the input images and the tracking results. 
 * --videoport [num] .. the port number to connect to a VideoServer.
 * --videohost [hostname] .. the host computer that runs a VideoServer.
 * -r [0,1] .. an option for saving Rois, default to 0(false).
 * --minROIarea [value] This will set the minimum size of a roi to accept. 
 *            Proportional to screen resolution. Defaults to 0.0025, 1/4 of a standard block.
 * --maxROIarea [value] This will set the minimum size of a roi to accept. 
 *            Defaults to 0.25, roughly 1/4 of the entire view.
 * 
 */
void ObjectTracker::configure(map<string,string> & _config) 
{
    ManagedProcess::configure(_config);
    VisionProxy::configure(_config);


    //nah: changed to allow no database
    //if (_config["-c"] != "") {
    if (_config["-s"] == "1") {
      m_stereoMode=true;
      throw user_error(__HERE__, "Stereo option is not supported");
    }
    else {
      m_stereoMode=false;
    }

    m_tracker = new BlobTracker(_config["-c"], m_stereoMode);
	//}

    if (_config["-t"] != "")
	m_framerate_ms = strtol(_config["-t"].c_str(), NULL, 10);

    if (_config["-out"] != "") 
	m_outfile.open(_config["-out"].c_str());

    if (m_tracker == 0)
	throw user_error(__HERE__, "no tracker was specified");

    if (_config["-v"] == "1") {
	m_tracker->setShow(true);
	m_tracker->setBaseWinName(string("track"));
	m_tracker->setViewId(0);
    }
    else
	m_tracker->setShow(false);
    
    if (_config["-r"] == "1")
	SetSaveRoiOption(true);
    else 
	SetSaveRoiOption(false);

    if (_config["-d"] == "1") 
	m_tracker->setSaveOrig(true);
    else if (_config["-d"] == "2") 
	m_tracker->setSaveResults(true);
    else if (_config["-d"] == "3") {
	m_tracker->setSaveOrig(true);
	m_tracker->setSaveResults(true);
    }
    else { // any numbers other than 1,2,3 will turn off the saving option.
	m_tracker->setSaveOrig(false);
	m_tracker->setSaveResults(false);
    }
    
    //Added to do minimum and maximum ROI sizes
    
    if (_config["--minROIarea"]!="") {
        m_tracker->min_area=atof(_config["--minROIarea"].c_str());
    }

    if (_config["--maxROIarea"]!="") {
        m_tracker->max_area=atof(_config["--maxROIarea"].c_str());
    }

}


void ObjectTracker::taskAdopted(const string &_taskID) 
{
    std::map<string, string>::iterator tit = m_tasks.find(_taskID);
  
    if (tit != m_tasks.end()) {
	string task = m_tasks[_taskID];
	m_tasks.erase(tit);

	if (task == VisionGoals::UPDATE_CAMERA_PARAMETERS_TASK) 
	{
	    CameraDataMap::iterator cit = m_cameraDataToProcess.find(_taskID);
	    if (cit != m_cameraDataToProcess.end()) 
	    {
		shared_ptr<const Vision::Camera> spCam 
		    = m_cameraDataToProcess[_taskID];
		UpdateCameraProjection(spCam);
		m_cameraDataToProcess.erase(cit);
	    }
	}
	else if (task == VisionGoals::INITIALIZE_ROI_TRACKING_TASK)
	{
	    RoiDataMap::iterator rit = m_roiDataToProcess.find(_taskID);
	    if (rit != m_roiDataToProcess.end()) 
	    {
		shared_ptr<const Vision::ROI> spROI 
		    = m_roiDataToProcess[_taskID];
		InitializeRoiTracking(spROI);
		m_roiDataToProcess.erase(rit);
	    }
	}
	else if (task == VisionGoals::TRACK_OBJECTS_TASK)
	{
	    Track();
	}
	else 
	    throw user_error(__HERE__, "%s is not my task\n", task.c_str());
	
	// tell the goal manager that the task is done successfully.
	taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);

    }
} 

void ObjectTracker::taskRejected(const string &_taskID) 
{
    std::map<string, string>::iterator tit = m_tasks.find(_taskID);

    if (tit != m_tasks.end()) {
	string task = m_tasks[_taskID];
	m_tasks.erase(tit);

	if (task == VisionGoals::UPDATE_CAMERA_PARAMETERS_TASK) 
	{
	    CameraDataMap::iterator cit = m_cameraDataToProcess.find(_taskID);
	    if (cit != m_cameraDataToProcess.end()) 
		m_cameraDataToProcess.erase(cit);
	}
	else if (task == VisionGoals::INITIALIZE_ROI_TRACKING_TASK)
	{
	    RoiDataMap::iterator rit = m_roiDataToProcess.find(_taskID);
	    if (rit != m_roiDataToProcess.end()) 
		m_roiDataToProcess.erase(rit);
	}

	user_printf(__HERE__, "Task, %s, is rejected\n", task.c_str());	
    }    
}


bool ObjectTracker::checkForTask(string taskName)
{
    std::map<string, string>::iterator tit;
    for (tit=m_tasks.begin(); tit!=m_tasks.end(); tit++) {
	if (tit->second == taskName)
	    return true;
    }
    return (false);
}

void ObjectTracker::HandleIdleEvent()
{
    if (checkForTask(VisionGoals::TRACK_OBJECTS_TASK)) 
	return;
    
    // Get the ID for the next task.
    string taskID = newTaskID();
    
    // Register the task
    m_tasks[taskID] = VisionGoals::TRACK_OBJECTS_TASK;
    
    // Propose the task to TaskManager
    proposeInformationProcessingTask(taskID, VisionGoals::TRACK_OBJECTS_TASK);
}

void ObjectTracker::HandleCameraEvent(const cdl::WorkingMemoryChange &_wmc)
{
    // Get the ID for the next task.
    string taskID = newTaskID();

    // Register Camera data for the proposed task
    shared_ptr<const CASTTypedData<Camera> > spCastCam =
	getWorkingMemoryEntry<Camera>(_wmc.m_address);    
    m_cameraDataToProcess[taskID] = spCastCam->getData();

    // Register the task
    m_tasks[taskID] = VisionGoals::UPDATE_CAMERA_PARAMETERS_TASK;
    
    // Propose the task to TaskManager
    proposeInformationProcessingTask(taskID, VisionGoals::UPDATE_CAMERA_PARAMETERS_TASK);

}


void ObjectTracker::HandleNewRoiEvent(const cdl::WorkingMemoryChange &_wmc)
{
    // Get the ID for the next task.
    string taskID = newTaskID();

    // Register ROI data for the proposed task
    shared_ptr<const CASTTypedData<Vision::ROI> > spCastROI =
	getWorkingMemoryEntry<ROI>(_wmc.m_address);    
    m_roiDataToProcess[taskID] = spCastROI->getData();

    // Register the task
    m_tasks[taskID] = VisionGoals::INITIALIZE_ROI_TRACKING_TASK;
    
    // Propose the task to TaskManager
    proposeInformationProcessingTask(taskID, VisionGoals::INITIALIZE_ROI_TRACKING_TASK);

}



void ObjectTracker::start()
{
  ManagedProcess::start();
  addChangeFilter(createLocalTypeFilter<Camera>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ObjectTracker>(this,
								  &ObjectTracker::HandleCameraEvent));
  addChangeFilter(createLocalTypeFilter<Camera>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ObjectTracker>(this,
								  &ObjectTracker::HandleCameraEvent));


  
  addChangeFilter(createLocalTypeFilter<ROI>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ObjectTracker>(this,
								  &ObjectTracker::HandleNewRoiEvent));
}


void ObjectTracker::AcquireCameraSetupInfo() 
{
    if (m_tracker->readyToVisualize())
	return;

    vector<Vision::Camera *> cameras;
    vector<Vision::ImageFrame *> imageFrames;
    GetCamerasAndImages(cameras, imageFrames);
    
    if ((cameras.size() != imageFrames.size()) || (cameras.size()==0)) 
    {
	for (unsigned i=0; i<cameras.size(); i++) 
	    delete(cameras[i]);
	for (unsigned i=0; i<imageFrames.size(); i++) 
	    delete(imageFrames[i]);
	throw user_error(__HERE__, "Camera and Video servers are not ready");
    }
    else {
	m_tracker->setProjection(cameras);
	for (unsigned i=0; i<imageFrames.size(); i++) {
	    IplImage *srcimg = buffer2image(imageFrames[i]);
	    m_tracker->CloneAndPushImages(srcimg);
	    cvReleaseImage(&srcimg);
	}
    }

    for (unsigned i=0; i<cameras.size(); i++) {
	delete(cameras[i]);
	delete(imageFrames[i]);
    }
}

void ObjectTracker::UpdateImageBuffer()
{
    // Clear the image buffer in Visualization 
    m_tracker->ClearImages(); 
    
    // Get the camera projection in Visualization
    vector<shared_ptr<CameraProj> > v_spProjs = m_tracker->GetProjections();

    if (v_spProjs.size() == 0) 
	throw user_error(__HERE__, "CameraProjection data is not ready");
    else {
	for (unsigned i=0; i<v_spProjs.size(); i++) 
	  {
	    IplImage *srcimg = GetIplImage(v_spProjs[i]->m_num);
	    m_tracker->CloneAndPushImages(srcimg);
	    cvReleaseImage(&srcimg);
	  }
    }
}


void ObjectTracker::runComponent() 
{
    sleepProcess(1000);
    AcquireCameraSetupInfo();

    AttendedObjectList* attendedObjList = new AttendedObjectList();
    m_attendedListMemoryID = newDataID();
    addToWorkingMemory<AttendedObjectList>(m_attendedListMemoryID, 
					   attendedObjList, cdl::BLOCKING);
    
    while(m_status == STATUS_RUN) 
    {
	HandleIdleEvent();
	sleepProcess(m_framerate_ms); 
    }
}


/**
 *  Update visualization's camera parameters.
 */ 
void ObjectTracker::UpdateCameraProjection(shared_ptr<const Camera> spCam) 
{
    m_tracker->UpdateProjection(spCam); 
}


void ObjectTracker::InitializeRoiTracking(shared_ptr<const ROI> spROI) 
{
  AcquireCameraSetupInfo();
    
    if (m_stereoMode)
	throw user_error(__HERE__, "Stereo tracking is not supported");
    else {
	string roi_objectWMA(spROI->m_objId);
	shared_ptr<const CASTTypedData<SceneObject> > spCastSceneObj =
	    getWorkingMemoryEntry<SceneObject>(roi_objectWMA);    
	shared_ptr<const SceneObject> spSceneObj = spCastSceneObj->getData();
	string label = CORBA::string_dup(spSceneObj->m_label.m_string);
	
	// don't propose a hand roi
	if (label != string("Vision:ObjectType:Hand"))
	    m_tracker->UpdateObjectAppearanceModelFromROI(spROI);
    }
}

void ObjectTracker::Track()
{
    UpdateImageBuffer();
    
    BALTTime scene_time = BALTTimer::getBALTTime();
    static int framenum = 0;
    m_tracker->saveOrig(framenum); // visualization's save
    
    m_tracker->track();

    m_tracker->visualize(framenum);
    m_tracker->saveResults(framenum);
    
    framenum++;

    // Log tracking results to a file
    ostringstream out;
    out << "[ObjectTracker::Track] total: " 
	<< m_tracker->NumObjects() 
	<< ", actives: " << m_tracker->NumActiveObjects();
    log(out.str());
    

    #ifdef TRACKER_DEBUG
    cout << "[ObjectTracker::Track] total: " 
         << m_tracker->NumObjects() 
    	   << ", actives: " << m_tracker->NumActiveObjects() << endl;
    #endif

    if (m_outfile.is_open())
	m_outfile << scene_time.m_s << "," << scene_time.m_us << ": ";

    // Update info in visual working memory 
    AttendedObjectList *attendedObjList = new AttendedObjectList();
    attendedObjList->m_memoryIDList.length(m_tracker->NumActiveObjects());
    int activeID = 0;
    for (int i=0; i<m_tracker->NumObjects(); i++) 
    {
	if (m_tracker->IsObjectActive(i)) 
	{
	    string strMemID;
	    updateObjectInfoInWorkingMemory(i, strMemID, scene_time);
	    attendedObjList->m_memoryIDList[activeID] = CORBA::string_dup(strMemID.c_str());
	    activeID++;
	}

        #ifdef ACTIVELY_MANAGE_ROI
	else if (m_tracker->IsObjectLost(i)) 
	{
	    string strMemID = m_tracker->getObjectMemoryID(i);
	    if (strMemID != "")
	    {
                try{
                    // Was not taking into account that the object may have been deleted
                    shared_ptr<const CASTTypedData<SceneObject> > spCastSceneObj =
                        getWorkingMemoryEntry<SceneObject>(strMemID);    
                    shared_ptr<const SceneObject> spSceneObj = spCastSceneObj->getData();
                    string manager=CORBA::string_dup(spSceneObj->m_managingProcess);
                    bool okToDel = (manager=="") || (manager==getProcessIdentifier());
                    if (okToDel)
                    {
                        // delete all rois associated with the object to delete
                        unsigned total_rois = spSceneObj->m_ROIsMemoryIDs.length(); 
                        for (unsigned id=0; id<total_rois; id++) {
                            string roi_address(spSceneObj->m_ROIsMemoryIDs[id]);
                            deleteFromWorkingMemory(roi_address);
                            user_printf(__HERE__, "Removed roi at %s\n",roi_address.c_str());			
                        }
                        
                        deleteFromWorkingMemory(strMemID);
                        user_printf(__HERE__, "Removed lost object %d\n", i);
                        //println("Removed WM SceneObject entry");
                    }
                }
                catch(cast::DoesNotExistOnWMException){
                    user_printf(__HERE__, "Object deleted while working\n");
                };
	    }
	    //nah: removed memory id reset to keep wm id later
	    //nah: put back in... maybe it's not a simple as that
	    m_tracker->putObjectMemoryID(i, "");
	}
	#endif
    }

    attendedObjList->m_time = scene_time;
    overwriteWorkingMemory<AttendedObjectList>(m_attendedListMemoryID, 
					       attendedObjList, cdl::BLOCKING);
}




void ObjectTracker::updateObjectInfoInWorkingMemory(int mobid, string &strMemID,
						    BALTTime scene_time) 
{

  strMemID = m_tracker->getObjectMemoryID(mobid);

  if(strMemID.empty()) {
    strMemID = newSceneObject();
    log("new scene object: %s", strMemID.c_str());
  }
  else {    
    log("existing scene object: %s", strMemID.c_str());
    try {
      loadSceneObject(strMemID);    
    }
    catch(const DoesNotExistOnWMException & e) {
      log("not recreating existing scene object: %s", strMemID.c_str());
      return;
    }
  }

  // updates m_bbox, m_pose, m_label, m_color of obj.  
  m_tracker->UpdateSceneObject(mobid, currentObject());
  currentObject().m_time = scene_time;
  objectPositionUpdated();

  // take over the right to delete SceneObject and its ROIs
  if (strcmp(currentObject().m_managingProcess,"") == 0) {
    currentObject().m_managingProcess = CORBA::string_dup(getProcessIdentifier().c_str());
  }

    
  if (!isNewObject()) {

    // Updates m_ROIsMemoryIDs of obj
    for (unsigned int i=0; i<currentObject().m_ROIsMemoryIDs.length(); i++) {
      
      // Get WM address of roi
      string roi_address(currentObject().m_ROIsMemoryIDs[i]);

      // Get WM address of roi & lock it
      lockEntry(roi_address, cast::cdl::LOCKED_ODR);
      
      // Copy ROI from WM 
      shared_ptr<const CASTTypedData<ROI> > spCastROI 
	= getWorkingMemoryEntry<ROI>(roi_address);        
      ROI *roi = new Vision::ROI(*spCastROI->getData());
      
      m_tracker->updateROI(roi->m_camNum , mobid, *roi);
      
      // SXH: Compute m_region, m_mask, m_time, m_contourPoints here, if needed. 
      roi->m_time = scene_time;
      
      overwriteWorkingMemory<ROI>( roi_address, roi, cdl::BLOCKING);

      unlockEntry(roi_address);
    }    
  }
  else {
    m_tracker->putObjectMemoryID(mobid, strMemID);
  
    unsigned rois_length = m_tracker->numViews();
    vector<ROI *> vNewROIs;
    m_tracker->updateROIs(mobid, vNewROIs);
    
    for (unsigned int i=0; i<rois_length; i++) {
      // Create a wm address for a new roi.
      string roi_address = newDataID(); 
      
      // SXH: Compute m_region, m_mask, m_time, m_contourPoints here, if needed. 
      vNewROIs[i]->m_objId =  CORBA::string_dup(strMemID.c_str());
      vNewROIs[i]->m_time = scene_time;
      vNewROIs[i]->m_address = CORBA::string_dup(roi_address.c_str());
      
      addToWorkingMemory<ROI>( roi_address, vNewROIs[i], cdl::BLOCKING);
      
      addObjectROI(roi_address.c_str());
    }
  
  }

  //write back scene object
  writeObject();
   
}
 


// void ObjectTracker::updateObjectInfoInWorkingMemory(int mobid, string &strMemID,
// 						    BALTTime scene_time) 
// {

//     SceneObject *obj = new Vision::SceneObject();

//     // updates m_bbox, m_pose, m_label, m_color of obj.
//     m_tracker->UpdateSceneObject(mobid, *obj);
//     obj->m_time = scene_time;
    
//     #ifdef TRACKER_DEBUG
//     cout << obj->m_label.m_string
// 	 << ", position: " << obj->m_pose.m_position.m_x 
// 	 << ", " << obj->m_pose.m_position.m_y
// 	 << ", " << obj->m_pose.m_position.m_z	<< endl;
//     #endif
    
//     if (m_outfile.is_open()) {
// 	m_outfile << obj->m_label.m_string
// 		  << ", position: " << obj->m_pose.m_position.m_x 
// 		  << ", " << obj->m_pose.m_position.m_y
// 		  << ", " << obj->m_pose.m_position.m_z	<< endl;
//     }	    
    
//     strMemID = m_tracker->getObjectMemoryID(mobid);
//     shared_ptr<const CASTTypedData<SceneObject> > spCastSceneObj;
//     //If there is an ID, try to get it, if that fails then assume ID is ""
//     if (strMemID!="")
//     {
//         try{
//             // obtains the original SceneObject from working memory
//             spCastSceneObj = getWorkingMemoryEntry<SceneObject>(strMemID);    
            
//         }
//         catch (cast::DoesNotExistOnWMException){
//             //should do some removal of deleted objects
//             cout<<"Object Tracker - Error: Segmentor has deleted object."<<endl;
//             strMemID="";
//         };
//     }
//     if (strMemID!="")
//     {
// 	#ifdef TRACKER_DEBUG
// 	user_printf(__HERE__, "Update previously found SceneObject at %s\n",
// 		    strMemID.c_str());
// 	#endif
//         shared_ptr<const SceneObject> spSceneObj = spCastSceneObj->getData();
        
//         //Ah, but this is also edited by the segmentor! Must lock the sceneobj and the rois.
//         lockEntry(CORBA::string_dup( strMemID.c_str()), cast::cdl::LOCKED_OD);
//         unsigned rois_length = spSceneObj->m_ROIsMemoryIDs.length();
//         for (unsigned i=0; i<rois_length; i++) 
//         {
//             // Get WM address of roi & lock it
//             lockEntry(CORBA::string_dup(spSceneObj->m_ROIsMemoryIDs[i]),cast::cdl::LOCKED_OD);
//         };	
// 	// copies m_surfaces, m_shape from the SceneObject in wm to obj.
// 	unsigned length = spSceneObj->m_surfaces.length();
// 	obj->m_surfaces.length(length);
// 	for (unsigned i=0; i<length; i++) {
// 	    obj->m_surfaces[i].m_id = spSceneObj->m_surfaces[i].m_id;
// 	    unsigned vertex_length = spSceneObj->m_surfaces[i].m_vertices.length();
// 	    obj->m_surfaces[i].m_vertices.length(vertex_length);
// 	    for (unsigned j=0; j<vertex_length; j++) {
// 		obj->m_surfaces[i].m_vertices[j].m_pos.m_x 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_pos.m_x;
// 		obj->m_surfaces[i].m_vertices[j].m_pos.m_y 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_pos.m_y;
// 		obj->m_surfaces[i].m_vertices[j].m_pos.m_z 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_pos.m_z;
// 		obj->m_surfaces[i].m_vertices[j].m_normal.m_x 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_normal.m_x;
// 		obj->m_surfaces[i].m_vertices[j].m_normal.m_y 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_normal.m_y;
// 		obj->m_surfaces[i].m_vertices[j].m_normal.m_z 
// 		    = spSceneObj->m_surfaces[i].m_vertices[j].m_normal.m_z;
// 	    }
// 	}
// 	obj->m_shape.m_int = spSceneObj->m_shape.m_int;
// 	obj->m_shape.m_confidence = spSceneObj->m_shape.m_confidence;

// 	// take over the right to delete SceneObject and its ROIs
// 	string managingProc = CORBA::string_dup(spSceneObj->m_managingProcess);
// 	if (managingProc == "")
// 	    obj->m_managingProcess = CORBA::string_dup(getProcessIdentifier().c_str());
// 	else
// 	    obj->m_managingProcess = spSceneObj->m_managingProcess;


// 	// Obtains the working memory addresses of rois from spSceneObj.
	
	
// 	// Updates m_ROIsMemoryIDs of obj
// 	obj->m_ROIsMemoryIDs.length(rois_length);
// 	for (unsigned i=0; i<rois_length; i++) 
// 	{
// 	    // Get WM address of roi
// 	    string roi_address = CORBA::string_dup(spSceneObj->m_ROIsMemoryIDs[i]);
// 	    obj->m_ROIsMemoryIDs[i] = CORBA::string_dup(roi_address.c_str());

// 	    // Create a ROI with new info 
// 	    shared_ptr<const CASTTypedData<ROI> > spCastROI 
// 		= getWorkingMemoryEntry<ROI>(roi_address);  
// 	    shared_ptr<const ROI> spROI = spCastROI->getData();
// 	    int viewID = spROI->m_camNum;
// 	    ROI *roi = new Vision::ROI();
// 	    m_tracker->updateROI(viewID , mobid, *roi);
	    
// 	    // SXH: Compute m_region, m_mask, m_time, m_contourPoints here, if needed. 
// 	    roi->m_objId =  CORBA::string_dup(strMemID.c_str());
// 	    roi->m_time = scene_time;
// 	    roi->m_address = CORBA::string_dup(roi_address.c_str());

// 	    #ifdef TRACKER_DEBUG
// 	    user_printf(__HERE__, "writing roi at %s\n", roi_address.c_str());
// 	    cout << ", roi: " << roi->m_bbox.m_center.m_x 
// 		 << ", " << roi->m_bbox.m_center.m_y
// 		 << ", " << roi->m_bbox.m_size.m_x
// 		 << ", " << roi->m_bbox.m_size.m_y << endl;
//             #endif 

// 	    overwriteWorkingMemory<ROI>( roi_address, roi, cdl::BLOCKING);
// 	}

// 	overwriteWorkingMemory<SceneObject>( strMemID, obj, cdl::BLOCKING);
//         //Now we can unlock it
//         unlockEntry(CORBA::string_dup(strMemID.c_str()));
//         for (unsigned i=0; i<rois_length; i++) 
//         {
//             // Get WM address of roi & unlock it
//             string roi_address = CORBA::string_dup(spSceneObj->m_ROIsMemoryIDs[i]);
//             unlockEntry(CORBA::string_dup(roi_address.c_str()));
//         };
//     }
// else
//     {	
// 	strMemID = newDataID();
// 	m_tracker->putObjectMemoryID(mobid, strMemID);
		    
// 	#ifdef TRACKER_DEBUG
// 	user_printf(__HERE__, "writing newly found SceneObject at %s\n",
// 		    strMemID.c_str());
// 	#endif

// 	// take over the right to delete roi.
// 	obj->m_managingProcess 
// 	    = CORBA::string_dup(getProcessIdentifier().c_str());

// 	unsigned rois_length = m_tracker->numViews();
// 	obj->m_ROIsMemoryIDs.length(rois_length);
	
// 	vector<ROI *> vNewROIs;
// 	m_tracker->updateROIs(mobid, vNewROIs);

// 	for (unsigned i=0; i<rois_length; i++) 
// 	{
// 	    // Create a wm address for a new roi.
// 	    string roi_address = newDataID(); 
    	    
// 	    // SXH: Compute m_region, m_mask, m_time, m_contourPoints here, if needed. 
// 	    vNewROIs[i]->m_objId =  CORBA::string_dup(strMemID.c_str());
// 	    vNewROIs[i]->m_time = scene_time;
// 	    vNewROIs[i]->m_address = CORBA::string_dup(roi_address.c_str());

// 	    #ifdef TRACKER_DEBUG
// 	    user_printf(__HERE__, "writing roi at %s\n", roi_address.c_str());
// 	    cout << ", roi: " << vNewROIs[i]->m_bbox.m_center.m_x 
// 		 << ", " << vNewROIs[i]->m_bbox.m_center.m_y
// 		 << ", " << vNewROIs[i]->m_bbox.m_size.m_x
// 		 << ", " << vNewROIs[i]->m_bbox.m_size.m_y << endl;
// 	    #endif 
    
// 	    addToWorkingMemory<ROI>( roi_address, vNewROIs[i], cdl::BLOCKING);

// 	    obj->m_ROIsMemoryIDs[i] =  CORBA::string_dup(roi_address.c_str());
// 	}

// 	addToWorkingMemory<SceneObject>(strMemID, obj, cdl::BLOCKING);

//     }

// }
 
