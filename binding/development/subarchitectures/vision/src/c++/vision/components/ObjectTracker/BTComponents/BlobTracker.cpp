/** @file BlobTracker.cpp
 *  @brief A blob tracker.
 *
 *  Tracks image blobs of a 3D objects using color distributions and shapes.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "BlobTracker.h"
#include <sstream>
#include <vision/components/common/SystemUtils/Common.h>
#include <vision/components/common/GeomUtils/GeomUtils.hpp>
#include "TrackAux.h"

using namespace std;
using namespace Common;
using namespace Vision;
using namespace Geom;

BlobTracker::BlobTracker(string cfgFilename, bool bStereo /*=false*/) :
    Tracker(), ObjectTypeManage()
{
    ObjectTypeManage::Initialize();
    min_area=0.0025;
    max_area=0.25;
    //ofile.open("blobtracker.log");
    Configure(cfgFilename);
    m_bStereo = bStereo; 
 } // BlobTracker::BlobTracker

 BlobTracker::~BlobTracker() 
 {
     //ofile.close();

     map<int,MobileObject*>::iterator obj_iter;
     for (obj_iter=mpMobileObjects.begin(); obj_iter!=mpMobileObjects.end(); obj_iter++)
     {
	 delete(obj_iter->second);
     }
     mpMobileObjects.clear();

     map<int,ViewTracker*>::iterator iter;
     for (iter=mpViewTrackers.begin(); iter!=mpViewTrackers.end(); iter++){
	 delete(iter->second);
     }
     mpViewTrackers.clear();

 } // BlobTracker::~BlobTracker


void BlobTracker::Configure(string cfgFilename) 
{
  if(!cfgFilename.empty()) {

    map<string,string> config;
    map<string,string>::iterator iter;
    
    ParseConfigFile(cfgFilename, config);
    if ((iter=config.find("OBJECT_DB")) != config.end()) 
      {
	string dbase_filename 
	  = completeFilename(cfgFilename, iter->second);
	InitializeObjectDatabase(dbase_filename);
      }
    else 
      throw user_error(__HERE__, "Cannot initialize Object Database"); 
    // ignore other parameters for now
  }
 }


 void BlobTracker::InitializeObjectDatabase(string dbaseDescriptionFilename) 
 {
     ifstream ifile(dbaseDescriptionFilename.c_str());

     if (!ifile.good()) 
	 throw user_error(__HERE__, "ObjectDatabaseList %s does not exist",
			     dbaseDescriptionFilename.c_str());
     else { // need to check the validity of content format try-catch exception
	 int numOfModels;
	 ifile >> numOfModels;
	 string modelFilenameFormat;
	 ifile >> modelFilenameFormat;	
	 ifile.close();

	 string complete_modelFilenameFormat
	     = completeFilename(dbaseDescriptionFilename, modelFilenameFormat);

	 if (numOfModels != 0) 
	     allocateMobileObjects(numOfModels, complete_modelFilenameFormat);
     }
 }


 void BlobTracker::allocateMobileObjects(int _numMobileObjects,
					 string _modelFilenameFormat) 
{
     char charString[1024];
     for (int i=0; i< _numMobileObjects; i++) 
     {
	 MobileObject* pMobileObject = new MobileObject(i);
	 snprintf(charString, 1024, _modelFilenameFormat.c_str(), i);
	 pMobileObject->init_config(string(charString));
	 mpMobileObjects[i] = pMobileObject;
     }
 }


 void BlobTracker::updateMobileObjectProperties(TrackAux::EnvironmentType _updateType, int frameNumber)
 {
     map<int,ViewTracker*>::iterator iter;
     map<int,MobileObject*>::iterator obj_iter;
     ViewTracker* pViewTracker=NULL;

     if (_updateType == TrackAux::MONOCAM_GROUND) { 

	 // Get first viewTracker
	 iter = mpViewTrackers.begin();
	 if (iter!= mpViewTrackers.end())
	     pViewTracker = iter->second;
	 else 
	     throw user_error(__HERE__, "First ViewTracker not available");

	 // Access the pose and confidence of objects in viewTracker
	 for (obj_iter=mpMobileObjects.begin(); 
	      obj_iter!=mpMobileObjects.end(); obj_iter++) 
	 {
	     MobileObject* pMobileObj = obj_iter->second;
	     pViewTracker->updatePoseProperties(pMobileObj);
	 }
     }
     else 
	 throw user_error(__HERE__, "Supports only ground projection");
 }


//  Note: ViewID for blob-tracking is hard-coded to be CAM_LEFT.
void BlobTracker::track() 
{
     static bool bInitialFrame=true;

     if (m_bStereo == false) // track only CAM_LEFT or 0 
     {
	 shared_ptr<CameraProj> spProj;
	 IplImage *img; // no need to release img!!! warning
	 if (!GetProjectionAndImage(CAM_LEFT, spProj, img))
	     throw user_error(__HERE__, "Camera and Video servers not ready");
	
	// Create ViewTracker for the first camera
	if (mpViewTrackers.size() == 0) { 
	    ViewTracker* pViewTracker = new ViewTracker(CAM_LEFT);	    
	    pViewTracker->configure(GetImageWidth(), GetImageHeight(), spProj);
	    mpViewTrackers[CAM_LEFT] = pViewTracker;
	}
	
	// Initialize ObjectViews of NEW mobile objects 
	if (mpMobileObjects.size() != 0) 
	   mpViewTrackers[CAM_LEFT]->initializeObjectViews(mpMobileObjects);

	// Track views of objects
	mpViewTrackers[CAM_LEFT]->track(img);
    }
    else 
    {
	// to fill in for stereo case
    }
   
    if (bInitialFrame == true)
	bInitialFrame = false;
    else {
	if (m_bStereo == false)
	    updateMobileObjectProperties(TrackAux::MONOCAM_GROUND);
	else {
	    // to fill in for stereo case
	}
    }
}

int BlobTracker::NumObjects()
{
    return mpMobileObjects.size();
}

bool BlobTracker::IsObjectActive(int objId)
{
    if (mpMobileObjects[objId] == 0) 
	throw user_error(__HERE__, "mobileObject %d does not exist", objId);
    else {
	if (mpMobileObjects[objId]->GetDetectionState() == RELIABLE)
	    return true;
	else
	    return false;
    }
}


bool BlobTracker::IsObjectLost(int objId)
{
    if (mpMobileObjects[objId] == 0) 
        throw user_error(__HERE__, "mobileObject %d does not exist", objId);
    else {
        if (mpMobileObjects[objId]->GetDetectionState() == DISAPPEARED)
	    return true;
	else
	    return false;
    }
}


void BlobTracker::UpdateSceneObject(int objId, Vision::SceneObject &obj)
{
    Math::Vector3D vec3D;
    char label[1024];
    
    if (mpMobileObjects[objId] == 0)
	throw user_error(__HERE__, "mobileObject %d does not exist", objId);
    else {
	mpMobileObjects[objId]->getPosition(vec3D.m_x, vec3D.m_y, vec3D.m_z);	
	obj.m_pose.m_position.m_x = vec3D.m_x;
	obj.m_pose.m_position.m_y = vec3D.m_y;
	obj.m_pose.m_position.m_z = vec3D.m_z;


	mpMobileObjects[objId]->getOrientation(vec3D.m_x, vec3D.m_y, vec3D.m_z);
	obj.m_pose.m_orientation.m_x = vec3D.m_x;
	obj.m_pose.m_orientation.m_y = vec3D.m_y;
	obj.m_pose.m_orientation.m_z = vec3D.m_z;


	obj.m_bbox.m_centroid.m_x = obj.m_pose.m_position.m_x;
	obj.m_bbox.m_centroid.m_y = obj.m_pose.m_position.m_y;
	obj.m_bbox.m_centroid.m_z = obj.m_pose.m_position.m_z;
	mpMobileObjects[objId]->getBoxSize(vec3D.m_x, vec3D.m_y, vec3D.m_z);
	obj.m_bbox.m_size.m_x = vec3D.m_x;
	obj.m_bbox.m_size.m_y = vec3D.m_y;
	obj.m_bbox.m_size.m_z = vec3D.m_z;

	// See also Vision.idl. E.g., Hand_001, Undefined_002
	string strtype = ObjtypeToString(mpMobileObjects[objId]->type());
	snprintf(label, 1024, "%s_%03d", strtype.c_str(), objId);	

	//int length = strlen(label);
        //char* newlabel = CORBA::string_alloc(length);
	//strcpy(newlabel, label);
	//obj.m_label.m_string = newlabel;

	obj.m_label.m_string = CORBA::string_dup(label);
	obj.m_label.m_confidence = mpMobileObjects[objId]->confidence();

	// hack: For now, we assign objId as m_color.m_int 
	// This is because a color signature file is the same as objId.
	obj.m_color.m_int = mpMobileObjects[objId]->patchID();
	obj.m_color.m_confidence = mpMobileObjects[objId]->confidence();


	// Assume recognizer assigns all undefined objects as shape. 
	if (mpMobileObjects[objId]->type() == Vision::UNDEF)
	    obj.m_objtype.m_int = (int)Vision::SHAPE;
	else
	    obj.m_objtype.m_int = mpMobileObjects[objId]->type();
	obj.m_objtype.m_confidence = 1.0;
    }
}


string BlobTracker::getObjectMemoryID(int objID) {
    if (mpMobileObjects[objID] == 0)
	throw user_error(__HERE__, "mobileObject %d does not exist", objID);
    else {
	return(mpMobileObjects[objID]->memoryID());
    }
}

void BlobTracker::putObjectMemoryID(int objID, string newMemoryID) 
{
    if (mpMobileObjects[objID] == 0)
	throw user_error(__HERE__, "mobileObject %d does not exist", objID);
    else {
	mpMobileObjects[objID]->putMemoryID(newMemoryID);
    }
}

bool BlobTracker::check_new_roi(CvRect rectRoi, int img_width, int img_height)
{
    int area  = rectRoi.width*rectRoi.width;
    if ((rectRoi.x < 0) || (rectRoi.y < 0) ||
	(rectRoi.x+rectRoi.width > img_width) ||
	(rectRoi.y+rectRoi.height > img_height)) {
	user_printf(__HERE__, "Proposed ROI's location is bad\n");
	return false;
    }
    else if ((area < (int)(min_area*img_width*img_height)) ||
	     (area > (int)(max_area*img_width*img_height))) {
	user_printf(__HERE__, "Proposed ROI's size is bad\n");
	return false;
    }
    else {
	std::map<int,ViewTracker*>::iterator vit 
	    = mpViewTrackers.find(CAM_LEFT);
	if (vit == mpViewTrackers.end())
	    throw user_error(__HERE__, "CAM_LEFT is not available");
	if (mpViewTrackers[CAM_LEFT]->check_overlap(rectRoi)) {
	    user_printf(__HERE__, "Proposed ROI is occluded by other objects\n");
	    return false;	
	}
	return true;
    }
}


bool BlobTracker::matchDatabase(int objID, int viewID, 
				IplImage *histImage, IplImage *mask)
{
    map<int,ViewTracker*>::iterator vit = mpViewTrackers.find(viewID);
    if (vit != mpViewTrackers.end()) {
	return (mpViewTrackers[viewID]->matchDatabase(objID, histImage, mask));
    }
    return false;
}


/**
 *  NOTE: must make sure that Configure has been done.
 */
void BlobTracker::UpdateObjectAppearanceModelFromROI(shared_ptr<const Vision::ROI> spROI)
{
    // Get info from roi 
    int roi_camID = spROI->m_camNum;
    Vision::BBox2D roi_bbox = spROI->m_bbox;
    string roi_objectWMA(spROI->m_objId);    
    string roi_address(spROI->m_address);
    
    if (roi_camID != CAM_LEFT) 
	return; // Only initialize with View 0
    
    #if 0
    user_printf(__HERE__, "Received a new roi from %s in view %d\n", 
		roi_address.c_str(), roi_camID);     
    #endif

    CvRect rectRoi = cvRect((int)(roi_bbox.m_center.m_x) - (int)(ceil((double)roi_bbox.m_size.m_x/2.0)) + 1,
			    (int)(roi_bbox.m_center.m_y) - (int)(ceil((double)roi_bbox.m_size.m_y/2.0)) + 1,
			    (int)roi_bbox.m_size.m_x, (int)roi_bbox.m_size.m_y);
    

    // Get image from view viewid
    IplImage *img=GetSourceImage(roi_camID);    
    if (img == NULL)
	throw user_error(__HERE__, "VideoServer is not ready");
    
    if (!check_new_roi(rectRoi, img->width, img->height)) 
	return;
        
    // Check the histogram of roi image 
    CvRect rectRoi_orig = cvGetImageROI(img);
    cvSetImageROI(img, rectRoi);
    IplImage *histImage = cvCloneImage(img);


    // MASKING
    IplImage *mask = cvCreateImage(cvSize((int)spROI->m_mask.m_width, 
					  (int)spROI->m_mask.m_height), 
				   IPL_DEPTH_8U, 1);
    unsigned char* dst = (unsigned char*) mask->imageData;
    unsigned char* src = (unsigned char*)&(spROI->m_mask.m_image[0]);
    memcpy(dst, src, spROI->m_mask.m_image.length());
    
    // IplImage *invHistImage = cvCloneImage(img);
//     cvNot(invHistImage,invHistImage);
//     cvNot(mask,mask);
//     cvAnd(histImage,invHistImage,histImage,mask); // src1, src2, dst, mask
    //cvReleaseImage(&mask);
    // cvReleaseImage(&invHistImage);  
    
    ///// END MASKING

    // Reject ROI if it is a skin-colored blob.
    int handObjId = TrackAux::HAND_ID;
    if (matchDatabase(handObjId, CAM_LEFT, histImage, mask)) {
	user_printf(__HERE__, "Segmented ROI is similar to Hand\n");
	static int rejected_counter=0;
	char fname[1024];
	snprintf(fname, 1024, "rejected-%d.jpg", rejected_counter++);
	cvSaveImage("fname", histImage);
	cvReleaseImage(&histImage);
	cvReleaseImage(&mask);
	cvSetImageROI(img, rectRoi_orig);
	return;
    }
    
    // Create a new MobileObject and add roi image to the MobileObject
    int newObjID = mpMobileObjects.size();
    MobileObject* pMobileObject = new MobileObject(newObjID);
    pMobileObject->addRoiImage(roi_camID, histImage, mask);
    cvReleaseImage(&histImage);
    cvReleaseImage(&mask);

    cvSetImageROI(img, rectRoi_orig);
    mpMobileObjects[newObjID] = pMobileObject;    
    
    // initialize ObjView in the ViewTracker
    if (m_bStereo == false)  // Assuming monocular setting
    {
	std::map<int,ViewTracker*>::iterator viter 
	    = mpViewTrackers.find(roi_camID);
	if (viter != mpViewTrackers.end()) 
	{ 
            // initialize ObjView of the new MobileObject
	    ViewTracker* pViewTracker = viter->second;
	    pViewTracker->initializeObjectView(pMobileObject);
	}
	else 
	    throw user_error(__HERE__, "ViewTracker for CAM_LEFT does not exist.");
    }
    else {
	throw user_error(__HERE__, "Stereo tracking is not supported");
    }
    
    putObjectMemoryID(newObjID, roi_objectWMA);
}


unsigned BlobTracker::numViews() 
{
    return mpViewTrackers.size();
} 

void BlobTracker::updateROIs(int objId, std::vector<Vision::ROI *> &rois)
{
    std::map<int,ViewTracker*>::iterator vit;
    for (vit=mpViewTrackers.begin(); vit!=mpViewTrackers.end(); vit++)
    {
	Vision::ROI *roi = new Vision::ROI();

	//assign defaults just in case
	roi->m_features.dimensions_.length(0);
	roi->m_features.data_.length(0);
	roi->m_contourPoints.length(0);

	updateROI(vit->second->getViewId(), objId, *roi);
	rois.push_back(roi);
    }
}


// Note: Only fills in the fields m_bbox and m_camNum.
void BlobTracker::updateROI(int viewId, int objId, Vision::ROI &roi)
{
    std::map<int,ViewTracker*>::iterator vit = mpViewTrackers.find(viewId);
    if (vit == mpViewTrackers.end()) {
	user_printf(__HERE__, "ViewTracker %d does not exist\n", viewId);
	return;
    }

    CvBox2D roiBox2D;
    bool res = mpViewTrackers[viewId]->getRoiBox2D(objId, roiBox2D);
    
    if (res==true) 
    { 
	roi.m_camNum = viewId;
	roi.m_bbox.m_center.m_x = roiBox2D.center.x;
	roi.m_bbox.m_center.m_y = roiBox2D.center.y;
	roi.m_bbox.m_size.m_x = roiBox2D.size.width;
	roi.m_bbox.m_size.m_y = roiBox2D.size.height;
    }
    else
	throw user_error(__HERE__, "Can't find a 2Dview of object %d.", objId);
}
