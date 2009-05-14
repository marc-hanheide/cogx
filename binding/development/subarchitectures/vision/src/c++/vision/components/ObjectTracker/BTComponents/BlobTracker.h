/** @file BlobTracker.h
 *  @brief Tracks objects as 3D blobs.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */

#ifndef BLOB_TRACKER_H
#define BLOB_TRACKER_H

#include "vision/components/ObjectTracker/Tracker.h"
#include <string>
#include <map>
#include <fstream>
#include "MobileObject.h"
#include "ViewTracker.h"
#include "ObjectTypeManage.hpp"

class BlobTracker : public Tracker, public ObjectTypeManage
{
 private:
    std::map<int,ViewTracker*> mpViewTrackers; //  maps viewId to ViewTracker.
    std::map<int,MobileObject*> mpMobileObjects; // maps objId to MobileObject.

     //std::ofstream ofile;  

    bool m_bStereo;
    void Configure(std::string cfgFilename);
    void InitializeObjectDatabase(std::string dbaseDescriptionFilename);
    void allocateMobileObjects(int _numMobileObjects, 
			       std::string _modelFilenameFormat);

    /** @brief Updates the properties of mobile objects.
     *   
     *  Updates the poses and confidence of objects in mpMobileObjects.
     *
     *  @return Void.
     */ 
    void updateMobileObjectProperties(TrackAux::EnvironmentType _updateType,
				      int frameNumber=0);

    /** @brief Checks the validity of rectRoi. 
     *  
     *  Checks if rectRoi corresponds to a new SceneObject
     *  to be tracked. A good roi must not be too large or small,
     *  must not look like skin, and must not be overlapped with 
     *  a hand.
     * 
     *  @param rectRoi The roi to be checked.
     *  @param img_width The width of the image.
     *  @param img_height The height of the image.
     *  @return Bool.
     */ 
    bool check_new_roi(CvRect rectRoi, int img_width, int img_height);


    /** @brief Matches an image against the object's patch. 
     *  
     *  @param objID the ID of object.
     *  @param viewID the view ID 
     *  @param histImage the image to match against the object.
     *  @return Bool.
     */ 
    bool matchDatabase(int objID, int viewID, IplImage *histImage, 
	IplImage *mask);

public:
    BlobTracker(string cfgFilename, bool bStereo=false);
    virtual ~BlobTracker();

    virtual void track();
    virtual int NumObjects();

    virtual bool IsObjectActive(int objId);
    virtual bool IsObjectLost(int objId);

    virtual void UpdateSceneObject(int objId, Vision::SceneObject &obj);
    virtual string getObjectMemoryID(int objId);
    virtual void putObjectMemoryID(int objID, string newMemoryID);
    virtual void UpdateObjectAppearanceModelFromROI(shared_ptr<const Vision::ROI> spROI);
    virtual unsigned numViews();
    virtual void updateROI(int viewId, int objId, Vision::ROI &roi);
    virtual void updateROIs(int objId, std::vector<Vision::ROI *> &rois);
};

#endif
