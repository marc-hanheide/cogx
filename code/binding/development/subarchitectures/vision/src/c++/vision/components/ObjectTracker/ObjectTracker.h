/** @file ObjectTracker.h
 *  @brief A 3D object tracker.
 *
 * @author Somboon Hongeng
 * @date march 2007
 * @bug No known bugs.
 */
#ifndef OBJECT_TRACKER_H_
#define OBJECT_TRACKER_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include "Tracker.h"
#include "vision/utils/VisionProxy.h"
#include <fstream>
#include "BTComponents/TrackerAbstract.hpp"
#include <vision/utils/SceneObjectWriter.hpp>

using namespace std;

class ObjectTracker : public VisionProxy, 
  public TrackerAbstract,  public SceneObjectWriter
{
    typedef map <string, shared_ptr<const Vision::Camera> > CameraDataMap;
    typedef map <string, shared_ptr<const Vision::ROI> > RoiDataMap;
    typedef map<string, string> TaskMap;

  public:
    ObjectTracker(const string &_id);
    virtual ~ObjectTracker();
    virtual void start();

  protected:
    virtual void taskAdopted(const string &_taskID);
    virtual void taskRejected(const string &_taskID);
    virtual void runComponent();
    virtual void configure(map<string,string> & _config);

 private:
    Tracker  *m_tracker;

    string   m_attendedListMemoryID;

    ofstream m_outfile; // for debuging. may remove later.
    
    bool m_stereoMode;

    int m_framerate_ms;

    // Hashtables used to record tasks and data for processing
    TaskMap m_tasks; 

    CameraDataMap m_cameraDataToProcess;  

    RoiDataMap m_roiDataToProcess;
    
    bool checkForTask(string taskName);

    void AcquireCameraSetupInfo();

    void UpdateImageBuffer();

    void UpdateCameraProjection(shared_ptr<const Vision::Camera> spCam);

    /** @brief Registers an appearance model of a new SceneObject,  
     *   from the ROI (or in stereo case, from the ROI from CAM_LEFT).
     */ 
    void InitializeRoiTracking(shared_ptr<const Vision::ROI> spROI);

    /** @brief Tracks objects. 
     */ 
    void Track();

    string attendedListMemoryID() {return m_attendedListMemoryID;};

    /** @brief A function that handles an overwrite signal of Camera. 
     */
    void HandleCameraEvent(const cdl::WorkingMemoryChange &_wmc);

    /** @brief A function that handles an overwrite signal of ROI. 
     */ 
    void HandleNewRoiEvent(const cdl::WorkingMemoryChange &_wmc);

    /** @brief A function that proposes tracking while idling. 
     */
    void HandleIdleEvent();

    /** @brief Computes m_bbox, m_camNum, m_time, m_objID fields of ROIs. 
     *
     *  Other fields are left blank.
     */
    void updateObjectInfoInWorkingMemory(int mobid, string &strMemID,
					 FrameworkBasics::BALTTime scene_time);

}; // class ObjectTracker

#endif
