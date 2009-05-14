/** @file Tracker.h
 *  @brief Generic tracker header. 
 *   
 *  Any specialized traker should derive from this class.
 *
 * @author Somboon Hongeng
 * @date march 2007
 * @bug No known bugs.
 */
#ifndef Tracker_H
#define Tracker_H

#include <vision/components/common/VisualizationUtils/Visualization.h>
#include <vector>
#include <vision/idl/Vision.hh>
#include <CoSyCommon/idl/Math.hh>

using namespace std;

class Tracker : public Visualization
{
public:
    Tracker();
    virtual ~Tracker();
    float min_area;
    float max_area;
    // Tracks objects
    virtual void track() = 0;

    // Returns the number of tracked objects
    virtual int NumObjects() = 0;
    
    // Checks whether objId is active (visually present)
    virtual bool IsObjectActive(int objId) = 0;

    // Checks whether objId is lost
    virtual bool IsObjectLost(int objId) = 0;

    /** @brief Updates the SceneObject
     *
     *  Updates the following data about SceneObject:
     *  m_bbox, m_pose, m_label (in format "OBJTYPE_%02d"), m_color.
     *  
     *  @param objId the ID of the mobile object to use for update.
     *  @param obj the "SceneObject" data structure to be updated.
     *  @return Void. 
     */
    virtual void UpdateSceneObject(int objId, Vision::SceneObject &obj) = 0;
    
    /** @brief Returns the working memory address of objID.
     */
    virtual string getObjectMemoryID(int objId) = 0;

    /** @brief Assigns a working memory address to objID.
     */
    virtual void putObjectMemoryID(int objID, string newMemoryID) =0;
    
    /** @brief Gets working memory address of ROI.
     */ 
    //virtual void getRoiMemoryID(int _viewID, int _objID) = 0;
    
    /** @brief Gets working memory address of ROI.
     */ 
    //virtual void putRoiMemoryID(int _viewID, int _objID, string newMemoryID) = 0;

    /** @brief Create a new MobileObject, and initilze the color histogram.
     */
    virtual void UpdateObjectAppearanceModelFromROI(shared_ptr<const Vision::ROI> spROI)=0;

    /** @brief Returns the number of active objects.
     */
    int NumActiveObjects();

    virtual unsigned numViews() = 0;
    virtual void updateROI(int viewId, int objId, Vision::ROI &roi) = 0;
    virtual void updateROIs(int objId, std::vector<Vision::ROI *> &rois) = 0;
};

#endif

