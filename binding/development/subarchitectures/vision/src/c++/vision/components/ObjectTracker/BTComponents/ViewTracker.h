/** @file ViewTracker.h
 *  @brief A tracker that tracks views of objects.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _VIEW_TRACKER_
#define _VIEW_TRACKER_ 

#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>

#include <vision/components/common/GeomUtils/Vector3D.h>
#include <vision/components/common/GeomUtils/CameraProj.h>

#include "TrackAux.h"
#include "TrackerConfig.h"
#include "MobileObject.h"
#include "ObjView.h"

#define NUM_CORNERS 19

using namespace TrackAux;
using namespace Geom;
using namespace boost;

class ViewTracker  
{
 public:
    int viewId;
    
    CameraProj *pCamera;

    /** set in configure */
    int downsample;
    
    /** condition for first-time state initialization */
    bool m_bInitialCondition; 
    
    /** set in track */
    bool bTmpimages_allocated;
    CvSize imgsize;
    CvSize bsize;
    IplImage * inputimg; // no need to delete this
    IplImage *grayimg;
    IplImage *cannyimg;
    IplImage *distimg;
    IplImage *prev_resizeimg;
    
    std::map<int, ObjView*> mpObjectViews;
    
    /** set in configure */
    bool bRoiIsSet;
    TrackerConfig::ROI Roi;
    
    std::string objPatchNameFormat;
    
 public:
    ViewTracker(int _viewId);
    virtual ~ViewTracker();
    
    int getViewId() {return viewId;}

    IplImage* getDistImg() {return (distimg);};
    
    ResolutionType resolutionType();
    
    /** @brief Configure with cam parameters provided as argument.
     */
    void configure(int width, int height, Vision::Camera *pcam);

    /** @brief Configure with cam parameters that must be read from a file.
     */ 
    void configure(int width, int height, shared_ptr<CameraProj> spProj);
    
    
    void configure(int width, int height, TrackerConfig &trackerConfig, 
		   std::string cameraCfg="");    

    /** @brief Initializes ObjViews of all MobileObjects.
     */
    void initializeObjectViews(std::map<int,MobileObject*> &mpMobileObjects);

    /** @brief Initializes an ObjView of a MobileObject.  
     */
    void initializeObjectView(MobileObject *pMobileObject);
    

    void init_tmpImages(CvSize viewSize);

    void track(IplImage* pViewImg);
    
    bool is_in_ROI(int _x, int _y);
    
    void DrawASample(IplImage *img);

    void DrawAllSamples(IplImage *img);

    void drawResults(void);
    
    
    /** @brief Updates 3D ground positions of an object.
     */
    void getGroundPosition(MobileObject* pMobileObject);
    
    
    /** @brief Update the pose properties of a mobile object.
     */ 
    void updatePoseProperties(MobileObject* pMobileObject);


    /** @brief Get the working memory address and 2D bounding box
     *        of the object.
     *  @param objId the id of the object.
     *  @param rectRoi the bounding box of ROI
     *  @return true if OK, or false if ROI of objId is not available.
     */
    bool getRoiBox2D(int objId, CvBox2D &roiBox2D);

    
    
    /** @brief Matches an image against the object's patch. 
     *  
     *  @param objID the ID of object.
     *  @param histImage the image to match against the object.
     *  @return Bool.
     */ 
    bool matchDatabase(int objID, IplImage *histImage, IplImage *mask);

    
    /** @brief Checks if rectRoi overlaps with a view of any object. 
     *  
     *  @param rectRoi a CvRect to check for overlap.
     *  @return Bool.
     */ 
    bool check_overlap(CvRect rectRoi);
    
    /** @brief Checks if rectRoi overlaps with a view of objID. 
     *  
     *  @param rectRoi a CvRect to check for overlap.
     *  @param objID the ID of object, whose view to check against.
     *  @return Bool.
     */ 
    bool check_overlap(CvRect rectRoi, int objID);

    
};
#endif
