/** @file MobileObject.h
 *  @brief A moving object.
 *
 *  This class contains a data structure for keeping 3D poses
 *  and intrinsic properties of a moving object such as  
 *  color and shape types.  
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _MOBILE_OBJECT_H_
#define _MOBILE_OBJECT_H_

#include <iostream>
#include <string>
#include <map>
#include <vision/components/common/GeomUtils/Model3D.h>
#include "TrackAux.h"
#include <opencv/cv.h>
//#include <vision/idl/Vision.hh>
#include "ObjectTypeManage.hpp"
#include "TrackerAbstract.hpp"

using namespace Geom;

class MobileObject : public ObjectTypeManage, public TrackerAbstract
{ 
 private:
    Vision::ObjType m_type; // object class to be filled by Recognizer

    TrackAux::ObjShape m_shape;

    string m_memoryID;

    float m_confidence;  // weight assigned by color/edge score

    //bool m_bLost; // lost if undetected for 3 frames, requiring pose resampling
    //bool m_bActive; // active when color and edge scores are good

    DetectionState m_detectionState;

    map<string,string> m_config;

    // hack for nick
    int m_patchID;

    //Images for histogram computation 
    map<int, IplImage*> m_roiImages;
    map<int, IplImage*> m_maskImages;

 public:
    int m_id;
    std::string m_patchFileformat;
    Vector3D m_boundingBox[8];
    int m_totalPoints;

    BBox3D m_bbox;
    Pose3D m_pose;
        
 public:
    MobileObject();
    MobileObject(int _id);
    virtual ~MobileObject();
    
    void initialize();
    //void init_config(map<string,string> &config);
    void init_config(string modelFilename);

    int patchID() { return m_patchID;};
    int id() { return m_id;};
    Vision::ObjType type() { return(m_type);};
    TrackAux::ObjShape shape() {return(m_shape);};
    string memoryID() {return m_memoryID;};
    float confidence() {return m_confidence;};
    map<string,string> config() {return m_config;};

    void SetDetectionState(DetectionState s) { m_detectionState=s; }
    DetectionState GetDetectionState() { return m_detectionState; }

/*     bool lost() {return m_bLost;}; */
/*     bool active() {return m_bActive;};     */
/*     void lost(bool bLost) {m_bLost = bLost;}; */
/*     void active(bool bActive) {m_bActive = bActive;}; */
    
    void putType(Vision::ObjType _type) {m_type=_type;};

    void putShape(TrackAux::ObjShape _shape) {m_shape=_shape;};
    
    void putMemoryID(string newMemoryID) {m_memoryID=newMemoryID;};
    
    //void putType(std::string _strtype);
    void putShape(std::string _strshape);
    void putPatch(std::string _strpatch);
    
    void putConfidence(float confidence) {m_confidence = confidence;};
    
    
    void addBBpoint(Vector3D point, int idx);
    Vector3D &get_centroid(void);
    void show(void);
  
    // for testing, dont use
    void completeBB(int idxA, int idxB,
		    int idxC, int newidx);
    void clear(void);
    

    void getPosition(float &x, float &y, float &z);
    void getOrientation(float &x, float &y, float &z);
    void getBoxSize(float &x, float &y, float &z);
    

    void addRoiImage(int camID, IplImage *roiImage, 
		     IplImage *maskImage=0);
    IplImage *getRoiImage(int viewID);
    
    IplImage *getMaskImage(int viewID);

    friend std::ostream &operator<<(std::ostream &of, MobileObject &sobj);
};

#endif
