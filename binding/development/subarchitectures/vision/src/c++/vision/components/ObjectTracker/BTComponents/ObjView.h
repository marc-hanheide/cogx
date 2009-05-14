/** @file ObjectView.h
 *  @brief An Abstract class of object's view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#ifndef _OBJ_VIEW_H_
#define _OBJ_VIEW_H_

#include <map>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "TrackAux.h"
#include <vision/components/common/GeomUtils/Vector3D.h>
#include <vision/idl/Vision.hh>

using std::map;
using std::string;
using namespace Geom;

class ViewTracker;

enum evidenceType {
    COLOR,
    EDGE,
    COMBINED
};


class ObjView 
{ 
 protected:
    // should move these to trackAux, to maintain consistency
    static const int NUM_STATES = 200;
    static const int NUM_PTS = 36;
    static const int NUM_DIRS = 36;
    
    static const int MAX_CONTOURS = 20;
    static const int MAX_CONTOUR_NUM = 20;
    
    static const int DISTTHRESH = 10;

    // Out-dated. This can be removed.
    static const float OutOfRangeCOST = 10.0; // 30.0

    static const float MaxColorMatchingSCORE = 255.0;
    static const int DOWNSAMPLE = 2;

    static const int LOSTCOUNT_THRESHOLD = 3;
    
    int m_id; // ID of mobile object

    int m_view;

    TrackAux::ObjShape m_shape;

    Vision::ObjType m_type;
    
    CvHistogram *m_hist;
    bool m_bhistvalid;

    IplImage *m_histImage;
    IplImage *m_weightImage;
    IplImage *m_backprojectImage;
    CvSize m_bsize;
    bool m_bImagesAllocated;

    int m_missingCounter;

    //bool m_bReinitialize;
    //bool m_initialCond;

    DetectionState m_detectionState;

    float m_maxWeight;

    CvPoint2D32f m_orient[NUM_PTS]; 

 public:
    ObjView();
    virtual ~ObjView();

    ViewTracker *m_pViewTracker;
    
    int ID() {return m_id;}
    
    void initHistogram(IplImage *roiImage, IplImage *mask);
    void initHistogram(char patchFilename[]);

    /** @brief Checks evidence against thresholds to determine
     *   if the detection is reliable.
     */ 
    bool is_reliable(evidenceType basis=COMBINED);

    DetectionState GetDetectionState() { return m_detectionState; }

    void SetDetectionState(DetectionState s) { m_detectionState = s; };

    /** @brief Checks if objview is lost and needs to be re-initialized.
     */ 
    //bool is_lost(void) { return (m_bReinitialize); }
    //void initialCond(bool onOff) {m_initialCond=onOff;}
    //bool initialCond() {return m_initialCond;}
    //void SetReinitiailize(bool bInit) { m_bReinitialize=bInit; }


    /** @brief Compute a score from 0 to MaxColorMatchingSCORE.
     */
    float Evaluate_backProjection(float state[]);

    void computePotentialImg(float max_radius);
    void CalBack(IplImage *src, IplImage *back, CvHistogram *hist_to_calback);


    CvHistogram * getHistogram();

    virtual void allocateImages(CvSize _bsize);

    virtual void initStateParameters(map<string,string> &config)=0;
    virtual void drawAllSamples(IplImage *img)=0;
    virtual void drawASample(IplImage *img)=0;
    virtual float colorCost()=0;
    virtual float edgeCost()=0;

    virtual float averageBackProjection(float state[], bool bCompletePose=false)=0;
    /** @brief Evaluate using distimage. [0,DISTTHRESH]. 
     */
    virtual float Evaluate_distimg(float state[])=0;

    /** @brief Computes a matching score. [0,1].
     */ 
    virtual float Evaluate_combined(float state[])=0;

    virtual void get2DGroundPoint(float &x, float &y)=0;
    virtual void getROI(CvBox2D &roi, float &outWeight)=0;

    virtual void detect(IplImage *resizedImg, bool initCondition)=0;
  
    virtual void initstate(float * initstate)=0; 
    virtual void initUsingBackprojectWeight(int particlenum)=0;

    virtual void CalculateWeights()=0;

    virtual void Resample()=0;
    virtual void DSystem()=0;
    
    
    /** @brief Estimates the view of object. 
     *   
     *  @param method If 0, output maxlikelyhood state.
     *                If 1, output the average state.
     */
    virtual void OutputTESTING(int method)=0;

};


#endif
