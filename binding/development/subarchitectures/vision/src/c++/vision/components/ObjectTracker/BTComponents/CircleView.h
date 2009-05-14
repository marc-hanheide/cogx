/** @file CircleView.h
 *  @brief A circle view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#ifndef _CIRCLE_VIEW_H_
#define _CIRCLE_VIEW_H_

#include "ObjView.h"

using namespace TrackAux;

class CircleView : public ObjView
{
 private:
    static const int DOFNUM = 3;

    float m_oldstate[NUM_STATES][DOFNUM];
    float m_allstate[NUM_STATES][DOFNUM];
    float m_weight[NUM_STATES];

    float m_radius[2]; 
    float m_dofrange[DOFNUM][2];
    float m_dofvar[DOFNUM];

    float m_beststate[DOFNUM];
    float m_bestweight;

    

 public:
    CircleView(ViewTracker *parent, int viewId, 
	       int objId, Vision::ObjType _type, 
	       TrackAux::ObjShape _shape,
	       map<string,string> config);
    ~CircleView();    

    void initRadius(int newRadius);

    void ResampleAlgorithm(float myoldstate[][DOFNUM], 
			   float oldsampleweight[], 
			   int oldparticlenum, 
			   float mynewstate[][DOFNUM], 
			   int newparticlenum);

    virtual void initStateParameters(map<string,string> &config);
    virtual void drawAllSamples(IplImage *img);
    virtual void drawASample(IplImage *img);
    virtual float colorCost();
    virtual float edgeCost();

    virtual float averageBackProjection(float state[], bool bCompletePose);
    virtual float Evaluate_distimg(float state[]);
    virtual float Evaluate_combined(float state[]);

    virtual void get2DGroundPoint(float &x, float &y);
    virtual void getROI(CvBox2D &roi, float &outWeight);

    virtual void detect(IplImage *resizedImg, bool initCondition);

    virtual void initstate(float * initstate);
    virtual void initUsingBackprojectWeight(int particlenum);

    virtual void CalculateWeights();
    virtual void Resample();
    virtual void DSystem();
    
    virtual void OutputTESTING(int method);
};

#endif
