/** @file RectView.h
 *  @brief A rectangle view.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#ifndef _RECT_VIEW_H_
#define _RECT_VIEW_H_

#include <vector>
#include "ObjView.h"

using std::vector;
using namespace TrackAux;

class RectView : public ObjView
{
 private:
    static const int DOFNUM_BOX = 5;
    CvPoint m_boxOutlines[4][NUM_PTS];

    float m_oldstate_box[NUM_STATES][DOFNUM_BOX];
    float m_allstate_box[NUM_STATES][DOFNUM_BOX];
    float m_weight[NUM_STATES];

    float m_ellipseRadius[2][2];
    float m_dofrange_box[DOFNUM_BOX][2];
    float m_dofvar_box[DOFNUM_BOX]; 

    float m_beststate_box[DOFNUM_BOX];
    float m_bestweight;

    IplImage *m_mask;
    float m_dirWeights[NUM_DIRS];

    // structure for evalutating hand shape and for storing posture templates
    CvMemStorage* m_storage;
    vector<CvBox2D> m_vContours;
    

    void ResampleAlgorithm_BOX(float myoldstate[][DOFNUM_BOX], 
			       float oldsampleweight[], 
			       int oldparticlenum, 
			       float mynewstate[][DOFNUM_BOX], 
			       int newparticlenum);


    bool validateBox(CvBox2D ell_box);
 public:
    RectView(ViewTracker *parent, int viewId, 
	     int objId, Vision::ObjType _type, 
	     TrackAux::ObjShape _shape,
	     map<string,string> config);
    ~RectView();

    void compBoxOutlineTemplates(int c_x, int c_y, int c_w, int c_h, int c_d);
    void drawBoxOutline(IplImage *img, float *state, CvScalar color);
    
    /** @brief Aligns boxpoints and put them in pts[].
     */
    void alignPointsOnBBox(CvPoint2D32f boxpts[], Vector3D pts[]);


    void computeBBoxPoints(float state[], CvPoint2D32f boxpts[]);

    void computeCC(IplImage *resizedImg);

    void computeDirectionWeights(void);
    void computeBoxAngles_orig(CvBox2D ell_box,float &angle_in_deg, float &std_angle);
    void computeBoxAngles(CvBox2D ell_box, float &angle_in_deg, float &std_angle);
    /** @brief Computes the angle between p0p1 and p0p2
     */
    double computeAngle(CvPoint2D32f &p0, CvPoint2D32f &p1, CvPoint2D32f &p2);

    void computeGaussianWeight(double gaussMask[], 
			       int total_steps, int std_steps);
    int degree_to_dirIdx(double angle_in_degrees, double step_width);
    int adjust_dirIdx(int dirIdx, int adjustment);
    int adjust_dirIdx_0_18(int dirIdx, int adjustment);

    /** @brief Computes the difference of angles. [0,180]
     */
    double compute_angle_difference(double ang1, double ang2);
    
    void normalize_dirWeights(void);
    
    /////// Interfaces ///////
    virtual void allocateImages(CvSize _bsize);

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

    // Move this to ObjView ??
    virtual void initstate(float * initstate);
    virtual void initUsingBackprojectWeight(int particlenum);

    virtual void CalculateWeights();
    virtual void Resample();
    virtual void DSystem();
    
    virtual void OutputTESTING(int method);
};

#endif
