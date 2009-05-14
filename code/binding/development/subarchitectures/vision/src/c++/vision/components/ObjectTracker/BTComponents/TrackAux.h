/** @file TrackAux.h
 *  @brief Data structures and utility functions for tracking. 
 *
 *  Provides interfaces to various utility structures and functions.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#ifndef _TRACK_AUX_H_
#define _TRACK_AUX_H_

#include <string>
#include <opencv/cv.h>
#include <opencv/cxcore.h>


enum DetectionState {
    UNINITIALIZED,
    RELIABLE,        // high supporting evidence
    TEMPORARY_LOST,  // require reinitialization.
    DISAPPEARED      // tracking finished.
};


namespace TrackAux
{
    static const int DOWNSAMPLE = 2;

    static const int HAND_ID = 0;
    
    enum ResolutionType {RES_UNKNOWN, RES_VGA, RES_QVGA};    
    enum ObjShape {SH_UNDEF, SH_CIRCLE, SH_RECT};
    enum EnvironmentType {MONOCAM_GROUND, STEREO};
    
    static const double PI = 3.1415927;
    static const float CONFIDENCE_THRES = 0.20;
    
    ObjShape StringToShapetype(std::string _str);
    std::string ShapetypeToString(ObjShape _shape);
    
    double urandom(void);
    double grandom(void);
    double grandom(double val, double sigma);
    void AccumulateProb(float prob[], float accprob[], int arraysize);
    int bsearch(float cumulWei[], int arraysize, float choice);

    void updateHist(CvHistogram *crt_hist, IplImage * histsrcimg,
		    CvRect roirect, IplImage *mask, bool notclean=false);

    CvHistogram *createHist();
    
    void ShowHistogram(CvHistogram &hist);
}
#endif

