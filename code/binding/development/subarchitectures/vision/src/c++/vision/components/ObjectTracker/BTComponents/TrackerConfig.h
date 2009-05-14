/** @file TrackerConfig.h
 *  @brief Manages the configuration of tracker.
 *  
 *  @author Somboon Hongeng
 *  @date march 2008
 *  @bug No known bugs.
 */
#ifndef _TRACKER_CONFIG_H_
#define _TRACKER_CONFIG_H_

#include <string>
#include <vision/components/common/GeomUtils/Vector2D.h>

using namespace Geom;

class TrackerConfig {
 public:
    struct ROI {
	Vector2D m_center;
	Vector2D m_size;
    };
    
    TrackerConfig();
    virtual ~TrackerConfig();
    
    void configure(std::string filename);

 public:
    int MAX_OBJECTS;
    int MAX_VIEWS;
    int MAX_CONTOURS;
    
    int DOFNUM;
    int PARTICLENUM;
    int SAMPLEPTNUM;
    int SAMPLE_ANGLE_NUM;
    
    
    float DEPTH_THRESHOLD;
    int DOWNSAMPLE;
    float OutOfRangeCOST;
    float MaxColorMatchingSCORE;
    
    int DISTTHRESH;
    int DOFNUM_BOX;
    
    int MAX_CONTOUR_NUM;
    int HAND_DOFNUM;
    
    bool roi_is_set;
    ROI Roi;
};


#endif
