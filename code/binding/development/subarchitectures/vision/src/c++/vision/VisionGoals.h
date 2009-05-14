/**
 * @file VisionGoals.h
 * @brief header file for VisionGoals
 *  
 * @date February 2008
 */

#ifndef CAST_VISION_GOALS_H
#define CAST_VISION_GOALS_H

#include <cast/core/CASTCore.hpp>

using namespace std;

/**
 * @class VisionGoals
 * @brief Defines a set of tasks that vision sub-architecture supports.
 */
class VisionGoals
{
 public:
    ;
    /**  Find regions of interest */
    static const string SEGMENT_ROI_TASK;
    
    /**  Update camera parameters in ObjectTracker*/
    static const string UPDATE_CAMERA_PARAMETERS_TASK;

    /**  Initialize ROI tracking in ObjectTracker*/
    static const string INITIALIZE_ROI_TRACKING_TASK;

    /**  Track objects */
    static const string TRACK_OBJECTS_TASK;

   /** Find ROI being pointed on */
    static const string GET_POINTED_ROI_TASK;

    /** Extract features from ROI */
    static const string EXTRACT_FEATURES_TASK;

    /** Recognise object properties */
    static const string RECOGNISE_PROPERTIES_TASK;

    /** Update visual knowledge */
    static const string LEARN_TASK;

};

#endif
