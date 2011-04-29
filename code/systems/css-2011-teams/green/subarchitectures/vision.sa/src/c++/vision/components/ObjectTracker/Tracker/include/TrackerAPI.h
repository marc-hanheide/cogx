/** @file TrackerAPI.h
 * 
 * public interface for Tracker Library
 * attempts to minimise leakage of #define's and globals
 * 
 * @author	Sebastian Zurek (UoB)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _TRACKER_TRACKERAPI_H_
#define _TRACKER_TRACKERAPI_H_

#include "headers.h"
#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"

/* now clean up */

#undef GL_ORTHO
#undef GL_PERSPECTIVE
#undef NONE
#undef BARREL
#undef SOBEL_THRESHOLD
#undef THINNING_THRESHOLD
#undef SPREADING_THRESHOLD
#undef DISTANCE_SCALING
#undef SPREADING_LOOPS
#undef PI
#undef EPSILON
#undef DEG2RAD
#undef RAD2DEG
#undef FN_LEN
#undef myalloc
#undef FTOL
#undef PIOVER180
#undef g_Resources
#undef OFFSETOF
#undef FLOAT_NULL
#undef DISTLEN

#endif /*_TRACKER_TRACKERAPI_H_*/
