/**
 * @file NurbsCreator
 * @author Thomas Mörwald
 * @date June 2011
 * @version 0.1
 * @brief Fitting NURBS to point clouds
 */

#ifndef _NURBS_CREATOR_H_
#define _NURBS_CREATOR_H_

#include <opencv2/core/core.hpp>
#include <v4r/TomGine/tgTomGine.h>
#include "NurbsTools.h"



class NurbsCreator
{
private:

public:
	static void FitNurbsSurface(unsigned order, unsigned refinement,
			unsigned iterations, cv::Mat_<cv::Vec4f> matCloud,
			cv::Mat_<uchar> mask, cv::Mat_<uchar> contour,
			TomGine::tgTomGineThread* dbgWin);

};

#endif
