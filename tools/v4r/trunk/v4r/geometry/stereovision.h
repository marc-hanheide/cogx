/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2010  Markus Bader

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef STEREOVISION_H
#define STEREOVISION_H


#include <v4r/geometry/structs.h>

namespace V4R {
class CameraGeometry;

class StereoVision
{
public:
	
    enum Type {
	IMG_A = 0,
	IMG_B = 1
    };
    StereoVision();
    StereoVision(CameraGeometry &rGeoA, CameraGeometry &rGeoB);
    ~StereoVision();
    void set(CameraGeometry &rGeoA, CameraGeometry &rGeoB);
    void updateEssential();
		cv::Point3_<double> triangulatePoint(const cv::Point_<double> &rPointA, const cv::Point_<double> &rPointB);
    cv::Mat triangulatePoints(const cv::Mat &pPointsA, const cv::Mat &pPointsB);

    const cv::Mat_<double> &MEssA() const {
        return mEssA;
    };
    const cv::Mat_<double> &MEssB() const {
        return mEssB;
    };
private:
    CameraGeometry *mpGeoA;
    CameraGeometry *mpGeoB;
    cv::Mat_<double> mEssA;
    cv::Mat_<double> mEssB;

};
};

#endif // STEREO_H
