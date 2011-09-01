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

#include "stereovision.h"
#include "camerageometry.h"

using namespace V4R;

StereoVision::StereoVision()
        : mpGeoA(NULL)
        , mpGeoB(NULL) {
}
StereoVision::StereoVision(CameraGeometry &rGeoA, CameraGeometry &rGeoB)
        : mpGeoA(NULL)
        , mpGeoB(NULL) {
    set(rGeoA, rGeoB);
}
StereoVision::~StereoVision() {
}
void StereoVision::set(CameraGeometry &rGeoA, CameraGeometry &rGeoB) {
    mpGeoA = &rGeoA, mpGeoB = &rGeoB;
}
void StereoVision::updateEssential() {
    mEssA = CameraGeometry::computeEssential(*mpGeoA, *mpGeoB);
    mEssB = CameraGeometry::computeEssential(*mpGeoB, *mpGeoA);
}


cv::Point3_<double> StereoVision::triangulatePoint(const cv::Point_<double> &rPointA, const cv::Point_<double> &rPointB) {
    cv::Point3_<double> des;
    cv::triangulatePoint(mpGeoA->MIntExt3x4(), mpGeoB->MIntExt3x4(), rPointA, rPointB, des);
    return des;
}

cv::Mat StereoVision::triangulatePoints(const cv::Mat &rPointsA, const cv::Mat &rPointsB) {
    cv::Mat_<double> p3D(3, rPointsA.rows);
    cv::triangulatePoints(mpGeoA->MIntExt3x4(), mpGeoB->MIntExt3x4(), rPointsA, rPointsB, p3D);
    return p3D;
}
