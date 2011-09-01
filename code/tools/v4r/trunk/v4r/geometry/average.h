/***************************************************************************
 *   Copyright (C) 2011 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/**
 * @file average.hpp
 * @author Markus Bader
 * @brief
 **/

#include <iostream>
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#ifndef V4R_GEOMETRY_AVERAGE_H
#define V4R_GEOMETRY_AVERAGE_H

namespace V4R {
double mean_arithmetic(double *v, unsigned int n);
float mean_arithmetic(float *v, unsigned int n);
double mean_arithmetic(int *v, unsigned int n);
cv::Point3_<double> mean_arithmetic(const std::vector<cv::Point3_<double> > &_samples);
cv::Point3_<double> mean_arithmetic(const std::list<cv::Point3_<double> > &_samples);
double varianz_arithmetic(const std::vector<cv::Point3_<double> > &_samples, const cv::Point3_<double> &_mean);
double varianz_arithmetic(const std::list<cv::Point3_<double> > &_samples, const cv::Point3_<double> &_mean);
    

}
#endif // V4R_GEOMETRY_AVERAGE_H