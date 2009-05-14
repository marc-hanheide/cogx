/** @file GeomUtils.hpp
 *  @brief Useful data structures and functions for geometrical manipulation.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _GEOM_UTILS_HPP_
#define _GEOM_UTILS_HPP_

#include <vector>
#include <opencv/cv.h>
#include <vision/components/common/GeomUtils/Vector2D.h>

namespace Geom {

    class Vector2D;

    /** @brief Convert a CvRect to an array of Vector2D points.
     *  
     *  @param rect a CvRect to convert.
     *  @param pts an array of four Vectors2D points.
     *  @return Void.
     */
    void convert2Points(CvRect rect, std::vector<Vector2D> &pts);

    /** @brief Show the content of CvRect on stdout.
     *  
     *  @param rect a CvRect to show.
     *  @return Void.
     */
    void show(CvRect rect);

    /** @brief Scale a CvRect.
     *  
     *  @param rect a CvRect to scale.
     *  @param scale_factor the scale factor 
     *  @return Void.
     */
    void scale(CvRect &rect, float scaleFactor); 
}

#endif
