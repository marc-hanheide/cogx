/** @file  Polygon.h
 *  @brief Defines an arcs-structure of a polygon (POLYGONARC).   
 *
 *  @author Somboon Hongeng
 *  @date March 2007
 *  @bug No known bugs.
 */
#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <vector>
#include "Vector2D.h"

namespace Geom {

struct seg{
    double debut[2];
    double fin[2];
}typedef SEGMENT;


class POLYGONARC {
 private:
    int     flag_max;
    double xmin,xmax,ymin,ymax; // bounding box of the polygon
    std::vector<Vector2D> v0;  // start points
    std::vector<Vector2D> v1;  // end points

    void Space_min_max_segment(SEGMENT seg,double *borne);
    int Space_min_max_polygon(double *borne); 
    int Space_point_is_on_segment(double *X,SEGMENT seg);
    int Space_inter_segment_segment(SEGMENT seg1,SEGMENT seg2,double *u1);
    int Space_is_odd(int nb);


 public:
    POLYGONARC();
    POLYGONARC(const POLYGONARC& polyarc);
    ~POLYGONARC();
    
    unsigned nb_vertices() const { return v0.size(); };
    const Vector2D& get_v0(int i) const;
    const Vector2D& get_v1(int i) const;
    
    double get_xmin() const {return xmin;};
    double get_xmax() const {return xmax;};
    double get_ymin() const {return ymin;};
    double get_ymax() const {return ymax;};

    void compute_bbox();    
    void append_vertice(int idx, double x, double y);

    
    double Space_distance(double *X1,double *X2);
    void Space_get_segment_from_poly(int num, SEGMENT *seg);
    int Space_point_is_inside_polygon(double *X);
};


}

#endif
