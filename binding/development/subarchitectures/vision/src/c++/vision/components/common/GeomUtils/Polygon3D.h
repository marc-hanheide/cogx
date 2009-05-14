/** @file Polygon3D.h
 *  @brief A specification of a 3D region.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _POLYGON3D_H_
#define _POLYGON3D_H_

#include <list>
#include <vector>
#include <iostream>
#include "Vector3D.h"

namespace Geom {

struct seg_3d{
    float debut[3];
    float fin[3];  
}typedef SEGMENT_3D;

class Polygon3D {
private:
    std::list<Vector3D> poly_3D;
    
    void init(const std::vector<Vector3D> contour3D);
    
public:
    // Creation.
    Polygon3D(void);
    Polygon3D(const std::vector<Vector3D> contour3D);
    ~Polygon3D(void);
    
    Vector3D compute_center(void);
    
    Polygon3D& operator=(const Polygon3D &src);    
    friend std::ostream& operator<<(std::ostream &out, Polygon3D &p);
    void print(std::ostream *os);    
};
typedef Polygon3D *Polygon3D_r;

}

#endif /* _POLYGON3D_H_ */
