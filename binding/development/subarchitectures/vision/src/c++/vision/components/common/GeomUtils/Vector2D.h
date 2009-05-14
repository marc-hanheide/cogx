/** @file Vector2D.h
 *  @brief A 2D vector.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#ifndef _VECTOR_2D_H_
#define _VECTOR_2D_H_

#include "GeomMath.h"
#include <iostream>

namespace Geom {

class Vector2D  
{
 public: // Intentionally make it public
    float    x;   
    float    y;   
        
 public:
    Vector2D(const Vector2D& pp);
    Vector2D(float xx=0, float yy=0);
    Vector2D& operator=(const Vector2D& pp);
    ~Vector2D(void);

    // Accessors.
    void   p_x(float xx) {x=xx;};
    void   p_y(float yy) {y=yy;};
    float g_x(void) const {return x;};
    float g_y(void) const {return y;};
    
    // Operations.
    float distance(const Vector2D *pr) const;
    float distance(const Vector2D& pt) const;
    
    float cosinus(const Vector2D *pr) const;
    float sinus(const Vector2D *pr) const;
    float angle(const Vector2D *pr) const;

    float cosinus(const Vector2D &p) const;
    float sinus(const Vector2D &p) const;
    float angle(const Vector2D &p) const; 

    void   transform(affine aff_new);
    
    bool operator==(const Vector2D& pp) const;
    bool operator!=(const Vector2D& pp) const;

    bool lies_between(Vector2D *pr1, Vector2D *pr2) const; 
    
    // Display.
    friend std::ostream& operator<<(std::ostream& out, const Vector2D &pt);
    /* friend std::ostream& operator<<(std::ostream& out, const Vector2D *ptr); */
};

extern const Vector2D VOID_VECTOR2D;

}

#endif 
