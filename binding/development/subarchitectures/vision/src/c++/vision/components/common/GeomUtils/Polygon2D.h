/** @file Polygon2D.h
 *  @brief Definition of a 2D polygon.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _POLYGON2D_H_
#define _POLYGON2D_H_

#include <iostream>
#include <vector>
#include "Vector2D.h"
#include "Polygon.h"

namespace Geom {
    
class Polygon2D {
 private:
    POLYGONARC* poly; 
    Vector2D center;  
    
    void compute_center(void);
	
 public:
    // Creation.
    Polygon2D(void);
    Polygon2D(const Polygon2D& p2D);
    ~Polygon2D(void);
  
    Polygon2D& operator=(const Polygon2D& rhs);

    // Operations.
    const POLYGONARC& g_poly(void) const { return(*poly); };
    const Vector2D& g_center(void) const { return(center); };
    
    /** @brief Initialize polygonarc by points pol[].
     */ 
    void init(int *pol, int nb_pts);

    
    void init(std::vector<Vector2D> points);

    int is_pt_in_zone(Vector2D* pr) const;
    int is_pt_close_to(Vector2D* pr) const;
    double distance(Vector2D* pr) const; /* diff_x + diff_y */
    void chgt_ref(void);

    int is_pt_within_range(Vector2D* pr, float thres_dist=200) const;
    
    // Affichage.
    void draw(void);
    void print(std::ostream *os) const { *os << poly->nb_vertices();};
};
 
}


//std::ostream& operator<<(std::ostream& out, const Geom::Polygon2D& p);


#endif /* _POLYGON2D_H_ */
