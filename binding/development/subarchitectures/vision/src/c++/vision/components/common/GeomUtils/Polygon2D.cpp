/** @file Polygon2D.cpp
 *  @brief A 2D polygon.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include <cmath>
#include "Polygon2D.h"
#include <vision/components/common/SystemUtils/Common.h>

namespace Geom {

    using std::vector;
    using std::ostream;
    using namespace Common;

Polygon2D::Polygon2D(void) : 
    poly(NULL) {
}

Polygon2D::Polygon2D(const Polygon2D& p2D) {
    poly = new POLYGONARC(p2D.g_poly());
    compute_center();
}


Polygon2D::~Polygon2D(void) {
    if(poly!=NULL) {
	delete(poly);
	poly = NULL;
    }
}



Polygon2D& Polygon2D::operator=(const Polygon2D& rhs) {
    if (this == &rhs)
	return(*this);
    
    if (rhs.poly != NULL)
	poly = new POLYGONARC(rhs.g_poly());
    compute_center();
    
    return(*this);
}



void Polygon2D::init(vector<Vector2D> points) {
    if (poly!=NULL) delete(poly);
    
    poly = new POLYGONARC();
    if (poly == NULL)
	throw user_error(__HERE__, "Fail to allocate polygon");
    else {
	
	int numpoints = points.size();
	for (int i=0; i<numpoints; i++) {
	    /*org*/
	    poly->append_vertice( 0, points[i].g_x(), points[i].g_y() ); 
	    /*fin*/
	    poly->append_vertice( 1, points[(i+1)%numpoints].g_x(),
				  points[(i+1)%numpoints].g_y() );
	}
	
	poly->compute_bbox(); // compute xmin, xmax, ymin, ymax of poly
	compute_center(); // use bbox to compute center
    }
    
}


void Polygon2D::init(int *pol, int nb_pts) {

    if (poly!=NULL) delete(poly);

    poly = new POLYGONARC();
    if (poly == NULL)
	throw user_error(__HERE__, "Fail to allocate polygon");
    else {
      for (int i=0; i<nb_pts; i++) {
	  /*org*/
	  poly->append_vertice(0, pol[2*i], pol[2*i+1] );
	  /*fin*/
	  poly->append_vertice( 1, 
			        pol[(2*i+2) % (2*nb_pts)],
			        pol[(2*i+3) % (2*nb_pts)] );
      }
    
      poly->compute_bbox(); // compute xmin, xmax, ymin, ymax of poly
      compute_center(); // use bbox to compute center
    }
}

void Polygon2D::compute_center(void) {

    if (poly == NULL) { 
	center.p_x(0.);
	center.p_y(0.);
    }
    else {
	double xx = (poly->get_xmin() + poly->get_xmax())/2.0;
	double yy = (poly->get_ymin() + poly->get_ymax())/2.0;
	
	/* When the camera is moving. */
//    gr_base.image_affine(base.g_img_mvt(), xx, yy, &ptx, &pty);
	
	center.p_x(xx);
	center.p_y(yy);
    }
}


int Polygon2D::is_pt_in_zone(Vector2D* pr) const {
    static double X[2];
//	affine inv_aff;
    
    if ((poly == NULL) || (pr == NULL))
	return(0);
    else {
//    invertAffine(base.g_img_mvt(), &inv_aff);
//    base.image_affine(inv_aff, pr->g_x(), pr->g_y(), &X[0], &X[1]);
	
	X[0] = pr->g_x();
	X[1] = pr->g_y();
	    
	if ( poly->Space_point_is_inside_polygon(X) == 0 )
	    return(0);
	else
	    return(1);
    }
}


int Polygon2D::is_pt_close_to(Vector2D* pr) const {
    //static double X[2];
    //affine inv_aff;
    
    if ((poly == NULL) || (pr == NULL))
	return(0);
    else {
//    affinity = (pgr->xmax - pgr->xmin) + (pgr->ymax - pgr->ymin);
	double affinity = ((poly->get_xmax() - poly->get_xmin())+
			   (poly->get_ymax() - poly->get_ymin()))/2.;
	double dist = distance(pr);
	if (dist > affinity)
	    return(0);
	else
	    return(1);
    }
}



int Polygon2D::is_pt_within_range(Vector2D* pr, float thres_dist) const {
    //static double X[2];
    
    if ((poly == NULL) || (pr == NULL))
	return(0);
    else {
	float affinity = thres_dist;
	double dist = distance(pr);
	
	if (dist > (double)affinity)
	    return(0);
	else
	    return(1);
    }
}

double Polygon2D::distance(Vector2D* pr) const {

    double    ptx, pty, dist_x, dist_y;
//	affine inv_aff;
  
    if ((poly == NULL) || (pr == NULL))
	return(-1);
    else {
    /* When the camera is moving. */
//    invertAffine(base.g_img_mvt(), &inv_aff);
//    base.image_affine(inv_aff, pr->g_x(), pr->g_y(), &ptx, &pty);
 
	ptx = pr->g_x(); pty = pr->g_y();
	
	if (ptx < poly->get_xmin())
	    dist_x = std::abs(ptx - poly->get_xmin());
	else if (poly->get_xmax() < ptx)
	    dist_x = std::abs(ptx - poly->get_xmax());
	else
	    dist_x = 0;
	
	if (pty < poly->get_ymin())
	    dist_y = std::abs(pty - poly->get_ymin());
	else if (poly->get_ymax() < pty)
	    dist_y = std::abs(pty - poly->get_ymax());
	else
	    dist_y = 0;
	
	return(dist_x + dist_y);
    }
}

void Polygon2D::chgt_ref(void) {
  //polygon_r pgr;
  //int j;
  //double affX, affY;
  //affine aff_new = base.new_affine();

  /*
  center->chgt_ref();
  
  if ((pgr = g_poly()) != NULL) {
    for (j=0; j<pgr->nb_vertices; j++) {
      
      base.image_affine(aff_new, pgr->x0_vertices[j],
			pgr->y0_vertices[j], &affX, &affY);
      pgr->x0_vertices[j] = affX;
      pgr->y0_vertices[j] = affY;
      base.image_affine(aff_new, pgr->x1_vertices[j],
			pgr->y1_vertices[j], &affX, &affY);
      pgr->x1_vertices[j] = affX;
      pgr->y1_vertices[j] = affY;
    }
  }
  */
}

//----------------
// Visualization.
//----------------
void Polygon2D::draw(void) {
  
  if (poly != NULL)
      for (unsigned i=0; i<poly->nb_vertices(); i++) {
	  /*
	  //draw_color_line((int)pgr->x0_vertices[i],(int)pgr->y0_vertices[i],
	  //(int)pgr->x1_vertices[i],(int)pgr->y1_vertices[i],
	  //base.in_img, white_color_pixel);
	  */
    }
}	

}


//----------------
// Display.
//----------------
// std::ostream& operator<<(std::ostream& out, const Geom::Polygon2D&  p) {
//     out<< "\t poly= ";
//     p.print(&out); 
//     out << "pnts";
//     return( out );
// }

