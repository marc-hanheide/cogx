/** @file Segment2D.h
 *  @brief A 2D segment.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#ifndef _SEGMENT2D_H_
#define _SEGMENT2D_H_

#include <cmath>
#include "Vector2D.h"
#include <boost/shared_ptr.hpp>
#include "vision/components/common/SystemUtils/ETimer.h"
#include <opencv/cv.h>
#include <iostream>
#include "CameraProj.h"
#include <vision/components/common/SystemUtils/Report.h>

namespace Geom {

    class Segment2D : 
	public ETimer, 
	public Report
{
 private:
    static const int NB_OUT_SEGMENT; // Nbr de points toleres endehors de la bande.
    static const int WIDTH_SEGMENT;  // Largeur de la bande entourant le segment.
    
    Vector2D m_start;     // Point de depart du segment.
    Vector2D m_end;       // Point d'arrive du segment.
    double   m_cosa;      // Cosinus de la direction.
    double   m_sina;      // Sinus de la direction.
    unsigned m_ti;        // Temps initial (creation de 'end').
    unsigned m_tf;        // Temps final.
    int      m_nb_out_pt; // Nbre de points en dehors de la bande.
    
 public:
    static const float DIST_SEGMENT; 

    
    Segment2D(const Segment2D& seg);
    Segment2D(const Vector2D pt1, unsigned t1);
    Segment2D(const Vector2D pt1=VOID_VECTOR2D, 
	      const Vector2D pt2=VOID_VECTOR2D, 
	      unsigned t1=0, unsigned t2=0);
    ~Segment2D(void);
    
    // Operations
    Segment2D& operator=(const Segment2D& src);
    bool operator==(const Segment2D& seg) const;
    bool operator!=(const Segment2D& seg) const;

    Vector2D g_start(void) const { return m_start; }
    Vector2D g_end(void) const { return m_end; }
    int       g_ti(void) const { return m_ti; }
    int       g_tf(void) const { return m_tf; }
    int       g_time(void) const { return (m_tf - m_ti); }
    int       g_nb_out_pt(void) const { return m_nb_out_pt; }
    double    g_cosa(void) const { return m_cosa; }
    double    g_sina(void) const { return m_sina; }
    
    void set(const Segment2D& seg);
    void p_start(const Vector2D pt) { m_start = pt; }
    void p_end(const Vector2D pt) { m_end = pt; }
    void p_nb_out_pt(int ival) { m_nb_out_pt = ival; }
    void p_ti(unsigned t) { m_ti = t; }
    void p_tf(unsigned t) { m_tf = t; }
    void p_time(int t) { m_tf = std::max(0, (int)m_tf+t); }
    void p_cosa(double val) { m_cosa = val; }
    void p_sina(double val) { m_sina = val; }
    
    void incr_time(void) { m_tf++; }
    void incr_nb_out_pt(void) { m_nb_out_pt++; }
    
    
    void g_direction(double& cos, double& sin) { cos = g_cosa();  sin = g_sina(); }
    void compute_direction(void);
    void spread_begin(Vector2D p2, unsigned time_init);
    int  spread_end(Vector2D p2);
    bool has_valid_points() const;

    bool   is_out(Vector2D pt);
    bool   is_valid();

    double speed(void);
    int    correct(Vector2D pt, unsigned time_f, int time_suspend);
    double deviation(const Segment2D seg) const;

    void   find_point(Vector2D& pt, int time);

    void   angle_difference(Segment2D seg2, double& cosinus, double& sinus);
    int    distinct_time(Segment2D seg2);
/*   void   chgt_ref(void); */
    
//    bool is_in_valid();
    
    void   draw(IplImage *img);
    void   draw(IplImage *img, CameraProj* pCameraModel,
		CvScalar lineColor=cvScalar(255,255,0));
    friend std::ostream& operator<<(std::ostream& out, const Segment2D& seg);

/*   friend ostream& operator<<(ostream& out, const Segment2D_r seg); */
    
};

extern const Segment2D VOID_SEGMENT2D;

}

#endif
