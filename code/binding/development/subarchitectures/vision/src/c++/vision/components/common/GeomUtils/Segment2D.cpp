/** @file Segment2D.cpp
 *  @brief A 2D segment.
 *
 *
 *  @author Somboon Hongeng
 *  @date April 2008
 *  @bug No known bugs.
 */
#include "Segment2D.h"
#include <vision/components/common/SystemUtils/Common.h>

namespace Geom {

    using namespace Common;

    const float Segment2D::DIST_SEGMENT=40.0; // 50 mm (or cm) 
    const int Segment2D::NB_OUT_SEGMENT=3;
    const int Segment2D::WIDTH_SEGMENT=25; //` used to be 40 mm 
    extern const Segment2D VOID_SEGMENT2D=Segment2D(VOID_VECTOR2D,0);

Segment2D::Segment2D(const Segment2D& seg) :   
    ETimer(), Report()
{
    set(seg);
}

Segment2D::Segment2D(const Vector2D p1, unsigned time) {
    p_end(VOID_VECTOR2D);
    p_cosa(0);
    p_sina(0);
    p_nb_out_pt(0);
    
    /* On cree p1. */
    p_start(p1);
    
    /* On etablit les dates par defaut. */
    p_ti(time);
    p_tf(time);
}


Segment2D::Segment2D(const Vector2D pt1, const Vector2D pt2, unsigned t1, unsigned t2)
{
    /* On etablit les temps. */
    //if ((t1<0) || (t2<0))
    //throw user_error(__HERE__, "invlaid times");
    m_ti = t1;
    // if (t2 < 0)
// 	m_tf = GetTime();
//     else
    m_tf = t2;
    
    /* Si les deux points sont trop proches. */
    if (pt1.distance(pt2) < Segment2D::DIST_SEGMENT) // 50 millimeter
    /* On ne cree que le debut du segment. */
    {
	p_start(pt1);
	p_end(VOID_VECTOR2D);
	p_cosa(0);
	p_sina(0);
	p_nb_out_pt(1);
    }
    
    /* Sinon, on cree les deux points. */
    else
    {
	/* On change les points. */
	p_start(pt1);
	p_end(pt2);
	
	/* On calcule la direction. */
	compute_direction();
	p_nb_out_pt(0);
    }
}

Segment2D::~Segment2D()
{
}


Segment2D& Segment2D::operator=(const Segment2D& src)
{
    if (this == &src)
	return(*this);
    set(src);
    return (*this);
}

bool Segment2D::operator==(const Segment2D& seg) const
{
    if ((m_start==seg.g_start()) && (m_end==seg.g_end()))
	return true;
    else
	return false;
}

bool Segment2D::operator!=(const Segment2D& seg) const
{
    if ((m_start==seg.g_start()) && (m_end==seg.g_end()))
	return false;
    else
	return true;
}


void Segment2D::set(const Segment2D& seg) 
{
    p_start(seg.g_start());    
    p_end(seg.g_end());
    p_cosa(seg.g_cosa());
    p_sina(seg.g_sina());
    p_ti(seg.g_ti());
    p_tf(seg.g_tf());
    p_nb_out_pt(seg.g_nb_out_pt());
}


void Segment2D::compute_direction(void)
{   
    if ((m_start == VOID_VECTOR2D) || (m_end == VOID_VECTOR2D))
	throw user_error(__HERE__, "invalid segment");
    p_cosa(m_start.cosinus(m_end));
    p_sina(m_start.sinus(m_end));
}


// Establish a complete segment, if start->p2 is longer than the
// minimum segment length, and assigned (ti,tf) accordingly
void Segment2D::spread_begin(Vector2D p2, unsigned time_init) 
{
    // Le segment doit etre cree. 
    if ((m_start==VOID_VECTOR2D) || (p2==VOID_VECTOR2D))
	throw user_error(__HERE__, "invalid segment points");
    
    /* Si start a deja ete cree au temps 'time_init' = base.top. */
    if ((m_ti == time_init) && (m_ti==GetTime()) ) {// m_ti is up-to-date 
	//cout << "[Segment2D::spread_begin] m_ti uptodate\n";
	throw user_error(__HERE__, "segment is uptodate");
	return; // On ne fait rien. 
    }
    
    /* Si p2 est suffisament eloigne. */
    if (m_start.distance(p2) > Segment2D::DIST_SEGMENT) 
    {
	if (debug())
	    cout << "[Segment2D::spread_begin] m_start->p2 OK, adding m_end=p2\n";
	p_ti(time_init - 1); // On etablit les temps. 
	p_tf(time_init);
	
	m_end = p2; // On initialise le segment. 
	p_cosa(m_start.cosinus(p2));
	p_sina(m_start.sinus(p2));
    }
    else
    {
	if (debug())
	    cout << "[Segment2D::spread_begin] m_start->p2 too small, resting)\n";
	p_ti(time_init); // On etablit les temps. 
	p_tf(time_init);
    }
}

// m_end is either extended or stay the same.
//
// Returns 1, if the endpoint was extended successfully
//         2, if this segment is too ambiguous, due to many "out" points 
int Segment2D::spread_end(Vector2D p2) {
    /* On est dans la phase de prolongement. */
    if ((p2 == VOID_VECTOR2D) || has_valid_points()==false)
	throw user_error(__HERE__, "invalid segment points");

    /* Si p2 est en dehors de la bande. */
    if (is_out(p2) == true) 
    {
	incr_nb_out_pt();
	
	/* S'il y a suffisament de points en dehors. */
	if (g_nb_out_pt() > Segment2D::NB_OUT_SEGMENT) {
	    if (debug()) 
		cout << "[Segment2D::spread_end] to start a new segment)\n";
	    return(2); // La phase de prolongement se finit. 
	}
	
    }
    else 
	m_end = p2; // Sinon on change 'end'.

    m_tf = GetTime(); 
    if (debug()) 
	cout << "[Segment2D::spread_end] keep spreading this segment)\n";
    /* La phase de prolongement continue. */
    return(1);
}


bool Segment2D::has_valid_points() const
{
    if ((m_start == VOID_VECTOR2D) || (m_end == VOID_VECTOR2D))
	return false;
    else
	return true;
}


// returns : true, if the distance to pt is shorter than the length of the segment
//                 or if pt is outside the band
bool Segment2D::is_out(Vector2D pt)
{
    /* On est dans la phase de prolongement. */
    if (pt==VOID_VECTOR2D || has_valid_points()==false)
	throw user_error(__HERE__, "invalid segment points");
    
    /* Si pr rebrousse chemin. */
    if (m_start.distance(m_end) > m_start.distance(pt))
	return(true);
    
    /* Calcul de l'eloignement de pr de la bande. */
    double xx = pt.g_x() - m_start.g_x();
    double yy = pt.g_y() - m_start.g_y();
    double height = std::abs(g_cosa()*yy - xx*g_sina());
    
    /* Si pr est en dehors de la bande. */
    if (height > Segment2D::WIDTH_SEGMENT)
	return(true);
    else
	return(false);
}


double Segment2D::speed(void)
{
    /* Si le segment n'est pas complet. */
    if ( ( has_valid_points()==false) || (g_time()<=0) )
	return(0);
    else
	/* On calcule la vitesse. */
	return( m_start.distance(m_end) / g_time() );
}
  

// Corrects the segment with end point "pt"
int Segment2D::correct(Vector2D pt, unsigned time_f, int time_suspend)
{
    if (pt==VOID_VECTOR2D)
	return(1);
    
    if (m_end==VOID_VECTOR2D)
    {
	spread_begin(pt, time_f);
	return(1);
    }
    
    /* Si le nouveau point est dedans, */
    /* il devient le 'end' du segment. */
    if (is_out(pt) == false)
    {
	m_tf = time_f;
	m_end = pt;
	return(1);
    }

    /* On calcule le temps de suspension de 'this' seg */
    /* et du nouveau point. */
    int time_suspend2 = std::max(time_suspend, ((int)time_f - (int) m_tf - 1));
    
    /* Sinon, le nouveau point est dehors, */
    /* Si la distance au 'end' est faible, */
    /* on met 'pr' dans les pts dehors.    */
    if (m_end.distance(pt) < Segment2D::DIST_SEGMENT) // too short the length, pt is out
    {
	m_tf = time_f;
	incr_nb_out_pt();
	return(1);
    }

    /* Sinon, la distance au 'end' est importante. */
    /* Si les temps sont trop rapproches, ou si    */
    /* le mobile n'a pas ete suspendu.             */
    else if (((m_ti+1) >= time_f) || (time_suspend2 == 0))
    {
	/* On change 'end'. */
	m_tf = time_f;
	m_end = pt;
	return(1);
    }
    /* Si le temps de suspension n'est pas coherent. */
    else if (m_tf > (time_f+time_suspend))
    {
	//gr_base.warning("segment2D::correct", 2);
	std::cout << "[Segment2D::correct] soemthing fishy" << std::endl;
	/* On change 'end'. */
	m_end = pt;
	return(1);
    }
    else
    /* On cree un nouveau segment. */
	return(2);
}


// Calculate the radian angle [-pi,pi] 
// between this segment and 'sr'.
double Segment2D::deviation(const Segment2D seg) const
{
  /* On calcule la valeur de l'angle:      */
  /* cos(sr-t)=cos(sr)cos(t)+sin(sr)sin(t) */
  double val = seg.g_cosa()*g_cosa() + seg.g_sina()*g_sina();

  /* On prend l'arc-cosinus. */
  val = acos(val);

  /* On calcule le signe de l'angle:       */
  /* sin(sr-t)=sin(sr)cos(t)-cos(sr)sin(t) */
  double sig = seg.g_sina()*g_cosa() + seg.g_cosa()*g_sina();

  if (sig < 0)
    return(-val);
  else
    return(val);
}


void Segment2D::find_point(Vector2D& pt, int time)
{
    if (m_start == VOID_VECTOR2D)
	throw user_error(__HERE__, "segment is invalid");
    else if (m_end == VOID_VECTOR2D) 
	pt = m_start;
    else
    {
	/* Sinon, on calcule la projection de 'start'. */
	double dist = (time - m_ti) * speed();
	pt.p_x(m_start.g_x() + dist*g_cosa());   // projection by linear speed
	pt.p_y(m_start.g_y() + dist*g_sina());   // projection by linear speed
    }
}


// Calculates the angle measuring the difference
// between the two segments
void Segment2D::angle_difference(Segment2D seg2, double& cosinus, double& sinus)
{
  /* On calcule le cosinus de l'angle:              */
  /* cos(seg2-seg1)=cos(seg2)cos(seg1)+sin(seg2)sin(seg1) */
  cosinus = seg2.g_cosa()*g_cosa() + seg2.g_sina()*g_sina();

  /* On calcule le sinus de l'angle:                */
  /* sin(seg2-seg1)=sin(seg2)cos(sr1)-cos(seg2)sin(seg1) */
  sinus = seg2.g_sina()*g_cosa() + seg2.g_cosa()*g_sina();
}


//
// Calculate how long the two segments are identical
//
int Segment2D::distinct_time(Segment2D seg2)
{
    //if (seg2 == VOID_VECTOR2D)
//	throw user_error(__HERE__, "invalid segment");

    Vector2D pt1 = m_end;
    unsigned t1;
    int end_ok=0;
    if (pt1 == VOID_VECTOR2D)
    { /* Sinon, on prend le debut de sr1. */
	pt1 = m_start;
	t1 = g_ti();
    }
    else
    { 
	t1 = m_tf;
	end_ok++;
    }
    
    /* Si le point 'fin' de seg2 est non nul. */
    Vector2D pt2 = seg2.g_end();
    unsigned t2;
    if (pt2 == VOID_VECTOR2D)
    {/* Sinon, on prend le debut de sr1. */
	pt2 = seg2.g_start();
	t2 = seg2.g_ti();
    }
    else {
	t2 = seg2.g_tf();
	end_ok++;
    }
    
    /* Si pr1 et pr2 sont differents. */
    if (pt1.distance(pt2) > 1)
	return(std::max((int)t1, (int)t2));
    
    /* Sinon, si on a regarde un des points 'fin'. */
    else if (end_ok == 1)
	return(std::max( (int)g_ti(), (int)seg2.g_ti()));
    
    /* Sinon, si on a regarde les points 'fin'. */
    else if (end_ok == 2)
    {
	/* On regarde les debuts. */
	if (((pt1 = g_start()) != VOID_VECTOR2D) &&
	    ((pt2 = seg2.g_start()) != VOID_VECTOR2D))
	{
	    /* S'ils sont differents. */
	    if (pt1.distance(pt2) > 1)
		return(std::max( (int)g_ti(), (int)seg2.g_ti()));
	}
    }
    return(-1);
}


void Segment2D::draw(IplImage *img) 
{
    if (has_valid_points() == true)
    {
	cvLine(img, cvPoint((int)m_start.g_x(), (int)m_start.g_y()),
	       cvPoint((int)m_end.g_x(), (int)m_end.g_y()),
	       cvScalar(255,255,0), 2);
    }
}


void Segment2D::draw(IplImage *img, CameraProj* pCameraModel,
		     CvScalar lineColor) 
{
    if (has_valid_points() == true)
    {
	Vector2D start_imgPoint 
	    = pCameraModel->projectToImage(Vector3D(m_start.g_x(),
						    m_start.g_y(), 0));
	Vector2D end_imgPoint 
	    = pCameraModel->projectToImage(Vector3D(m_end.g_x(), 
						    m_end.g_y(), 0));
	
	cvLine(img, cvPoint((int)start_imgPoint.g_x(), 
			    (int)start_imgPoint.g_y()),
	       cvPoint((int)end_imgPoint.g_x(), (int)end_imgPoint.g_y()),
	       lineColor, 2);
    }
}



std::ostream& operator<<(std::ostream& out, const Segment2D& seg)
{
    Vector2D pt;
    out<< "[";
    if ((pt=seg.g_start()) != VOID_VECTOR2D)
	out<< "start:" << pt;
    if ((pt=seg.g_end()) != VOID_VECTOR2D)
	out<< ", end:" << pt;
    out<< ", cosa:" << seg.g_cosa();
    out<< ", sina:" << seg.g_sina();
    out<< ", out pts:" << seg.g_nb_out_pt();
    out<< ", ti:" << seg.g_ti();
    out<< ", tf:" << seg.g_tf() << "]";
    return( out );
}



}
