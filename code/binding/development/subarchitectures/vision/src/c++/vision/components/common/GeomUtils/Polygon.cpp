/** @file  Polygon.cpp
 *  @brief An arcs-structure of a polygon (POLYGONARC).   
 *
 *  @author Somboon Hongeng
 *  @date March 2007
 *  @bug No known bugs.
 */
#include <iostream>
#include "Polygon.h"

namespace Geom {

#define SMALL 0.01
#define VERY_SMALL 0.05 


POLYGONARC::POLYGONARC() {
    flag_max = 0;
    xmin = 0.0;    xmax = 0.0;
    ymin = 0.0;    ymax = 0.0;
}

POLYGONARC::POLYGONARC(const POLYGONARC& polyarc) {
    flag_max = 0;
    for (unsigned i=0; i<polyarc.nb_vertices(); i++) {
	v0.push_back(polyarc.get_v0(i));
	v1.push_back(polyarc.get_v1(i));
    }
    compute_bbox();
}

POLYGONARC::~POLYGONARC() {
    v0.clear();
    v1.clear();
}



void POLYGONARC::append_vertice(int idx, double x, double y) {
    if (idx == 0)
	v0.push_back(Vector2D(x,y));
    else if (idx == 1)
	v1.push_back(Vector2D(x,y));
}

const Vector2D& POLYGONARC::get_v0(int idx) const {
    if ((idx>=0) && (idx< (int)v0.size()))
	return v0[idx];
    else {
	std::cerr << "POLYGONARC::v0 index out out bound\n"; 
	abort();
    }
}

const Vector2D& POLYGONARC::get_v1(int idx) const {
    if ((idx>=0) && (idx<(int)v1.size()))
	return v1[idx];
    else {
	std::cerr << "POLYGONARC::v1, index out out bound\n"; 
	abort();
    }
}

void POLYGONARC::compute_bbox() {
    if (v0.size() < 0) {
	std::cerr << "POLYGONARC::compute_bbox, Null inpu polygon\n";
	abort();
    }
    
    float x_min = v0[0].g_x();    
    float y_min = v0[0].g_y();
    float x_max = v0[0].g_x();    
    float y_max = v0[0].g_y();

    for (unsigned i=0; i<nb_vertices(); i++) {
	x_min = std::min( v0[i].g_x(), x_min );
	x_min = std::min( v1[i].g_x(), x_min );
	y_min = std::min( v0[i].g_y(), y_min );
	y_min = std::min( v1[i].g_y(), y_min );
	x_max = std::max( v0[i].g_x(), x_max );
	x_max = std::max( v1[i].g_x(), x_max );
	y_max = std::max( v0[i].g_y(), y_max );
	y_max = std::max( v1[i].g_y(), y_max );
    }

    xmin = x_min;
    ymin = y_min;
    xmax = x_max;
    ymax = y_max;
}

// 
void POLYGONARC::Space_get_segment_from_poly(int num,SEGMENT *seg) {
    seg->debut[0]=v0[num].g_x();
    seg->debut[1]=v0[num].g_y();
    seg->fin[0]=v1[num].g_x();
    seg->fin[1]=v1[num].g_y();
}


void POLYGONARC::Space_min_max_segment( SEGMENT seg, double *borne ) {
    borne[0]=(seg.debut[0]<seg.fin[0])?seg.debut[0]:seg.fin[0];
    borne[1]=(seg.debut[0]>seg.fin[0])?seg.debut[0]:seg.fin[0];
    borne[2]=(seg.debut[1]<seg.fin[1])?seg.debut[1]:seg.fin[1];
    borne[3]=(seg.debut[1]>seg.fin[1])?seg.debut[1]:seg.fin[1];
}



// double borne[4]
int POLYGONARC::Space_min_max_polygon(double *borne) {
    double  b1[4], b2[4];
    
    if(nb_vertices()==0)return(-1);
    
    if(flag_max) {
	borne[0]=xmin;
	borne[1]=xmax;
	borne[2]=ymin;
	borne[3]=ymax;
	return(1);
    }

    SEGMENT seg;
    Space_get_segment_from_poly( 0, &seg );
    Space_min_max_segment(seg,b1);

    for(unsigned i=1;i<nb_vertices();i++) {
	Space_get_segment_from_poly(i,&seg);
	Space_min_max_segment(seg,b2);
	b1[0]=(b2[0]<b1[0])?b2[0]:b1[0];
	b1[1]=(b2[1]>b1[1])?b2[1]:b1[1];
	b1[2]=(b2[2]<b1[2])?b2[2]:b1[2];
	b1[3]=(b2[3]>b1[3])?b2[3]:b1[3];
    }
    for(int i=0;i<4;i++)
	borne[i]=b1[i];
    
    flag_max=1;
    xmin=borne[0];
    xmax=borne[1];
    ymin=borne[2];
    ymax=borne[3];
    return(1);
}


// double X[2]
int POLYGONARC::Space_point_is_on_segment(double *X,SEGMENT seg) {
    double X2[2],X12[2];
    double x12[2],x2[2];
    double norm,norm1,d1,d2;
    int i;
    
    d1=(X[0]-seg.debut[0]);
    d2=(X[1]-seg.debut[1]);
    if(sqrt(d1*d1+d2*d2)<=VERY_SMALL)return(1);

    d1=(X[0]-seg.fin[0]);
    d2=(X[1]-seg.fin[1]);
    if(sqrt(d1*d1+d2*d2)<=VERY_SMALL)return(1);
    
    X12[0]=seg.fin[0]-seg.debut[0];
    X12[1]=seg.fin[1]-seg.debut[1];
    
    norm=sqrt(X12[0]*X12[0]+X12[1]*X12[1]);
    for(i=0;i<2;i++)x12[i]=X12[i]/norm;

    X2[0]=X[0]-seg.debut[0];
    X2[1]=X[1]-seg.debut[1];
    
    norm=sqrt(X2[0]*X2[0]+X2[1]*X2[1]);
    for(i=0;i<2;i++)x2[i]=X2[i]/norm;
    
    norm1=x2[0]*x12[1]-x2[1]*x12[0];
    
    if(fabs(norm1)>VERY_SMALL)return(0);
    
    norm=1000;
    if(fabs(X12[0])>=fabs(X12[1]) && fabs(X12[0])>VERY_SMALL)
	norm=X2[0]/X12[0];
    if(fabs(X12[1])>=fabs(X12[0]) && fabs(X12[1])>VERY_SMALL)
	norm=X2[1]/X12[1];
    
    if(norm>=0 && norm <=1)return(1);
    return(0);
}


int POLYGONARC::Space_inter_segment_segment(SEGMENT seg1,SEGMENT seg2,double *u1) {
    double x1,y1,x2,y2,X1,Y1,X2,Y2;
    double delt,lam,alp,u11,u2,x21,y21;
    double XM1,YM1,XM2,YM2,xm1,ym1,xm2,ym2;
    
    x1=seg1.debut[0];y1=seg1.debut[1];
    x2=seg1.fin[0];y2=seg1.fin[1];
    xm1=x2;XM1=x1;if(x2>x1){XM1=x2;xm1=x1;}
    ym1=y2;YM1=y1;if(y2>y1){YM1=y2;ym1=y1;}
    
    
    X1=seg2.debut[0];Y1=seg2.debut[1];
    X2=seg2.fin[0];Y2=seg2.fin[1];
    xm2=X2;XM2=X1;if(X2>X1){XM2=X2;xm2=X1;}
    ym2=Y2;YM2=Y1;if(Y2>Y1){YM2=Y2;ym2=Y1;}
    
    if(XM1 <xm2 || XM2 <xm1 || YM1 < ym2 || YM2 <ym1)return(0);
    
    x21=(x1-X1)*(x1-X1);
    y21=(y1-Y1)*(y1-Y1);
    u11=sqrt(x21+y21);
    
    x21=(x2-X2)*(x2-X2);
    y21=(y2-Y2)*(y2-Y2);
    u2=sqrt(x21+y21);
    
    if(u11<=VERY_SMALL && u2<=VERY_SMALL)return(0);
    
    x21=(x1-X2)*(x1-X2);
    y21=(y1-Y2)*(y1-Y2);
    u11=sqrt(x21+y21);

    x21=(x2-X1)*(x2-X1);
    y21=(y2-Y1)*(y2-Y1);
    u2=sqrt(x21+y21);

    if(u11<=VERY_SMALL && u2<=VERY_SMALL)return(0);
    
    delt=(x2-x1)*(Y1-Y2)-(y2-y1)*(X1-X2);
    
    if(fabs(delt) <VERY_SMALL)return(0);
    
    lam=((X1-x1)*(Y1-Y2)-(Y1-y1)*(X1-X2))/delt;
    if(lam <0 || lam >1)return(0);
    alp=((x2-x1)*(Y1-y1)-(y2-y1)*(X1-x1))/delt;
    if(alp <0 || alp >1)return(0);
    
    u1[0]=x1+lam*(x2-x1);
    u1[1]=y1+lam*(y2-y1);
    return(1);
}

double POLYGONARC::Space_distance(double *X1,double *X2) {
    double d1=(X1[0]-X2[0]);
    double d2=(X1[1]-X2[1]);
    return(sqrt(d1*d1+d2*d2));
}


int POLYGONARC::Space_is_odd(int nb) {
    double f=(int)(nb/2.);
    if(2*f-nb!=0)return(1);
    return(0);
}


// double X[2]
int POLYGONARC::Space_point_is_inside_polygon(double *X)  {
    double LMAX;
    
    double d, u1[2],decal;
    SEGMENT  seg,segp;
    int nbi; /* nombre d'intersection */
    int nbia, nbu;
    
    double borne[4];
    Space_min_max_polygon(borne);
    if(X[0]<borne[0]-VERY_SMALL || X[0]>borne[1]+VERY_SMALL || 
       X[1]<borne[2]-VERY_SMALL || X[1]>borne[3]+VERY_SMALL)  {
	return(0);
    }
    
    LMAX=borne[1]-borne[0];
    if(borne[3]-borne[2]>LMAX)LMAX=borne[3]-borne[2];
    LMAX*=3;
    
    for(unsigned i=0; i<nb_vertices(); i++) {
	Space_get_segment_from_poly(i,&seg);
	if(Space_point_is_on_segment(X,seg)) {
	    return(2);
	}
    }

    /* printf (" lmax %d",(int)LMAX);*/
    d = LMAX;
    nbia= -1;
  
    seg.debut[0]=X[0]; seg.debut[1]=X[1];
    seg.fin[0]=X[0]; seg.fin[1]=X[1]+d;
    
    decal=0;
    
 again:

    nbia++;
    
    if(nbia>1000) return(1);
    seg.fin[0]=X[0]+d*cos(decal);
    seg.fin[1]=X[1]+d*sin(decal);
    
    nbi=0;
    
    for(unsigned i=0; i<nb_vertices(); i++)    {
	
	Space_get_segment_from_poly(i,&segp);
	nbu = Space_inter_segment_segment(seg,segp,u1);
	nbi+=nbu;
	
	/*si on trouve un point d'intersection trop proche
	  d'un sommet de polygone on repart au debut en decalant
	  le x,y final du segment */
	
	if( (Space_distance(u1,segp.debut) <SMALL ||
	     Space_distance(u1,segp.fin) <SMALL) && nbu !=0)
	{
	    decal+=0.01;
	    goto again;
	}
	
    }
    if(Space_is_odd(nbi)) {
	return(1);
    }
    return(0);
}


}
