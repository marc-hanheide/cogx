//
// = FILENAME
//    HSSRedundantLine2DRep.cc
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1999 Patric Jensfelt
//                  2009 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSRedundantLine2DRep.hh"
#include "HSSutils.hh"

#ifndef DEPEND
#include <float.h>
#endif

namespace HSS {

#define HSS_LINE_DTHRESHOLD   1e-10

RedundantLine2DRep::RedundantLine2DRep()
  :m_Xs(0),
   m_Ys(0),
   m_Xe(0),
   m_Ye(0),
   m_Xc(0),
   m_Yc(0),
   m_Z(0), 
   m_Theta(0),
   m_Alpha(0),
   m_C(0),    
   m_D(0),    
   m_H(0),
   m_R(4,4),
   m_EndPtModel(NO_END_POINTS),
   m_Key(-1),
   m_Weight(0),
   m_Source(NULL)
{
  // Set all elemenst to infinit uncertainty
  for (int i = 0; i < 4; i++)
    for (int j  =0; j < 4; j++)
      m_R(i,j) = DBL_MAX;
}

RedundantLine2DRep::RedundantLine2DRep(double xS, double yS, double xE, double yE) 
  :m_R(4,4),
   m_EndPtModel(NO_END_POINTS),
   m_Key(-1),
   m_Weight(0),
   m_Source(NULL)
{
  setEndPts(xS, yS, xE, yE);
  
  // Set all elemenst to infinit uncertainty
  for (int i = 0; i < 4; i++)
    for (int j  =0; j < 4; j++)
      m_R(i,j) = DBL_MAX;

}

RedundantLine2DRep::RedundantLine2DRep(const RedundantLine2DRep& src)
  :m_R(4,4)
{
  (*this) = src;
}

RedundantLine2DRep::~RedundantLine2DRep()
{
}


void
RedundantLine2DRep::setEndPts(double xS, double yS, double xE, double yE) 
{
  m_Xs = xS;
  m_Ys = yS;
  m_Xe = xE;
  m_Ye = yE;

  m_Xc = 0.5 * (xS + xE);
  m_Yc = 0.5 * (yS + yE);

  // Direction of line
  m_Theta = atan2(yE - yS, xE - xS);

  // Angle of line from origin normal to line
  m_Alpha = m_Theta - M_PI_2;
  if (fabs(HSS::pi_to_pi(atan2(m_Yc,m_Xc) - m_Alpha)) > M_PI_2) {
    m_Alpha += M_PI;
  }

  // Make sure that alpha is (-pi,pi]
  m_Alpha = HSS::pi_to_pi(m_Alpha);

  // Half length of the line
  m_H = 0.5 * sqrt((yE - yS) * (yE - yS) + (xE - xS) * (xE - xS));

  // Perpendicular distance to the line
  m_C = fabs(HSS::distPt2Line(0, 0, m_Xc, m_Yc, m_Theta));

  // Tangential distance to line middle 
  m_D = fabs(HSS::distPt2Line(0, 0, m_Xc, m_Yc, m_Alpha));

  // Make sure that d has right sign as absDistOrig2Line > 0 This is
  // done by checking if the formula involving alpha, theta, c and d
  // holds.
  double tmpX_ = m_C * cos(m_Alpha) + m_D * cos(m_Theta);
  double tmpY_ = m_C * sin(m_Alpha) + m_D * sin(m_Theta);
  if (fabs(HSS::pi_to_pi(atan2(tmpY_, tmpX_) - atan2(m_Yc, m_Xc))) > 
      HSS_LINE_DTHRESHOLD){
    m_D = -m_D;
  }
}

void 
RedundantLine2DRep::setAll(double xS, double yS, double xE, double yE,
                           double xC, double yC, double alpha, double theta, 
                           double c, double d, double h)
{
  m_Xs = xS;
  m_Ys = yS;

  m_Xe = xE;
  m_Ye = yE;

  m_Xc = xC;
  m_Yc = yC;

  m_Alpha = alpha;
  m_Theta = theta;

  m_C = c;
  m_D = d;
  m_H = h;
}

void
RedundantLine2DRep::rotate(double angle)
{
  double tmpX, tmpY, sina, cosa;

  sina = sin(angle);
  cosa = cos(angle);

  // Rotate start point
  tmpX = m_Xs;
  tmpY = m_Ys;
  m_Xs  = tmpX * cosa - tmpY * sina;
  m_Ys  = tmpX * sina + tmpY * cosa;

  // Rotate mid point
  tmpX = m_Xe;
  tmpY = m_Ye;
  m_Xe  = tmpX * cosa - tmpY * sina;
  m_Ye  = tmpX * sina + tmpY * cosa;

  // Rotate end point
  tmpX = m_Xc;
  tmpY = m_Yc;
  m_Xc  = tmpX * cosa - tmpY * sina;
  m_Yc  = tmpX * sina + tmpY * cosa;

  // Rotate alpha angle
  m_Alpha = HSS::zero_to_2pi(m_Alpha + angle);

  // Rotate theta angle
  m_Theta = HSS::zero_to_2pi(m_Theta + angle);
}

}; // namespace HSS

void
HSS::RedundantLine2DRep::print(std::ostream &os)
{
  os << "xS=" << xS() 
     << " yS=" << yS() 
     << " xE=" << xE()
     << " yE=" << yE()
     << " c=" << c()
     << " d=" << d()
     << " h=" << h()
     << " alpha=" << alpha()
     << " theta=" << theta()
     << std::endl;
}

std::ostream&
operator<<(std::ostream &os, const HSS::RedundantLine2DRep& l)
{
  os << "xS=" << l.xS() 
     << " yS=" << l.yS() 
     << " xE=" << l.xE()
     << " yE=" << l.yE()
     << " c=" << l.c()
     << " d=" << l.d()
     << " h=" << l.h()
     << " alpha=" << l.alpha()
     << " theta=" << l.theta()
     << " em=" << l.getEndPointModel()
     << " w=" << l.getWeight();
  return os;
}

