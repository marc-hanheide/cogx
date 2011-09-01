//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"
#include "syEllipse.hpp"

NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
// constructor
CzEllipse::CzEllipse()
{
   Set(0., 0., 0., 0., 0.);
}

////////////////////////////////////////////////////////////////////////////////
// constructor
CzEllipse::CzEllipse(double x_in, double y_in, double a_in, double b_in, double phi_in)
{
   Set(x_in, y_in, a_in, b_in, phi_in);
}

////////////////////////////////////////////////////////////////////////////////
// Set ellipse parameters
void CzEllipse::Set(double x_in, double y_in, double a_in, double b_in, double phi_in)
{
   dX = x_in;
   dY = y_in;
   dA = a_in;
   dB = b_in;
   dPhi = phi_in;
   iMarkerID = 0;
   bEllipseOK = false;
   dFitError = 0.;
   dSupport = 0.; 

   // these are frequently needed by Distance(), so let's cache them
   a2 = Sqr(dA);
   a4 = Sqr(a2);
   b2 = Sqr(dB);
   b4 = Sqr(b2);
}

////////////////////////////////////////////////////////////////////////////////
// Compute support
unsigned CzEllipse::EllipseSupport(CzArray<CzEdgel> &points, double inlDist, CzArray<bool> &inlIdx)
{
   double co = cos(-dPhi), si = sin(-dPhi), n, d, dist;
   unsigned z;
   unsigned nbInl=0;

   CzVector2 p, q;

   dFitError = 0.;
   inlIdx.Resize(points.Size());

   for(z=0; z<points.Size(); z++)
   {
      // transform point in image coords to ellipse coords
      // Note that this piece of code is called often.
      // Implementing this explicitely here is faster than using
      // TransformToEllipse(), as cos and sin only need to be evaluated once.
      //p = Vector2(points[z].x,points[z].y);
      p=points[z].p;
      p.x -= dX;
      p.y -= dY;
      q.x = co*p.x - si*p.y;
      q.y = si*p.x + co*p.y;
      // calculate absolute distance to ellipse
      if(IsZero(q.x) && IsZero(q.y))
         dist = dB;
      else
      {
         x2 = Sqr(q.x);
         y2 = Sqr(q.y);
         n = fabs(x2/a2 + y2/b2 - 1.);
         d = 2.*sqrt(x2/a4 + y2/b4);
         dist = n/d;
      }
      if(dist <= inlDist)
      {
         inlIdx[z]=true;
         nbInl++;
         dFitError += dist;
      }
      else
         inlIdx[z]=false;
   }

   double circumference = EllipseCircumference(dA, dB);

//cout<<"nbInl:"<<nbInl<<"  circumference:"<<circumference<<endl;

   if (nbInl>0 && !IsZero(circumference))
   {
      dFitError /= (double)nbInl;
      dSupport = (double)nbInl / circumference;
   }
   else
   {
      dFitError = DBL_MAX;
      dSupport=0;
   }

   return nbInl;
}


////////////////////////////////////////////////////////////////////////////////
// Fit ellipse using given edgels.
bool CzEllipse::FitEllipse(CzArray<CzEdgel> &ps, double &x, double &y, double &a, double &b, double &phi)
{
   unsigned i;
   CvPoint2D32f *points = 0;
   CvBox2D params;

   points = new CvPoint2D32f[ps.Size()];
   if (points == 0)
      throw CzExcept(__HERE__, "Could not create class CvPoint2D32f! Out of memory?");
   assert(points != 0);
   for(i = 0; i <ps.Size(); i++)
   {
      points[i].x = (float) ps[i].p.x;
      points[i].y = (float) ps[i].p.y;
   }

   cvFitEllipse(points, ps.Size(), &params);
   x = params.center.x;
   y = params.center.y;
   // box size is double the axis lengths
   a = params.size.width/2.;
   b = params.size.height/2.;
   // note: the angle returned is in degrees!
   phi = ScaleAngle_0_2pi(params.angle*M_PI/180.);
   // note: for unknown reasons sometimes a < b!
   if(a < b)
   {
      swap(a, b);
      phi = ScaleAngle_0_2pi(phi + M_PI_2);
   }
   delete[] points;
   return true;
}

////////////////////////////////////////////////////////////////////////////////
// Tangent vector at given ellipse point.
// Always points counterclockwise.
CzVector2 CzEllipse::Tangent(const CzVector2 &p)
{
   return Rotate(TangentCentAxPar(TransformToEllipse(p)), dPhi);
}

////////////////////////////////////////////////////////////////////////////////
// Tangent vector at given ellipse point, for centered, axis-parallel ellipse.
// Always points counterclockwise.
CzVector2 CzEllipse::TangentCentAxPar(const CzVector2 &p)
{
   double t = atan2(dA*p.y, dB*p.x);
   return CzVector2(-dA*sin(t), dB*cos(t));
}

////////////////////////////////////////////////////////////////////////////////
// Calculate ellipse parameters x/y/A/B/phi from conic equation Ax^2+Bxy+Cy^2....+F=0
bool CzEllipse::computeAndSetGeomFromConic(double Ac, double Bc, double Cc, double Dc, double Ec, double Fc) {

   double BcBcAcCc = (Bc*Bc - Ac*Cc);
   if (BcBcAcCc == 0) {
      return false;
   }
   double x0 = (Cc*Dc - Bc*Ec) / BcBcAcCc;
   double y0 = (Ac*Ec - Bc*Dc) / BcBcAcCc;
   double a0, a0_2, b0, b0_2, phi0;
   double SqrAcCc4BcBc = Sqr(Ac-Cc) + 4*Bc*Bc;
   if (SqrAcCc4BcBc < 0) {
      return false;
   }
   a0_2 = (2*(Ac*Ec*Ec + Cc*Dc*Dc + Fc*Bc*Bc - 2*Bc*Dc*Ec - Ac*Cc*Fc)) / ((Bc*Bc - Ac*Cc)*((sqrt(SqrAcCc4BcBc)) - (Ac+Cc)));
   b0_2 = (2*(Ac*Ec*Ec + Cc*Dc*Dc + Fc*Bc*Bc - 2*Bc*Dc*Ec - Ac*Cc*Fc)) / ((Bc*Bc - Ac*Cc)*(-(sqrt(SqrAcCc4BcBc)) - (Ac+Cc)));
   if ((a0_2 < 0) || (b0_2 < 0)) {
      return false;
   }
   a0 = sqrt(a0_2);
   b0 = sqrt(b0_2);
   
   //	double tempPhi=phi;
   phi0=0;
   if (Bc == 0) {
      if (Ac < Cc) {
         //cout << "1" << endl;
         phi0=M_PI_2;
      }
   } else {
      double z = (Ac-Cc) / (2*Bc);
      if (Ac < Cc) {
         phi0=0.5*(atan(1./z));	
         //cout << "m2" << endl;		
      }
      if (Ac > Cc) {
         phi0=M_PI_2 + 0.5*(atan(1./z));
         //cout << "m3" << endl;
      }
   }

   dX = x0;
   dY = y0;
   dA = a0;
   dB = b0;

   //phi = ScaleAngle_0_2pi(phi0);
   if (dA < dB) {
      swap(dA, dB);
      //phi = ScaleAngle_0_2pi(phi + M_PI_2);
   }

   //phi=tempPhi;
   
   return true;
}


NAMESPACE_CLASS_END()
