//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syEllMethodSubpixel.hpp"
#include "ImgUtils.h"


#include "syArray.hpp"
#include "syVector2.hpp"
#include "syEllipse.hpp"
#include "syExcept.hpp"


NAMESPACE_CLASS_BEGIN( RTE )

////////////////////////////////////////////////////////////////////////////////
CzEllMethodSubpixel::CzEllMethodSubpixel()
{

}

////////////////////////////////////////////////////////////////////////////////
CzEllMethodSubpixel::~CzEllMethodSubpixel()
{

}

////////////////////////////////////////////////////////////////////////////////
// find ellipses on given image
bool CzEllMethodSubpixel::Subpixel(IplImage *dx, 
                                   IplImage *dy,
                                   CzArray<CzEdgel> &points, 
                                   CzEllipse *ell)
{
		CzArray<CzEdgel> subPixelPoints(10000);

    subPixelPoints.Clear();
    
		double ex,ey,ea,eb,phi;

							for(int i=0; i < (int)points.Size(); i=i+1) 
							{
								CzVector2 dir, normal;
								dir = ell->Tangent(points[i].p); //is more stable than fitting line if the first approximation is accurate
								//dir = atan2((double)GetPx16SC1(dy,points[i].p.x,points[i].p.y),(double)GetPx16SC1(dx,points[i].p.x,points[i].p.y));										
								dir.Normalise();
								normal = CzVector2(-dir.y, dir.x); //less sensitive to gaussian noise
								//normal = CzVector2((double)GetPx16SC1(dx,points[i].p.x,points[i].p.y), (double)GetPx16SC1(dy,points[i].p.x,points[i].p.y));
								//normal.Normalise();
								//Using normal vector, get pixels around points[i].{x,y}
								CzVector2 pointA, pointB, pointC;
								pointB = points[i].p;
								pointA = CzVector2(pointB.x - cvRound(normal.x), pointB.y - cvRound(normal.y));
								pointC = CzVector2(pointB.x + cvRound(normal.x), pointB.y + cvRound(normal.y));

			//					cout << normal.x << " " << cvRound(normal.x) << " " << normal.y << " " << cvRound(normal.y) << endl;

								float bx=(float)pointB.x;
								float ax=(float)pointA.x;
								float cx=(float)pointC.x;
								float by=(float)pointB.y;
								float ay=(float)pointA.y;
								float cy=(float)pointC.y;
								float fa, fb, fc;
								fa = (float)(sqrt(Sqr((float)GetPixelI16(dx, (int)ax,(int)ay)) + Sqr((float)GetPixelI16(dy, (int)ax,(int)ay))));
								fb = (float)(sqrt(Sqr((float)GetPixelI16(dx, (int)bx,(int)by)) + Sqr((float)GetPixelI16(dy, (int)bx,(int)by))));
								fc = (float)(sqrt(Sqr((float)GetPixelI16(dx, (int)cx,(int)cy)) + Sqr((float)GetPixelI16(dy, (int)cx,(int)cy))));
								float minx = bx - 1/2.f*( ((bx-ax)*(bx-ax)*(fb-fc) - (bx-cx)*(bx-cx)*(fb-fa)) / ((bx-ax) * (fb-fc) - (bx-cx)*(fb-fa)) );
								float miny = by - 1/2.f*( ((by-ay)*(by-ay)*(fb-fc) - (by-cy)*(by-cy)*(fb-fa)) / ((by-ay) * (fb-fc) - (by-cy)*(fb-fa)) );
								if(cvRound(normal.x) == 0)
										minx = bx;
								if(cvRound(normal.y) == 0)
										miny = by;
								subPixelPoints.PushBack(CzEdgel(CzVector2(minx,miny)));
							}

							// Fit ellipse using sub-pixel accuracy edges
              CzArray<bool> inlIdx(10000);
							inlIdx.Clear();
							RTE::CzEllipse::FitEllipse(subPixelPoints,ex,ey,ea,eb,phi);
							ell->Set(ex,ey,ea,eb,phi);

							ell->EllipseSupport(subPixelPoints, .1, inlIdx);

//cout<<"Points: "<<subPixelPoints.Size()<<"   Support: "<<ell->dSupport<<endl;

return true;
}


NAMESPACE_CLASS_END()
