/**
 * @file mxDrawTools.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef MXDRAW_TOOLS_H_
#define MXDRAW_TOOLS_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include "mxCvTools.h"
#include "stdint.h"

namespace mx
{

	static const uint16_t SYM_NA  = 0x0000;
	static const uint16_t SYM_POINT = 0x0001;
	static const uint16_t SYM_CROSS = 0x0002;
	static const uint16_t SYM_DOT = 0x0004;
	static const uint16_t SYM_BOX = 0x0008;
	static const uint16_t SYM_CIRCLE = 0x0010;
	static const uint16_t OPT_NA = 0x0000;
	static const uint16_t OPT_ARROW = 0x0100;

	template <class T>
	inline T clip ( T v ) {if ( v > 0xFF ) return 0xFF; else if ( v < 0 ) return 0; else return v;}


	void DrawWrite ( IplImage *pImg, const char* pText, const CvPoint &rPosition, const CvScalar &rColor, double fScale = 1., int iFontFace = CV_FONT_HERSHEY_PLAIN );
	inline void DrawWrite ( IplImage *pImg, const char* pText, const CvPoint2D64f &rPosition, const CvScalar &rColor, double fScale = 1., int iFontFace = CV_FONT_HERSHEY_PLAIN )
	{DrawWrite ( pImg, pText, cvPoint ( rPosition ), rColor, fScale, iFontFace );}

	inline void DrawWriteInt ( IplImage *pImg, int iInteger, const CvPoint &rPosition, const CvScalar &rColor, double fScale = 1., int iFontFace = CV_FONT_HERSHEY_PLAIN )
	{
		char pText[16]; sprintf ( pText, "%i", iInteger );
		DrawWrite ( pImg, pText, rPosition, rColor, fScale, iFontFace );
	}
	inline void DrawWriteInt ( IplImage *pImg, int iInteger, const CvPoint2D64f &rPosition, const CvScalar &rColor, double fScale = 1., int iFontFace = CV_FONT_HERSHEY_PLAIN )
	{
		char pText[16]; sprintf ( pText, "%i", iInteger );
		DrawWrite ( pImg, pText, cvPoint ( rPosition ), rColor, fScale, iFontFace );
	}
	void DrawPoint ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor );
	void DrawDot ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 1 );
	void DrawCross ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 3 );
	void DrawBox ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 3 );
	void DrawCircle ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 3 );
	void DrawRectangle ( IplImage *pImg, const CvRect &rRect, const CvScalar &rColor );
	void DrawArrow ( IplImage *pImg, const CvPoint &rPoint, double dDirection, const CvScalar &rColor, int iSize = 10 );

	void DrawLine ( IplImage *pImg, const CvPoint &rPointA, const CvPoint &rPointB, const CvScalar &rColor, uint16_t defOptions = OPT_NA, int iSize  = 10 );
	inline void DrawLine ( IplImage *pImg, const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB, const CvScalar &rColor, uint16_t defOptions = OPT_NA, int iSize  = 10 )
	{DrawLine ( pImg, cvPoint ( rPointA ), cvPoint ( rPointB ), rColor, defOptions, iSize );}

	inline void DrawClosure ( IplImage *pImg, const CvPoint *pPoints, uint iNrOfPoints, const CvScalar &rColor, uint16_t defOptions = OPT_NA, int iSize  = 10 )
	{
		for ( uint i = 0; i < iNrOfPoints-1; i++ )
		{
			DrawLine ( pImg, pPoints[i], pPoints[i+1], rColor, defOptions, iSize );
		}
		DrawLine ( pImg, pPoints[iNrOfPoints-1], pPoints[0], rColor, defOptions, iSize ) ;
	}

	inline void DrawClosure ( IplImage *pImg, const CvPoint2D64f *pPoints, uint iNrOfPoints, const CvScalar &rColor, uint16_t defOptions = OPT_NA, int iSize  = 10 )
	{
		for ( uint i = 0; i < iNrOfPoints-1; i++ )
		{
			DrawLine ( pImg, pPoints[i], pPoints[i+1], rColor, defOptions, iSize );
		}
		DrawLine ( pImg, pPoints[iNrOfPoints-1], pPoints[0], rColor, defOptions, iSize );
	}

	void DrawSymbol ( IplImage *pImg, uint16_t defType, const CvPoint &rPoint, const CvScalar &rColor, int iSize = 3 );
	inline void DrawSymbol ( IplImage *pImg, uint16_t symType, const CvPoint2D64f &rPoint, const CvScalar &rColor, int iSize = 3 )
	{DrawSymbol ( pImg, symType, cvPoint ( rPoint ), rColor, iSize );}
	inline void DrawSymbol ( IplImage *pImg, uint16_t symType, const CvPoint2D32f &rPoint, const CvScalar &rColor, int iSize = 3 )
	{DrawSymbol ( pImg, symType, cvPoint ( rPoint ), rColor, iSize );}


	inline void YUVToBGR ( unsigned char* pYUV, CvScalar* pBGR )
	{
		int y,u,v;

		y = pYUV[0];
		v = pYUV[1]*2 - 255;
		u = pYUV[2]*2 - 255;

		pBGR->val[0]  = clip<int> ( y + v );
		pBGR->val[1]  = clip<int> ( ( int ) ( y - 0.51*u - 0.19*v ) );
		pBGR->val[2]  = clip<int> ( y + u );
	}


}

#endif //MXDRAW_TOOLS_H_
