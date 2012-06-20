	/**
 * @file mxDrawTools.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include "mxDrawTools.h"

namespace mx
{

	void DrawWrite ( IplImage *pImg, const char* pText, const CvPoint &rPosition, const CvScalar &rColor, double fScale, int iFontFace )
	{
		CvFont tFont;
		cvInitFont ( &tFont, iFontFace, fScale, fScale );
		cvPutText ( pImg, pText, rPosition, &tFont, rColor );
	}

	void DrawPoint ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor )
	{
		if ( ( rPoint.x >= pImg->width ) || ( rPoint.x  < 0 ) ||
		        ( rPoint.y >= pImg->height ) || ( rPoint.y < 0 ) )
		{
			return;
		}
		cvSet2D ( pImg, rPoint.y, rPoint.x, rColor );
	};

	void DrawDot ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize )
	{
		if ( ( rPoint.x + iSize >= pImg->width ) || ( rPoint.x - iSize < 0 ) ||
		        ( rPoint.y + iSize >= pImg->height ) || ( rPoint.y - iSize < 0 ) )
		{
			return;
		}
		for ( int i = -iSize; i <= iSize; i++ )
		{
			for ( int j = -iSize; j <= iSize; j++ )
			{
				cvSet2D ( pImg, rPoint.y+i, rPoint.x+j, rColor );
			}
		}
	};

	void DrawCross ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize )
	{
		if ( ( rPoint.x + iSize >= pImg->width ) || ( rPoint.x - iSize < 0 ) ||
		        ( rPoint.y + iSize >= pImg->height ) || ( rPoint.y - iSize < 0 ) )
		{
			return;
		}
		CvPoint pt1 = cvPoint ( rPoint.x + iSize, rPoint.y + iSize );
		CvPoint pt2 = cvPoint ( rPoint.x - iSize, rPoint.y - iSize );
		CvPoint pt3 = cvPoint ( rPoint.x - iSize, rPoint.y + iSize );
		CvPoint pt4 = cvPoint ( rPoint.x + iSize, rPoint.y - iSize );
		cvLine ( pImg, pt1, pt2, rColor );
		cvLine ( pImg, pt3, pt4, rColor );
	};
	void DrawBox ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize )
	{
		if ( ( rPoint.x + iSize >= pImg->width ) || ( rPoint.x - iSize < 0 ) ||
		        ( rPoint.y + iSize >= pImg->height ) || ( rPoint.y - iSize < 0 ) )
		{
			return;
		}
		CvPoint pt1 = cvPoint ( rPoint.x - iSize, rPoint.y - iSize );
		CvPoint pt2 = cvPoint ( rPoint.x + iSize, rPoint.y + iSize );
		cvRectangle ( pImg, pt1, pt2, rColor );
	}
	void DrawCircle ( IplImage *pImg, const CvPoint &rPoint, const CvScalar &rColor, int iSize )
	{
		if ( ( rPoint.x + iSize >= pImg->width ) || ( rPoint.x - iSize < 0 ) ||
		        ( rPoint.y + iSize >= pImg->height ) || ( rPoint.y - iSize < 0 ) )
		{
			return;
		}
		cvCircle ( pImg, rPoint, iSize, rColor );
	}

	void DrawRectangle ( IplImage *pImg, const CvRect &rRect, const CvScalar &rColor )
	{
		cvRectangle ( pImg,
		              cvPoint ( rRect.x, rRect.y ),
		              cvPoint ( rRect.x+rRect.width, rRect.y+rRect.height ),
		              rColor );
	}

	void DrawLine ( IplImage *pImg, const CvPoint &rPointA, const CvPoint &rPointB, const CvScalar &rColor, uint16_t defOptions, int iSize )
	{
		cvLine ( pImg, rPointA, rPointB, rColor );
		if ( defOptions == OPT_ARROW )
		{
			double dx = rPointA.x - rPointB.x;
			double dy = rPointA.y - rPointB.y;
			double dDirection = atan2 ( dy,dx );
			DrawArrow ( pImg, rPointB, dDirection, rColor, iSize );
		}
	}

	void DrawArrow ( IplImage *pImg, const CvPoint &rPoint, double dDirection, const CvScalar &rColor, int iSize )
	{
		const double ArrowOpen = CV_PI/180. * 20.;
		int dx1 = cvRound ( cos ( dDirection + ArrowOpen ) * ( double ) iSize );
		int dy1 = cvRound ( sin ( dDirection + ArrowOpen ) * ( double ) iSize );
		int dx2 = cvRound ( cos ( dDirection - ArrowOpen ) * ( double ) iSize );
		int dy2 = cvRound ( sin ( dDirection - ArrowOpen ) * ( double ) iSize );
		CvPoint pt1 = cvPoint ( rPoint.x + dx1, rPoint.y + dy1 );
		CvPoint pt2 = cvPoint ( rPoint.x + dx2, rPoint.y + dy2 );
		cvLine ( pImg, rPoint, pt1, rColor );
		cvLine ( pImg, rPoint, pt2, rColor );
	}

	void DrawSymbol ( IplImage *pImg, uint16_t symType, const CvPoint &rPoint, const CvScalar &rColor, int iSize )
	{
		if ( symType & SYM_POINT )
			DrawPoint ( pImg, rPoint, rColor );
		if ( symType & SYM_CROSS )
			DrawCross ( pImg, rPoint, rColor, iSize );
		if ( symType & SYM_CIRCLE )
			DrawCircle ( pImg, rPoint, rColor, iSize );
		if ( symType & SYM_DOT )
			DrawDot ( pImg, rPoint, rColor, 1 );
		if ( symType & SYM_BOX )
			DrawBox ( pImg, rPoint, rColor, iSize );
	}

}
