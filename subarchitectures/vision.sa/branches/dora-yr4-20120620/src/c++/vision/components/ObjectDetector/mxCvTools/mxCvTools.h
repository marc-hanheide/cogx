/**
 * @file mxCvTools
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef MXCVTOOLS_H
#define MXCVTOOLS_H

#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

inline void cvPrint ( CvMat *pMatix, const char *pInfo = NULL )
{
	if ( pInfo != NULL )
	{
		printf ( "*** %s  [%i x %i] ***\n", pInfo, pMatix->rows, pMatix->cols );
	}
	for ( int row = 0; row < pMatix->rows; row++ )
	{
		for ( int col = 0; col < pMatix->cols; col++ )
		{
			printf ( " %-12.4f", cvGet2D ( pMatix, row, col ).val[0] );
		}
		printf ( "\n" );
	}
}

inline void cvMatCopy ( CvMat *pSrc, CvMat *pDes )
{
	int rows = ( pDes->rows < pSrc->rows?pDes->rows:pSrc->rows ) ;
	int cols = ( pDes->cols < pSrc->cols?pDes->cols:pSrc->cols );
	for ( int row = 0; row < rows; row++ )
	{
		for ( int col = 0; col < cols; col++ )
		{
			cvSetReal2D ( pDes, row, col, cvmGet ( pSrc, row, col ) );
		}
	}
}

inline  CvPoint  cvPoint ( const CvPoint2D64f &rPoint )
{
	CvPoint ipt;
	ipt.x = cvRound ( rPoint.x );
	ipt.y = cvRound ( rPoint.y );

	return ipt;
}
inline  CvPoint  cvPoint ( const CvPoint2D32f &rPoint )
{
	CvPoint ipt;
	ipt.x = cvRound ( rPoint.x );
	ipt.y = cvRound ( rPoint.y );

	return ipt;
}

inline  CvPoint2D64f  cvPoint2D64f ( const CvPoint &rPoint )
{
	CvPoint2D64f ipt;
	ipt.x =  rPoint.x ;
	ipt.y =  rPoint.y ;

	return ipt;
}

inline  CvPoint2D64f  cvPoint2D64f ( const CvPoint2D32f &rPoint )
{
	CvPoint2D64f ipt;
	ipt.x =  rPoint.x ;
	ipt.y =  rPoint.y ;

	return ipt;
}

inline bool IsPointIn(const CvPoint &rPoint, const CvSize &rSize){
	return ((rPoint.x >= 0) && (rPoint.x < rSize.width) && (rPoint.y >= 0) && (rPoint.y < rSize.height)); 
}

inline void cvDiff(const CvPoint3D64f &rPointA, const CvPoint3D64f &rPointB, CvPoint3D64f *pPointDes)
{
	pPointDes->x =  rPointA.x - rPointB.x;
	pPointDes->y =  rPointA.y - rPointB.y;
	pPointDes->z =  rPointA.z - rPointB.z;
}
inline void cvDiff(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB, CvPoint2D64f *pPointDes)
{
	pPointDes->x =  rPointA.x - rPointB.x;
	pPointDes->y =  rPointA.y - rPointB.y;
}


inline CvPoint3D64f cvDiff(const CvPoint3D64f &rPointA, const CvPoint3D64f &rPointB)
{
	CvPoint3D64f ipt;
	cvDiff(rPointA, rPointB, &ipt);
	return ipt;
}

inline CvPoint2D64f cvDiff(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB)
{
	CvPoint2D64f ipt;
	cvDiff(rPointA, rPointB, &ipt);
	return ipt;
}

inline void cvAdd(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB, CvPoint2D64f *pPointDes)
{
	pPointDes->x =  rPointB.x + rPointA.x;
	pPointDes->y =  rPointB.y + rPointA.y;
}

inline void cvAdd(const CvPoint3D64f &rPointA, const CvPoint3D64f &rPointB, CvPoint3D64f *pPointDes)
{
	pPointDes->x =  rPointB.x + rPointA.x;
	pPointDes->y =  rPointB.y + rPointA.y;
	pPointDes->z =  rPointB.z + rPointA.z;
}

inline CvPoint3D64f cvAdd(const CvPoint3D64f &rPointA, const CvPoint3D64f &rPointB)
{
	CvPoint3D64f ipt;
	cvAdd(rPointA, rPointB, &ipt);
	return ipt;
}

inline CvPoint2D64f cvAdd(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB)
{
	CvPoint2D64f ipt;
	cvAdd(rPointA, rPointB, &ipt);
	return ipt;
}

inline void cvMult(CvPoint3D64f *pPoint, double scalar)
{
	pPoint->x *= scalar;
	pPoint->y *= scalar;
	pPoint->z *= scalar;
}

inline CvPoint3D64f cvMult(const CvPoint3D64f &rPoint, double scalar)
{
	CvPoint3D64f ipt = rPoint;
	cvMult(&ipt, scalar);
	return ipt;
}

inline double cvNorm(const CvPoint3D64f &rPoint)
{
	return cvSqrt(rPoint.x * rPoint.x + rPoint.y * rPoint.y + rPoint.z * rPoint.z);
}

inline double cvNorm(const CvPoint2D64f &rPoint)
{
	return cvSqrt(rPoint.x * rPoint.x + rPoint.y * rPoint.y);
}

inline void cvMitPoint(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB, CvPoint2D64f *pPointDes)
{
	pPointDes->x =  rPointA.x +  (rPointB.x - rPointA.x) / 2;
	pPointDes->y =  rPointA.y +  (rPointB.y - rPointA.y) / 2;
}


inline CvPoint2D64f cvMitPoint(const CvPoint2D64f &rPointA, const CvPoint2D64f &rPointB)
{
	CvPoint2D64f ipt;
	cvMitPoint(rPointA, rPointB, &ipt);
	return ipt;
}


inline void cvNormalize(CvPoint3D64f *pPoint)
{
	double norm = cvNorm(*pPoint);
	pPoint->x /= norm;
	pPoint->y /= norm;
	pPoint->z /= norm;
}

inline CvPoint3D64f cvNormalize(const CvPoint3D64f &rPoint)
{
	CvPoint3D64f ipt = rPoint;
	cvNormalize(&ipt);
	return ipt;
}



#endif //MXCVTOOLS_H
