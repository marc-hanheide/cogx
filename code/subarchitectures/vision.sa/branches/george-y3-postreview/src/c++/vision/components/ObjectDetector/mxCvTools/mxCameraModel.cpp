/**
 * @file mxCameraModel
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/
#include "mxCameraModel.h"
#include <iostream>

namespace mx
{

	CCameraModel::CCameraModel() :
			m_tImgSize ( cvSize ( 0,0 ) ),
			m_pIntrinsic ( NULL ),
			m_pDistortion ( NULL ),
			m_pExtrinsic ( NULL ),
			m_pMintMext ( NULL ),
			m_pInvMintMext ( NULL ),
			m_fDistoreError ( 0.5 )
	{
		m_pUndistMap[0] = NULL; m_pUndistMap[1] = NULL;
		m_pDistMap[0] = NULL; m_pDistMap[1] = NULL;
	}


	CCameraModel::~CCameraModel()
	{
		cvReleaseMat ( &m_pIntrinsic );
		cvReleaseMat ( &m_pDistortion );
		cvReleaseMat ( &m_pExtrinsic );
		cvReleaseMat ( &m_pUndistMap[0] );
		cvReleaseMat ( &m_pUndistMap[1] );
		cvReleaseMat ( &m_pDistMap[0] );
		cvReleaseMat ( &m_pDistMap[1] );
	}


	void CCameraModel::SetIntrinsicMatrix ( double *pMatrix )
	{
		if (	m_pIntrinsic == NULL )
		{
			m_pIntrinsic = cvCreateMat ( 3, 3, CV_64F );
		}
		cvSetIdentity ( m_pIntrinsic );
		double *pData = ( double* ) m_pIntrinsic->data.ptr;
		for ( int i = 0; i < 9; i++ ) pData[i] = pMatrix[i];
	}

	void CCameraModel::SetIntrinsic ( double *pPara )
	{
		if (	m_pIntrinsic == NULL )
		{
			m_pIntrinsic = cvCreateMat ( 3, 3, CV_64F );
		}
		cvSetIdentity ( m_pIntrinsic );
		double *pData = ( double* ) m_pIntrinsic->data.ptr;
		pData[0] = pPara[0];	pData[4] = pPara[1];
		pData[2] = pPara[2];	pData[5] = pPara[3];
	}


	bool CCameraModel::LoadIntrinsic ( const char* pFilename )
	{
		CvFileStorage* fs = cvOpenFileStorage ( pFilename, 0, CV_STORAGE_READ );
		if ( fs == NULL ) return false;
		CvMat *pMatIn;
		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"Intrinsic" );

		double IntrinsicPara[] =
		{
			cvmGet ( pMatIn, 0, 0 ),
			cvmGet ( pMatIn, 1, 1 ),
			cvmGet ( pMatIn, 0, 2 ),
			cvmGet ( pMatIn, 1, 2 )
		};
		SetIntrinsic ( IntrinsicPara );
		cvReleaseMat ( &pMatIn );
		cvReleaseFileStorage ( &fs );
		return true;
	}

	void CCameraModel::SetDistortion ( double *pPara )
	{
		if (	m_pDistortion == NULL )
		{
			m_pDistortion = cvCreateMat ( 1, 4, CV_64F );
		}
		double *pData = ( double* ) m_pDistortion->data.ptr;
		for ( int i = 0; i < 4; i++ )
		{
			pData[i] = pPara[i];
		}
	}


	bool CCameraModel::LoadDistortion ( const char* pFilename )
	{
		CvFileStorage* fs = cvOpenFileStorage ( pFilename, 0, CV_STORAGE_READ );
		if ( fs == NULL ) return false;
		CvMat *pMatIn;
		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"Distortions" );
		SetDistortion ( pMatIn->data.db );
		cvReleaseMat ( &pMatIn );
		cvReleaseFileStorage ( &fs );
		return true;
	}


	void CCameraModel::SetExtrinsic (	double *pPara )
	{
		if (	m_pExtrinsic == NULL )
		{
			m_pExtrinsic = cvCreateMat ( 4, 4, CV_64F );
		}
		cvSetIdentity ( m_pExtrinsic );

		double *pData = ( double* ) m_pExtrinsic->data.ptr;
		for ( int i = 0; i < 12; i++ )
		{
			pData[i] = pPara[i];
		}

	}



	bool CCameraModel::Load ( const char* pFilename )
	{
		CvFileStorage* fs = cvOpenFileStorage ( pFilename, 0, CV_STORAGE_READ );
		CvMat *pMatIn;


		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"ImageSize" );
		m_tImgSize.width  = ( int ) pMatIn->data.db[0];
		m_tImgSize.height = ( int ) pMatIn->data.db[1];
		cvReleaseMat ( &pMatIn );

		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"Intrinsic" );
		SetIntrinsic ( pMatIn->data.db );
		cvReleaseMat ( &pMatIn );

		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"Distortions" );
		SetDistortion ( pMatIn->data.db );
		cvReleaseMat ( &pMatIn );

		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"Extrinsic" );
		SetExtrinsic ( pMatIn->data.db );
		cvReleaseMat ( &pMatIn );

		cvReleaseFileStorage ( &fs );
		return true;
	}

	bool CCameraModel::Save ( const char* pFilename )
	{
		CvFileStorage* fs = cvOpenFileStorage ( pFilename, 0, CV_STORAGE_WRITE_TEXT );

		CvMat* matImgSize = cvCreateMat ( 1, 2, CV_32F );
		matImgSize->data.fl[0] = m_tImgSize.width;
		matImgSize->data.fl[1] = m_tImgSize.height;

		if ( ( m_tImgSize.width != 0 ) && ( m_tImgSize.height != 0 ) )
		{
			cvWrite ( fs, "ImageSize", matImgSize, cvAttrList ( 0,0 ) );
		}
		if ( m_pIntrinsic != NULL )
		{
			cvWrite ( fs, "Intrinsic", m_pIntrinsic, cvAttrList ( 0,0 ) );
		}
		if ( m_pDistortion != NULL )
		{
			cvWrite ( fs, "Distortions", m_pDistortion, cvAttrList ( 0,0 ) );
		}
		if ( m_pExtrinsic != NULL )
		{
			cvWrite ( fs, "Extrinsic", m_pExtrinsic, cvAttrList ( 0,0 ) );
		}

		cvReleaseFileStorage ( &fs );
		cvReleaseMat ( &matImgSize );
		return true;
	}

	void CCameraModel::InitUndistortMap()
	{
		for ( int i = 0; i < 2; i++ )
		{
			if ( ( m_pUndistMap[i] != NULL ) &&
			        ( m_tImgSize.height != m_pUndistMap[i]->height ) &&
			        ( m_tImgSize.width != m_pUndistMap[i]->width ) )
			{
				cvReleaseMat ( &m_pUndistMap[i] );
				m_pUndistMap[i] = NULL;
			}
			if ( m_pUndistMap[i] == NULL )
			{
				m_pUndistMap[i]  = cvCreateMat ( m_tImgSize.height,
				                                 m_tImgSize.width,
				                                 CV_32FC1 );
			}
		}
		cvInitUndistortMap ( m_pIntrinsic, m_pDistortion,
		                     m_pUndistMap[0], m_pUndistMap[1] );
	}


	void CCameraModel::Undistort ( IplImage *pImgSrc, IplImage *pImgDes )
	{
		cvRemap ( pImgSrc, pImgDes, m_pUndistMap[0], m_pUndistMap[1] );
	}

	void CCameraModel::Undistort ( const CvPoint &rPointSrc, CvPoint2D64f *pPointDes )
	{
		int iArrayPos = m_pUndistMap[0]->width * rPointSrc.y + rPointSrc.x;
		pPointDes->x = m_pUndistMap[0]->data.fl[iArrayPos];
		pPointDes->y = m_pUndistMap[1]->data.fl[iArrayPos];
	}


	void CCameraModel::Undistort ( const CvPoint &rPointSrc, CvPoint *pPointDes )
	{
		if ( IsPointIn ( rPointSrc, m_tImgSize ) )
		{
			int iArrayPos = m_pUndistMap[0]->width * rPointSrc.y + rPointSrc.x;
			pPointDes->x = cvRound ( m_pUndistMap[0]->data.fl[iArrayPos] );
			pPointDes->y = cvRound ( m_pUndistMap[1]->data.fl[iArrayPos] );
		}
		else
		{
			CvPoint2D64f tPoint = cvPoint2D64f ( rPointSrc );
			Undistort ( tPoint, &tPoint );
			pPointDes->x = ( int ) tPoint.x;
			pPointDes->y = ( int ) tPoint.y;
		}
	}

	void CCameraModel::Undistort ( const CvPoint2D64f &rPointSrc, CvPoint2D64f *pPointDes )
	{
		// center x
		double cx = m_pIntrinsic->data.db[2];
		// center y
		double cy = m_pIntrinsic->data.db[5];
		// focallength x
		double fx = m_pIntrinsic->data.db[0];
		// focallength y
		double fy = m_pIntrinsic->data.db[4];
		// x postion in distorted image
		double x = ( ( double ) rPointSrc.x );
		// y postion in distorted image
		double y = ( ( double ) rPointSrc.y );
		// relative x position in distorted image to the center
		double dx = ( x - cx ) / fx;
		// relative y position in distorted image to the center
		double dy = ( y - cy ) / fy;
		// relative distance of the distorted point to the center
		double r = sqrt ( pow ( dx,2 ) + pow ( dy,2 ) );
		// k1 radial distortion coefficients
		double k1 = m_pDistortion->data.db[0];
		// k2 radial distortion coefficients
		double k2 = m_pDistortion->data.db[1];
		// p1 tangential distortion coefficients
		double p1 = m_pDistortion->data.db[2];
		// p2 tangential distortion coefficients
		double p2 = m_pDistortion->data.db[3];

		//x = dx*(1 + k1r^2 + k2r^4) + 2*p1*dx*dy + p2(r^2+2*dx^2)
		double Tx1 = dx* ( 1.0 + k1*pow ( r,2 ) + k2*pow ( r,4 ) );
		double Tx2 = 2*p1*dx*dy;
		double Tx3 = p2* ( pow ( r,2 ) + 2*pow ( dx,2 ) );

		//y = dy*(1 + k1r^2 + k2r^4) + p1(r^2+2*dy^2) + 2*p2*dx*dy
		double Ty1 = dy* ( 1.0 + k1*pow ( r,2 ) + k2*pow ( r,4 ) );
		double Ty2 = p1* ( pow ( r,2 ) + 2*pow ( dy,2 ) );
		double Ty3 = 2*p2*dx*dy;

		pPointDes->x = ( Tx1 + Tx2 + Tx3 ) *fx + cx;
		pPointDes->y = ( Ty1 + Ty2 + Ty3 ) *fx + cy;
	}

	int CCameraModel::InitDistortMap()
	{
		for ( int i = 0; i < 2; i++ )
		{
			if ( ( m_pDistMap[i] != NULL ) &&
			        ( m_tImgSize.height != m_pDistMap[i]->height ) &&
			        ( m_tImgSize.width != m_pDistMap[i]->width ) )
			{
				cvReleaseMat ( &m_pDistMap[i] );
				m_pDistMap[i] = NULL;
			}
			if ( m_pDistMap[i] == NULL )
			{
				m_pDistMap[i]  = cvCreateMat ( m_tImgSize.height,
				                               m_tImgSize.width,
				                               CV_32FC1 );
			}
		}
		int iNrOfMaxInterations = 0;
		for ( int y = 0; y < m_tImgSize.height; y++ )
		{

			int iArrayPos = m_pDistMap[0]->width * y;
			for ( int x = 0; x < m_tImgSize.width; x++ )
			{
				CvPoint2D64f tPointDes;
				if ( !Distort ( cvPoint2D64f ( x,y ), &tPointDes ) )
				{
					iNrOfMaxInterations++;
				}
				m_pDistMap[0]->data.fl[iArrayPos] = tPointDes.x;
				m_pDistMap[1]->data.fl[iArrayPos] = tPointDes.y;
				iArrayPos++;
			}
		}
		return iNrOfMaxInterations;
	}

	void CCameraModel::Distort ( const CvPoint &rPointSrc, CvPoint2D64f *pPointDes )
	{
		int iArrayPos = m_pDistMap[0]->width * rPointSrc.y + rPointSrc.x;

		pPointDes->x = m_pDistMap[0]->data.fl[iArrayPos];
		pPointDes->y = m_pDistMap[1]->data.fl[iArrayPos];

		Distort ( cvPoint2D64f ( rPointSrc.x,rPointSrc.y ), pPointDes );

		if ( pPointDes->x < 0 ) pPointDes->x = 0;
		if ( pPointDes->x >= m_tImgSize.width ) pPointDes->x = m_tImgSize.width-1;
		if ( pPointDes->y < 0 ) pPointDes->y = 0;
		if ( pPointDes->y >= m_tImgSize.height ) pPointDes->x = m_tImgSize.height-1;
	}

	bool CCameraModel::Distort ( const CvPoint2D64f &rPointSrc, CvPoint2D64f *pPointDes )
	{
		const int iMaxInterations = 100;
		double fError = m_tImgSize.width*m_tImgSize.width*m_tImgSize.width;
		CvPoint2D64f tGradient = cvPoint2D64f ( 0,0 );
		*pPointDes = rPointSrc;
		CvPoint2D64f tCurrent;
		int i;
		for ( i = 0; ( ( i < iMaxInterations ) && ( fError > m_fDistoreError ) ); i++ )
		{
			pPointDes->x += tGradient.x;
			pPointDes->y += tGradient.y;
			Undistort ( *pPointDes, &tCurrent );
			tGradient.x = rPointSrc.x - tCurrent.x;
			tGradient.y = rPointSrc.y - tCurrent.y;
			fError = tGradient.x*tGradient.x + tGradient.y*tGradient.y;
		}
		if ( pPointDes->x < 0 ) pPointDes->x = 0;
		if ( pPointDes->x >= m_tImgSize.width ) pPointDes->x = m_tImgSize.width-1;
		if ( pPointDes->y < 0 ) pPointDes->y = 0;
		if ( pPointDes->y >= m_tImgSize.height ) pPointDes->x = m_tImgSize.height-1;
		if ( i < iMaxInterations ) return true;
		else return false;
	}
	void CCameraModel::Image2World ( const CvPoint &rPointSrc, CvPoint3D64f *pPointDes, bool bImgIsDistort )
	{
		CvPoint2D64f tUndistor;
		if ( bImgIsDistort )
		{
			Distort ( rPointSrc, &tUndistor );
		}
		else
		{
			tUndistor.x = rPointSrc.x;
			tUndistor.y = rPointSrc.y;
		}
		Image2World ( tUndistor, pPointDes, false );
	}

	void CCameraModel::Image2World ( const CvPoint2D64f &rPointSrc, CvPoint3D64f *pPointDes, bool bImgIsDistort )
	{
		CvMat *pSrc = cvCreateMat ( 4,1,CV_64F );

		if ( bImgIsDistort )
		{
			Distort ( rPointSrc, ( CvPoint2D64f * ) pSrc->data.db );
		}
		else
		{
			pSrc->data.db[0] = rPointSrc.x;
			pSrc->data.db[1] = rPointSrc.y;
		}

		pSrc->data.db[2] = 1;
		pSrc->data.db[3] = 1;

		CvMat *pDes = cvCreateMat ( 4,1,CV_64F );

		cvMatMul ( m_pInvMintMext, pSrc, pDes );

		pPointDes->x = cvmGet ( pDes, 0, 0 ) ;
		pPointDes->y = cvmGet ( pDes, 1, 0 ) ;
		pPointDes->z = cvmGet ( pDes, 2, 0 ) ;

		cvReleaseMat ( &pSrc );
		cvReleaseMat ( &pDes );
	}


	void CCameraModel::World2GroundPlane ( const CvPoint3D64f &rPointSrc, CvPoint3D64f *pPointDes, double dOffset )
	{
		double d = ( - rPointSrc.z - dOffset ) / ( rPointSrc.z - m_tCameraPosition.z );

		pPointDes->x = rPointSrc.x + ( rPointSrc.x - m_tCameraPosition.x ) * d;
		pPointDes->y = rPointSrc.y + ( rPointSrc.y - m_tCameraPosition.y ) * d;
		pPointDes->z = dOffset;
	}


	void CCameraModel::World2YPlane ( const CvPoint3D64f &rPointSrc, CvPoint3D64f *pPointDes, double dYOffset )
	{
		double t = ( - rPointSrc.y + dYOffset ) / ( rPointSrc.y - m_tCameraPosition.y );	// t = (-Py + dO)/dy

		pPointDes->x = rPointSrc.x + ( rPointSrc.x - m_tCameraPosition.x ) * t;
		pPointDes->y = dYOffset;
		pPointDes->z = rPointSrc.z + ( rPointSrc.z - m_tCameraPosition.z ) * t;
	}


	void CCameraModel::World2Image ( const CvPoint3D64f &rPointSrc, CvPoint2D64f *pPointDes, bool bImgIsDistort )
	{
		CvMat *pSrc = cvCreateMat ( 4,1,CV_64F );
		pSrc->data.db[0] = rPointSrc.x;
		pSrc->data.db[1] = rPointSrc.y;
		pSrc->data.db[2] = rPointSrc.z;
		pSrc->data.db[3] = 1;


		CvMat *pDes = cvCreateMat ( 4,1,CV_64F );

		cvMatMul ( m_pMintMext, pSrc, pDes );

		pPointDes->x = cvmGet ( pDes, 0, 0 ) / cvmGet ( pDes, 2, 0 );
		pPointDes->y = cvmGet ( pDes, 1, 0 ) / cvmGet ( pDes, 2, 0 );

		if ( bImgIsDistort )
		{
			Undistort ( *pPointDes, pPointDes );
		}

		cvReleaseMat ( &pSrc );
		cvReleaseMat ( &pDes );
	}

	void CCameraModel::World2Image ( const CvPoint3D64f &rPointSrc, CvPoint *pPointDes, bool bImgIsDistort )
	{
		CvMat *pSrc = cvCreateMat ( 4,1,CV_64F );
		pSrc->data.db[0] = rPointSrc.x;
		pSrc->data.db[1] = rPointSrc.y;
		pSrc->data.db[2] = rPointSrc.z;
		pSrc->data.db[3] = 1;


		CvMat *pDes = cvCreateMat ( 4,1,CV_64F );

		cvMatMul ( m_pMintMext, pSrc, pDes );

		pPointDes->x = cvRound ( cvmGet ( pDes, 0, 0 ) / cvmGet ( pDes, 2, 0 ) );
		pPointDes->y = cvRound ( cvmGet ( pDes, 1, 0 ) / cvmGet ( pDes, 2, 0 ) );

		if ( bImgIsDistort )
		{
			Undistort ( *pPointDes, pPointDes );
		}

		cvReleaseMat ( &pSrc );
		cvReleaseMat ( &pDes );
	}




	void CCameraModel::ComputeExtrinsic ( const std::vector< CvPoint2D64f> &rImagePoints,
	                                      const std::vector< CvPoint3D64f> &rWorldPoints )
	{
		if ( rImagePoints.size() != rWorldPoints.size() ) return;
		CvMat* pWorldPoints = cvCreateMat ( rWorldPoints.size(), 3, CV_64F );
		CvMat* pImagePoints  = cvCreateMat ( rImagePoints.size(), 2, CV_64F );
		CvMat* pRotationVector ( cvCreateMat ( 3, 1, CV_64F ) );
		CvMat* pRotation ( cvCreateMat ( 3, 3, CV_64F ) );
		CvMat* pTranslationVector ( cvCreateMat ( 3, 1, CV_64F ) );

		for ( unsigned int i = 0; i < rImagePoints.size(); i++ )
		{
			cvSetReal2D ( pWorldPoints, i, 0, rWorldPoints[i].x );
			cvSetReal2D ( pWorldPoints, i, 1, rWorldPoints[i].y );
			cvSetReal2D ( pWorldPoints, i, 2, rWorldPoints[i].z );
			cvSetReal2D ( pImagePoints, i, 0, rImagePoints[i].x );
			cvSetReal2D ( pImagePoints, i, 1, rImagePoints[i].y );
		}

		cvFindExtrinsicCameraParams2 ( pWorldPoints, pImagePoints,
		                               m_pIntrinsic, m_pDistortion,
		                               pRotationVector, pTranslationVector );

		cvRodrigues2 ( pRotationVector, pRotation );

		double *R = pRotation->data.db;
		double *T = pTranslationVector->data.db;


		double pEx[12] = {	R[0], R[1], R[2], T[0],
		                   R[3], R[4], R[5], T[1],
		                   R[6], R[7], R[8], T[2]
		                 };

		SetExtrinsic ( pEx );

		cvReleaseMat ( &pRotationVector );
		cvReleaseMat ( &pRotation );
		cvReleaseMat ( &pTranslationVector );
		cvReleaseMat ( &pWorldPoints );
		cvReleaseMat ( &pImagePoints );
	}

	bool CCameraModel::LoadReferencePointsToComputeExtrinsic ( const char* pFilename )
	{
		CvFileStorage* fs = cvOpenFileStorage ( pFilename, 0, CV_STORAGE_READ );
		CvMat *pMatIn;
		pMatIn = ( CvMat* ) cvReadByName ( fs,0,"ReferencePoints" );

		std::vector<CvPoint2D64f> vImagePoints;
		std::vector <CvPoint3D64f> vWorldPoints;

		double ix, iy, wx, wy, wz;
		for ( int r = 0; r < pMatIn->rows; r++ )
		{
			ix = cvmGet ( pMatIn, r, 0 );
			iy = cvmGet ( pMatIn, r, 1 );
			wx = cvmGet ( pMatIn, r, 2 );
			wy = cvmGet ( pMatIn, r, 3 );
			wz = cvmGet ( pMatIn, r, 4 );
			vImagePoints.push_back ( cvPoint2D64f ( ix, iy ) );
			vWorldPoints.push_back ( cvPoint3D64f ( wx, wy, wz ) );
		}
		ComputeExtrinsic ( vImagePoints, vWorldPoints );

		cvReleaseMat ( &pMatIn );
		cvReleaseFileStorage ( &fs );

		return true;
	}


	void CCameraModel::ComputeCameraPosition ( )
	{
		CvMat tMatCameraPositionInCameraCoodinates;
		CvMat tMatCameraPositionInWorldCoodinates;
		double pCameraCenterInCameraCoodinates[] = { 0, 0, 0, 1};
		cvInitMatHeader ( &tMatCameraPositionInWorldCoodinates,
		                  4, 1, CV_64F, ( double* ) &m_tCameraPosition );
		cvInitMatHeader ( &tMatCameraPositionInCameraCoodinates,
		                  4, 1, CV_64F, pCameraCenterInCameraCoodinates );
		CvMat *pInvExtrinsic = cvCreateMat ( 4,4,CV_64F );
		cvInvert ( m_pExtrinsic, pInvExtrinsic );
		cvMatMul (	pInvExtrinsic,
		           &tMatCameraPositionInCameraCoodinates,
		           &tMatCameraPositionInWorldCoodinates );
		cvReleaseMat ( &pInvExtrinsic );
	}

	void CCameraModel::ComputeMintMext ()
	{

		if ( m_pMintMext == NULL )
		{
			m_pMintMext = cvCreateMat ( 4, 4, CV_64F );
		}

		if ( m_pInvMintMext == NULL )
		{
			m_pInvMintMext = cvCreateMat ( 4, 4, CV_64F );
		}

		CvMat *pMint = cvCreateMat ( 4, 4, CV_64F );

		cvSetIdentity ( pMint );
		cvMatCopy ( m_pIntrinsic, pMint );
		cvMatMul ( pMint, m_pExtrinsic,  m_pMintMext );
		cvInvert ( m_pMintMext,  m_pInvMintMext );

		cvReleaseMat ( &pMint );
	}


	void CCameraModel::PreProcessing ( const CvSize &rImgSize )
	{
		m_tImgSize = rImgSize;
		InitUndistortMap();
		InitDistortMap();
		if ( m_pExtrinsic != NULL )
		{
			ComputeMintMext ();
			ComputeCameraPosition ( );
		}
	}


	void CCameraModel::DrawCoordCross ( IplImage *pImg, double dSize, const CvScalar &rColor, bool bImgIsDistort )
	{
		CvPoint3D64f tZero3D = cvPoint3D64f ( 0.,0.,0. );
		CvPoint2D64f tZero2D;
		World2Image ( tZero3D, &tZero2D, bImgIsDistort );

		CvPoint3D64f tX3D = cvPoint3D64f ( dSize,0.,0. );
		CvPoint2D64f tX2D;
		World2Image ( tX3D, &tX2D, bImgIsDistort );
		DrawLine ( pImg, tZero2D, tX2D, rColor, OPT_ARROW );
		CvPoint3D64f tX3DText = cvPoint3D64f ( dSize*1.1,0.,0. );
		CvPoint2D64f tX2DText;
		World2Image ( tX3DText, &tX2DText, bImgIsDistort );
		DrawWrite ( pImg, "X", tX2DText, rColor );

		CvPoint3D64f tY3D = cvPoint3D64f ( 0.,dSize,0. );
		CvPoint2D64f tY2D;
		World2Image ( tY3D, &tY2D, bImgIsDistort );
		DrawLine ( pImg, tZero2D, tY2D, rColor, OPT_ARROW );
		CvPoint3D64f tY3DText = cvPoint3D64f ( 0.,dSize*1.1,0. );
		CvPoint2D64f tY2DText;
		World2Image ( tY3DText, &tY2DText, bImgIsDistort );
		DrawWrite ( pImg, "Y", tY2DText, rColor );

		CvPoint3D64f tZ3D = cvPoint3D64f ( 0.,0.,dSize );
		CvPoint2D64f tZ2D;
		World2Image ( tZ3D, &tZ2D, bImgIsDistort );
		DrawLine ( pImg, tZero2D, tZ2D, rColor, OPT_ARROW );
		CvPoint3D64f tZ3DText = cvPoint3D64f ( 0.,0.,dSize*1.1 );
		CvPoint2D64f tZ2DText;
		World2Image ( tZ3DText, &tZ2DText, bImgIsDistort );
		DrawWrite ( pImg, "Z", tZ2DText, rColor );
	}

	void CCameraModel::DrawLineProjection ( IplImage *pImg, const CvPoint3D64f &rPointStart, const CvPoint3D64f &rPointEnd, const CvScalar &rColor, bool bImgIsDistort, unsigned int defOptions, double dSize )
	{
		CvPoint2D64f tPoint2D_Start;
		World2Image ( rPointStart, &tPoint2D_Start, bImgIsDistort );

		CvPoint2D64f tPoint2D_End;
		World2Image ( rPointEnd, &tPoint2D_End, bImgIsDistort );
		CvPoint2D64f v = cvDiff(tPoint2D_Start, tPoint2D_End);
		double d = cvNorm(v);
		DrawLine ( pImg, tPoint2D_Start, tPoint2D_End, rColor, defOptions, (int) (d * dSize) );
	}

	void CCameraModel::DrawSymbol ( IplImage *pImg, unsigned int symType, const CvPoint3D64f &rPoint, const CvScalar &rColor, bool bImgIsDistort, double dSize )
	{

		CvPoint tPoint;
		World2Image ( rPoint, &tPoint, bImgIsDistort );
		mx::DrawSymbol ( pImg, symType, tPoint, rColor, ( int ) dSize );
	}
}
