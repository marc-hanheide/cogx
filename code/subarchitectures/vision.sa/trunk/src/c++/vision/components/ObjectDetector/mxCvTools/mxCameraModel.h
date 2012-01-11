/**
 * @file mxCameraModel
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/
#ifndef MXCAMERAMODEL_H
#define MXCAMERAMODEL_H

#include "mxCvTools.h"
#include "mxDrawTools.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

namespace mx
{

	/**
		@author Markus Bader <bader@acin.tuwien.ac.at>
	*/
	class CCameraModel
	{
		public:
			/**
			* @brief Constructor 
			**/
			CCameraModel();
			/**
			* @brief Destructor 
			**/
			~CCameraModel();

			/**
			* @brief Sets the intrinsic camera parameters
			* @param  pPara array with fx, fy, cx, cy
			* @post CCameraModel::PreProcessing
			**/
			void SetIntrinsic ( double *pPara);

			/**
			* @brief Sets the intrinsic camera matrix
			* @param  pMatrix array with 3x3 elemets
			* @post CCameraModel::PreProcessing
			**/
			void SetIntrinsicMatrix ( double *pMatrix);

			/**
			* @brief loads the intrinsic camera parameter
			* @param pFilename calibration from a xml file
			* @return false on problems
			**/
			bool LoadIntrinsic(const char* pFilename = "calibration.xml");

			/**
			* @brief Sets the distortion camera parameters
			* @param  pPara array with k1, k2, p1, p2
			* @post CCameraModel::PreProcessing
			**/
			void SetDistortion ( double *pPar );

			/**
			* @brief loads the distortion camera parameter 
			* @param pFilename calibration from a xml file
			* @return false on problems
			**/
			bool LoadDistortion(const char* pFilename = "calibration.xml");

			/**
			* @brief Sets the extrinsic camera parameters
			* @param  pPara array with R(3x3)+T(3x1)
			* @post CCameraModel::PreProcessing
			**/
			void SetExtrinsic (	double *pPar );

			/**
			* @brief loads the matrixes needed
			* @param pFilename calibration file in OpenCv xml format
			* @return false on problems
			**/
			bool Load(const char* pFilename = "calibration.xml");

			/**
			* @brief Saves the matrixes needed
			* @param pFilename calibration file in OpenCv xml format
			* @return false on problems
			**/
			bool Save(const char* pFilename = "calibration.xml");

			/**
			* @brief Computes the extrinsic camera parameters\n
			* At least four points are needed
			* @param  rImagePoints Points in the image
			* @param  rWorldPoints Corresponding points in world coordinates to rImagePoints
			**/
			void ComputeExtrinsic ( const std::vector<CvPoint2D64f> &rImagePoints, const std::vector <CvPoint3D64f> &rWorldPoints );

			/**
			* @brief loads the reference points to compute the extrinsic camera parameters
			* @param pFilename calibration from a xml file
			* @return false on problems
			**/
			bool LoadReferencePointsToComputeExtrinsic (const char* pFilename = "calibration.xml");


			/**
			* @brief Removes the distortions from a camera image\n
			* @param  pImgSrc camera image distort
			* @param  pImgDes image without distortions
			**/
			void Undistort ( IplImage *pImgSrc, IplImage *pImgDes );

			/**
			* @brief Computes the location of a pixel in the distort image\n
			* It uses the lookup table
			* @param  rPointSrc point in the undistort image "image without distortions"
			* @param  pPointDes point in the distort images "camera image"
			**/
			void Undistort ( const CvPoint &rPointSrc, CvPoint *pPointDes );

			/**
			* @brief Computes the location of a pixel in the distort image\n
			* It uses the lookup table
			* @param  rPointSrc point in the undistort image "image without distortions"
			* @param  pPointDes point in the distort images "camera image"
			**/
			void Undistort ( const CvPoint &rPointSrc, CvPoint2D64f *pPointDes );

			/**
			* @brief Computes the location of a pixel in the distort image\n
			* It computes every time the undistortions
			* @param  rPointSrc point in the undistort image "image without distortions"
			* @param  pPointDes point in the distort images "camera image"
			**/
			void Undistort ( const CvPoint2D64f &rPointSrc, CvPoint2D64f *pPointDes );

			/**
			* @brief Computes the location of a pixel in the undistort image\n
			* It uses the lookup table
			* @param  rPointSrc point in the distort image "camera image"
			* @param  pPointDes point in the undistort images "image without distortions"
			**/
			void Distort ( const CvPoint &rPointSrc, CvPoint2D64f *pPointDes );

			/**
			* @brief Computes the location of a pixel in the undistort image\n
			* It computes every time the distortions
			* @param  rPointSrc point in the distort image "camera image"
			* @param  pPointDes point in the undistort images "image without distortions"
			**/
			bool Distort ( const CvPoint2D64f &rPointSrc, CvPoint2D64f *pPointDes );

			/**
			* @brief Computes the location of a pixel in world coordinates\n
			* It returns the 3D location of the pixel on the image plane in world coordinates.\n
			* If bImgIsDistortit is true it will compute the point using gradienten relegation
			* @param  rPointSrc point on the image plane
			* @param  pPointDes point in world coordinates
			* @param  bImgIsDistortit should be true if the source is a distort image "camera" 
			**/
			void Image2World ( const CvPoint2D64f &rPointSrc, CvPoint3D64f *pPointDes, bool bImgIsDistort = true );

			/**
			* @brief Computes the location of a pixel in world coordinates\n
			* It returns the 3D location of the pixel on the image plane in world coordinates.\n
			* If bImgIsDistortit is true it use the precomputed lookup table
			* @param  rPointSrc point on the image plane
			* @param  pPointDes point in world coordinates
			* @param  bImgIsDistortit should be true if the source is a distort image "camera" 
			**/
			void Image2World ( const CvPoint &rPointSrc, CvPoint3D64f *pPointDes, bool bImgIsDistort = true );

			/**
			* @brief Computes the location of a point in world coordinates if it lies on the ground\n
			* @param  rPointSrc point on the image plane in world coodinates
			* @param  pPointDes point in world coordinates an the groundplane
			* @param  dOffset Offset in [mm] if it lies aboth the ground
			* @pre CCameraModel::Image2World
			**/
			void World2GroundPlane ( const CvPoint3D64f &rPointSrc, CvPoint3D64f *pPointDes, double dOffset = 0 );

			/**
			* @brief Computes the location of a point in world coordinates if it lies on a x-z plane with y=dYOffset\n
			* @param  rPointSrc point on the image plane in world coodinates
			* @param  pPointDes point in world coordinates an the x-z
			* @param  dYOffset Offset in [mm] of the y-axis
			* @pre CCameraModel::Image2World
			**/
			void World2YPlane ( const CvPoint3D64f &rPointSrc, CvPoint3D64f *pPointDes, double dYOffset = 0 );

			/**
			* @brief Computes the location of a world point in the image\n
			* Optional it gives the option to distore the image pixels so that it fits into the camera image. \n
			* If bImgIsDistortit is true it will compute the point using gradienten relegation
			* @param  rPointSrc World point
			* @param  pPointDes Image point
			* @param  bImgIsDistort it should be true if the target is a distort (camera) image
			* @pre CCameraModel::PreProcessing
			**/
			void World2Image ( const CvPoint3D64f &rPointSrc, CvPoint2D64f *pPointDes, bool bImgIsDistort = true );

			/**
			* @brief Computes the location of a world point in the image\n
			* Optional it gives the option to distore the image pixels so that it fits into the camera image. \n
			* If bImgIsDistortit is true it use the precomputed lookup table
			* @param  rPointSrc World point
			* @param  pPointDes Image point
			* @param  bImgIsDistort it should be true if the target is a distort (camera) image
			* @pre CCameraModel::PreProcessing
			**/
			void World2Image ( const CvPoint3D64f &rPointSrc, CvPoint *pPointDes, bool bImgIsDistort = true );	

			/**
			* @brief Preprocessing step to compute matices need for the transformations and lookup tables \n
			* It should be called after the instrinic, distortions and extrinsic parameters a set
			* @pre CCameraModel::SetIntrinsic, CCameraModel::SetDistortion, CCameraModel::SetExtrinsic oder  CCameraModel::ComputeExtrinsic
			**/
			void PreProcessing ( const CvSize &rImgSize );
			/**
			* @brief Defines new minimal distortions error \n
			* the values is used as threshold in teh function CCameraModel::Distort
			* @post CCameraModel::PreProcessing
			**/
			void SetDistoreError ( double fError = 0.5 ) {m_fDistoreError = fError;};
			/**
			* @brief Current use minimal distortions error \n
			* @return min distore error
			**/
			double GetDistoreError() {return m_fDistoreError;};
			/**
			* @brief Current intrinsic matrix \n
			* @return intrinsic matrix
			**/
			CvMat *GetIntrinsic() {return m_pIntrinsic;};
			/**
			* @brief Current distortion parameter \n
			* @return distortion parameter
			**/
			CvMat *GetDistortion() {return m_pDistortion;};
			/**
			* @brief Current Extrinsic matrix \n
			* @return extrinsic matrix
			**/
			CvMat *GetExtrinsic() {return m_pExtrinsic;};

			/**
			* @brief Draws a cross with the coordinates at <0,0,0>
			* @param dSize
			* @return extrinsic matrix
			**/
			void DrawCoordCross(IplImage *pImg, double dSize = 500, const CvScalar &rColor = cvScalar(128,128,128,0), bool bImgIsDistort = true);

			/**
			* @brief Draws a line onto the image plane
			* @param pImg
			* @param rPointStart
			* @param rPointEnd
			* @param rColor
			* @param bImgIsDistort
			* @param defOptions
			* @param iSize
			* @return extrinsic matrix
			**/
			void DrawLineProjection(IplImage *pImg, const CvPoint3D64f &rPointStart, const CvPoint3D64f &rPointEnd, const CvScalar &rColor, bool bImgIsDistort = true, unsigned int defOptions = OPT_NA, double iSize  = 0.5  );

			/**
			* @brief Draws a symbol
			* @param pImg
			* @param defType see mx::DrawSymbol (SYM_CROSS)
			* @param rPoint
			* @param bImgIsDistort
			* @param dSize
			* @return extrinsic matrix
			**/
			void DrawSymbol(IplImage *pImg, unsigned int defType, const CvPoint3D64f &rPoint, const CvScalar &rColor = cvScalar(255,255,255,0) , bool bImgIsDistort = true, double dSize  = 10  );

		private:
			CvSize m_tImgSize;
			CvMat *m_pIntrinsic;
			CvMat *m_pDistortion;
			CvMat *m_pExtrinsic;
			CvMat *m_pMintMext;
			CvMat *m_pInvMintMext;
			CvMat *m_pUndistMap[2];
			CvMat *m_pDistMap[2];
			CvPoint3D64f m_tCameraPosition;
			double m_fDistoreError;

			//Private functions
			void ComputeMintMext ();
			void ComputeCameraPosition ();
			void InitUndistortMap();
			int InitDistortMap();
	};

}

#endif
