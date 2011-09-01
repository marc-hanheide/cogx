/**
 * @author Kai ZHOU
 * @date June 2009
 *
 * Just receives stereo point clouds and displays them.
 */

#ifndef PLANE_POPOUT_H
#define PLANE_POPOUT_H

#include <cast/architecture/ManagedComponent.hpp>
#include <PointCloudClient.h>
#include <VisionData.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "TomGineWraper/TomGineThread.hh"

#include "StereoCamera.h"


#include <v4r/PCLAddOns/PlanePopout.hh>
#include <v4r/PCLAddOns/utils/PCLUtils.h>
#include <v4r/PCLAddOns/functions/PCLFunctions.h>

//#undef FEAT_VISUALIZATION

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{
using namespace cogx;
using namespace cogx::Math;

class PlanePopOut : public PointCloudClient,
                    public ManagedComponent
{
public:
typedef struct ObjP
{
	Vector3 c;
	Vector3 s;
	double r;
	std::string id;
	bool bComCurrentPre;
	bool bInWM;
	int count;
	PointCloud::SurfacePointSeq pointsInOneSOI;
	PointCloud::SurfacePointSeq BGInOneSOI;
	PointCloud::SurfacePointSeq EQInOneSOI;
	CvHistogram* hist;
	CvRect rect;
}ObjPara;

typedef struct Particle
{
    vector<double> v; 		// velocity
    vector<double> p; 		// position in parameter space
    vector<double> pbest;	// best position for this particle
    double fCurr;		// current fitness
    double fbest;		// best fitness
    bool operator < (const Particle& rhs) const
    {
      return fCurr<rhs.fCurr;
    }
}PSOParticle;

typedef struct MatchingSOI
{
    int p;	//index in previous list of SOIs
    int c;	//index in current list of SOIs
    double pro;	//probability of matching
}SOIMatch;

private:

///--------------------------------------------------------------------
  
    void GetImageData();
    void GetPlaneAndSOIs();				/// Dominant plane detection and Euclidean Clustering
    void CalSOIHist(PointCloud::SurfacePointSeq pcl_cloud, std::vector< int > label, std::vector <CvHistogram*> & vH);
    
    void ConvexHullOfPlane(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
      Matrix33 GetAffineRotMatrix();
      inline Vector3 AffineTrans(Matrix33 m33, Vector3 v3);
      Vector3 ProjectOnDominantPlane(Vector3 InputP);
    void BoundingPrism(PointCloud::SurfacePointSeq &pointsN, std::vector <int> &labels);
      void DrawOnePrism(vector <Vector3> ppSeq, double hei, Vector3& v3c);
    void DrawCuboids(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
    void BoundingSphere(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
    
    void DisplayInTG();
      void Points2Cloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud);
      
    void AddConvexHullinWM();
    void SOIManagement();
      VisionData::SOIPtr createObj(Vector3 center, Vector3 size, double radius, PointCloud::SurfacePointSeq psIn1SOI, PointCloud::SurfacePointSeq BGpIn1SOI, PointCloud::SurfacePointSeq EQpIn1SOI);
      int IsMatchingWithOneSOI(int index, std::vector <SOIMatch> mlist);
      float Compare2SOI(ObjPara obj1, ObjPara obj2);
	double CompareHistKLD(CvHistogram* h1, CvHistogram* h2);
      void SaveHistogramImg(CvHistogram* hist, std::string str);
	
      
  
///---------------------------------------------------------------------
      
      double A, B, C, D;					///< Plane Coefficients, Ax+By+Cz+D=0
      pcl::ModelCoefficients::Ptr dpc;				///< Dominant plane Coefficients
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tablehull;		///< Table hull
      pcl::PointIndices::Ptr planepoints;			///< the indices of points on the dominant plane
      
      VisionData::Vector3Seq mConvexHullPoints;
      Vector3 mCenterOfHull;
      double mConvexHullRadius;
      double mConvexHullDensity;

      Vector3 pre_mCenterOfHull;
      double pre_mConvexHullRadius;
      std::string pre_id;
  
      
///---------------------------------------------------------------------
  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * whether to use stereo points in global or left camear coord system.
   */
	bool useGlobalPoints;
	// unit m, due to the error of stereo, >0.01 is suggested
	double min_height_of_obj;

	std::vector<ObjPara> PreviousObjList;
	std::vector<ObjPara> CurrentObjList;
	std::vector<ObjPara> Pre2CurrentList;

	vector< PointCloud::SurfacePointSeq > SOIPointsSeq;
	vector< PointCloud::SurfacePointSeq > BGPointsSeq;
	vector< PointCloud::SurfacePointSeq > EQPointsSeq; //equivocal points
	vector< Vector3 > v3center;
	vector<double> vdradius;
	vector< Vector3 > v3size;
	vector <Vector3> vlines;
	vector <double>	vlineConfidence;
	IplImage* previousImg;
	bool bIsMoving;
	double CurrentBestDistSquared;
	bool bHorizontalFound;
	bool bVerticalOn;
	std::string stereoconfig;                         ///< Config name of the stereo configuration file
	TGThread::TomGineThread *tgRenderer;              ///< 3D render engine

	/// When multiple point cloud servers are used one may want SOIs from only some of them.
	/// To disable SOI generation, add the parameter --generate-sois 0.
	/// The SOIs can still be obtained through a filter/wm-call (GetStableSOIs).
	/// Default: true.
	bool bWriteSoisToWm;

	
	
//-----------------------------------------------------------------------------------------------------------------------------------	
	/**
	* status of SOI matching results
	*/
	enum SOIMatchingResult {
	  MISSING, STABLE, ADDING
	};
	
	std::vector <CvHistogram*> vec_histogram;		 ///< vector with the histograms of all the SOIs
	PointCloud::SurfacePointSeq points;	  		 ///< PointCloud type all the points in the scene
	std::vector< int > points_label;			   ///< lables of all the points
	cv::Mat_<cv::Vec4f> kinect_point_cloud;                   ///< Point cloud from the kinect
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud ;        ///< PCL point cloud
	
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > sois;     ///< Estimated sois from the PlanePopout
	std::vector<CvHistogram*> hists;				///< vector with histograms of all the SOIs
	std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pre_sois; ///< keep the sois for tracking
	std::vector<CvHistogram*> pre_hists;				///< keep the histograms

	cv::Mat_<cv::Vec3b> patches;                              ///< 3D patches on 2D image
	int kinectImageWidth, kinectImageHeight;                  ///< width and height of the kinect color image
	int pointCloudWidth, pointCloudHeight;                    ///< width and height of the kinect point cloud
	Video::Image image_l, image_r, image_k;                   ///< Left and right stereo image and kinect image
	IplImage *iplImage_l, *iplImage_r, *iplImage_k;           ///< Converted left and right stereo images (openCV ipl-images)

      //  cv::Mat_<cv::Point3f> kinect_point_cloud;                 ///< point cloud with kinect 3d points                                  /// TODO delete later => change to cv::Vec4f
      //  cv::Mat_<cv::Point3f> kinect_color_point_cloud;           ///< point cloud with kinect color information
	
	bool single;                                              ///< Single shot mode for the stereo detector learner
	bool showImages;                                          ///< Show images in openCV windows
//-----------------------------------------------------------------------------------------------------------------------------------------------------------	

#ifdef FEAT_VISUALIZATION
	bool m_bSendPoints;
	bool m_bSendPlaneGrid;
	bool m_bSendImage;
	bool m_bSendSois;
	// Color the poitns by labels or send the color from image
	bool m_bColorByLabel;
	CMilliTimer m_tmSendPoints;

	class CDisplayClient: public cogx::display::CDisplayClient
	{
		PlanePopOut* pPopout;
	public:
		CDisplayClient() { pPopout = NULL; }
		void setClientData(PlanePopOut* pPlanePopout) { pPopout = pPlanePopout; }
		void handleEvent(const Visualization::TEvent &event); /*override*/
		std::string getControlState(const std::string& ctrlId); /*override*/
	};
	CDisplayClient m_display;

	void SendSyncAllSois();
	void SendImage(PointCloud::SurfacePointSeq& points, std::vector <int> &labels, const Video::Image& img);
	void SendPoints(const PointCloud::SurfacePointSeq& points, std::vector<int> &labels, bool bColorByLabels, CMilliTimer& tmSendPoints);
	void SendPlaneGrid();
	void SendOverlays();
	void SendSoi(PlanePopOut::ObjPara& soiobj);
	void SendRemoveSoi(PlanePopOut::ObjPara& soiobj);
#endif

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

public:
	
	CvPoint ProjectPointOnImage(Vector3 p, const Video::CameraParameters &cam);
	void CollectDensePoints(Video::CameraParameters &cam, PointCloud::SurfacePointSeq points);
	
	void GetStableSOIs(std::vector<VisionData::SOIPtr>& soiList);
	void onAdd_GetStableSoisCommand(const cast::cdl::WorkingMemoryChange& _wmc);
	
	double para_a;
	double para_b;
	double para_c;
	double para_d;


	PlanePopOut() : camId(0) 
	{
		para_a = 0.0;
		para_b = 0.0;
		para_c = 0.0;
		para_d = 0.0;
		previousImg = 0;
		min_height_of_obj = 0.04;
		bWriteSoisToWm = true;
	}
  virtual ~PlanePopOut() {}
};

}

#endif



