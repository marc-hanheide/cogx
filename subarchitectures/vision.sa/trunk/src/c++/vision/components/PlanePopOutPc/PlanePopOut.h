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
#include "OpenSURF/surflib.h"
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
	IpVec surf;
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
	double Calc_SplitThreshold(PointCloud::SurfacePointSeq &points, std::vector <int> &label);
	std::vector<ObjPara> PreviousObjList;
	std::vector<ObjPara> CurrentObjList;
	std::vector<ObjPara> Pre2CurrentList;
	VisionData::SOIPtr createObj(Vector3 center, Vector3 size, double radius, PointCloud::SurfacePointSeq psIn1SOI, PointCloud::SurfacePointSeq BGpIn1SOI, PointCloud::SurfacePointSeq EQpIn1SOI);
	float Compare2SOI(ObjPara obj1, ObjPara obj2);
	int IsMatchingWithOneSOI(int index, std::vector <SOIMatch> mlist);
	//bool Compare2SOI(ObjPara obj1, ObjPara obj2);
	void AddConvexHullinWM();
	void newVisualObject(const cdl::WorkingMemoryChange & _wmc);
	void deleteVisualObject(const cdl::WorkingMemoryChange & _wmc);
	void RefinePlaneEstimation(vector <Vector3> lines);

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

	void Points2Cloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud);
	void DisplayInTG();
	
//-----------------------------------------------------------------------------------------------------------------------------------	
	
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
	void GetImageData();
	void GetPlaneAndSOIs();					/// Dominant plane detection and Euclidean Clustering
	bool SimpleSOIMatch();					/// when the no. of SOIs doesn't change, do the one-by-one matching
	void ComplexSOIMatch();					/// when the no.of SOIs changes, find the missing/new SOIs
	void TrackSOIs();
	void GetHists(std::vector< CvHistogram* > &_hists);
	CvHistogram* CalSOIHist(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);
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


	bool RANSAC(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
	void SplitPoints(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
	void DrawOneCuboid(Vector3 Max, Vector3 Min);
	void DrawCuboids(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
	void BoundingSphere(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
	void BoundingPrism(PointCloud::SurfacePointSeq &pointsN, std::vector <int> &labels);
	void DrawOnePrism(vector <Vector3> ppSeq, double hei, Vector3& v3c);
	void ConvexHullOfPlane(PointCloud::SurfacePointSeq &points, std::vector <int> &labels);
	inline Vector3 AffineTrans(Matrix33 m33, Vector3 v3);
	Vector3 GetAffineTransVec(Vector3 v3p);
	Matrix33 GetAffineRotMatrix();
	Vector3 ProjectOnDominantPlane(Vector3 InputP);
	void DrawWireSphere(Vector3 center, double radius);

	vector<double> Hypo2ParaSpace(vector<Vector3> vv3Hypo);
	
	
	bool lessfitness(const Particle& p1, const Particle& p2);
	
	void CalRadiusCenter4BoundingSphere(PointCloud::SurfacePointSeq points, Vector3 &c, double &r);
	
	Vector3 ProjectPointOnPlane(Vector3 p, double A, double B, double C, double D);
	
	CvPoint ProjectPointOnImage(Vector3 p, const Video::CameraParameters &cam);
	void CollectDensePoints(Video::CameraParameters &cam, PointCloud::SurfacePointSeq points);
	CvHistogram* GetSurfAndHistogram(PointCloud::SurfacePointSeq points, Video::Image img, IpVec& ips, CvRect &r);
	void SOIManagement();
	void GetStableSOIs(std::vector<VisionData::SOIPtr>& soiList);
	void onAdd_GetStableSoisCommand(const cast::cdl::WorkingMemoryChange& _wmc);
	void SaveHistogramImg(CvHistogram* hist, std::string str);
	double CompareHistKLD(CvHistogram* h1, CvHistogram* h2);
	bool IsMoving(IplImage * subimg);
	Vector3 PixelRGB2HSV(cogx::Math::ColorRGB rgb);
	void FindVerticalPlanes(PointCloud::SurfacePointSeq &points, std::vector <int> &labels, double B, double C);
	
	inline Particle InitialParticle()
	{
	    Particle P;
	    P.v.assign(4,0.0); 		// velocity
	    P.p.assign(4,0.0); 		// position in parameter space
	    P.pbest.assign(4,0.0);	// best position for this particle
	    P.fCurr = 9999999;		// current fitness
	    P.fbest = 9999999;
	    return P;
	};	
	
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



