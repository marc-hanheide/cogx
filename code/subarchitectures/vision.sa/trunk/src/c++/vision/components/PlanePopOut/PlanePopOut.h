/**
 * @author Kai ZHOU
 * @date June 2009
 *
 * Just receives stereo point clouds and displays them.
 */

#ifndef PLANE_POPOUT_H
#define PLANE_POPOUT_H

#include <cast/architecture/ManagedComponent.hpp>
#include <StereoClient.h>
#include <VisionData.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "OpenSURF/surflib.h"

//#undef FEAT_VISUALIZATION

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{
using namespace cogx;
using namespace cogx::Math;

class PlanePopOut : public StereoClient,
                     public ManagedComponent
{
typedef struct ObjP
{
	Vector3 c;
	Vector3 s;
	double r;
	std::string id;
	bool bComCurrentPre;
	bool bInWM;
	int count;
	VisionData::SurfacePointSeq pointsInOneSOI;
	VisionData::SurfacePointSeq BGInOneSOI;
	VisionData::SurfacePointSeq EQInOneSOI;
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
	double Calc_SplitThreshold(VisionData::SurfacePointSeq &points, std::vector <int> &label);
	std::vector<ObjPara> PreviousObjList;
	std::vector<ObjPara> CurrentObjList;
	std::vector<ObjPara> Pre2CurrentList;
	VisionData::SOIPtr createObj(Vector3 center, Vector3 size, double radius, VisionData::SurfacePointSeq psIn1SOI, VisionData::SurfacePointSeq BGpIn1SOI, VisionData::SurfacePointSeq EQpIn1SOI);
	float Compare2SOI(ObjPara obj1, ObjPara obj2);
	int IsMatchingWithOneSOI(int index, std::vector <SOIMatch> mlist);
	//bool Compare2SOI(ObjPara obj1, ObjPara obj2);
	void AddConvexHullinWM();

	vector< VisionData::SurfacePointSeq > SOIPointsSeq;
	vector< VisionData::SurfacePointSeq > BGPointsSeq;
	vector< VisionData::SurfacePointSeq > EQPointsSeq; //equivocal points
	vector< Vector3 > v3center;
	vector<double> vdradius;
	vector< Vector3 > v3size;
	IplImage* previousImg;
	bool bIsMoving;
	double CurrentBestDistSquared;
	bool bHorizontalFound;
	bool bVerticalOn;

#ifdef FEAT_VISUALIZATION
	bool m_bSendPoints;
	bool m_bSendPlaneGrid;
	bool m_bSendImage;
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


	bool RANSAC(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	void SplitPoints(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	void DrawOneCuboid(Vector3 Max, Vector3 Min);
	void DrawCuboids(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	void BoundingSphere(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	void BoundingPrism(VisionData::SurfacePointSeq &pointsN, std::vector <int> &labels);
	void DrawOnePrism(vector <Vector3> ppSeq, double hei, Vector3& v3c);
	void ConvexHullOfPlane(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	inline Vector3 AffineTrans(Matrix33 m33, Vector3 v3);
	Vector3 GetAffineTransVec(Vector3 v3p);
	Matrix33 GetAffineRotMatrix();
	Vector3 ProjectOnDominantPlane(Vector3 InputP);
	void DrawWireSphere(Vector3 center, double radius);

	vector<double> Hypo2ParaSpace(vector<Vector3> vv3Hypo);
	void PSO_internal(vector < vector<double> > init_positions, VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	bool PSO_Label(VisionData::SurfacePointSeq &points, std::vector <int> &labels);
	double PSO_EvaluateParticle(Particle OneParticle, vector <Particle> optima_found, VisionData::SurfacePointSeq points, Vector3 cc, double rr);
	bool lessfitness(const Particle& p1, const Particle& p2);
	vector<double> UpdatePosition(vector<double> p, vector<double> v);
	vector<double> UpdateVelocity(vector<double> p, vector<double> v, vector<double> pbest, vector<double> gbest, float chi, float c1, float c2, float w);
	void CalRadiusCenter4BoundingSphere(VisionData::SurfacePointSeq points, Vector3 &c, double &r);
	double DistOfParticles(Particle p1, Particle p2, Vector3 c, double r, bool& bParallel);
	Vector3 ProjectPointOnPlane(Vector3 p, double A, double B, double C, double D);
	void Reinitialise_Parallel(vector<Particle>& vPar, vector<Particle>& vT, vector<Particle> vFO, VisionData::SurfacePointSeq points, Vector3 cc, double rr);
	CvPoint ProjectPointOnImage(Vector3 p, const Video::CameraParameters &cam);
	void CollectDensePoints(Video::CameraParameters &cam, VisionData::SurfacePointSeq points);
	CvHistogram* GetSurfAndHistogram(VisionData::SurfacePointSeq points, Video::Image img, IpVec& ips, CvRect &r);
	void SOIManagement();
	void SaveHistogramImg(CvHistogram* hist);
	double CompareHistKLD(CvHistogram* h1, CvHistogram* h2);
	bool IsMoving(IplImage * subimg);
	Vector3 PixelRGB2HSV(VisionData::ColorRGB rgb);
	void FindVerticalPlanes(VisionData::SurfacePointSeq &points, std::vector <int> &labels, double B, double C);
	
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
	}
  virtual ~PlanePopOut() {}
};

}

#endif



