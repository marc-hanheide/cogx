/**
 * @author Michael Zillich
 * @date Sept 2011
 *
 * This is a rewrite of the original plane popout, using the V4R library class
 * PlanePopout, which is based on PCL.
 */

#ifndef PLANE_POPOUT_H
#define PLANE_POPOUT_H

#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/compat.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <v4r/PCLAddOns/PlanePopout.hh>
#include <v4r/PCLAddOns/PCLUtils.h>
#include <v4r/PCLAddOns/PCLFunctions.h>
#include <PointCloudClient.h>
#include <VisionData.hpp>
#include "TomGineWraper/TomGineThread.hh"

//#undef FEAT_VISUALIZATION

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{
using namespace cogx;
using namespace cogx::Math;

class PlanePopOut : public PointCloudClient, public ManagedComponent
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
    class PlaneEntry
    {
    public:
        Plane3 plane;
        vector<PointCloud::SurfacePoint> planePoints;        
        vector<Vector3> hullPoints;
        bool valid;
        RGBValue dispColor;

        PlaneEntry() {
            dispColor.r = 255;
            dispColor.g = 255;
            dispColor.b = 255;
            dispColor.a = 0;
            valid = false;
        }
        void clear() {
            planePoints.clear();
            hullPoints.clear();
            valid = false;
        }
        void init(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
	        pcl::PointIndices::Ptr planepoints, pcl::ModelCoefficients::Ptr pcl_domplane,
	        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tablehull);
    };

    class SOIEntry
    {
    public:
        Sphere3 boundingSphere;
        Box3 boundingBox;
        std::vector<PointCloud::SurfacePoint> points;
        std::vector<PointCloud::SurfacePoint> BGpoints;
        std::string WMId;
        CvHistogram* hist;
        int numFramesNotSeen;
        int numStableFrames;
        bool hasMatch;
        RGBValue dispColor;

        SOIEntry() {
            hist = 0;
            numFramesNotSeen = 0;
            numStableFrames = 0;
            hasMatch = false;
            dispColor.r = rand()%255;
            dispColor.g = rand()%255;
            dispColor.b = rand()%255;
            dispColor.a = 0;
        }
        SOIEntry(const SOIEntry &other) {
            hist = 0;
            *this = other;
        }
        ~SOIEntry() {
            cvReleaseHist(&hist);
        }
        SOIEntry& operator= (const SOIEntry &other);
        void init(const PlaneEntry &domPlane);
        void calcHistogram();
        double compare(const SOIEntry &other);
        double matchProbability(const PlanePopOut::SOIEntry &other);
        void updateFrom(const SOIEntry &other);
        void establishMatch(SOIEntry &other);
        VisionData::SOIPtr createWMSOI(ManagedComponent *comp);
    };

private:
    /**
     * Camera ID from where to get images in case we want to display image and
     * backprojections etc. for visualisation.
     */
    int camId;
    /**
     * Rectified image from point cloud server, in case we want to display image and
     * backprojections etc. for visualisation.
     */
    IplImage *iplDispImage;
    /**
     * If set to true, use own OpenGL visualisation.
     */
    bool doDisplay;
    /**
     * 3D render engine for own OpenGL visualisation.
     */
    TGThread::TomGineThread *tgRenderer;

    /**
     * CAST type PointCloud, all the points in the scene
     */
    PointCloud::SurfacePointSeq points;
    /**
     * as above, but in PCL format
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
    /**
     * Core class: V4R Plane and SOI detection.
     */
    pclA::PlanePopout* m_planePopout;
    static IceUtil::Mutex m_planePopoutMutex; // global lock because of libqhull thread-un-safety
    static int m_componentCount;       // lock only if componentCount > 1

    /**
     * a SOI not seen for this time will be deleted from WM
     */
    int AgonalTime;
    /**
     * only if a SOI has been seen longer than that, will it be added to WM
     */
    int StableTime;
    /**
     * the single dominant plane, including its points and convec hull
     */
    PlaneEntry dominantPlane;
    /**
     * list of all tracked SOIs, where the stable ones have been added to WM 
     */
    list<SOIEntry> trackedSOIs;
    /**
     * TracSOIs and GetStableSOIs access the trackedSOIs list from two threads.
     * So need a mutex for that.
     */
    boost::interprocess::interprocess_mutex trackedSOIsMutex;
    /**
     * map of current SOIs. the SOI label as it is returned by the PlanePopout class
     * is used as the key.
     */
    std::map<unsigned, SOIEntry> currentSOIs;
    bool m_bWriteSoisToWm;

    void GetImageData();
    void GetPlaneAndSOIs();
    void TrackSOIs();
    void DisplayInTG();
    // void SaveHistogramImg(CvHistogram* hist, std::string str);

#ifdef FEAT_VISUALIZATION
    bool m_bSendPoints;
    bool m_bSendPlaneGrid;
    bool m_bSendImage;
    bool m_bSendSois;
    // Color the points by labels or send the color from image
    bool m_bColorByLabel;

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

    std::string guiid(const std::string &myid) {
	return myid + "_" + getComponentID();
    }
    void startV11N();
    void SendImage();
    void SendPoints(bool bColorByLabels);
    void SendPlaneGrid();
    void SendOverlays();
    void SendSOI(PlanePopOut::SOIEntry& soi);
    void SendSOIs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &sois);
    void SendRemovedSOI(PlanePopOut::SOIEntry& soi);
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
    PlanePopOut();
    virtual ~PlanePopOut();
    using CASTComponent::sleepComponent;

    /// When multiple point cloud servers are used one may want SOIs from only some of them.
    /// To disable SOI generation, add the parameter --generate-sois 0.
    /// The SOIs can still be obtained through a filter/wm-call (GetStableSOIs).
    /// Default: true.
    void GetStableSOIs(std::vector<VisionData::SOIPtr>& soiList);
    void onAdd_GetStableSoisCommand(const cast::cdl::WorkingMemoryChange& _wmc);
};

}

#endif
// vim: set sw=4 sts=4 ts=8 noet list :vim



