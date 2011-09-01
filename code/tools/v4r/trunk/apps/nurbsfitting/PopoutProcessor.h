 /**
 * @file PopoutProcessor
 * @author Thomas Mörwald
 * @date June 2011
 * @version 0.1
 * @brief Extract popouts from point clouds and segment them
 */

#ifndef _POPOUT_PROCESSOR_H_
#define _POPOUT_PROCESSOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <v4r/PCore/Point3fCol.hh>
#include <v4r/PCLAddOns/PlanePopout.hh>

#include <semaphore.h>
#include <v4r/TomGine/tgTomGine.h>

class PopoutProcessor
{
public:
	struct SharedData
	{
		cv::Mat_<cv::Vec4f> matCloud;
		cv::Mat_<cv::Vec3b> image;
		cv::Mat_<uchar> mask;
		cv::Mat_<uchar> contour;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	};

private:
	bool stop;
	bool waitForData;
	int pcl_id;
	cv::Ptr<TomGine::tgTomGineThread> dbgWin;

	pthread_mutex_t dataMutex;
	sem_t dataSem;



	cv::Mat_<ushort> labels;
	vector<unsigned> sizeClusters;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered;
	pcl::PointIndices popouts;
	pcl::ModelCoefficients::Ptr tableCoefficients;


	SharedData data;

	cv::Ptr<pcl::Grabber> interface;
	pclA::PlanePopout planePopout;

public:
	PopoutProcessor();
	~PopoutProcessor();

	/** The main loop.. */
	void
	CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

	bool Stopped()
	{
		return stop;
	}

	void WaitForData(PopoutProcessor::SharedData &data);

};

#endif
