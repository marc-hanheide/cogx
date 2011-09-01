#include "PopoutProcessor.h"
#include <v4r/PCLAddOns/utils/PCLUtils.h>

using namespace pclA;

PopoutProcessor::PopoutProcessor() :
	stop(false), pcl_id(-1), waitForData(false), planePopout(PlanePopout::Parameter(0.0, 2.0))
{
	pthread_mutex_init(&dataMutex, NULL);
	sem_init(&dataSem, 0, 0);

	tableCoefficients.reset(new pcl::ModelCoefficients());

	cloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	data.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

	interface = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)>
			f = boost::bind(&PopoutProcessor::CallbackCloud, this, _1);

	interface->registerCallback(f);

	interface->start();
}

PopoutProcessor::~PopoutProcessor()
{
	interface->stop();
	sem_post(&dataSem);
	pthread_mutex_destroy(&dataMutex);
	sem_destroy(&dataSem);
}

void PopoutProcessor::CallbackCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	if (!stop && waitForData)
	{

		pthread_mutex_lock(&dataMutex);

		pclA::CopyPointCloud(*cloud, *data.cloud);

		// PlanePopout
		planePopout.FilterZ(cloud, *cloudFiltered);
		planePopout.DetectPopout(cloud, popouts);
		planePopout.ConvertPopout2Mat(*cloud, popouts, data.matCloud);
		planePopout.LabelClusters(data.matCloud, labels, sizeClusters);
		planePopout.CreateMaskLargest(labels, sizeClusters, data.mask);
		//      planePopout.CreateMaskAll(labels, sizeClusters, mask);

		// Find contour of mask
		cv::Mat tmp_mask;
		data.mask.copyTo(tmp_mask);
		data.contour = cv::Mat::zeros(tmp_mask.rows, tmp_mask.cols, CV_8U);
		std::vector < std::vector<cv::Point> > points;
		cv::findContours(tmp_mask, points, CV_RETR_EXTERNAL,
				CV_CHAIN_APPROX_NONE);
		if (!points.empty())
		{
			for (unsigned i = 0; i < points[0].size(); i++)
			{
				data.contour(points[0][i].y, points[0][i].x) = 255;
			}
		}

		pclA::ConvertPCLCloud2CvMat(*cloud, data.matCloud);

		pclA::ConvertPCLCloud2Image(*cloud, data.image);


		if (dbgWin.empty())
		{
			dbgWin = new TomGine::tgTomGineThread(data.image.cols,
					data.image.rows);
			cv::Mat_<float> camR = cv::Mat::eye(3, 3, CV_32F);
			cv::Mat camT = cv::Mat::zeros(3, 1, CV_32F);
			cv::Mat camI(cv::Mat::zeros(3, 3, CV_64F));
			camI.at<double> (0, 0) = camI.at<double> (1, 1) = 525;
			camI.at<double> (0, 2) = 320;
			camI.at<double> (1, 2) = 240;
			camI.at<double> (2, 2) = 1;
			dbgWin->SetCamera(camI);
			dbgWin->SetCamera(camR, camT);
		}
		if (pcl_id < 0)
			pcl_id = dbgWin->AddPointCloud(data.matCloud);
		else
			dbgWin->SetPointCloud(pcl_id, data.matCloud);
		dbgWin->Update();

		cv::imshow("Image", data.image);
		cv::imshow("Mask", data.mask);
		cv::imshow("Contour", data.contour);
		sem_post(&dataSem);
		pthread_mutex_unlock(&dataMutex);
	} else
	{
		if (!dbgWin.empty())
		{
			cv::Mat_ < cv::Vec4f > matcloud;
			pclA::ConvertPCLCloud2CvMat(*cloud, matcloud);
			if (pcl_id < 0)
				pcl_id = dbgWin->AddPointCloud(matcloud);
			else
				dbgWin->SetPointCloud(pcl_id, matcloud);
			dbgWin->Update();
		}


	}

	int key = cv::waitKey(100);
	if (((char) key) == 27)
	{
		stop = true;
	}
}

void PopoutProcessor::WaitForData(PopoutProcessor::SharedData &data)
{
	pthread_mutex_lock(&dataMutex);
	waitForData = true;
	pthread_mutex_unlock(&dataMutex);

	sem_wait(&dataSem);
	pthread_mutex_lock(&dataMutex);
	if (!stop)
	{
		this->data.matCloud.copyTo(data.matCloud);
		this->data.image.copyTo(data.image);
		this->data.mask.copyTo(data.mask);
		this->data.contour.copyTo(data.contour);
		pclA::CopyPointCloud(*this->data.cloud, *data.cloud);
	}
	waitForData = false;
	pthread_mutex_unlock(&dataMutex);
}
