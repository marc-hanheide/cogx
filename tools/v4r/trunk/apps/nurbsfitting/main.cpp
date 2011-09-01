#include "PopoutProcessor.h"

#include "NurbsCreator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <v4r/PCLAddOns/utils/PCLUtils.h>


int main()
{
	bool save_cloud = false;
	bool save_normals = false;
	PopoutProcessor* popout;
	PopoutProcessor::SharedData data;
	data.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	cv::Mat_<float> depth;
	cv::Mat_<cv::Vec4f> normals;
	cv::Mat_<float> curvature;

	TomGine::tgEngine* m_engine;
	TomGine::tgTomGineThread* dbgWin = NULL;
	int surf_id = -1;
	int pcl_id = -1;

	unsigned order = 3;
	unsigned refinement = 5;
	unsigned iterations = 1;

	std::string normals_yml = std::string(V4R_DIR) + "/apps/nurbsfitting/Resources/normals.yml";
	std::string pcd_cloud = std::string(V4R_DIR) + "/apps/nurbsfitting/Resources/desktop.pcd";
	std::string edges_jpg = std::string(V4R_DIR) + "/apps/nurbsfitting/Resources/edges.jpg";
	std::string edges_yml = std::string(V4R_DIR) + "/apps/nurbsfitting/Resources/edges.yml";

	if(save_cloud){
		popout = new PopoutProcessor();
		printf("[main] WaitForData ...\n");
		popout->WaitForData(data);
		printf("[main] got data\n");

		printf("[main] saving: '%s'\n", pcd_cloud.c_str());
		pcl::io::savePCDFileBinary(pcd_cloud, *data.cloud);

		delete(popout);
//		return 0;

	}else{
		printf("[main] loading %s\n", pcd_cloud.c_str());
		if(pcl::io::loadPCDFile(pcd_cloud, *data.cloud)==-1)
			throw std::runtime_error("[nurbsfitting] PCD file not found.");

		pclA::ConvertPCLCloud2Image(*data.cloud, data.image);
		pclA::ConvertPCLCloud2CvMat(*data.cloud, data.matCloud, 0.1, 2.0);
		pclA::ConvertPCLCloud2Mask(*data.cloud, data.mask);

		if(save_normals){
			printf("[main] convert to cv normals\n");
			pclA::ConvertPCLCloud2Normals(data.cloud, normals, 0.02);
			cv::FileStorage fs;
			if (!fs.open(normals_yml, cv::FileStorage::WRITE))
				throw std::runtime_error(
						"[main] Cannot open normal yml file for writing.");
			printf("[main] save normals to '%s'\n", normals_yml.c_str());
			fs << "normals" << normals;
			fs.release();
		}else{
			printf("[main] load normals from '%s'\n", normals_yml.c_str());
			cv::FileStorage fs;
			if (!fs.open(normals_yml, cv::FileStorage::READ))
				throw std::runtime_error(
						"[main] Cannot open normal yml file for reading.");
			fs["normals"] >> normals;
			fs.release();
		}

		float max_curv(0.0f), min_curv(INFINITY);
		curvature = cv::Mat_<float>(normals.rows, normals.cols);
		for(unsigned v=0; v<normals.rows; v++){
			for(unsigned u=0; u<normals.cols; u++){
				cv::Vec4f &n = normals(v,u);
				float &curv = curvature(v,u);
				curv = n[3];
				if(max_curv < curv) max_curv = curv;
				if(min_curv > curv) min_curv = curv;
			}
		}

		float dcurv = 1.0f / (max_curv-min_curv);
		for(unsigned v=0; v<curvature.rows; v++){
			for(unsigned u=0; u<curvature.cols; u++){
				cv::Vec4f &n = normals(v,u);
				float &curv = curvature(v,u);
				if(n[0]==0.0f && n[1]==0.0f && n[2]==0.0f)
					curv = 0.0f;
				else
					curv = (curv-min_curv) * dcurv;
			}
		}

		depth = cv::Mat_<float>(data.matCloud.rows,data.matCloud.cols);
		float depth_max = 0.0;
		float depth_min = INFINITY;
		for(unsigned j=0; j<data.matCloud.cols; j++){
			for(unsigned i=0; i<data.matCloud.rows; i++){
				if(depth_max < data.matCloud(i,j)[2])
					depth_max = data.matCloud(i,j)[2];
				if(depth_min > data.matCloud(i,j)[2])
					depth_min = data.matCloud(i,j)[2];
			}
		}

		for(unsigned j=0; j<data.matCloud.cols; j++){
			for(unsigned i=0; i<data.matCloud.rows; i++){
				depth(i,j) = (data.matCloud(i,j)[2]-depth_min) / (depth_max-depth_min);
			}
		}
	}


	// Set up TomGine
	if (dbgWin==NULL)
	{
		dbgWin = new TomGine::tgTomGineThread(data.image.cols, data.image.rows);
		if(!save_cloud){
			m_engine = new TomGine::tgEngine(data.image.cols, data.image.rows);

			TomGine::tgImageProcessor ip( data.image.cols, data.image.rows );

			TomGine::tgTexture2D texColor, texDepth, texCurvature, texNormals, texMask;
			TomGine::tgTexture2D texResult;

			texColor.Load(&data.image(0,0),data.image.cols,data.image.rows,GL_RGB, GL_RGB, GL_UNSIGNED_BYTE);
			texDepth.Load(&depth(0,0),depth.cols, depth.rows, GL_LUMINANCE, GL_LUMINANCE, GL_FLOAT);
			texCurvature.Load(&curvature(0,0), curvature.cols, curvature.rows, GL_RED, GL_RED, GL_FLOAT);
			texNormals.Load(&normals(0,0), normals.cols, normals.rows, GL_RGBA, GL_RGBA, GL_FLOAT);
			texMask.Load(&data.mask(0,0),data.mask.cols,data.mask.rows, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_BYTE);

//			ip.setFBO(true);
			ip.rgbdEdges(texColor, texDepth, texCurvature, texResult, 1.0, 20.0, 20.0);

			m_engine->Update();

			dbgWin->SetImage(data.image);
			dbgWin->AddPointCloud(data.matCloud);
		}

		cv::Mat_<float> camR = cv::Mat::eye(3, 3, CV_32F);
		cv::Mat camT = cv::Mat::zeros(3, 1, CV_32F);
		cv::Mat camI(cv::Mat::zeros(3, 3, CV_64F));
		camI.at<double> (0, 0) = camI.at<double> (1, 1) = 525;
		camI.at<double> (0, 2) = 320;
		camI.at<double> (1, 2) = 240;
		camI.at<double> (2, 2) = 1;

		dbgWin->SetCamera(camI);
		dbgWin->SetCamera(camR, camT);

		if(save_cloud){
			printf("[main] fitting NURBS surface ...\n");
			NurbsCreator::FitNurbsSurface(order, refinement, iterations,
					data.matCloud, data.mask, data.contour, dbgWin);
		}else{

		}

		dbgWin->Update();

		while (!dbgWin->Stopped())
		{
			usleep(100000);
		}

		delete (dbgWin);
		delete (m_engine);
	}

	printf("[main] dbgWin deleted\n");


	return 0;
}

