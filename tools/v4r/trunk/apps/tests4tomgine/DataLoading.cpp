/*
 * DataLoading.cpp
 *
 *  Created on: Apr 10, 2011
 *      Author: moerwald
 */

#include "DataLoading.h"


char charbuffer[512];

DataManager::DataManager(){

	m_zerolvl = 32;
	m_inpaintlvl = 64;
	m_z_tol = 0.01;

}

DataManager::~DataManager(){

	std::vector<DataSet*>::iterator it;
	it = data.begin();
	for(it=data.begin(); it!=data.end(); it++){

		delete(*it);

	}

}

DataSet* DataManager::operator[](const unsigned &i){

	if(i<data.size())
		return data[i];

	return NULL;

}

void DataManager::getDataSetFilenames(	int setnr,
										const std::string file_path,
										std::string &pc_file_yml,
										std::string &mask_file,
										std::string &camera_file,
										std::string &pose_file)
{

	sprintf(charbuffer, "%scloud-GuteLaune-%04d.yml", file_path.c_str(), setnr);
	pc_file_yml = charbuffer;

	sprintf(charbuffer, "%smask-GuteLaune-%04d.png", file_path.c_str(), setnr);
	mask_file 	= charbuffer;

	sprintf(charbuffer, "%scamera-GuteLaune-%04d.yml", file_path.c_str(), setnr);
	camera_file = charbuffer;

	sprintf(charbuffer, "%spose-GuteLaune-%04d.yml", file_path.c_str(), setnr);
	pose_file	= charbuffer;

}

void DataManager::loadDataKinect(	const std::string &pc_file_yml,
									const std::string &mask_file,
									const std::string &camera_file,
									const std::string &pose_file,
									bool calc_normals)
{
	DataSet* ds = new DataSet();

	printf("[DataManager::loadDataKinect] Initialising data set:\n");

	// Read point cloud
	cv::FileStorage fs;
	if(!fs.open(pc_file_yml.c_str(), cv::FileStorage::READ))
		throw std::runtime_error("[DataManager::loadDataKinect] Cannot open pointcloud yml file.");
	cv::Mat cv_pc;
	fs["cloud"]>>cv_pc;
	printf("  cv_pc: %d %d %d\n", cv_pc.rows, cv_pc.cols, cv_pc.channels());
	fs.release();

	// Read mask
	ds->mask = cv::imread(mask_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	if(ds->mask.empty())
		throw std::runtime_error("[DataManager::loadDataKinect] Cannot open mask file.");
	printf("  ds->mask: %d %d %d\n", ds->mask.rows, ds->mask.cols, ds->mask.channels());

	// Read camera intrinsic
	if(!fs.open(camera_file.c_str(), cv::FileStorage::READ))
		throw std::runtime_error("[DataManager::loadDataKinect] Cannot open camera intrinsic file.");
	fs["intrinsic"]>>ds->camera;
	printf("  cv_camera: %d %d %d\n", ds->camera.rows, ds->camera.cols, ds->camera.channels());
	fs["distortion"]>>ds->distortion;
	printf("  ds->distortion: %d %d %d\n", ds->distortion.rows, ds->distortion.cols, ds->distortion.channels());
	fs.release();

	// Read camera pose
	if(!fs.open(pose_file.c_str(), cv::FileStorage::READ))
		throw std::runtime_error("[DataManager::loadDataKinect] Cannot open camera pose file.");
	fs["R"]>>ds->Rvec;
	printf("  ds->Rvec: %d %d %d\n", ds->Rvec.rows, ds->Rvec.cols, ds->Rvec.channels());
	cv::Rodrigues(ds->Rvec, ds->R);
	fs["T"]>>ds->T;
	printf("  ds->T: %d %d %d\n", ds->T.rows, ds->T.cols, ds->Rvec.channels());
	fs.release();

	// Initialize depth map
	ds->depth = cv::Mat(ds->mask.rows, ds->mask.cols, CV_64F, 0.0);

	int numNan = 0;
	int numMasked = 0;
	int numPoints = 0;
	int numBoundaries = 0;

	printf("[DataManager::loadDataKinect] Loading data ...\n");

	for(int i=0; i<cv_pc.rows; i++){
		for(int j=0; j<cv_pc.cols; j++){

			// get point from cloud;
			double x = (double)(cv_pc.at<cv::Vec3f>(i,j)[0]);
			double y = (double)(cv_pc.at<cv::Vec3f>(i,j)[1]);
			double z = (double)(cv_pc.at<cv::Vec3f>(i,j)[2]);

			if( isnan(x) || isnan(y) || isnan(z) ){

				numNan++;

			}else{


				int I = i; //(int)round(imgPoint.at<double>(1,0) / imgPoint.at<double>(2,0));
				int J = j; //(int)round(imgPoint.at<double>(0,0) / imgPoint.at<double>(2,0));


				if( I<0 || J<0 || I>ds->mask.rows || J>ds->mask.cols){

					printf("Out of range %d %d", I, J);
					break;

				}


				ds->depth.at<double>(I,J) = z;


				if( ds->mask.at<unsigned char>(I,J) > m_inpaintlvl){

					int nouter =	int(ds->mask.at<unsigned char>(I-1,J-1) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I-1,J+0) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I-1,J+1) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+0,J-1) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+0,J+1) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J-1) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J+0) <= m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J+1) <= m_inpaintlvl);

					int ninner =	int(ds->mask.at<unsigned char>(I-1,J-1) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I-1,J+0) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I-1,J+1) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+0,J-1) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+0,J+1) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J-1) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J+0) > m_inpaintlvl) +
									int(ds->mask.at<unsigned char>(I+1,J+1) > m_inpaintlvl);

					if( nouter >= 2 && ninner >= 4 )
					{

						ds->boundary.AppendPoint( ON_3dPoint( x, y, z ) );
						numBoundaries++;

					}else if( ninner == 8 ){

						ds->interior.AppendPoint( ON_3dPoint( x, y, z ) );
						numPoints++;

					}


				}else{

					ds->outer.AppendPoint( ON_3dPoint( x, y, z ) );
					numMasked++;

				}

			}

		}

	}

	data.push_back(ds);

}

void DataManager::inpaintMask(DataSet* data1, DataSet* data2){

	for(int i=0; i<data1->interior.PointCount(); i++){

		cv::Mat point1(3,1,CV_64F);
		point1.at<double>(0,0) = data1->interior[i].x;
		point1.at<double>(1,0) = data1->interior[i].y;
		point1.at<double>(2,0) = data1->interior[i].z;

		// transform from frame 1 to 0
		cv::Mat point0 = data1->R.t() * point1 - data1->R.t() * data1->T;
		// transform from 0 to frame 2
		cv::Mat point2 = data2->R * (point0 + data2->R.t() * data2->T);

		// project to image space
		cv::Mat imgPoint = data2->camera * point2;

		int I = (int)round(imgPoint.at<double>(1,0) / imgPoint.at<double>(2,0));
		int J = (int)round(imgPoint.at<double>(0,0) / imgPoint.at<double>(2,0));

		if( data2->depth.at<double>(I,J) == 0.0 )
			data2->depth.at<double>(I,J) = point2.at<double>(2,0);
		else if( data2->depth.at<double>(I,J) > point2.at<double>(2,0) )
			data2->depth.at<double>(I,J) = point2.at<double>(2,0);

		if( data2->mask.at<unsigned char>(I-1,J+0) > m_zerolvl )
			data2->mask.at<unsigned char>(I-1,J+0) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J-1) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J-1) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J+0) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J+0) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J+1) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J+1) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+1,J+0) > m_zerolvl)
			data2->mask.at<unsigned char>(I+1,J+0) = m_inpaintlvl;

	}

	for(int i=0; i<data1->boundary.PointCount(); i++){

		cv::Mat point0(3,1,CV_64F);
		point0.at<double>(0,0) = data1->boundary[i].x;
		point0.at<double>(1,0) = data1->boundary[i].y;
		point0.at<double>(2,0) = data1->boundary[i].z;

		// transform to frame of data2
		cv::Mat point2 = data2->R * (point0 + data2->R.t() * data2->T);

		// project to image space
		cv::Mat imgPoint = data2->camera * point2;

		int I = (int)round(imgPoint.at<double>(1,0) / imgPoint.at<double>(2,0));
		int J = (int)round(imgPoint.at<double>(0,0) / imgPoint.at<double>(2,0));

		if( data2->depth.at<double>(I,J) == 0.0 )
			data2->depth.at<double>(I,J) = point2.at<double>(2,0);
		else if( data2->depth.at<double>(I,J) > point2.at<double>(2,0) )
			data2->depth.at<double>(I,J) = point2.at<double>(2,0);

		if( data2->mask.at<unsigned char>(I-1,J+0) > m_zerolvl )
			data2->mask.at<unsigned char>(I-1,J+0) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J-1) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J-1) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J+0) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J+0) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+0,J+1) > m_zerolvl )
			data2->mask.at<unsigned char>(I+0,J+1) = m_inpaintlvl;
		if( data2->mask.at<unsigned char>(I+1,J+0) > m_zerolvl)
			data2->mask.at<unsigned char>(I+1,J+0) = m_inpaintlvl;

	}

	cv::namedWindow("myWindow",CV_WINDOW_AUTOSIZE);
	cv::imshow("myWindow", data2->mask);
	cv::waitKey(0);

}

void DataManager::mergePointClouds(ON_PointCloud &pc1, ON_PointCloud &bnd1, ON_PointCloud &pc2, ON_PointCloud &bnd2, ON_PointCloud &pcn, ON_PointCloud &bndn){

	double max_dist_pc = 0.01;
	double max_dist_bnd = 0.01;
	int idx;

	pcn = pc1;

	for(int i=0; i<pc2.PointCount(); i++){

		if( !pc1.GetClosestPoint(pc2[i], &idx, max_dist_pc) )
			pcn.AppendPoint(pc2[i]);

	}

	for(int i=0; i<bnd2.PointCount(); i++){

		if( !pc1.GetClosestPoint(bnd2[i], &idx, max_dist_bnd) )
			bndn.AppendPoint(bnd2[i]);

	}

	for(int i=0; i<bnd1.PointCount(); i++){

		if( !pc2.GetClosestPoint(bnd1[i], &idx, max_dist_bnd) )
			bndn.AppendPoint(bnd1[i]);

	}

	for(int i=0; i<bnd1.PointCount(); i++){

		if( bnd2.GetClosestPoint(bnd1[i], &idx, max_dist_bnd) )
			bndn.AppendPoint(bnd1[i]);

	}

}


