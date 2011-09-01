/*
 * DataLoading.h
 *
 *  Created on: Apr 10, 2011
 *      Author: moerwald
 */

#include <string>
#include <opennurbs.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#ifndef DATALOADING_H_
#define DATALOADING_H_

struct DataSet{
	int ID;
	cv::Mat mask, depth;
	cv::Mat camera, distortion;
	cv::Mat R, Rvec, T;
	ON_PointCloud interior;
	ON_PointCloud interior_normal;
	ON_PointCloud boundary;
	ON_PointCloud outer;
};

class DataManager{
public:
	DataManager();
	~DataManager();

	DataSet* operator[](const unsigned &i);

	void getDataSetFilenames(	int setnr,
							const std::string file_path,
							std::string &pc_file_yml,
							std::string &mask_file,
							std::string &camera_file,
							std::string &pose_file);

	void loadDataKinect(const std::string &pc_file_yml,
						const std::string &mask_file,
						const std::string &camera_file,
						const std::string &pose_file,
						bool calc_normals);

	void inpaintMask(DataSet* data1, DataSet* data2);

	void mergePointClouds(ON_PointCloud &pc1, ON_PointCloud &bnd1, ON_PointCloud &pc2, ON_PointCloud &bnd2, ON_PointCloud &pcn, ON_PointCloud &bndn);

	void randomPointCloud();

	unsigned char m_inpaintlvl;
	unsigned char m_zerolvl;
	double m_z_tol;


	std::vector<DataSet*> data;

};

#endif /* DATALOADING_H_ */
