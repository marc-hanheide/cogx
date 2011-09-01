/*
 * DataLoading.h
 *
 *  Created on: Apr 10, 2011
 *      Author: moerwald
 */

#include <string>
#include "opennurbs.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "blztools/src/blztools.h"

#ifndef DATALOADING_H_
#define DATALOADING_H_

struct DataSet
{
	int ID;
	cv::Mat mask, depth;
	JImg2d normals;
	cv::Mat camera, distortion;
	cv::Mat R, Rvec, T;
	ON_PointCloud interior;
	ON_PointCloud interior_normal;
	ON_PointCloud boundary;
	ON_PointCloud outer;
};

class DataManager
{
public:
	DataManager();
	~DataManager();

	DataSet* operator[](const unsigned &i);

	void getDataSetFilenames(int setnr, const std::string file_path,
			std::string &pc_file_yml, std::string &mask_file,
			std::string &camera_file, std::string &pose_file);

	void FromCvMat2DataSet(cv::Mat_<cv::Vec4f> matCloud,
			cv::Mat_<uchar> mask, cv::Mat_<uchar> contour);

	void loadDataKinect(const std::string &pc_file_yml,
			const std::string &camera_file, const std::string &pose_file);

	void loadDataKinectMasked(const std::string &pc_file_yml,
			const std::string &mask_file, const std::string &camera_file,
			const std::string &pose_file, bool calc_normals);

	cv::Mat getNormal(int i, int j, cv::Mat &pc);

	cv::Mat getNormalFast(int i, int j, cv::Mat &pc);

	void saveNormalsVTK(DataSet* ds, const std::string &normal_file);

	void inpaintMask(DataSet* data1, DataSet* data2);

	void mergePointClouds(ON_PointCloud &pc1, ON_PointCloud &bnd1,
			ON_PointCloud &pc2, ON_PointCloud &bnd2, ON_PointCloud &pcn,
			ON_PointCloud &bndn);

	void randomPointCloud();

	unsigned char m_inpaintlvl;
	unsigned char m_zerolvl;
	double m_z_tol;

	std::vector<DataSet*> data;

};

#endif /* DATALOADING_H_ */
