
#include <external/sba-1.6/SBAWrapper.hh>

#include <opencv/cv.h>

#include <map>
#include <iostream>
#include <stdexcept>
#include <v4r/PGeometry/Pose.hh>

int main(int argc, char *argv[] ){
	
	CvMat* pRvecs = (CvMat*)cvLoad("rvecs.xml");
	CvMat* pTvecs = (CvMat*)cvLoad("tvecs.xml");
	CvMat* pImagePoints = (CvMat*)cvLoad("imagePoints.xml");
	CvMat* pObjectPoints = (CvMat*)cvLoad("objectPoints.xml");
	CvMat* pIntrinsic = (CvMat*)cvLoad("intrinsicDistored.xml");
	
	cv::Mat Rvecs = pRvecs;
	cv::Mat Tvecs = pTvecs;
	cv::Mat ImagePoints = pImagePoints;
	cv::Mat ObjectPoints = pObjectPoints;
	cv::Mat Intrinsic = pIntrinsic;
	
	unsigned frames = Rvecs.size().height;
	unsigned corners = ImagePoints.size().height / frames;
	
	if(frames != Tvecs.size().height){
		throw std::runtime_error("ImagePoints and ObjectPoints don't match");
	}
	
	cout << "Frames: " << frames << endl;
	cout << "Corners: " << corners << endl;
	
	P::SBAWrapper bundler(500,Intrinsic);
	cv::Mat R, T, matR;
	vector< pair<unsigned,cv::Point2d> > projs;
	P::Pose pose, invPose;
	
	for(unsigned i=0; i<frames; i++){
		R = Rvecs.row(i);
		cv::Rodrigues(R,matR);
		T = Tvecs.row(i);
		bundler.InsertCamera(matR, T);
	}
	
	for(unsigned i=0; i<corners; i++){
		cv::Point3d po(ObjectPoints.at<double>(i,0), ObjectPoints.at<double>(i,1), ObjectPoints.at<double>(i,2));
		projs.clear();
		for(unsigned j=0; j<frames; j++){
			unsigned k = j * corners + i;
			cv::Point2d pi(ImagePoints.at<double>(k,0), ImagePoints.at<double>(k,1));
			pair<unsigned,cv::Point2d> proj(j,pi);
			projs.push_back(proj);
		}
		bundler.InsertPoints(po, projs);
	}
	
	bundler.BundleMot();
	
	cv::Mat RvecsRefined = Rvecs;
	cv::Mat TvecsRefined = Tvecs;
	
	std::vector< cv::Ptr<P::Pose> > cameras;
	std::vector<cv::Point3d> points;
	bundler.GetData(cameras, points);
	
	for(unsigned i=0; i<cameras.size(); i++)
	{
		TvecsRefined.at<double>(i,0) =  cameras[i]->t.at<double>(0,0);
		TvecsRefined.at<double>(i,1) =  cameras[i]->t.at<double>(0,1);
		TvecsRefined.at<double>(i,2) =  cameras[i]->t.at<double>(0,2);
		
		cv::Rodrigues(cameras[i]->R, R);
		RvecsRefined.at<double>(i,0) = R.at<double>(0,0);
		RvecsRefined.at<double>(i,1) = R.at<double>(0,1);
		RvecsRefined.at<double>(i,2) = R.at<double>(0,2);
	}
	
	CvMat cvRvecsRefined = RvecsRefined;
	CvMat cvTvecsRefined = TvecsRefined;
	cvSave("rvecsRefined.xml", &cvRvecsRefined);
	cvSave("tvecsRefined.xml", &cvTvecsRefined);
	
	
	return 0;
}
