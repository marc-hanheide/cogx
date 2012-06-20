/**
 * @file TomGineThread.hh
 * @author Richtsfeld, Prankl
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the tomGine, implementing a threaded 3d render window.
 */

#ifndef TGTHREAD_TOMGINE_THREAD_HH
#define TGTHREAD_TOMGINE_THREAD_HH


#include <opencv2/core/core.hpp>

#include <vector>
#include "Pose.hh"
#include "tgEngine.h"
#include "tgTexture.h"
#include "tgCamera.h"
#include "tgMathlib.h"
#include "tgRenderModel.h"
#include "tgMaterial.h"
#include "tgModel.h"
#include "tgLabel.h"
#include "tgPose.h"


namespace TGThread
{

using namespace std;
  
class TomGineThread
{
private:
  int width, height;                             // window width and height
  int mode;                                      // 
  bool stopTomGineThread;                        // stop the tomeGine thread

  pthread_t thread;                              // tomGine thread
  pthread_mutex_t dataMutex;                     // mutex for data

  //--- data for drawing ---------------
  TomGine::tgCamera tgCam;                       // tomGine camera model
  TomGine::tgMaterial tgMat;                     // tomGine material model
  TomGine::tgTexture *tgTex;                     // tomGine texture model

  TomGine::tgLabel l;

  struct Label                                   // label structure
  {
    bool render;
    std::string lL;
    std::string lLS;
    std::string lLE;
    cv::Point3d lP;
    cv::Point3d lPS;
    cv::Point3d lPE;
    TomGine::tgLabel lLabel;                     // tomGine label of link
    TomGine::tgLabel lLabelS;                    // tomGine label of start node
    TomGine::tgLabel lLabelE;                    // tomGine label of end node
    
  };
  vector<Label> vLabel;                          // vector with all labels and poses

  bool drawImage, draw3D, drawPointCloud;        // draw image, 3D and point clouds
  bool drawVPointCloud;                          // draw vec4f point cloud
  bool drawLabels;                               // draw labels
  bool useProbLevel;                             // use a probability level for pruning
  bool showCoordinateFrame;                      // show a coordinate frame
  
  cv::Mat img;                                   // image
  cv::Mat intrinsic;                             // intrinsic parameters of the camera
  bool camPoseChanged;
  Pose camPose;                                  // camera pose
  cv::Vec3d rotCenter;                           // rotation center
  double coordinateFrameSize;                    // size of coordinate frame
  
  // render elements
  TomGine::tgModel object;                       // object model to display
  
  cv::Mat_<cv::Point3f> mCloud;                  // point cloud
  cv::Mat_<cv::Point3f> cCloud;                  // rgb color of point cloud
  cv::Mat_<cv::Vec4f> vCloud;                    // point cloud with rgb color (float encoded)
  cv::Mat_<cv::Vec4f> vClouds;                   // point clouds with rgb color (float encoded)

  vector<cv::Point3f> points3D;                  // points in 3D
  vector<cv::Point3f> colPoints3D;               // color of the 3D points
  vector<float> sizePoints3D;                    // size of each 3D point
  
  vector<pair<cv::Vec3f, cv::Vec3f> > lines3D;   // lines in 3D
  vector<cv::Vec3f> lineCols3D;                  // color of 3D lines
  vector<float> lineProbs3D;                     // probability of 3D lines
  vector<std::string> lineLabels;                // string with line labels
  vector<std::string> lineLabelsStart;           // string with node label @ start
  vector<std::string> lineLabelsEnd;             // string with node label @ end
  
  vector<vector<cv::Vec3f> > polygons3D;         // polygons in 3D
  vector<cv::Vec4f> colPolygons3D;               // color of 3D polygons
  vector<bool> filledPolygon3D;                  // draw filled 3D polygons


  //--- drawing methods --------------
  void Draw3D(TomGine::tgEngine &render);  
  bool KeyHandler(std::vector<blortGLWindow::Event> &events);
  void SetTGCamera(unsigned width, unsigned height, double zFar, double zNear, Pose &pose,
         cv::Mat &intrinsic, TomGine::tgCamera &tgCam);
  
  void DrawPointCloud();
  void DrawVPointCloud();
  void DrawPoints3D();
  void DrawLines3D();
  void DrawLabels3D(TomGine::tgEngine &render);

  friend void* ThreadDrawing(void* c);


public:
  TomGineThread(int w, int h);
  ~TomGineThread();

  void SetParameter(cv::Mat &_intrinsic);
  void SetCamera(cv::Mat &R, cv::Mat &t, cv::Vec3d &_rotCenter);
  void SetRotationCenter(cv::Vec3d &_rotCenter);
  void SetShapeModel(TomGine::tgModel &tgmodel);
  void SetImage(cv::Mat &_img);
  void SetCoordinateFrame(double size = 0.5);
  
  void AddPoint3D(double x, double y, double z, uchar r=0, uchar g=0, uchar b=0, double size=1);
  void AddLine3D(double x1, double y1, double z1, double x2, double y2, double z2, uchar r=0, uchar g=0, uchar b=0, 
		 float probability = 1.0, string lLabel = "l", string lLabelS = "s", string lLabelE = "e");
  void AddLine3D(cv::Point3d p0, cv::Point3d p1, float probability = 1.0, string lLabel = "l", string lLabelS = "s",
		 string lLabelE = "e", uchar r=0, uchar g=0, uchar b=0);
  void AddGraphModel(std::vector<cv::Point3d> first, std::vector<cv::Point3d> second, std::vector<double> probability, 
		      std::vector<std::string> link, std::vector<std::string> node_0, std::vector<std::string> node_1,
		      uchar r=0, uchar g=0, uchar b=0);
          
  void SetPointCloud(cv::Mat_<cv::Point3f> &matCloud, cv::Mat_<cv::Point3f> &colCloud);       // TODO  Remove this command => Use AddPointCloud instead!
  void AddPointCloud(cv::Mat_<cv::Vec4f> &matCloud);                                          // TODO Das sind Set Functionen, weil sie nur einmal verwendet werden können: Ändern
  void AddPointCloud(std::vector<cv::Vec4f> &vecCloud);
  void AddPointClouds(vector< cv::Mat_<cv::Vec4f> > &vecClouds);                              // TODO Nur mehr eine Repräsentation für Punkte und die wird vergrößert!!
  
  void AddConvexHull(cv::Mat_<cv::Vec4f> &vecHull);
  void AddConvexHull(std::vector<cv::Vec4f> &vecHull);
  void AddConvexHulls(vector< cv::Mat_<cv::Vec4f> > &vecHulls);
  
  void AddHullPrism(std::vector<cv::Vec4f> &vecHull);

  void Clear();
};

}

#endif

