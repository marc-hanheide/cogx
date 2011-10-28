/**
 * @file KinectPclEdges.cpp
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D edge images from Kinect data with Tom's RGBD-Segmenter.
 * 
 * 
 */

// // #include <time.h>
#include "KinectCore.h"
#include "KinectPclEdges.h"
#include "PclLine3D.h"

namespace Z
{

/**
 * @brief Constructor of KinectPclEdges
 * @param kc Kinect core
 * @param vc Vision core
 */
KinectPclEdges::KinectPclEdges(KinectCore *kc, VisionCore *vc) 
               : KinectBase(kc, vc)
{
  numEdgels = 0;
  tgEngine = NULL;
  rgbdSegment = NULL;
}

/**
 * @brief Clear all arrays.
 */
void KinectPclEdges::ClearResults()
{
  numEdgels = 0;
}


/**
 * @brief Calculate 3D edges from kinect data.
 */
void KinectPclEdges::Process()
{
  double fx, fy, cx, cy;
  kcore->GetScaledCameraParameters(fx, fy, cx, cy);
  cv::Mat_<cv::Vec4f> normals = kcore->GetCvNormals();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pc;
  int rows = kcore->GetPointCloudHeight();
  int cols = kcore->GetPointCloudWidth();
  
//   pclA::Dilation(kcore->GetPclCloud(), pcl_pc, fx, fy, cx, cy, 1);
//   pclA::Dilation(pcl_pc2, pcl_pc, kcore->fx/rgb2depthScale, kcore->fy/rgb2depthScale, kcore->cx/rgb2depthScale,kcore->cy/rgb2depthScale, 1);
  pclA::CopyPointCloud(kcore->GetPclCloud(), pcl_pc);

  // set parameter for segmenting edges
  float zMin = 0.3;
  float zMax = 2.3;
  float weightColorEdge = 0.05;       // 0.05
  float weightDepthEdge = 10;         // 4
  float weightCurvatureEdge = 40.;    // 40
  float weightMaskEdge = 0.8;         // 0.8
  RGBDSegment::Parameter p = RGBDSegment::Parameter(zMin, zMax, weightColorEdge, weightDepthEdge,
                          weightCurvatureEdge, weightMaskEdge, 50.0, 25.0, 0.02, 20, 1);

  if(tgEngine == NULL)
    tgEngine = new TomGine::tgEngine(cols, rows);
  if(rgbdSegment == NULL)
    rgbdSegment = new RGBDSegment(tgEngine, p);

  // Filter points with z-value
  pclA::FilterZ(pcl_pc, zMin, zMax);
  

  /// TODO We need color, depth, mask, curvature
  
  cv::Mat_<float> color_edges = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> depth_edges = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> curvature_edges = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> mask_edges = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> edges = cv::Mat_<float>(rows, cols);
  
  // get image and mask
  cv::Mat_<cv::Vec3b> image = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertPCLCloud2Image(pcl_pc, image);
  cv::Mat_<uchar> mask = cv::Mat_<uchar>(rows, cols);
  pclA::ConvertPCLCloud2Mask(pcl_pc, mask, false, false);
 
  cv::Mat_<float> depth = cv::Mat_<float>(rows, cols);
  cv::Mat_<float> curvature = cv::Mat_<float>(rows, cols);
  rgbdSegment->DetectEdges(kcore->GetPointCloud(), normals, image, mask, depth, curvature, color_edges, depth_edges, curvature_edges, mask_edges, edges);
  
  /// basic images (mask, depth, curvature)
  cv::Mat_<cv::Vec3b> color_edge_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertCvMat2Image(color_edges, color_edge_image_2, true);
  cv::imshow("Grey-level color edge image 2", color_edge_image_2);
  
  cv::Mat_<cv::Vec3b> cur_edge_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertCvMat2Image(curvature_edges, cur_edge_image_2, true);
  cv::imshow("Grey-level curvature edge image 2", cur_edge_image_2);

  cv::Mat_<cv::Vec3b> mask_edge_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertCvMat2Image(mask_edges, mask_edge_image_2, true);
  cv::imshow("Grey-level mask edge image 2", mask_edge_image_2);
  
//   cv::Mat_<cv::Vec3b> edge_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
//   pclA::ConvertCvMat2Image(edges, edge_image_2, true);
//   cv::imshow("Grey-level edge image 2", edge_image_2);
  
//   cv::Mat_<cv::Vec3b> gradient_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
//   pclA::ConvertCvMat2Image(gradients, gradient_image_2, true);
//   cv::imshow("Grey-level gradient image 2", gradient_image_2);
  
  /// TODO TODO TDOO depth edges verarbeiten!!!
  cv::Mat_<cv::Vec3b> depth_edge_image = cv::Mat_<cv::Vec3b>(rows, cols);
  pclA::ConvertCvMat2Image(depth_edges, depth_edge_image, true);
  cv::imshow("Grey-level depth edge image orginal", depth_edge_image);
  
  float depthThreshold = 0.5;
  for(unsigned row=0; row<rows; row++)
    for(unsigned col=0; col<cols; col++)
    {
      if(depth_edges(row, col) > depthThreshold)
        depth_edges(row, col) = 1.;
      else
        depth_edges(row, col) = 0.;
    }
  
  /// Get edges from vision core  // TODO Funktioniert nicht für 320x240!!!
//   IdImage *idImage = ((FormSegments*) vcore->Principles(GestaltPrinciple::FORM_SEGMENTS))->edge_img;
//   cv::Mat_<float> vs3_edges = cv::Mat_<float>(rows, cols);
// 
//   for(unsigned i=0; i<idImage->height; i++)
//     for(unsigned j=0; j<idImage->width; j++)
//       if(idImage->data[i*idImage->width + j] != -1)
//       {
//         if(depth_edges(i, j) == 0.) // only if edge is not already a depth edge
//           depth_edges(i, j) = 0.5;
//         vs3_edges(i,j) = 1.;
//       }
//       else vs3_edges(i, j) = 0.;
//       
//   cv::Mat_<cv::Vec3b> depth_edge_image_2 = cv::Mat_<cv::Vec3b>(rows, cols);
//   pclA::ConvertCvMat2Image(depth_edges, depth_edge_image_2, true);
//   cv::imshow("depth edge + vs3 color image", depth_edge_image_2);
//   
//   cv::Mat_<cv::Vec3b> vs3_edge_image = cv::Mat_<cv::Vec3b>(rows, cols);
//   pclA::ConvertCvMat2Image(vs3_edges, vs3_edge_image, true);
//   cv::imshow("vs3 edge image", vs3_edge_image);
  

  tgEngine->Update();
printf("KinectPclLines::Process: ended\n");

}


}








