/**
 * @brief Demo program to use Kinect interface with OpenNI driver.
 * @author Richtsfeld
 * @date June 2011
 * 
 * Shows the color image from the kinect and grabs images and point clouds.
 * 
 * Usage: cmake .
 *        make
 *        ./kinectDemo
 */

#include "Kinect.h"
#include "FileReaderWriter.h"


int main(int argc, char** argv)
{
  printf("kinectDemo: Usage:\n"); 
  printf("    q ... Quit: Press button q to quit program.\n"); 
  printf("    g ... Grab: Press button g to grab a jpg color image and a point cloud (binary).\n\n"); 
  printf("    now wait for initialisation ... \n");

  cvNamedWindow("Kinect color image", CV_WINDOW_AUTOSIZE);

  IplImage* kinect_image = 0;
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;

  int imgcnt = 0;
  char filename_kinect[1024];
  char filename_kinect_cloud[1024];

  Kinect::Kinect *kinect;
  kinect = new Kinect::Kinect("KinectConfig.xml");
  kinect->StartCapture(0);
  printf("    initialisation done: started grabing from Kinect.\n\n");

  int key;
  while((key = (char)cvWaitKey(10)) != 'q')
  {
    kinect->GetColorImage(&kinect_image);
    cvShowImage("Kinect color image", kinect_image);
    
    if(key == 'g')
    {
      snprintf(filename_kinect, 1024, "img%03d.jpg", imgcnt);
      snprintf(filename_kinect_cloud, 1024, "img%03d-C.pts", imgcnt++);

      kinect->Get3dWorldPointCloud(cloud, colCloud, 1);
      AR::writeToFile(filename_kinect_cloud, cloud, colCloud);

      cvSaveImage(filename_kinect, kinect_image);

      printf("kinectDemo: Write image %s and point cloud %s to disk.\n", filename_kinect, filename_kinect_cloud); 
    }

  }
  
  cvReleaseImage(&kinect_image);
  cvDestroyWindow("Stereo left");
}