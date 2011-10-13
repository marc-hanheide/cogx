/**
 * $Id$
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <v4r/PCore/Point3fCol.hh>
#include <v4r/PCLAddOns2/PlanePopout.hh>
#include <v4r/PCLAddOns2/utils/PCLUtils.h>


using namespace std;



/**
 * The openni grabber and the main loop
 */
class OpenNIProcessor
{
public:
  bool stop;

  cv::Mat_<cv::Vec4f> matCloud;
  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<ushort> labels;
  cv::Mat_<uchar> mask;
  vector<unsigned> sizeClusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered;
  pcl::PointIndices popouts;

  cv::Ptr<pcl::Grabber> interface;
  pclA::PlanePopout planePopout;

  OpenNIProcessor () : stop(false)
  {
    cloudFiltered.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  }

  /**
   * The main loop..
   */
  void CallbackCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    cout<<"Have a new image: "<<cloud->width<<"x"<<cloud->height<<endl;

    if (!stop)
    {
      planePopout.FilterZ(cloud, *cloudFiltered);
      planePopout.DetectPopout(cloudFiltered, popouts);
      planePopout.ConvertPopout2Mat(*cloud, popouts, matCloud);
      planePopout.LabelClusters(matCloud, labels, sizeClusters);
      planePopout.CreateMaskLargest(labels, sizeClusters, mask);
      //planePopout.CreateMaskAll(labels, sizeClusters, mask);
      //planePopout.ConvertPointCloud2Mat(*cloud, matCloud);

      pclA::ConvertPCLCloud2Image(cloud, image);

      cv::imshow("Image",image);
      cv::imshow("Mask",mask);
    }

    int key = cv::waitKey(100);
    if (((char)key)==27 ) {stop = true;}
  }
  
  /**
   * just start the grabber and init the bind callback
   */
  void run ()
  {
    interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
      boost::bind (&OpenNIProcessor::CallbackCloud, this, _1);

    interface->registerCallback (f);

    interface->start ();

    while (!stop)
      sleep(1);

    interface->stop ();
  }
};




/**
 * main
 */
int main ()
{
  cv::namedWindow("Image", 1);
  cv::namedWindow("Mask", 1);

  OpenNIProcessor v;
  v.run ();

  cv::destroyWindow("Image");
  cv::destroyWindow("Mask");

  return 0;
}


/***************************** SOME METHODES ***********************************/

