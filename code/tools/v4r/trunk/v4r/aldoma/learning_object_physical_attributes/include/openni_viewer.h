#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

template <typename PointType>
class SimpleOpenNIViewer
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  SimpleOpenNIViewer(pcl::OpenNIGrabber& grabber)
    : cloud_viewer_("PCL OpenNI Viewer"),
      grabber_(grabber)
  {
  }

  /**
   * @brief Callback method for the grabber interface
   * @param cloud The new point cloud from Grabber
   */
  void
  cloud_callback (const CloudConstPtr& cloud)
  {
    //FPS_CALC ("callback");
    boost::mutex::scoped_lock lock (cloud_mutex_);
    cloud__ = *cloud;
    cloud_ = cloud;
  }

  void keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
  {
    string* message = (string*)cookie;
    cout << (*message) << " :: ";
    if (event.getKeyCode())
      cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
    else
      cout << "the special key \'" << event.getKeySym() << "\' was";
    if (event.keyDown())
      cout << " pressed" << endl;
    else
      cout << " released" << endl;
  }

  void mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
  {
    string* message = (string*) cookie;
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
    {
      cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
  }
  /**
   * @brief swaps the pointer to the point cloud with Null pointer and returns the cloud pointer
   * @return boost shared pointer to point cloud
   */
  CloudConstPtr
  getLatestCloud ()
  {
    //lock while we swap our cloud and reset it.
    boost::mutex::scoped_lock lock(cloud_mutex_);
    CloudConstPtr temp_cloud;
    temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
    //it is safe to set it again from our
    //callback
    return (temp_cloud);
  }

  /**
   * @brief starts the main loop
   */
  void
  run()
  {
    //pcl::Grabber* interface = new pcl::OpenNIGrabber(device_id_, pcl::OpenNIGrabber::OpenNI_QQVGA_30Hz, pcl::OpenNIGrabber::OpenNI_VGA_30Hz);

    /*string mouseMsg3D("Mouse coordinates in PCL Visualizer");
    string mouseMsg2D("Mouse coordinates in image viewer");
    string keyMsg3D("Key event for PCL Visualizer");
    string keyMsg2D("Key event for image viewer");*/

    //cloud_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg3D));
    //cloud_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg3D));

//    image_viewer_.registerMouseCallback (&SimpleOpenNIViewer::mouse_callback, *this, (void*)(&mouseMsg2D));
//    image_viewer_.registerKeyboardCallback(&SimpleOpenNIViewer::keyboard_callback, *this, (void*)(&keyMsg2D));

    boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&SimpleOpenNIViewer::cloud_callback, this, _1);
    boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

    //boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&SimpleOpenNIViewer::image_callback, this, _1);
    //boost::signals2::connection image_connection = grabber_.registerCallback (image_cb);

    grabber_.start();

    while (!cloud_viewer_.wasStopped ())
    {
      if (cloud_)
      {
        //the call to get() sets the cloud_ to null;
        cloud_viewer_.showCloud (getLatestCloud ());
      }

    }

    grabber_.stop();

    cloud_connection.disconnect();
  }

  pcl::visualization::CloudViewer cloud_viewer_;
  pcl::OpenNIGrabber& grabber_;
  boost::mutex cloud_mutex_;
  CloudConstPtr cloud_;
  Cloud cloud__;
};
