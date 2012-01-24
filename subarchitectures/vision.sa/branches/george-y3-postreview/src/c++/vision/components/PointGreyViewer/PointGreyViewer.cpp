/**
 * @file PointGreyViewer.cpp
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2010, Februar 2009
 * @version 0.1
 * @brief Just receives stereo images and displays them.
 * A dummy component showing how to get images. 
 */

#include <opencv/highgui.h>
#include <VideoUtils.h>
#include "PointGreyViewer.h"


/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::PointGreyViewer();
  }
}


namespace cast
{

using namespace std;
using namespace VisionData;


/**
 * @brief Configure component.
 * @param _config Configuration
 */
void PointGreyViewer::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

	if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if(videoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));

	getImage = false;
}


/**
 * @brief Start PointGreyViewer component.
 */
void PointGreyViewer::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

	// open cv-windows
  cvNamedWindow("PointGreyViewer: LEFT", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("PointGreyViewer: RIGHT", CV_WINDOW_AUTOSIZE);

  // start receiving images pushed by the video server
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
  receiving = true;
}


/**
 * @brief Destroy PointGreyViewer component.
 */
void PointGreyViewer::destroy()
{
  cvDestroyWindow("PointGreyViewer: LEFT");
  cvDestroyWindow("PointGreyViewer: RIGHT");
}


/**
 * @brief Receive images from the PointGreyServer
 * @param images Images from the PointGreyServer
 */
void PointGreyViewer::receiveImages(const std::vector<Video::Image>& images)
{
	if(images.size() == 2)
	{
		IplImage *iplImage0 = convertImageToIpl(images[0]);
		cvShowImage("PointGreyViewer: LEFT", iplImage0);
		cvReleaseImage(&iplImage0);

		IplImage *iplImage1 = convertImageToIpl(images[1]);
		cvShowImage("PointGreyViewer: RIGHT", iplImage1);
		cvReleaseImage(&iplImage1);
	}
	else 
		println("wrong size of image array.");
}


/**
 * @brief Run component
 */
void PointGreyViewer::runComponent()
{
  println("press <s> to stop/start receving images");
  while(isRunning())
  {
    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    int key = cvWaitKey(100);
		if (key != -1) debug("pressed key: %c", (char) key);

    switch((char) key)
    {
      case 's':  // start/stop getting images
        if(receiving)
        {
          if(!getImage) videoServer->stopReceiveImages(getComponentID().c_str());
          println("stopped receving images");
        }
        else
        {
          if(!getImage) videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0); // changes the size not
          println("started receving images");
        }
        receiving = !receiving;
        break;
      case 'c':  // change receiving images mode
          println("change between getImage and receiveImage");
          getImage = !getImage;
				break;
      case '1':  // change between different image resolutions
          println("set image resolution: 320x240.");
					if(!getImage)
					{
						int width = 320;
						int height = 240;
						videoServer->stopReceiveImages(getComponentID().c_str());
						videoServer->startReceiveImages(getComponentID().c_str(), camIds, width, height);
					}
				break;
      case '2':  // change between different image resolutions
          println("set image resolution: 640x480.");
					if(!getImage)
					{
						int width = 640;
						int height = 480;
						videoServer->stopReceiveImages(getComponentID().c_str());
						videoServer->startReceiveImages(getComponentID().c_str(), camIds, width, height);
					}
				break;
      case '3':  // change between different image resolutions
          println("set image resolution: 800x600.");
					if(!getImage)
					{
						int width = 800;
						int height = 600;
						videoServer->stopReceiveImages(getComponentID().c_str());
						videoServer->startReceiveImages(getComponentID().c_str(), camIds, width, height);
					}
				break;
      case '4':  // change between different image resolutions
          println("set image resolution: 1024x768.");
					if(!getImage)
					{
						int width = 1024;
						int height = 768;
						videoServer->stopReceiveImages(getComponentID().c_str());
						videoServer->startReceiveImages(getComponentID().c_str(), camIds, width, height);
					}
				break;
			case '5':  // change between different image resolutions
					println("set image resolution: 1280x960.");
					if(!getImage)
					{
						int width = 1280;
						int height = 960;
						videoServer->stopReceiveImages(getComponentID().c_str());
						videoServer->startReceiveImages(getComponentID().c_str(), camIds, width, height);
					}
				break;
			case 'g': // get image with VideoMode7 pruned (640x480) from the high-resolution image (1280x960)
				{				// we need this face brackets for the initialisation of variables
					videoServer->stopReceiveImages(getComponentID().c_str());
					lockComponent();

					cout << "VideoServer: " << videoServer->getServerName() << endl;

					// set Format7 properties
					println("get image pruned from high resolution: 1280x960 => 640x480.");
					int offsetX = 0;
					int offsetY = 0;
					videoServer->changeFormat7Properties(640, 480, offsetX, offsetY, 0, 1600);

					// get images and display
					Video::Image image_l, image_r;
					videoServer->getImage(camIds[0], image_l);
					videoServer->getImage(camIds[1], image_r);
					IplImage *iplImage0 = convertImageToIpl(image_l);
					cvShowImage("PointGreyViewer: LEFT", iplImage0);
					cvReleaseImage(&iplImage0);
					IplImage *iplImage1 = convertImageToIpl(image_r);
					cvShowImage("PointGreyViewer: RIGHT", iplImage1);
					cvReleaseImage(&iplImage0);

					// reset Format7 properties
					videoServer->changeFormat7Properties(640, 480, 0, 0, 1, 1600);

					unlockComponent();
					videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
				}
				break;
			case 'm':
					static int toggle = 0;
					if(toggle == 0)
						videoServer->changeFormat7Properties(640, 480, 0, 0, 0, 1600);
					else if(toggle == 1)
						videoServer->changeFormat7Properties(640, 480, 640, 0, 0, 1600);
					else if(toggle == 2)
						videoServer->changeFormat7Properties(640, 480, 0, 480, 0, 1600);
					else if(toggle == 3)
						videoServer->changeFormat7Properties(640, 480, 640, 480, 0, 1600);
					else if(toggle == 4)
						videoServer->changeFormat7Properties(640, 480, 0, 0, 1, 1600);
					toggle++;
					if(toggle > 4) toggle = 0;
        break;
      case 'h':  // show help
					println("c ... change between getImage() and receiveImage() call\n"
									"s ... start/stop the receive image pushing from the video server.\n"
									"m ... change Format7-mode properties.\n"
									"g ... get image pruned from high resolution image.\n"
									"1-5 ... set different image sizes: 320x240 / 640x480 / 800x600 / 1024x768 / 1280x960");
        break;
      default:
        break;
    }

		// get image testing
		if (getImage)
		{
			printf("getImage\n");
			Video::Image image_l, image_r;

			videoServer->getImage(camIds[0], image_l);
			videoServer->getImage(camIds[1], image_r);

			IplImage *iplImage0 = convertImageToIpl(image_l);
			cvShowImage("PointGreyViewer: LEFT", iplImage0);
			cvReleaseImage(&iplImage0);

			IplImage *iplImage1 = convertImageToIpl(image_r);
			cvShowImage("PointGreyViewer: RIGHT", iplImage1);
			cvReleaseImage(&iplImage0);
		}
  }
}

}

