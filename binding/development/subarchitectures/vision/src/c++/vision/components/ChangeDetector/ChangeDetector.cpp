#include "ChangeDetector.h"
#include <vision/VisionGoals.h>
#include <vision/idl/Vision.hh>

#include <vision/utils/VisionUtils.h>
#include <opencv/highgui.h>

#include <sstream>

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new ChangeDetector(_id);
  }
}

ChangeDetector::ChangeDetector(const string &_id) : 
  
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id),
  m_lastImage(NULL), m_bAnythingHappened(false), m_nImagesBeforeSignal(0)
{
  //set default value for frame delay
  m_frameSkip = 2;
  m_signalDelay = 3;

  //0.3% of the image changed
  m_threshold = 0.003;

  m_camera = 0;

  m_observedComponent = "";

  //what to write as default in the scene processed field when scene is static
  m_processed = false;

} // ChangeDetector::ChangeDetector

ChangeDetector::~ChangeDetector() 
{
  if (m_lastImage != NULL)
    cvReleaseImage(&m_lastImage);
} // ChangeDetector::~ChangeDetector


/// Options: \n
/// -c .. specify which camera to observe, default \b 0 \n
/// -s .. frameskip (currently not used, \p sleepProcess(100) is used instead), deafult \b 2 \n
/// -d .. signal delay: how many consecutive still frames do we need to consider the scene stable (not changing), default \b 3 \n
/// -p .. value for the m_sceneProcessed flag in the \p Vision::SceneChanged struct (will be obsolete soon), default \b false \n
/// -i .. ID of observed component (currently not used), default \b "" \n
void ChangeDetector::configure(map<string,string> & _config) {

  ManagedProcess::configure(_config);
  VideoClientProcess::configure(_config);

  ostringstream outStream;
 
  if(_config["-s"] != "") {
    istringstream configStream(_config["-s"]);
    configStream >> m_frameSkip;

    outStream.str("");
    outStream<<"setting frame skip: "<<m_frameSkip;
    log(outStream.str());
  }

  if(_config["-d"] != "") {
    istringstream configStream(_config["-d"]);
    configStream >> m_signalDelay;

    outStream.str("");
    outStream<<"setting signal delay: "<<m_signalDelay;
    log(outStream.str());
  }

  if(_config["-c"] != "")
    {
    istringstream configStream(_config["-c"]);
    configStream >> m_camera;

    outStream.str("");
    outStream<<"setting camera: "<<m_camera;
    log(outStream.str());
  }

  if(_config["-i"] != "") {
    istringstream configStream(_config["-i"]);
    configStream >> m_observedComponent;
    
    outStream.str("");
    outStream<<"setting observed ID: "<<m_observedComponent;
    log(outStream.str());
  }

  if(_config["-p"] != "") {
    string pval = _config["-p"];
    cout<<pval<<endl;
    if(pval == "true") {
      m_processed = true;
    }
    else {
      m_processed = false;
    }

    outStream.str("");
    outStream<<"setting scene processed value: "<<m_processed;
    log(outStream.str());
  }


}

void ChangeDetector::taskAdopted(const string &_taskID) 
{
  // Do nothing because the tasks are not proposed by this component.
} // ChangeDetector::taskAdopted

void ChangeDetector::taskRejected(const string &_taskID) 
{
  // Do nothing.
}

void ChangeDetector::runComponent() 
{
  sleepProcess(3000);

  // Add empty struct to the working memory. Whenever this struct will be
  // overwritten, this will indicate that the scene has changed
  Vision::SceneChanged* sc = new Vision::SceneChanged();
  sc->m_sceneChanging = false;
  sc->m_sceneChanged = false;
  sc->m_sceneProcessed = false;
  sc->m_camNum = m_camera;
  this->m_memoryID = this->newDataID();
  this->addToWorkingMemory<Vision::SceneChanged>(m_memoryID, sc, cdl::BLOCKING);

  while(m_status == STATUS_RUN) {
    GetAndProcessImage();
    //make it a little less keen ;) -- should just put a skip in 
    sleepProcess(100);
  }
}

/// Copies image data from \p ImageFrame structure to a \p IplImage object.
IplImage* ChangeDetector::buffer2image(Vision::ImageFrame* _pImage)
{
  IplImage *tmpimg = cvCreateImage(cvSize(_pImage->m_width, _pImage->m_height), 
      IPL_DEPTH_8U, 3);
  unsigned char* dst = (unsigned char*) tmpimg->imageData;
  char* src = (char *)&(_pImage->m_image[0]);
  memcpy(dst, src, _pImage->m_image.length());

  return tmpimg;
} // ChangeDetector::buffer2image

/// Converts color format from RGB to HSV.
IplImage* ChangeDetector::rgb2hsv(IplImage* _pImage)
{
  IplImage* hsvImage = cvCreateImage(
      cvSize(_pImage->width, _pImage->height), IPL_DEPTH_8U, 3);
  
  cvCvtColor(_pImage, hsvImage, CV_BGR2HSV);

  return hsvImage;
} // ChangeDetector::rgb2hsv

/// Gets a new image frame from the \p VideoServer and compares it to the previous frame.
/// If the difference exceeds the threshold, the scene is considered to be under
/// change for the current frame. After a certain number of unchanged frames (3 by default),
/// the scene is considered stable.\n \n While the scene is changing, the function is constantly
/// overwriting the \p Vision:SceneChanged struct in working memory setting the
/// \p m_sceneChanging flag to true and \p m_sceneChanged flag to false. After the scene has
/// stabilised, the \p m_sceneChanging flag is set to false and \p m_sceneChanged flag to true.
/// After that the \p Vision:SceneChanged struct is not overwriten until the scene starts changing again 
void ChangeDetector::GetAndProcessImage()
{
  Vision::ImageFrame Image;
  getImage(m_camera, Image);
  int imageWidth = Image.m_width;
  int imageHeight = Image.m_height;
  //how many changed pixels to look for
  float changedPixels = (imageWidth * imageHeight) * m_threshold;    
  IplImage* image = this->buffer2image(&Image);

  IplImage* grayImage = cvCreateImage(cvSize(image->width, image->height), IPL_DEPTH_8U, 1);
  cvCvtColor(image,grayImage, CV_BGR2GRAY);

  if (this->m_lastImage)  {

    /*cvNamedWindow("test");
    cvShowImage("test",m_lastImage);
    cvWaitKey(100);*/

    IplImage* diffImage = cvCreateImage(cvSize(imageWidth, imageHeight), 
					IPL_DEPTH_8U, 1);
    

    // Get the difference image.
    cvAbsDiff(grayImage, this->m_lastImage, diffImage);
    cvThreshold(diffImage, diffImage, 20, 255,
      CV_THRESH_BINARY);

 
    if (cvCountNonZero(diffImage) > changedPixels) {
      this->m_nImagesBeforeSignal = m_signalDelay;
      this->m_bAnythingHappened = true;
    }
    else {
      this->m_nImagesBeforeSignal--;
    } // if - else

    cvReleaseImage(&diffImage);


    // scene change was happening but no change in the last couple of images
    // so scene is static again and has changed
    if ((m_bAnythingHappened) && (m_nImagesBeforeSignal == 0)) {

      this->m_bAnythingHappened = false;

      Vision::SceneChanged* sc = new Vision::SceneChanged();

      sc->m_sceneChanging = false;
      sc->m_sceneChanged = true;
      sc->m_sceneProcessed = m_processed;
      sc->m_camNum = m_camera;

      this->overwriteWorkingMemory<Vision::SceneChanged>(this->m_memoryID, sc, cdl::BLOCKING);
      log("scene changed");
    } // if
    // scene is undergoing change and has not settled yet
    else if(m_bAnythingHappened)
    {
      Vision::SceneChanged* sc = new Vision::SceneChanged();

      sc->m_sceneChanging = true;
      sc->m_sceneChanged = false;
      sc->m_sceneProcessed = false;
      sc->m_camNum = m_camera;

      this->overwriteWorkingMemory<Vision::SceneChanged>(this->m_memoryID, sc, cdl::BLOCKING);

      log("scene changing");
    }

    // Destroy previous image.
    cvReleaseImage(&(this->m_lastImage));
  } // if

  this->m_lastImage = grayImage;

  // Free the input image.
  if (image != NULL)
    cvReleaseImage(&image);	
}

void ChangeDetector::redrawGraphicsText()
{
//  printText("tralala");
}


