#ifndef CAST_VISUALIZATOR_H_
#define CAST_VISUALIZATOR_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

#include <opencv/cv.h>

#include <vector>
#include <map>


//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision;
using namespace cast;
using namespace boost;

typedef map<string, string> TaskMap;


class Visualizer : public VideoClientProcess
{
  public:
  Visualizer(const string &_id);
  virtual ~Visualizer();
  
  virtual void runComponent();
  virtual void configure(map<string,string> & _config);
  virtual void start();
  
protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
private:
  
  void ShowResultingWindow();
  void drawObjectsOnLayer();
  void drawLayerOnImage();

  //captured image
  Vision::ImageFrame Image;
  IplImage *img;
  //image for objects layer
  IplImage *lay;

  /// input camera to use
  int m_camera;
  
  /// number of frames to skip for reading WM data
  int frameSkip;  
 
}; // Class Visualizer

#endif
