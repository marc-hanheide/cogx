#ifndef CARCLASSIFIER_H_
#define CARCLASSIFIER_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

#include <opencv/cv.h>

#include <vector>
#include <map>

#include "carhogjbwrapper.h"

using namespace std;
using namespace Vision;
using namespace cast; 
using namespace std; 
using namespace boost; //default useful namespaces, fix to reflect your own code

//not used right now
typedef map<string, string> TaskMap;


class CarClassifier : public VideoClientProcess //public ManagedProcess // 
{
public:
  CarClassifier(const string &_id);
  virtual ~CarClassifier();
  virtual void configure(map<string,string> & _config);

  virtual void runComponent();
  virtual void start();
  
protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
private:
  void HandleROIChange(const cdl::WorkingMemoryChange &change);
  CarHOGJBWrapper* _hogjbcw;
  IplImage* image_classes;
  IplImage* image_cropped;
  Vision::ImageFrame* lastframe_;
  int camera_;
};
#endif
