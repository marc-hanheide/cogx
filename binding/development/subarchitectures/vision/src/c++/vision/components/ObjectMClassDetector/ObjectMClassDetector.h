#ifndef OBJECTMCLASSDETECTOR_H_
#define OBJECTMCLASSDETECTOR_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

#include <opencv/cv.h>

#include <vector>
#include <map>
#include <string>
#include "SW_MultHOG.h"

using namespace std;
using namespace Vision;
using namespace cast; 
using namespace std; 
using namespace boost; //default useful namespaces, fix to reflect your own code

//not used right now
typedef map<string, string> TaskMap;


class ObjectMClassDetector : public VideoClientProcess //public ManagedProcess // 
{
public:
  ObjectMClassDetector(const string &_id);
  virtual ~ObjectMClassDetector();
  virtual void configure(map<string,string> & _config);

  virtual void runComponent();
  virtual void start();
  
protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
          void processImageFromCamera();
private:
  void HandleWMChange(const cdl::WorkingMemoryChange &change);
  void loadConfFile(const char* file);
  SW_MultHOG swMULTHOG_;
  IplImage* imgdet_;
  Vision::ImageFrame* lastframe_;
  int camera_;
  int fixwidth_;
  int frameno_;
  int framestosave_;
  bool alwayson_; 
  bool visualize_;
  bool nonmax_;
  int saveone_;
  bool saveidl_;
  int numsavedidls_;
  //AnnotationList annolist_;
  string outprefix;
  vector<AnnotationList> classannolists_;
  vector<AnnotationList> classannolistsnms_;
};
#endif
