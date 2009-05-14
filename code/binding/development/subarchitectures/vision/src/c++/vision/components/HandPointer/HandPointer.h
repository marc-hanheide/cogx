#ifndef CAST_HANDPOINTER_H_
#define CAST_HANDPOINTER_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

#include <opencv/cv.h>

#include <vector>
#include <map>

#include "HandDetector.h"

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision;
using namespace cast;
using namespace boost;

typedef map<string, string> TaskMap;


class HandPointer : public VideoClientProcess
{
  public:
    HandPointer(const string &_id);
    virtual ~HandPointer();

    virtual void runComponent();
    virtual void configure(map<string,string> & _config);
    virtual void start();

  protected:
    virtual void taskAdopted(const string &_taskID);
    virtual void taskRejected(const string &_taskID);

  private:

    void HandleSceneChanging(const cdl::WorkingMemoryChange &change);
    void GetPointedROI(const string &_taskID);
    void getPointer(); 
    void findPointedROI(); 
    void grabAndProcessImage(int camNum);
    
    // Hashtable used to record the tasks we want to carry out
    TaskMap m_tasks;

    // Scene changing status
    string m_sceneStatusID;

    //hand detector
    HandDetector *proc;

    //first frame to process
    bool first_frame;

    //ID of the entry in working memory
    string m_memoryID;

    //input camera to use
    int m_camera;

    //show window status
    bool showOpenCVWindow;

    //minimal number present hand features in scene
    long h_minPresent;
 
    //captured image
    Vision::ImageFrame Image;

    //location of configuration file
    string file_loc; 

    //stack of detected parameters   
    struct PARAMETER_STACK
    {
     long* position_hor;      //horizontal pointing position
     long* position_ver;      //vertical pointing position
     long* direction;         //pointing direction
     long nmbr;               //number of elements in stack 
     long size;
    } pstack;
    
    //detected hand pointer parameters
    struct POINTER
    {
     float pos_x;  //pointers horizontal position
     float pos_y;  //pointers vertical position
     float ang_x;  //pointers horizontal angle component
     float ang_y;  //pointers vertical angle component
     bool exist;   //pointers existance indicator
    } hpointer;
  
    //detected salient ROI parameters
    struct DETECTED_SALIENT_ROI
    {
     bool is_detected;   //inidicator of detected ROI
     float cent_x;       //center x coordinate
     float cent_y;       //center y coordinate
     float roi_sx;       //size x coordinate
     float roi_sy;       //size y coordinate
     string wm_addr;     //WM address of ROI
     string wm_obj;      //wm address of scene object 
    } detectedroi;


    //horizontal and vertical image resolution
    long frameW;
    long frameH;

}; //HandPointer

#endif
