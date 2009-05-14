#ifndef VISION_SIFT_COMPONENT_H_
#define VISION_SIFT_COMPONENT_H_

#include "vision/idl/Vision.hh"
#include "vision/utils/VideoClientProcess.h"

#include <map>
#include <vector>
#include <algorithm>

#include <cast/architecture/ManagedProcess.hpp>

#include "SiftDetectEngine.h"

// Unfortunately the namespaces have to be set in this crude manner...
using namespace std;
using namespace Vision;
using namespace cast; using namespace boost; 

typedef map<string, string> MemoryIDMap;

class SIFTComponent : public VideoClientProcess {

public:
  SIFTComponent(const string &_id);
  virtual ~SIFTComponent();
  
  virtual void configure(map<string,string> & _config);
  virtual void runComponent();
  virtual void start();

protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
  void proposeObjectDetectorGoal(const cdl::WorkingMemoryChange& _wmChange);

  /**
   * Function that actually sets up the sift engine to process the ROI
   * region of the current image...
   */
  string doSIFT(shared_ptr<const Vision::ROI> _pROI);
  
  void updateSceneObject(shared_ptr<const Vision::ROI> _pROI, const string & _label);

  /**
   * Function inherited from Inspectable component for display
   * purposes...
   */
  void redrawGraphics2D();

private:

  SiftDetectEngine *m_pSiftEngine;

  // Hashtable used to record the tasks we want to carry out
  MemoryIDMap * m_pProposedProcessing;

  string m_curTaskID;

  // Store the current action's address, that triggered the sift
  // operation...
  map<string, cdl::WorkingMemoryAddress> m_taskActionMap;

  // Name of the database file that contains the trained feature
  // vectors...
  string m_dbfilename;

  // Probability estimate of the detected object...
  double m_objectConfidence;

  // Storage for the image output used for debugging purposes...
  IplImage* currImage;
  string dispLabel;
  Math::Vector2D dispPoint;
};


#endif
