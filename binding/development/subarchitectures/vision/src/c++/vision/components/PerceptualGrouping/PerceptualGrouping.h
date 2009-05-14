/**
 * Perceptual Grouping Component -- detects perceptual groups in a
 * ROI...
 *
 * @author Mohan
 * @date December 11 2007
 */

#ifndef VISION_PERCEPTUAL_GROUPING_H_
#define VISION_PERCEPTUAL_GROUPING_H_

#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>
#include <opencv/cv.h>

#include <vector>
#include <algorithm>
#include <map>

#include <cast/architecture/ManagedProcess.hpp>

#define PG_CIRCLE 1
#define PG_RECTANGLE 3


// Unfortunately the namespaces have to be set in this crude manner...
using namespace std;
using namespace Vision;
using namespace cast; using namespace boost; 


class PerceptualGrouping : public VideoClientProcess { 

public:
  PerceptualGrouping( const string &_id ); 
  virtual ~PerceptualGrouping();
  
  virtual void runComponent(); 
  virtual void start(); 
 
  virtual void taskAdopted( const string &_taskID ); 
  virtual void taskRejected( const string &_taskID );
  
  void handlePlanningActionRequest( const cdl::WorkingMemoryChange& _wmc ); 

  /**
   * Function that responds to a ROI addition in vision's WM,
   * resulting in a task to find the perceptual groups in it...
   */
  void handleROIPGDetection( const cdl::WorkingMemoryChange& _wmc );

  /**
   * Function that actually computes the perceptual groups, and
   * returns a numerical index corresponding to the groups found...
   */
  int doPerceptualGroups( shared_ptr<const Vision::ROI> _pROI ); 
  
  /**
   * Function takes the input ROI and determines the perceptual groups
   * in the image region corresponding to this ROI, and sets an
   * integer variable to denote the perceptual 'class' the ROI is
   * classified into...
   */
  void detectPerceptualGroups( const BBox2D& _bbox, int& pgLabel );

  /**
   * Function to update the scene object once the labels have been
   * found...
   */
  void updateSceneObject( shared_ptr<const Vision::ROI> _pROI, const int& _label );

  /**
   * Function inherited from Inspectable component for display
   * purposes...
   */
  void redrawGraphics2D();

private:

  // Store the current action's address, that triggered the perceptual
  // grouping operation...
  map<string, cdl::WorkingMemoryAddress> m_taskActionMap;
  
  // ID of task currently being processed...
  string m_curTaskID;

  // Storage for the images -- also used for debugging purposes...
  IplImage* currImage;
  IplImage* roiImage;
  string dispLabel;
  Math::Vector2D dispPoint;
};


#endif
