/**
 * Shape Component -- detects shape of a ROI...
 *
 * @author Mohan
 * @date November 12 2007
 */

#ifndef VISION_SHAPE_COMPONENT_H_
#define VISION_SHAPE_COMPONENT_H_

#define NUM_SHAPES 4 // 3  // Number of independent shapes considered...
#define NUM_MOMENTS 7      // Number of moments computed per shape...
#define NUM_SAMPLES 5 // NUmber of samples per shape...

#define EMPTY_CLASS_THRESH ( 1/NUM_SHAPES )
#define UNKNOWN_CLASS_THRESH 0.99

#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>
#include <opencv/cv.h>

#include <vector>
#include <algorithm>
#include <map>

#include <cast/architecture/ManagedProcess.hpp>

// Unfortunately the namespaces have to be set in this crude manner...
using namespace std;
using namespace Vision;
using namespace cast; using namespace boost; 


class ShapeComponent : public VideoClientProcess { 

public:
  ShapeComponent( const string &_id ); 
  virtual ~ShapeComponent();  
  
  virtual void configure( map<string,string> & _config ); 
  virtual void runComponent(); 
  virtual void start(); 
  
  virtual void taskAdopted( const string &_taskID ); 
  virtual void taskRejected( const string &_taskID ); 
  
  /**
   * Function loads the shape hists database into the memory...
   */
  void loadDataBase( string strDBFile );

  /**
   * Function that responds to a ROI addition in vision's WM,
   * resulting in a shape detection task...
   */
  void handleROIShapeDetection( const cdl::WorkingMemoryChange& _wmc ); 

  /**
   * Function that actually does the shape histogram-based comparison,
   * and spews out a label...
   */
  int doSHAPE( shared_ptr<const Vision::ROI> _pROI ); 

  /**
   * Function that gets the shape of the ROI, from the contourpoints
   * sent in as input...
   */
  void detectShape( ImageFrame& _pimg, const vector2DSequence& contourPoints, int& sLabel );

  /**
   * Function determines the shape class of the current ROI, based on
   * previously stored moments -- simple NNr based on a distance
   * measure...
   */
  void determineShapeClass( int& sLabel );  

  /**
   * Function computes distance between test vector moments and the
   * stored vector of moments whose index is provided...
   */
  double distMeasure( int idX, int method );

  /**
   * Function that is used just to write out the histogram values to a
   * file, for later analysis...
   */
  void writeVectors2File();

  /**
   * Function to update the scene object once the shape label has been
   * found...
   */
  void updateSceneObject( shared_ptr<const Vision::ROI> _pROI, const int& _label );

  /**
   * Function inherited from Inspectable component for display
   * purposes...
   */
  void redrawGraphics2D();

private:

  // Store the current action's address, that triggered the shape
  // operation...
  map<string, cdl::WorkingMemoryAddress> m_taskActionMap;
  
  // ID of task currently being processed...
  string m_curTaskID;

  // Name of file that stores the trained data samples -- read when
  // the component is created...
  string m_dbfilename;
  // File to store training samples into...
  string m_trnfilename;
  // Flag to decide whether to write out the learned vectors to a
  // file...
  bool m_writeVectors;

  // Samples read from stored file...
  double shapeMoments[ NUM_SHAPES * NUM_SAMPLES ][ NUM_MOMENTS ];

  // Samples from current image ROI...
  double testMoments[ NUM_MOMENTS ];

  // Array to store the probabilitites of each of the known shapes
  // (used for each ROI analyzed...)
  double m_shapeProbs[ NUM_SHAPES ];

  // Storage for the image output used for debugging purposes...
  IplImage* currImage;
  string dispLabel;
  Math::Vector2D dispPoint;

  // Number of shapes that shall be considered (currently 0-4)...
  int m_numShapes; // default = NUM_SHAPES = 4;

  // Number of samples per shape class (that has been trained for)...
  int m_numSamples; // default = NUM_SAMPLES = 5;

  // Probability threshold -- if the highest match probability is
  // below this threshold, the class label should be 'empty'...
  double m_emptyClassProbThresh; // default = EMPTY_CLASS_THRESH = 1/NUM_SHAPES;

  // Probability threshold -- if the ratio of the best to second-best
  // matches is below this threshold, the class label is 'unknown' --
  // could be multiple objects in the ROI...
  double m_unknownClassProbRatioThresh; // default = UNKNOWN_CLASS_THRESH = 0.91;
};


#endif
