/**
 * Color Component -- detects color of a ROI...
 *
 * @author Mohan
 * @date November 12 2007
 */

#ifndef VISION_COLOR_COMPONENT_H_
#define VISION_COLOR_COMPONENT_H_

#define NUM_COLORS 4 // Number of independent colors considered...
#define NUM_SAMPLES 4 // Number of sample vectors for each color...
#define NUM_BINS 512 // Number of bins in each histogram...

#define EMPTY_CLASS_THRESH ( 1/NUM_COLORS )
#define UNKNOWN_CLASS_THRESH 0.99

#define NUM_BINS_DIM 8 // 4
#define SS_RATIO (256/NUM_BINS_DIM)

#define NUM_HIST_SAMPS_MIN 3500
#define NUM_HIST_SAMPS_MAX 60000

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


class ColorComponent : public VideoClientProcess {

public:
  ColorComponent( const string &_id );
  virtual ~ColorComponent();
  
  virtual void configure( map<string,string> & _config );
  virtual void runComponent();
  virtual void start();
  
  virtual void taskAdopted( const string &_taskID );
  virtual void taskRejected( const string &_taskID );
  
  /**
   * Function loads the color hists database into the memory...
   */
  void loadDataBase( string strDBFile );

  /**
   * Function that responds to a ROI addition in vision's WM,
   * resulting in a color detection task...
   */
  void handleROIColorDetection( const cdl::WorkingMemoryChange& _wmc ); 

  /**
   * Function that actually does the color histogram-based comparison,
   * and spews out a label...
   */
  int doCOLOR( shared_ptr<const Vision::ROI> _pROI ); 
 
  /**
   * Function that gets the color of the ROI, from the image sent in
   * as input...
   */
  void detectColor( ImageFrame* _pimg, const BBox2D& _bbox, int& cLabel );

  /**
   * Function to update the scene object once the color label has been
   * found...
   */
  void updateSceneObject( shared_ptr<const Vision::ROI> _pROI, const int& _label );

  /**
   * Function to determine the histogram, assuming that a certain
   * number of samples have already been stored in an array (a statis
   * member variable)...
   */
  void generateHist( int numVals );

  /**
   * Function does a simple NNr operation to determine the best-suited
   * color class label (integer values: 1,2,3) using JS as the
   * distance measure...
   */
  void determineColorClass( int& cLabel );  

  /**
   * Function computes jsDistance between test histogram and the
   * staored histogram whose index is provided...
   */
  double jsDistance( int idX );

  /**
   * Function that is used just to write out the histogram values to a
   * file, for later analysis...
   */
  void writeHist2File();

  /**
   * Function inherited from Inspectable component for display
   * purposes...
   */
  void redrawGraphics2D();

private:
  // Histogram storing N (=3*4 currently) histograms, NUM_SAMPLES (=4)
  // vectors corresponding to each color class already trained for...
  double colorHists[ NUM_COLORS * NUM_SAMPLES ][ NUM_BINS ];
 
  // Test histogram created from input ROI -- compared with stored
  // hists to determine color class...
  double testHist[ NUM_BINS ];

  // Hacky declaration of array to hold RGB pixel samples but will do
  // for now...
  int rgbPixels[ NUM_HIST_SAMPS_MAX ][3];

  // Store the current action's address, that triggered the color
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
  bool m_writeHists;

  // Array to store the probabilitites of each of the known colors
  // (used for each ROI analyzed...)
  double m_colorProbs[ NUM_COLORS ];

  // Image that stores the image region corresponding to the ROI...
  IplImage* roiImg;

  // Storage for the image output used for debugging purposes...
  IplImage* currImage;
  string dispLabel;
  Math::Vector2D dispPoint;

  // Flags to set the number of colors being considered and the number
  // of samples for each color...
  int m_numColors;
  int m_numSamples;

  // Probability threshold -- if the highest match probability is
  // below this threshold, the class label should be 'empty'...
  double m_emptyClassProbThresh; // default = EMPTY_CLASS_THRESH = 1/NUM_COLORS;

  // Probability threshold -- if the ratio of the best to second-best
  // matches is below this threshold, the class label is 'unknown' --
  // could be multiple objects in the ROI...
  double m_unknownClassProbRatioThresh; // default = UNKNOWN_CLASS_THRESH = 0.91;  
};


#endif
