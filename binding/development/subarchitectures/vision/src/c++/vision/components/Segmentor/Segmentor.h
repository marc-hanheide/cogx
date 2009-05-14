#ifndef CAST_SEGMENTOR_H_
#define CAST_SEGMENTOR_H_

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>
#include <vision/utils/SceneObjectWriter.hpp>

#include <opencv/cv.h>

#include <vector>
#include <map>

#include "backModel.h"

//default useful namespaces, fix to reflect your own code
using namespace std;
using namespace Vision;
using namespace cast;
using namespace boost;

typedef map<string, string> TaskMap;


class Segmentor : public VideoClientProcess, public SceneObjectWriter {
  public:
  Segmentor(const string &_id);
  virtual ~Segmentor();
  
  virtual void runComponent();
  virtual void configure(map<string,string> & _config);
  virtual void start();
  
protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  
private:
  


  // functions for filling in sceneObject
  void GetCamera(Camera &cam, int camId);
  
  /// Trains the distribution of the background color.
  void learnBackground();
  
  double inline normpdf(double _x, double _mu, double _sigma);
  
  /**
   * This method is used to push all ROIs on the given image to Working
   * Memory.
   */
  void pushROIsToWorkingMemory(IplImage* _image);
  
  
  /**
   * Function converts a CvBox (from OpenCV) into a bounding box of
   * the form that is supported by the vision idl file...
   */
  void cvBoxToROIBox( CvBox2D cvbox, Vision::BBox2D& roibox );

  /**
   * Uses the normal distribution to distinguish between foreground and
   * background and detect ROIs based on this distinction.
   */
  void getSalientRegions(IplImage* _image,
			 std::vector<Vision::ROI*>& _regions);
  
  float overlap(const Vision::ROI* _roi1, const Vision::ROI* _roi2) const;
  
  void saveImage(IplImage* _image, const char* _prefix = "test",
		 bool _showImage = false);
  
  /// Gets image frame from the VideoServer.
  IplImage* getIplImage(int camNum);
  
  /// RGB --> HSV conversion.
  IplImage* rgb2hsv(IplImage* _pImage);
  
  void HandleSceneChanged(const cdl::WorkingMemoryChange &change);
  void AdoptSegmentROI(const string &_taskID);
  
  // sxh added:
  class mybox
  {
  public:
    int tLeft_x; int tLeft_y;
    int bRight_x; int bRight_y;
    mybox()
    {
      tLeft_x = tLeft_y = INT_MIN;
      bRight_x = bRight_y = INT_MAX;
    }
  } m_box;
 
  bool m_bBox_is_set;
  // is point in the bbox of interest?
  bool is_interesting(int x, int y);
  
  /// Means for H, S and V channels of the background.
  CvScalar m_bgMeans;
  
  /// Variances for H, S and V channels of the background.
  CvScalar m_bgVars;
  
  // Hashtable used to record the tasks we want to carry out
  TaskMap m_tasks;
  
  //nah added
  /// Shadow sensitivity
  double m_shadowThreshold;
  
  /// ROIs overlap threshold - is this a new object or an existing one?
  double m_overlapThreshold;
  
  /// minimum acceptable contour area
  unsigned int m_minContourArea;
  int m_minContourPoints;

  /// input camera to use
  int m_camera;
  
  /// Number of background training images
  unsigned int m_trainImgCount;
  
  /// Number of normalization background images
  unsigned int m_normImgCount;
  
  _back_model *ptrBackModel;
  
  /// Scene changing status
  string m_sceneStatusID;

  /// Show segmentation window
  bool m_segWindow;

  /// If true,Segmentor does no delete old ROIs.
  bool m_noROIDelete;


  
}; // class Segmentor

#endif
