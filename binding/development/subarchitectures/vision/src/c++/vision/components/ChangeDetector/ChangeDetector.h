#ifndef CAST_CHANGE_DETECTOR_H_
#define CAST_CHANGE_DETECTOR_H_

#include <vector>
#include <map>
#include <opencv/cv.h>

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/utils/VideoClientProcess.h>
#include <vision/idl/Vision.hh>

using namespace std;
using namespace cast; using namespace std; using namespace boost; //default useful namespaces, fix to reflect your own code

class ChangeDetector : public VideoClientProcess
{
  public:
    ChangeDetector(const string &_id);
    virtual ~ChangeDetector();

  protected:
    virtual void taskAdopted(const string &_taskID);
    virtual void taskRejected(const string &_taskID);
    virtual void runComponent();
    virtual void configure(map<string,string> & _config);
    virtual void redrawGraphicsText();

  private:
    IplImage* rgb2hsv(IplImage* _pImage);
    IplImage* buffer2image(Vision::ImageFrame* _pImage);
    /// Get a new frame and process it.
    void GetAndProcessImage();

    /// Points to the previous image frame. The current frame is compared to the previous frame to determine whether the scene is changing.
    IplImage* m_lastImage;
    /// When the scene stabilises this flag shows whether the information about that was already submitted to the working memory.
    bool m_bAnythingHappened;
    /// Stores the number of frames that need to be unchanged before the scene is considered
    /// stable (not changing). For each still frame its value is decreased by 1, for each
    /// changing frame it is set to \p m_signalDelay .
    unsigned m_nImagesBeforeSignal;

    /**
     * This is the ID of the entry in working memory that is used to signal
     * that scene is stationary again.
     */
    string m_memoryID;
    /// How many consecutive unchanged frames do we need to consider scene stable (not changing).
    int m_frameSkip;
	int m_signalDelay;
    int m_camera;  // which camera to observe
    string m_observedComponent;
    /// The portion of image pixels that need to change between two consecutive frames to consider
    /// the scene changing.
    float m_threshold;

    //what to write as default in the scene processed field when scene is static
    bool m_processed;
}; // class ChangeDetector

#endif
