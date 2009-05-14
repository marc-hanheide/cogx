#ifndef _VISION_PROXY_H_
#define _VISION_PROXY_H_

#include <vector>
#include <opencv/cv.h>
#include <vision/utils/VideoClientProcess.h>
#include <vision/idl/Vision.hh>

class VisionProxy : public VideoClientProcess
{
 protected:
    // The returned IplImage must be cvReleaseImage'ed by the caller. 
    IplImage* buffer2image(Vision::ImageFrame* _pImage);

    // The returned IplImage must be cvReleaseImage'ed by the caller. 
    IplImage* GetIplImage(int camID);

    // cameras and imageFrames must be deleted by the caller
    void GetCamerasAndImages(vector<Vision::Camera *> &cameras,
			     vector<Vision::ImageFrame *> &imageFrames);

    void GetCameras(vector<shared_ptr<Vision::Camera> > &cameras);


    void configure(map<string,string> & _config);

 public:
    VisionProxy(const string&_id);
    virtual ~VisionProxy();
};

#endif
