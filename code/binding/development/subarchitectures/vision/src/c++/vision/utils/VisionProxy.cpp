#include "VisionProxy.h"

VisionProxy::VisionProxy(const string&_id) :
    WorkingMemoryAttachedComponent(_id),
    VideoClientProcess(_id)
{
    
}

VisionProxy::~VisionProxy() 
{
}


void VisionProxy::configure(map<string,string> & _config) 
{
  VideoClientProcess::configure(_config);
}


IplImage* VisionProxy::buffer2image(Vision::ImageFrame* _pImage)
{
    IplImage *tmpimg = cvCreateImage(
	cvSize(_pImage->m_width, _pImage->m_height), IPL_DEPTH_8U, 3);
    unsigned char* dst = (unsigned char*) tmpimg->imageData;
    char* src = (char *)&(_pImage->m_image[0]);
    memcpy(dst, src, _pImage->m_image.length());
    return tmpimg;
} 


IplImage* VisionProxy::GetIplImage(int camID)
{
    Vision::ImageFrame Image;
    getImage(camID, Image);
    IplImage* image = buffer2image(&Image);
    return image;
}


void VisionProxy::GetCameras(vector<shared_ptr<Vision::Camera> > &cameras) 
{
    vector<shared_ptr<const CASTData<Vision::Camera> > > cams;
    //getWorkingMemoryEntries(VisionOntology::CAMERA_TYPE, 0, cams);
    getWorkingMemoryEntries(cams);

    for(unsigned i = 0; i < cams.size(); i++)
    {	
	shared_ptr<const Vision::Camera> spCam = cams[i]->getData();
	
	// HACK: for now LEFT camera is hardcoded	
	if((spCam->m_num == Vision::CAM_LEFT) || 
	   (spCam->m_num == Vision::CAM_RIGHT))
	{
	    // convert to non-const before pused into cameras  
	    shared_ptr<Vision::Camera> new_spCam 
		= shared_ptr<Vision::Camera>();
	    *new_spCam = *spCam;
	    cameras.push_back(new_spCam);
	}
     }
     cams.clear();
}

void VisionProxy::GetCamerasAndImages(vector<Vision::Camera *> &cameras,
				      vector<Vision::ImageFrame *> &imageFrames)
{
    vector<shared_ptr<const CASTData<Vision::Camera> > > cams;
    getWorkingMemoryEntries(cams);

    for(unsigned i = 0; i < cams.size(); i++)
    {	
	shared_ptr<const Vision::Camera> spCam = cams[i]->getData();

	// HACK: for now LEFT camera is hardcoded
	if((spCam->m_num == Vision::CAM_LEFT) || 
	   (spCam->m_num == Vision::CAM_RIGHT))
	{
	    Vision::Camera *pnewcam = new Vision::Camera();
	    *pnewcam = *spCam;
	    cameras.push_back(pnewcam);
	}
    }
    cams.clear();

    // Get images as fast as it can. Then, later convert to IplImage
    // Should really get synchronized images. 
    for(unsigned i = 0; i < cameras.size(); i++) 
    {
 	Vision::ImageFrame *imgFrame = new Vision::ImageFrame();
 	getImage(cameras[i]->m_num, *imgFrame);
 	imageFrames.push_back(imgFrame);
    }

    //BALTTime time = GetCameraTime(Vision::CAM_LEFT);
//     ImageFrame* imageFrame = GetImage(0);
//     imageFrames.push_back(imageFrame);    
//     BALTTime time = imageFrame->m_time;
//     for (unsigned i=1; i < totalcams; i++) 
//     {
// 	imageFrames.push_back(GetImage(i,time));
//     }
}


