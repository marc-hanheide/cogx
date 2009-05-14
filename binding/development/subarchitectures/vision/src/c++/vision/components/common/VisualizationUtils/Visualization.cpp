/** @file Visualization.cpp
 *  @brief Interfaces for visualizing 3D objects on 2D images.
 *
 *  It is possible that cameras are updated less frequent than images.
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Visualization.h"
#include "ColorDefs.h"
#include "vision/components/common/SystemUtils/Common.h"
#include "vision/components/common/GeomUtils/Vector3D.h"

using namespace Common;


Visualization::Visualization() : 
    m_bMultiCam(false),
    m_viewId(0),
    m_bWindowInitialized(false),
    m_baseWinName(""),
    m_bShow(false),
    m_bSave(false),
    m_bSaveOrig(false),
    m_bSaveResults(false)
{
}

Visualization::~Visualization()
{
    ClearImages();
    m_vCamProjs.clear();

    // destroy windows
    if(m_bMultiCam == false) {
	string imgname = "View" + m_viewId;
	if (m_bWindowInitialized == true) 
	    cvDestroyWindow(imgname.c_str());
    }    

    //delete(pGLdisplay);
}

int Visualization::GetImageWidth()
{
    user_assert(m_vImages.size()!=0, __HERE__, "no images");
    return m_vImages[0]->width;
}

int Visualization::GetImageHeight()
{
    user_assert(m_vImages.size()!=0, __HERE__, "no images");
    return m_vImages[0]->height;
}


bool Visualization::GetProjectionAndImage(int id, 
					  shared_ptr<CameraProj> &spProj,
					  IplImage *&img) 
{
    int maxid = (int)(m_vCamProjs.size())-1;
    if ((id < 0) || (id > maxid))
	throw user_error(__HERE__, 
			 "CameraProj ID %d does not exist. May need to wait for CameraServer to settle.", id);
    spProj = m_vCamProjs[id];
    if (m_vImages.size() == 0)
	return false;
    else {
	img = m_vImages[id];
	return true;
    }
}


IplImage * Visualization::GetSourceImage(int camID) 
{
    if ((camID<0) || (camID>(int)m_vSrcImages.size()-1))
	return NULL;
    else
	return m_vSrcImages[camID];
}


void Visualization::ClearImages()
{
    for (unsigned i=0; i<m_vImages.size(); i++) {
	cvReleaseImage(&m_vImages[i]);
	cvReleaseImage(&m_vSrcImages[i]);
    }
    m_vImages.clear();
    m_vSrcImages.clear();
}

void Visualization::CloneAndPushImages(IplImage* newimg) 
{ 
    if (newimg != NULL) {
	// Save a copy of the original image
	IplImage *srcImg = cvCloneImage(newimg);
	m_vSrcImages.push_back(srcImg);

	// Add a copy of new image for processing
	IplImage *processImg = cvCloneImage(newimg);
	m_vImages.push_back(processImg); 
    }
    else
	throw user_error(__HERE__, "Cannot clone a NULL image");
}


IplImage *Visualization::ViewImage()
{
    user_assert(m_vImages[m_viewId]!=NULL, __HERE__, "image doesn't exist");
    return  m_vImages[m_viewId];
}

shared_ptr<CameraProj> Visualization::ViewProjection()
{
    user_assert(m_viewId<m_vCamProjs.size(), __HERE__, "proj doesn't exist");
    return m_vCamProjs[m_viewId];
}


bool Visualization::readyToVisualize()
{
    if ((m_vCamProjs.size() == 0) || (m_vImages.size()==0))
	return false;
    else
	return true;
}


void Visualization::setVisualization(vector<const Vision::Camera *> &cameras,
				     vector<IplImage *> &images,
				     bool bMultiCamMode /*=false*/) 
{
    m_bMultiCam = bMultiCamMode;

    user_assert(cameras.size(), __HERE__, "number of cameras is 0");
    user_assert(cameras.size()==images.size(), 
		__HERE__, "mismatched numbers of cameras and views");

    setProjection(cameras);
    

    if (m_vImages.size() == 0) {
	for (unsigned int i=0; i<images.size(); i++)
	    CloneAndPushImages(images[i]);
    }
    else {
	if ( (images.size()!=m_vImages.size()) || 
	     (images.size()!=m_vSrcImages.size()) )
	    throw user_error( __HERE__, "mismatched number of views");
	for (unsigned int i=0; i<images.size(); i++) {
	    cvCopy(images[i], m_vImages[i]);
	    cvCopy(images[i], m_vSrcImages[i]);
	}
    }
}

void Visualization::UpdateProjection(shared_ptr<const Vision::Camera> spCam)
{
    int matchID=-1;

    if (m_vCamProjs.size()==0)
	matchID = -1;
    else {
	for (unsigned i=0; i<m_vCamProjs.size(); i++) 
	{
	    if (m_vCamProjs[i]->m_num == spCam->m_num) {
		m_vCamProjs[i]->readCamParms(*spCam);
		matchID = i;
		break;
	    }
	}
    }

    if (matchID == -1) 
    {
	// HACK: for now LEFT and RIGHT are hardcoded
	if ((spCam->m_num == Vision::CAM_LEFT) || 
	    (spCam->m_num == Vision::CAM_RIGHT)) 
	{
	    shared_ptr<CameraProj> spProj 
		= shared_ptr<CameraProj>(new CameraProj());
	    spProj->setDefault(spCam->m_num);
	    spProj->readCamParms(*spCam);
	    m_vCamProjs.push_back(spProj);
	}
	else
	    throw user_error(__HERE__, "only support left/right cameras");
    }
    else 
    {
	m_vCamProjs[matchID]->setDefault(spCam->m_num);
	m_vCamProjs[matchID]->readCamParms(*spCam);
	user_printf(__HERE__, "match camID %d \n", matchID);
    }

    // now pull images 
}


void Visualization::setProjection(vector<Vision::Camera *> &cameras) 
{
    if (m_vCamProjs.size()==0) {
	for (unsigned int i=0; i<cameras.size(); i++) {
	    shared_ptr<CameraProj> spProj 
		= shared_ptr<CameraProj>(new CameraProj());
	    spProj->setDefault(cameras[i]->m_num);
	    spProj->readCamParms(*(cameras[i]));
	    m_vCamProjs.push_back(spProj);
	}
    }
    else {
	user_assert(m_vCamProjs.size()==cameras.size(), 
		    __HERE__, "mismatched number of cameras");
	for (unsigned int i=0; i<cameras.size(); i++) {
	    m_vCamProjs[i]->setDefault(cameras[i]->m_num);
	    m_vCamProjs[i]->readCamParms(*(cameras[i]));
	}
    }
}


void Visualization::setProjection(vector<const Vision::Camera *> &cameras) 
{
    if (m_vCamProjs.size()==0) {
	for (unsigned int i=0; i<cameras.size(); i++) {
	    shared_ptr<CameraProj> spProj 
		= shared_ptr<CameraProj>(new CameraProj());
	    spProj->setDefault(cameras[i]->m_num);
	    spProj->readCamParms(*(cameras[i]));
	    m_vCamProjs.push_back(spProj);
	}
    }
    else {
	user_assert(m_vCamProjs.size()==cameras.size(), 
		    __HERE__, "mismatched number of cameras");
	for (unsigned int i=0; i<cameras.size(); i++) {
	    m_vCamProjs[i]->setDefault(cameras[i]->m_num);
	    m_vCamProjs[i]->readCamParms(*(cameras[i]));
	}
    }
}


void Visualization::drawLine(Vector3D src, Vector3D dst, 
			     CvScalar color /*= c_red */) 
{
    drawLine(m_viewId, src, dst, color);
}

void Visualization::drawLine(int viewId, Vector3D src, Vector3D dst, 
			     CvScalar color /*= c_red */) 
{
    Vector2D orig2D = m_vCamProjs[viewId]->projectToImage(src);
    Vector2D fin2D = m_vCamProjs[viewId]->projectToImage(dst);
    
    cvLine(m_vImages[viewId], 
	   cvPoint( (int) orig2D.x, (int) orig2D.y ),
	   cvPoint( (int) fin2D.x, (int) fin2D.y ),
	   color, 2);
}

// void Visualization::drawLine(Point_3 src, Point_3 dst, 
// 			     CvScalar color /*= c_red */) 
// {
//     drawLine(m_viewId, src, dst, color);
// }

// void Visualization::drawLine(int viewId, Point_3 src, Point_3 dst, 
// 			     CvScalar color /*= c_red */) 
// {
//     Vector3D orig, fin;
//     orig.set(src.x(), src.y(), src.z());
//     fin.set( dst.x(), dst.y(), dst.z());
    
//     Vector2D orig2D = m_vCamProjs[viewId]->projectToImage(orig);
//     Vector2D fin2D = m_vCamProjs[viewId]->projectToImage(fin);
    
//     cvLine(m_vImages[viewId], 
// 	   cvPoint( (int) orig2D.x, (int) orig2D.y ),
// 	   cvPoint( (int) fin2D.x, (int) fin2D.y ),
// 	   color, 2);
// }

void Visualization::saveOrig(int framenum)
{
    if (m_bSaveOrig == true) {
	char str_framenum[1024];
	snprintf(str_framenum, 1024, "%04d", framenum);

	string newname = string("image-0.") + str_framenum + ".ppm";
	cvSaveImage(newname.c_str(), m_vSrcImages[0]);
    }
}

void Visualization::saveResults(int framenum)
{
    if (m_bSaveResults == true) {
	char str_framenum[1024];
	snprintf(str_framenum, 1024, "%04d", framenum);

	string newname = string("result-0.") + str_framenum + ".jpg";
	cvSaveImage(newname.c_str(), m_vImages[0]);
    }
}



void Visualization::visualize(int framenum)
{
    if (m_bShow == true) 
    {
	char str_framenum[1024];
	snprintf(str_framenum, 1024, "%03d", framenum);
	
	if(m_bMultiCam == false) {
	    // create windows
	    string imgname = m_baseWinName.c_str() + m_viewId;
	    if (m_bWindowInitialized == false) {
		cvNamedWindow(imgname.c_str());
		m_bWindowInitialized = true;
	    }
	    
	    // draw
#ifdef MILLIMETER 
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(100,0,0), c_red);
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(0,100,0), c_green);
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(0,0,100), c_blue);
#else 
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(1,0,0), c_red);
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(0,1,0), c_green);
	    drawLine(m_viewId, Vector3D(0,0,0), Vector3D(0,0,1), c_blue);
#endif
	    int linetype=CV_AA; 
	    CvFont font1;
	    cvInitFont( &font1, CV_FONT_HERSHEY_SIMPLEX, 
			1.0, 1.0, 0, 1, linetype);
	    cvPutText(m_vImages[m_viewId], str_framenum,
		      cvPoint(250,30), &font1, cvScalar(125,255,255) );


	    // save image
	    if (m_bSave == true) {
		string newname = imgname + "." + str_framenum + ".jpg";
		cvSaveImage(newname.c_str(), m_vImages[m_viewId]);
	    }
	    
	    // show windows
	    cvShowImage(imgname.c_str(), m_vImages[m_viewId]);
	    cvWaitKey(2);

	}
    }
}    
