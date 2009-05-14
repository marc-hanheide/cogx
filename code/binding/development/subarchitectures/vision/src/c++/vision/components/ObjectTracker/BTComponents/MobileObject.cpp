/** @file MobileObject.cpp
 *  @brief A moving object.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include <iostream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "MobileObject.h"
#include "TrackAux.h"
#include <vision/components/common/SystemUtils/Common.h>

using namespace std;
using namespace TrackAux;
using namespace Common;
using namespace Geom;

MobileObject::MobileObject() : ObjectTypeManage(), TrackerAbstract()
{
    m_id = -1;
    initialize();
}

MobileObject::MobileObject(int _id) 
{
    m_id = _id;
    initialize();

    m_detectionState = UNINITIALIZED;
}

MobileObject::~MobileObject() 
{
    map<int, IplImage*>::iterator riter;    
    for (riter=m_roiImages.begin(); riter!=m_roiImages.end(); riter++)
	cvReleaseImage(&(riter->second));
    m_roiImages.clear();

    for (riter=m_maskImages.begin(); riter!=m_maskImages.end(); riter++) {
	if (riter->second !=0)
	    cvReleaseImage(&(riter->second));
    }
    m_maskImages.clear();
    
}

void MobileObject::initialize()
{
    m_type = Vision::UNDEF; 

    m_shape = SH_UNDEF;

    m_patchFileformat = "";

    for (int i=0; i<8; i++) 
	m_boundingBox[i].set(0.0, 0.0, 0.0);
    m_totalPoints = 0;

    m_confidence = 0.;

    m_memoryID = "";

    m_patchID = -1;
    
    // Calling init(config) will reset the default m_config.
    m_config["TYPE"] = Vision::TYPE_UNDEF;
    m_config["SHAPE"] = "SH_UNDEF";
    m_config["R_MIN"] = "35";
    m_config["R_MAX"] = "55";
    m_config["PATCH"] = "";
}

void MobileObject::addRoiImage(int camID, IplImage *roiImage, 
			       IplImage *maskImage /* =0 */) 
{
    m_roiImages[camID] = cvCloneImage(roiImage);
    if (maskImage!=0)
	m_maskImages[camID] = cvCloneImage(maskImage);
    else
	m_maskImages[camID] = 0;

    if (GetSaveRoiOption()) 
    {
	char fname[1024];
	snprintf(fname, 1024, "roi_saved_obj%02d_view%02d.jpg", m_id, camID);
	cvSaveImage(fname, m_roiImages[camID]);
	
	if (maskImage != 0) {
	    snprintf(fname, 1024, "mask_obj%02d_view%02d.jpg", m_id, camID);
	    cvSaveImage(fname, maskImage);
	}
    }
    
}

IplImage *MobileObject::getRoiImage(int viewID) 
{
    map<int, IplImage*>::iterator riter = m_roiImages.find(viewID);
    if (riter==m_roiImages.end())
	return NULL;
    else 
	return riter->second;
}

IplImage *MobileObject::getMaskImage(int viewID) 
{
    map<int, IplImage*>::iterator riter = m_maskImages.find(viewID);
    if (riter==m_maskImages.end())
	return NULL;
    else 
	return riter->second;
}

void MobileObject::init_config(string modelFilename)
{
    m_config.clear();
    ParseConfigFile(modelFilename, m_config);

    map<string,string>::iterator iter;
    
    if ((iter=m_config.find("TYPE")) != m_config.end()) {
	Vision::ObjType type = StringToObjtype(iter->second);
	putType(type);
    }
    else 
	throw user_error(__HERE__, "Object TYPE is not specified");
    
    if ((iter=m_config.find("SHAPE")) != m_config.end())
	putShape(iter->second);
    else 
	throw user_error(__HERE__, "Object SHAPE is not specified");
    
    if ((iter=m_config.find("PATCH")) != m_config.end()) 
    {
	string complete_patchFilename
	    = completeFilename(modelFilename, iter->second);
	putPatch(complete_patchFilename);
	//putPatch(iter->second);
    }
    else 
	throw user_error(__HERE__, "Object PATCH is not specified");
    
}

// void MobileObject::init_config(map<string,string> &config) 
// {
//     m_config.clear();
//     m_config = config;
    
//     map<string,string>::iterator iter;
    
//     if ((iter=m_config.find("TYPE")) != m_config.end()) {
// 	Vision::ObjType type = StringToObjtype(iter->second);
// 	putType(type);
//     }
//     else 
// 	throw user_error(__HERE__, "Object TYPE is not specified");
    
//     if ((iter=m_config.find("SHAPE")) != m_config.end())
// 	putShape(iter->second);
//     else 
// 	throw user_error(__HERE__, "Object SHAPE is not specified");
    
//     if ((iter=m_config.find("PATCH")) != m_config.end()) 
// 	putPatch(iter->second);
//     else 
// 	throw user_error(__HERE__, "Object PATCH is not specified");
// }

void MobileObject::getPosition(float &x, float &y, float &z) 
{
    x = m_pose.position().x(); 
    y = m_pose.position().y(); 
    z = m_pose.position().z();
}
void MobileObject::getOrientation(float &x, float &y, float &z) {
    x = m_pose.orientation().x(); 
    y = m_pose.orientation().y(); 
    z = m_pose.orientation().z();
}

void MobileObject::getBoxSize(float &x, float &y, float &z) {
    x = m_bbox.size().x();
    y = m_bbox.size().y();
    z = m_bbox.size().z();
}

// void MobileObject::putType(std::string _strtype) {
//     m_type=TrackAux::StringToObjtype(_strtype);
// }
  
void MobileObject::putShape(std::string _strshape) {
    m_shape=TrackAux::StringToShapetype(_strshape);
}

void MobileObject::putPatch(std::string _strpatch) {
    m_patchFileformat=_strpatch;

    // For nick: get patchfile number to be used as color signature
    int first_dot_pos =  m_patchFileformat.find_last_of(".");
    string substring = m_patchFileformat.substr(0,first_dot_pos);
    int second_dot_pos =  substring.find_last_of(".");
    string strPatchID = substring.substr(second_dot_pos+1);
    std::istringstream strin(strPatchID);
    strin >> m_patchID;
}

void MobileObject::addBBpoint(Vector3D point, int idx) {
    m_boundingBox[idx] = point;
    m_totalPoints += 1;
}

Vector3D &MobileObject::get_centroid(void) {
    static Vector3D center;
    double scale = 1.0/m_totalPoints;
    center.set(0.0, 0.0, 0.0);    

    for (int i=0; i<m_totalPoints; i++) 
	center = center + m_boundingBox[i];
    center = center*scale;
    return center; 
}


void MobileObject::clear(void) {
    for (int i=0; i<8; i++) 
	m_boundingBox[i].set(0.0, 0.0, 0.0);
    m_totalPoints = 0;
}


void MobileObject::show(void) {
    for (int i=0; i<8; i++) {
	cerr << i << ": " 
	     << m_boundingBox[i].x() << ", "
	     << m_boundingBox[i].y() << ", "
	     << m_boundingBox[i].z() << endl;
    }
    cerr << endl;
}






//////////////////////////////////
//
//      D
//      | 
//  A --|-- B
//    \ | /
//     \|/
//      C
//
void MobileObject::completeBB(int idxA, int idxB,
			      int idxC, int newidx) {
    
    CvMat pC, pB, pA, pAB, pCAB, pD;
    
    float a[] = {m_boundingBox[idxA].x(),
		 m_boundingBox[idxA].y(),
		 m_boundingBox[idxA].z()};
    float b[] = {m_boundingBox[idxB].x(),
		 m_boundingBox[idxB].y(),
		 m_boundingBox[idxB].z()};
    float c[] = {m_boundingBox[idxC].x(),
		 m_boundingBox[idxC].y(),
		 m_boundingBox[idxC].z()};
    float d[] = {0,0,0};
    float e[] = {0,0,0};
    float f[] = {0,0,0};
    
    cvInitMatHeader(&pA, 3,1, CV_32F, a);
    cvInitMatHeader(&pB, 3,1, CV_32F, b);
    cvInitMatHeader(&pC, 3,1, CV_32F, c);
  
    cvInitMatHeader(&pAB, 3,1, CV_32F, d);
    cvInitMatHeader(&pCAB, 3,1, CV_32F, e);
    cvInitMatHeader(&pD, 3,1, CV_32F, f);
    
    
    cvAddWeighted(&pA, 0.5, &pB, 0.5, 0, &pAB);
    cvSub(&pAB, &pC, &pCAB);
    cvAddWeighted(&pC, 1, &pCAB, 2, 0, &pD);
    
    Vector3D newpoint(cvGetReal1D(&pD,0),
		      cvGetReal1D(&pD,1),
		      cvGetReal1D(&pD,2));
    
    addBBpoint(newpoint, newidx);
    
    /*
      cerr << cvGetReal1D(&p0,0) << endl;
      cerr << cvGetReal1D(&p0,1) << endl;
      cerr << cvGetReal1D(&p0,2) << endl << endl;
      
      cerr << cvGetReal1D(&p2,0) << endl;
      cerr << cvGetReal1D(&p2,1) << endl;
      cerr << cvGetReal1D(&p2,2) << endl << endl;
      
      cerr << cvGetReal1D(&p6,0) << endl;
      cerr << cvGetReal1D(&p6,1) << endl;
      cerr << cvGetReal1D(&p6,2) << endl << endl;
    */
    
    //  cvAdd(p2, p6, p26);
}



std::ostream &operator<<(std::ostream &os, MobileObject &sobj) {    
    os << sobj.m_id << " " << sobj.m_bbox << endl;
    os << sobj.m_patchFileformat.c_str() << endl; 
    return os; 
} 
