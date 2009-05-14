/** @file CameraProj.h
 *  @brief Camera projection.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _CAMERA_PROJ_H_
#define _CAMERA_PROJ_H_

#include <opencv/cv.h>
#include "Model3D.h"
#include "Vector2D.h"
#include "Vector3D.h"

#include <vision/idl/Vision.hh>

using namespace Geom;

class CameraProj {
 public:
    CameraProj();
    ~CameraProj();
    
    CameraProj& operator=(const CameraProj& src);
    
    void setDefault(int _num=0);
    void readCamParms(char filename[]);
    void readCamParms(const Vision::Camera &campose);
    bool is_valid() {return bInitialized;};
    
    Geom::Vector3D projectImagePointToGroundplane(Vector2D p);
    double projectDistanceToGroundplane(Vector2D p, double s);
    void projectRoiToWorld(CvBox2D &roi, BBox3D &bbox, Pose3D &pose);
    Vector2D projectToImage(Geom::Vector3D p3);
    
    Pose3D pose() {return m_pose;};
  
 public:
    bool bInitialized;
  
    long         m_num;
    Pose3D       m_pose;
    // image width and height
    long m_width;
    long m_height;
    // the following are components of the camera projection matrix
    //     | fx  0  cx |
    // A = | 0  fy  cy |
    //     | 0   0   1 |
    float m_fx;
    float m_fy;
    float m_cx;
    float m_cy;
    // the following are distortion parameters (t1 and t2 typically 0):
    // xd = xu (1 + k1 r^2 + k2 r^4 + k3 r^6) + tangx
    // yd = yu (1 + k1 r^2 + k2 r^4 + k3 r^6) + tangy
    // where:
    // xd, yd are distorted, xu, yu undistorted image coords
    // tangx = 2 t1 xu yu + t2 (r^2 + 2 xu^2)  (typically 0)
    // tangy = 2 t2 xu yu + ti (r^2 + 2 yu^2)  (typically 0)
    // r^2 = xu^2 + yu^2
    float m_k1;
    float m_k2;
    float m_k3;
    float m_t1;
    float m_t2;
};

#endif
