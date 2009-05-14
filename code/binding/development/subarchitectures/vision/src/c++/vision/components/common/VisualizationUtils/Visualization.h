/** @file Visualization.h
 *  @brief Interfaces for visualizing 3D objects on 2D images.
 *
 *  It is possible that cameras are updated less frequent than images.
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_

#include <boost/shared_ptr.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "vision/components/common/GeomUtils/CameraProj.h"
#include "ColorDefs.h"
#include <vector>
#include <string> 
#include <vision/idl/Vision.hh>

using namespace std;
using namespace boost;
using namespace Color;
using namespace Geom;

class Visualization
{
 protected:
    vector<shared_ptr<CameraProj> > m_vCamProjs;
    vector<IplImage *> m_vImages;    // images to process and display
    vector<IplImage *> m_vSrcImages; // source images taken from camera

    bool m_bMultiCam; // initialized to false
    unsigned m_viewId; // initialized to 0
    bool m_bWindowInitialized; // initialized to false
    string m_baseWinName;
    bool m_bShow;
    bool m_bSave;
    bool m_bSaveOrig;
    bool m_bSaveResults;
    
 public:
    Visualization();
    virtual ~Visualization();

    
    IplImage * GetSourceImage(int camID);
    
    /** @brief Get the CameraProj and IplImage from the internal ring buffer.
     *  
     *  @return Bool. If successful return true, else return false.
     */
    bool GetProjectionAndImage(int id, shared_ptr<CameraProj> &spProj,
			       IplImage *&img);
    
    int GetImageWidth();
    int GetImageHeight();
    vector<shared_ptr<CameraProj> > &GetProjections() {return m_vCamProjs;}
    void ClearImages();

    /** @brief Clone images to use for processing.
     *  
     *  @param newimg an image to clone.
     *  @return Void.
     */
    void CloneAndPushImages(IplImage* newimg);

    /** @brief Retrieve the image chosen for viewing.
     *  @return A pointer to IplImage. 
     */
    IplImage *ViewImage();

    /** @brief Retrieve the parameters of the camera chosen for viewing
     *  @return A shared pointer to a CameraProj.
     */
    shared_ptr<CameraProj> ViewProjection();

    bool readyToVisualize();
    
    /** @brief Initialization of projection and images
     *  
     *  Make a copy of camera parameters and images that are retrieved from 
     *  the working memory. 
     * 
     *  @param cameras A set of camera parameters to be updated and returned.
     *  @param images A set of images to be updated and returned.
     *  @param bMuliCamMode A flag sepcifying if this is a multi-cam system.
     *  @return Void.
     */ 
    void setVisualization(vector<const Vision::Camera *> &cameras,
			  vector<IplImage *> &images,
			  bool bMultiCamMode=false);
    
    /** @brief Update projection matrices (e.g., when head turns).
     *
     */
    void setProjection(vector<const Vision::Camera *> &cameras);

    void setProjection(vector<Vision::Camera *> &cameras);

    void UpdateProjection(shared_ptr<const Vision::Camera> spCam);
    
    /** @brief Draw lines on the image.
     *
     */
    void drawLine(int viewId, Vector3D src, Vector3D dst, CvScalar color=c_red);

    /** @brief Draw lines on the image.
     *
     */
    void drawLine(Vector3D src, Vector3D dst, CvScalar color=c_red);

    //void drawLine(int viewId, Point_3 src, Point_3 dst, CvScalar color=c_red);
    //void drawLine(Point_3 src, Point_3 dst, CvScalar color=c_red);

    /** @brief Show the image.
     *
     */
    virtual void visualize(int framenum);

    void setViewId(int _viewID) {m_viewId=_viewID;}

    void setBaseWinName(string _name) {m_baseWinName=_name;}

    void setShow(bool _bOnOff) {m_bShow=_bOnOff;}

    void setSave(bool _bOnOff) {m_bSave=_bOnOff;}

    void setSaveOrig(bool _bOnOff) {m_bSaveOrig=_bOnOff;}

    void setSaveResults(bool _bOnOff) { m_bSaveResults=_bOnOff; }

    void saveOrig(int framenum);

    void saveResults(int framenum);
};

#endif 
