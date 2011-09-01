/**
 * @file tgTomGineThread.h
 * @author Richtsfeld, Prankl, Mörwald
 * @date March 2011
 * @version 0.1
 * @brief Multi-threaded wrapper for the TomGine.
 */

#ifndef TGTHREAD_TOMGINE_THREAD_H
#define TGTHREAD_TOMGINE_THREAD_H

#include <opencv/cxcore.h>
#include <vector>
#include <list>
#include <string>
#include <v4r/TomGine/tgTomGine.h>
#include <semaphore.h>

namespace TomGine {

/** @brief RGB-value of point clouds, accessable as float or long value. */
typedef union {
	struct {
		unsigned char Blue; // Blue channel
		unsigned char Green; // Green channel
		unsigned char Red; // Red channel
		unsigned char Alpha; // Alpha channel
	};
	float float_value;
	long long_value;
} tgRGBValue;

/** @brief Running TomGine rendering engine in an enclosed thread. No OpenGL commands are allowed outside of this thread. */
class tgTomGineThread {
protected:
	int width, height; // window width and height
	int mode; //
	bool stopTomGineThread; // stop the tomeGine thread
	bool renderingStopped;
	bool eventsStopped;
	bool clear;

	pthread_t thread_gl; // tomGine thread_gl
	pthread_t thread_event; // thread for event handling (keyboard, mouse, resize, etc ...)
	pthread_mutex_t dataMutex; // mutex for data
	pthread_mutex_t eventMutex; // mutex for data
	sem_t renderSem;
	sem_t renderFinishedSem;

	//--- data for drawing ---------------

	TomGine::tgEngine *m_engine;
	V4R::Event event;
	std::list<V4R::Event> m_eventlist;

	struct Label // label structure
	{
		bool render;
		std::string lL;
		std::string lLS;
		std::string lLE;
		cv::Point3d lP;
		cv::Point3d lPS;
		cv::Point3d lPE;
	};
	std::vector<Label> vLabel; // vector with all labels and poses

	bool loadImage;
	bool drawImage, draw3D, drawPointCloud; // draw image, 3D and point clouds
	bool drawVPointCloud; // draw vec4f point cloud
	bool drawLabels; // draw labels
	bool drawModels; // draw tomgine models
	bool drawNurbs; // draw non-uniform rational b-splines
	bool useProbLevel; // use a probability level for pruning
	bool showCoordinateFrame; // show a coordinate frame

	cv::Mat img; // image
	cv::Mat intrinsic; // intrinsic parameters of the camera
	cv::Mat extR, extT; // extrinsic camera
	cv::Vec3d rotCenter; // rotation center
	double coordinateFrameSize; // size of coordinate frame
	bool camChanged;
	bool rotCenterChanged;


	// Point3D
	std::vector<cv::Point3f> points3D; // points in 3D
	std::vector<cv::Point3f> pointCols3D; // color of the 3D points
	std::vector<float> pointSize3D; // size of each 3D point

	// Line3D
	std::vector<std::pair<cv::Vec3f, cv::Vec3f> > lines3D; // lines in 3D
	std::vector<cv::Vec3f> lineCols3D; // color of 3D lines
	std::vector<float> lineWidth3D; // probability of 3D lines

//	std::vector<std::string> lineLabels; // string with line labels
//	std::vector<std::string> lineLabelsStart; // string with node label @ start
//	std::vector<std::string> lineLabelsEnd; // string with node label @ end

//	std::vector<std::vector<cv::Vec3f> > polygons3D; // polygons in 3D
//	std::vector<cv::Vec4f> colPolygons3D; // color of 3D polygons
//	std::vector<bool> filledPolygon3D; // draw filled 3D polygons

	std::vector<TomGine::tgModel*> m_models;
	std::vector<TomGine::tgModel*> m_pointclouds;
	std::vector<TomGine::tgNurbsSurfacePatch> nurbsSurfaceData;
	std::vector<TomGine::tgNurbsSurface*> nurbsSurface;

	//--- drawing methods --------------

	void GL_Update(TomGine::tgEngine *render);
	void GL_Draw3D(TomGine::tgEngine *render);

	void GL_DrawPointCloud();
	void GL_DrawPoints3D();
	void GL_DrawLines3D();
	void GL_DrawModels();

	void GL_SyncNurbsData();
	void GL_DrawNurbs();

	void GL_Clear();

	friend void* ThreadDrawing(void* c);
	friend void* ThreadEventHandling(void* c);

	bool KeyHandler(V4R::Event &event);

public:
	/** @brief Initialize class and start threads. */
	tgTomGineThread(int w, int h);
	/** @brief Wait for all threads to finish, then destroy. */
	~tgTomGineThread();

	/** @brief Set the intrinsic parameter of the TomGine virtual camera. (See cv::calibrateCamera in OpenCV specs.) */
	void SetCamera(cv::Mat &_intrinsic);
	/** @brief Set extrinsic parameter of TomGine virtual camera (pose). */
	void SetCamera(cv::Mat &R, cv::Mat &t);
	/** @brief Set point to rotate the camera about using the mouse. */
	void SetRotationCenter(cv::Vec3d &_rotCenter);
	/** @brief Set a background image. */
	void SetImage(cv::Mat &_img);

	/** @brief	Add a point in 3D to the scene.
	 *  @param	x,y,z	position of the point.
	 *  @param	r,g,b	color of the point.
	 *  @param	size	size of the point in pixel. */
	void AddPoint3D(double x, double y, double z,
					uchar r = 0, uchar g = 0, uchar b = 0,
					double size = 1);

	/** @brief	Add a line in 3D to the scene.
	 *  @param	x1,y1,z1	Start point of the line.
	 *  @param 	x2,y2,z2	End point of the line.
	 *  @param	r,g,b		Color of the line.
	 *  @param	width		Width of the line in pixel.	 */
	void AddLine3D(	double x1, double y1, double z1,
					double x2, double y2, double z2,
					uchar r = 0, uchar g = 0, uchar b = 0,
					float width = 1.0);

	/** @brief	Adds a tgModel as pointer to the scene and calls the tgModel::Draw() function when rendered.
	 *  @brief	Attention: Creation and destruction of model is in control of the user.
	 *  Destruction of the model at a point where the OpenGL rendering thread (ThreadDrawing) is
	 *  still running leads to segmentation faults. \n
	 *  This function is meant to provide the user with an interface to use OpenGL commands
	 *  within an derived class of tgModel, overwriting the virtual void tgModel::Draw().
	 *  @param model	The pointer to a tgModel.
	 *  @return	Unique id of the model added.*/
	int AddModel(TomGine::tgModel *model);

	/** @brief	Add a NURBS surface to the scene.
	 *  @return	Unique id of the NURBS added.*/
	int AddNurbsSurface(const TomGine::tgNurbsSurfacePatch &nurbsData);

	/** @brief 	Adds a colored point cloud to the scene. tgModel::m_colorpoints
	 *  @return	Unique id of the point cloud added. */
	int AddPointCloud(const TomGine::tgModel &pcl);

	/** @brief Adds a colored point cloud to the scene.
	 *  @param cloud Cloud of points in OpenCV vector format. 4th entry is float-encoded RGBA color.
	 *  @return	Unique id of the point cloud added. */
	int AddPointCloud(cv::Mat_<cv::Vec4f> cloud);

	/** @brief  Sets the model pointer added with AddModel() to the model pointer specified.
	 *  @param	id The id of the model returned by AddModel().
	 *  @param  model The new model pointer. */
	void SetModel(int id, TomGine::tgModel *model);

	/** @brief	Sets the NURBS added with AddNurbsSurface().
	 *  @param	id		The id of the NURBS returned by AddNurbsSurface().
	 *  @param	nurbsData The new data defining the NURBS.	 */
	void SetNurbsSurface(int id, const TomGine::tgNurbsSurfacePatch &nurbsData);

	/** @brief	Sets the point cloud added with AddNurbsSurface().
	 *  @param	id		The id of the point cloud.
	 *  @param	pcl		The new point cloud.	 */
	void SetPointCloud(int id, const TomGine::tgModel &pcl);
	/** @brief	Sets the point cloud added with AddNurbsSurface().
	 *  @param	id		The id of the point cloud.
	 *  @param	pcl		The new point cloud.	 */
	void SetPointCloud(int id, cv::Mat_<cv::Vec4f> cloud);

	/** @brief	Trigger rendering thread to update data and draw it to screen. */
	void Update();
	/** @brief	Indicates if threads have stopped (finished). */
	bool Stopped();
	/** @brief	Clear content of the scene. */
	void Clear();

	/** @brief	Clear the points in the scene added with AddPoint3D(). (not point-clouds). */
	void ClearPoints3D();
	/** @brief	Clear the lines in the scene. */
	void ClearLines3D();
  /** @brief Clear the vector of tgModel pointer */
  void ClearModels();


};

}

#endif

