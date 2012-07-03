/** @file TrackerThread.h
 * 
 * @author      Sergio Roa (DFKI)
 * @author	Manuel Noll (DFKI)
 * @author      Thomas Mörwald
 *
 * @version 1.0
 *
 *
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef TRACKERTHREAD_H_
#define TRACKERTHREAD_H_


#include <ThreadObject/Thread.h>
#include <smltools/tracker_tools.h>

// #include <Golem/Sys/Thread.h>
// #include <ThreadObject/CameraThread.h>
// #include <Tracker/Tracker.h>
#include <TomGine/tgTimer.h>
#include <TomGine/tgFont.h>
#include <GLWindow/GLWindow.h>
// #include <opencv/cv.h>
// #include <opencv/highgui.h>
// #include <string>

namespace smlearning {

/** \brief The thread class permitting to run the blort tracker in a thread.
*
* The blort tracker is a standalone program. In order to use it in parallel for further applications like being the eye
* for an real world acting robot, it is implemented as a thread.
* Since intentionally the position of the tracked object is needed it is somehow apparent that the thread is able to return the 
* object position (getPose). 
* The thread runs in principle the standalone.cpp, where the initialization is done in the cto TrackerThread and the loop is 
* executed in the OnTask function.
*
* \param _object_pose contains the pose of the object
* \param _new_position is true if there was a new object position stored
* \param _quit is true if the program was ended
* \param _img captured camera image
* \param _m_tracker is tracking the object and containing all central information
* \param _m_window is the ouput OpenGL Window
* \param _id_2 is the object identification number
* \param _movement is the current state of the object movement (fast / slow / still )
* \param _quality is the current quality of object visibilty ( ok / occluded / lost / locked )
* \param _confidence is the current confidence of the model ( good / fair / bad )
* \param m_timer, m_evData thread specific variables
*
*/

class TrackerThread : public CThread /*golem::Runnable*/
{
public:
	// std cto initializing the thread
	TrackerThread(const std::string,const std::string,const std::string, const TomGine::tgPose);
	// std dto
	~TrackerThread();
	// executes the former standalone.cpp loop as the thread task
	virtual BOOL OnTask();
	// virtual void run () /*{ OnTask (); }*/;
	// void start ();
	// returns the current object position by reference
	TomGine::tgPose getPose();
protected:
	//golem::Thread thread;
	//golem::CriticalSection cs;
	// // contains the pose of the object
	// TomGine::tgPose _object_pose;
	// is true if the program was ended
	bool _quit;
	// is true if there was a new object position stored
	bool _new_position;
	// thread specific variable
	TomGine::tgTimer m_timer;
	// thread specific variable
	CEventClass m_evData;
	CMutexClass m_running;
	// font
	TomGine::tgFont font;
	/** Tracker */
	// golem::shared_ptr<Tracking::Tracker> m_tracker;
	Tracking::Tracker* m_tracker;
	/** Object model */
	TomGine::tgModel m_object;
	/* The path to load and save the ply model */
	std::string m_plypath;
	/** Captured camera Image */
	IplImage* _img;
	TomGine::tgPose m_track_pose, m_initialPose;
	/** Tracker window */
	// golem::shared_ptr<blortGLWindow::GLWindow> glWindow;
	blortGLWindow::GLWindow* glWindow;
	/** Tracker parameters */
	Tracking::Tracker::Parameter trackParams;
	/** Camera thread */
	// CCameraThread* pThreadCamera;
	/** Model id */
	int m_trackpred_id, m_ground_id, m_track_id;
	/** is the current state of the object movement (fast / slow / still ) */
	Tracking::movement_state _movement;
	/** is the current quality of object visibilty ( ok / occluded / lost / locked ) */
	Tracking::quality_state _quality;
	/** is the current confidence of the model ( good / fair / bad ) */
	Tracking::confidence_state _confidence;

	std::string tracker_ini_file;
	std::string cam_ini_file;
	std::string pose_ini_file;



};

}; // namespace smlearning 

#endif 

