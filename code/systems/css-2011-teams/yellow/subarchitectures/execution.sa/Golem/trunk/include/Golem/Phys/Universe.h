/** @file Universe.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_UNIVERSE_H_
#define _GOLEM_PHYS_UNIVERSE_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Phys/Recorder.h>
#include <Golem/Phys/Scene.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** NVIDIA driver error stream */
class NxErrorStream : public NxUserOutputStream {
private:
	Context &context;

public:
	NxErrorStream(Context &context);
	virtual void reportError(NxErrorCode code, const char* message, const char* file, int line);
	virtual NxAssertResponse reportAssertViolation(const char* message, const char* file, int line);
	virtual void print(const char* message);
};

//------------------------------------------------------------------------------

class Universe : protected Runnable {
	friend class Scene;
	
public:
	typedef shared_ptr<Universe> Ptr;
	typedef PrivateList<Scene*, Scene::Ptr> SceneList;
	
	class Desc {
	public:
		/** Program name */
		std::string name;

		/** Number of program arguments */
		int argc;
		/** Program arguments */
		char** argv;
		
		/** GUI window position X */
		int windowX;
		/** GUI window position Y */
		int windowY;
		/** GUI window width */
		int windowWidth;
		/** GUI window height */
		int windowHeight;

		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;
		
		bool realTimeSimulation;
		SecTmReal renderFPS;

		Real visualizationScale;
		Real skinWidth;
		Real sleepLinVelSquared;
		Real sleepAngVelSquared;
		Real maxAngularVelocity;
		Real bounceThreshold;
		//Real dynFrictScaling;
		//Real staFrictScaling;

		/** Screen capture tool */
		Recorder::Desc recorderDesc;
		/** Screen capture frame drop */
		U32 recorderFrameDrop;

		/** Constructs Universe description. */
		Desc() {
			setToDefault();
		}

		/** Destructor should be virtual */
		virtual ~Desc() {}

		/** Creates/initialises the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Universe, Universe::Ptr, Context&)
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			argc = 0;
			argv = NULL;
			
			name = "Golem";
			windowX = 0;
			windowY = 0;
			windowWidth = 1200;
			windowHeight = 800;

			threadPriority = Thread::ABOVE_NORMAL;
			threadTimeOut = 30000; //[msec]
			
			realTimeSimulation = false;	
			renderFPS = SecTmReal(30.0); // avr. frames/sec

			visualizationScale = Real(10.0);
			skinWidth = Real(0.005);//NX_SKIN_WIDTH 0.025
			sleepLinVelSquared = Real(0.05*0.05);//NX_DEFAULT_SLEEP_LIN_VEL_SQUARED 0.15*0.15
			sleepAngVelSquared = Real(0.05*0.05);//NX_DEFAULT_SLEEP_ANG_VEL_SQUARED 0.14*0.14
			maxAngularVelocity = Real(7.0);//NX_MAX_ANGULAR_VELOCITY 7
			bounceThreshold = Real(-2.0); // NX_BOUNCE_THRESHOLD -2

			recorderDesc.setToDefault();
			recorderFrameDrop = 0; // after capture drop 1
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (argc <= 0 || argv == NULL)
			//	return false;
			if (windowWidth <= 0 || windowHeight <= 0)
				return false;
			if (renderFPS <= SEC_TM_REAL_ZERO)
				return false;

			if (visualizationScale <= Real(0.0))
				return false;

			if (!recorderDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	static Universe* pUniverse;

	golem::Context &context;
	Desc desc;
	Thread thread;
	volatile bool bPause, bTerminate;
	CriticalSection csPhysX;
	CriticalSection csOpenGL;
	Event evLoop, evWaitKey;

	int key;
	bool keyInterrupted;

	NxPhysicsSDK* pNxPhysicsSDK;
	NxErrorStream nxErrorStream;

	int glWindowHandle;
	GLdouble glAspectRatio;

	bool bTitle;
	std::string sTitle; 

	bool bSize;

	/** Screen capture tool */
	Recorder::Ptr pRecorder;
	/** Screen capture total frames */
	U32 recorderFrameCounter;
	/** Screen capture dropped frames */
	U32 recorderFrameDrop;
	
	/** Collection owns pointers to all scenes */
	SceneList sceneList;
	SceneList::const_iterator currentScene;
	mutable CriticalSection csSceneList;

	Real visualizationScale;

	SecTmReal maxTimeStep;
	SecTmReal simulationTimeStamp;
	SecTmReal renderTimeStamp;

	virtual void run();
	virtual void runPhysics();
	virtual void setupCurrentScene();

	/** Glut initialisation */
	virtual bool initGlut();

	static void renderCallback();
	static void reshapeCallback(int width, int height);
	static void idleCallback();

	static void mouseCallback(int button, int state, int x, int y);
	static void motionCallback(int x, int y);
	static void keyboardCallback(unsigned char key, int x, int y);
	
	static void arrowKeyCallback(int key, int x, int y);

	/** Constructs the Universe without initialisation */
	Universe(golem::Context& context);

	/** Creates/initialises the Universe */
	bool create(const Desc& desc);
	
	/** Releases resources */
	void release();

public:
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Universe();

	/** Launches the Universe */
	virtual bool launch();

	/** Terminates the Universe */
	virtual void exit();
	
	/** Pauses/suspends the Universe */
	virtual void pause();

	/** Resumes the Universe */
	virtual void resume();

	/** Checks if the Universe is alive */
	virtual bool interrupted();

	/** Checks if the Universe is stopped/suspended */
	virtual bool suspended();

	/** Read a key, wait no longer than timeOut */
	virtual int waitKey(MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** Sets window title */
	virtual void setTitle(const char* title);
	
	/** Sets window size */
	virtual void setSize(int width, int height);
	
	/** Creates Scene from the Scene description. */
	virtual Scene* createScene(const Scene::Desc& desc);
	
	/** Releases the Scene. */
	virtual void releaseScene(Scene& scene);

	/** Returns collection of Scenes */
	virtual SceneList getSceneList() const;
	
	/** Returns render frame rate (FPS) */
	virtual SecTmReal getRenderFrameRate() const;
	
	/** Post number of frames capture */
	virtual void postScreenCaptureFrames(U32 numOfFrames);
	
	/** Returns number of frames to capture */
	virtual bool hasScreenCaptureFrames() const;
	
	/** Returns screen capture frame rate */
	virtual SecTmReal getScreenCaptureFrameRate() const;
	
	/** Returns screen capture frame rate */
	virtual void setScreenCaptureFrameDrop(U32 frameDrop);
	
	/** Returns window width */
	virtual int getWindowWidth() const;
	
	/** Returns window height */
	virtual int getWindowHeight() const;
	
	/** Synchronisation with PhysX */
	inline CriticalSection &getCSPhysX() {
		return csPhysX;
	}
	
	/** Synchronisation with OpenGL */
	inline CriticalSection &getCSOpenGL() {
		return csOpenGL;
	}
	
	inline golem::Context& getContext() {
		return context;
	}
	
	inline const golem::Context& getContext() const {
		return context;
	}
	
	inline NxPhysicsSDK* getNxPhysicsSDK() {
		return pNxPhysicsSDK;
	}

	inline const NxPhysicsSDK* getNxPhysicsSDK() const {
		return pNxPhysicsSDK;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_UNIVERSE_H_*/
