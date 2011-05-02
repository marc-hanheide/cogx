/** @file Universe.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Msg.h>
#include <Golem/Phys/NxCooking.h>
#include <GL/freeglut.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

NxErrorStream::NxErrorStream(Context &context) : context(context) {
}

void NxErrorStream::reportError(NxErrorCode code, const char* message, const char* file, int line) {
	Message::Level level;
	std::string codeStr;
	
	switch (code) {
	case NXE_INVALID_PARAMETER:
		codeStr = "invalid parameter";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_INVALID_OPERATION:
		codeStr = "invalid operation";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_OUT_OF_MEMORY:
		codeStr = "out of memory";
		level = Message::LEVEL_ERROR;
		break;
	case NXE_DB_WARNING:
		codeStr = "warning";
		level = Message::LEVEL_WARNING;
		break;
	case NXE_DB_INFO:
		codeStr = "info";
		level = Message::LEVEL_INFO;
		break;
	default:
		codeStr = "unknown error";
		level = Message::LEVEL_ERROR;
	}

	context.getMessageStream()->write(level, "NxErrorStream::reportError(): %s (%d): %s: %s", file, line, codeStr.c_str(), message);
}

NxAssertResponse NxErrorStream::reportAssertViolation(const char* message, const char* file, int line) {
	context.getMessageStream()->write(Message::LEVEL_ERROR, "NxErrorStream::reportAssertViolation(): %s (%d): %s", file, line, message);

#ifdef WIN32
	switch (MessageBox(0, message, "AssertViolation, see console for details.", MB_ABORTRETRYIGNORE)) {
	case IDRETRY:
		return NX_AR_CONTINUE;
	case IDIGNORE:
		return NX_AR_IGNORE;
	case IDABORT:
	default:
		return NX_AR_BREAKPOINT;
	}
#endif
}

void NxErrorStream::print(const char* message) {
	context.getMessageStream()->write(Message::LEVEL_ERROR, "NxErrorStream::print(): %s", message	);
}

//------------------------------------------------------------------------------

Universe* Universe::pUniverse = NULL;

Universe::Universe(golem::Context& context) :
	context(context), nxErrorStream(context)
{
	pNxPhysicsSDK = NULL;
	bPause = false;
	bTerminate = false;
	bTitle = false;
	bSize = false;
	key = 0;
	keyInterrupted = false;
	currentScene = sceneList.end();
}

Universe::~Universe() {
	release();
}

bool Universe::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgUniverseInvalidDesc(Message::LEVEL_CRIT, "Universe::create(): Invalid description");
	
	this->desc = desc;
	glAspectRatio = 1.0;

	// Initialise PhysicsSDK
	pNxPhysicsSDK = ::NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, 0, &nxErrorStream);
	if (pNxPhysicsSDK == NULL)
		throw MsgUniversePhysXInit(Message::LEVEL_CRIT, "Universe::create(): Unable to initialize PhysX");

	// Initialise cooking
	if (!::InitCooking())
		throw MsgUniversePhysXCookInit(Message::LEVEL_CRIT, "Universe::create(): Unable to initialize PhysX cooking library");
	
	visualizationScale = desc.visualizationScale;
	const Real visualizationScaleInv = REAL_ONE/visualizationScale;
	
	pNxPhysicsSDK->setParameter(NX_SKIN_WIDTH,
		NxReal(desc.skinWidth * visualizationScaleInv));
	pNxPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_LIN_VEL_SQUARED,
		NxReal(desc.sleepLinVelSquared * visualizationScaleInv * visualizationScaleInv));
	pNxPhysicsSDK->setParameter(NX_DEFAULT_SLEEP_ANG_VEL_SQUARED,
		NxReal(desc.sleepAngVelSquared));
	pNxPhysicsSDK->setParameter(NX_MAX_ANGULAR_VELOCITY,
		NxReal(desc.maxAngularVelocity));
	pNxPhysicsSDK->setParameter(NX_BOUNCE_THRESHOLD,
		NxReal(desc.bounceThreshold * visualizationScaleInv));
	//pNxPhysicsSDK->setParameter(NX_DYN_FRICT_SCALING,
	//	NxReal(desc.dynFrictScaling * visualizationScaleInv));
	//pNxPhysicsSDK->setParameter(NX_STA_FRICT_SCALING,
	//	NxReal(desc.staFrictScaling * visualizationScaleInv));

	simulationTimeStamp = renderTimeStamp = context.getTimer().elapsed();
	maxTimeStep = SecTmReal(1.0)/(SecTmReal(2.0)*desc.renderFPS);

	pRecorder = desc.recorderDesc.create(context); // throws
	postScreenCaptureFrames(0);
	
	return true;
}

void Universe::release() {
	bTerminate = true;
	if (!thread.join(desc.threadTimeOut))
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Universe::release(): Unable to stop physics thread");
	
	while (!sceneList.empty())
		releaseScene(*sceneList.back());

	if(pNxPhysicsSDK != NULL) {
		::CloseCooking();

		::NxReleasePhysicsSDK(pNxPhysicsSDK);
		pNxPhysicsSDK = NULL;
	}
}

//------------------------------------------------------------------------------

bool Universe::launch() {
	if (pUniverse != NULL)
		throw MsgUniverseMultipleInstances(Message::LEVEL_CRIT, "Universe::launch(): Universe has been already created");
	pUniverse = this;

	if (thread.isAlive()) {
		context.getMessageStream()->write(Message::LEVEL_WARNING, "Universe::launch(): Universe has been launched already");
		return true;
	}

	currentScene = sceneList.begin();
	if (currentScene == sceneList.end())
		throw MsgUniverseNoScenes(Message::LEVEL_CRIT, "Universe::launch(): no Scenes have been created");

	if (!thread.start(this))
		throw MsgUniverseThreadLaunch(Message::LEVEL_CRIT, "Universe::launch(): Unable to launch Universe thread");
	
	if (!thread.setPriority(desc.threadPriority))
		context.getMessageStream()->write(Message::LEVEL_WARNING, "Universe::launch(): Unable to change Universe thread priority");
	
	if (!evLoop.wait(desc.threadTimeOut))
		throw MsgUniverseThreadLaunch(Message::LEVEL_CRIT, "Universe::launch(): Universe thread is not alive");
	
	return true;
}

void Universe::run() {
	if (!initGlut())
		throw MsgUniverseGlutInit(Message::LEVEL_CRIT, "Universe::run(): Unable to initialize Glut");

	setupCurrentScene();
	
	// start the main loop
	::glutMainLoop();
	evLoop.set(false);
	context.getMessageStream()->write(Message::LEVEL_INFO, "Universe::run(): Closing Universe....");
}

void Universe::exit() {
	if (!evLoop.wait(0) && !thread.isAlive()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Universe::exit(): Universe has not been launched");
		return;
	}

	bTerminate = true;
}

void Universe::pause() {
	bPause = true;
}

void Universe::resume() {
	bPause = false;
}

bool Universe::interrupted() {
	return !evLoop.wait(0) || keyInterrupted;
}

bool Universe::suspended() {
	return bPause;
}

int Universe::waitKey(MSecTmU32 timeOut) {
	//if (interrupted())
	//	return 0;

	//evWaitKey.set(false);
	//return evWaitKey.wait(timeOut) ? key : 0;
	
	if (evWaitKey.wait(timeOut))
		evWaitKey.set(false);
	int key = this->key;
	this->key = 0;
	return interrupted() ? 0 : key;
}

//------------------------------------------------------------------------------

bool Universe::initGlut() {
	::glutInit(&pUniverse->desc.argc, pUniverse->desc.argv);
	::glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	::glutInitWindowPosition(pUniverse->desc.windowX, pUniverse->desc.windowY);
	::glutInitWindowSize(pUniverse->desc.windowWidth, pUniverse->desc.windowHeight);
	::glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glWindowHandle = ::glutCreateWindow(pUniverse->desc.name.c_str());
	::glutSetWindow(glWindowHandle);
	::glutDisplayFunc(renderCallback);
	::glutReshapeFunc(reshapeCallback);
	::glutIdleFunc(idleCallback);
	::glutKeyboardFunc(keyboardCallback);
	::glutSpecialFunc(arrowKeyCallback);
	::glutMouseFunc(mouseCallback);
	::glutMotionFunc(motionCallback);
	reshapeCallback(::glutGet(GLUT_WINDOW_WIDTH), ::glutGet(GLUT_WINDOW_HEIGHT));
	return true;
}

//------------------------------------------------------------------------------

void Universe::runPhysics() {
	const SecTmReal timeCurrent = context.getTimer().elapsed();
	const SecTmReal timeElapsed = desc.realTimeSimulation ? timeCurrent - simulationTimeStamp : SecTmReal(1.0)/desc.renderFPS;
	simulationTimeStamp = timeCurrent;

	{
		CriticalSectionWrapper csw(csSceneList);
		for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); i++)
			if (!(*i)->isAsync())
				(*i)->preprocess(timeElapsed);
	}
	{
		// simulate all synch scenes
		CriticalSectionWrapper csw(csSceneList);
		for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); i++)
			if (!(*i)->isAsync()) {
				CriticalSectionWrapper csw(csPhysX);
				NxScene* pNxScene = (*i)->getNxScene();
				pNxScene->simulate((NxReal)timeElapsed);
				pNxScene->flushStream();
				pNxScene->fetchResults(NX_RIGID_BODY_FINISHED, true); // blocking call
			}
	}
	{
		CriticalSectionWrapper csw(csSceneList);
		for (SceneList::iterator i = sceneList.begin(); i != sceneList.end(); i++)
			if (!(*i)->isAsync())
				(*i)->postprocess(timeElapsed);
	}
}

void Universe::setupCurrentScene() {
	if (currentScene == sceneList.end())
		return;

	setTitle((*currentScene)->desc.name.c_str());
	
	CriticalSectionWrapper csw(csOpenGL);
	(*currentScene)->initGL();
}

//------------------------------------------------------------------------------

void Universe::renderCallback() {
	{
		CriticalSectionWrapper csw(pUniverse->csSceneList);
		if (pUniverse->currentScene != pUniverse->sceneList.end()) {
			CriticalSectionWrapper csw(pUniverse->csOpenGL);
			
			if (pUniverse->bTitle) {
				pUniverse->bTitle = false;
				::glutSetWindowTitle(pUniverse->sTitle.c_str());
			}
			if (pUniverse->bSize) {
				pUniverse->bSize = false;
				::glutReshapeWindow(pUniverse->desc.windowWidth, pUniverse->desc.windowHeight);
			}
			
			if (!(*pUniverse->currentScene)->isAsync())
				(*pUniverse->currentScene)->render();
			
			::glutSwapBuffers();
		}
	}

	if (pUniverse->recorderFrameDrop == 0 && pUniverse->recorderFrameCounter > 0) {
		pUniverse->recorderFrameCounter--; // TODO thread safety
		pUniverse->pRecorder->capture(0, 0, pUniverse->desc.windowWidth, pUniverse->desc.windowHeight);
	}
	pUniverse->recorderFrameDrop = (pUniverse->recorderFrameDrop + 1)%(pUniverse->desc.recorderFrameDrop + 1);
}

void Universe::reshapeCallback(int width, int height) {
	CriticalSectionWrapper csw(pUniverse->csOpenGL);
	pUniverse->desc.windowWidth = width;
	pUniverse->desc.windowHeight = height;
	pUniverse->glAspectRatio = GLdouble(width)/GLdouble(height);
}

void Universe::idleCallback() {
	const SecTmReal timeCurrent = pUniverse->context.getTimer().elapsed();
	const SecTmReal delay = std::max(SecTmReal(0.0), SecTmReal(1.0)/pUniverse->desc.renderFPS - (timeCurrent - pUniverse->renderTimeStamp));
	pUniverse->context.getTimer().sleep(delay);
	pUniverse->renderTimeStamp = pUniverse->context.getTimer().elapsed();

	// TODO ughhh: use better stuff than freeglut
	if (pUniverse->bTerminate) {
		::glutLeaveMainLoop();
		return;
	}
		
	pUniverse->evLoop.set(true);

	if (!pUniverse->bPause)
		pUniverse->runPhysics();

	::glutPostRedisplay();
}

void Universe::mouseCallback(int button, int state, int x, int y) {
	CriticalSectionWrapper csw(pUniverse->csSceneList);
	if (pUniverse->currentScene != pUniverse->sceneList.end() && !(*pUniverse->currentScene)->isAsync())
		(*pUniverse->currentScene)->mouseHandler(button, state, x, y);
}

void Universe::motionCallback(int x, int y) {
	CriticalSectionWrapper csw(pUniverse->csSceneList);
	if (pUniverse->currentScene != pUniverse->sceneList.end() && !(*pUniverse->currentScene)->isAsync())
		(*pUniverse->currentScene)->motionHandler(x, y);
}

void Universe::keyboardCallback(unsigned char key, int x, int y) {
	CriticalSectionWrapper csw(pUniverse->csSceneList);
	
	switch (key) {
	case 27: // esc
		pUniverse->keyInterrupted = true;
		::glutLeaveMainLoop();
		goto WAIT_KEY;
	case 104:// pgup
		if (pUniverse->currentScene == pUniverse->sceneList.begin())
			pUniverse->currentScene = --pUniverse->sceneList.end();
		else
			--pUniverse->currentScene;

		// setup the new scene
		pUniverse->setupCurrentScene();
		goto WAIT_KEY;
	case 105: // pgdown
		if (pUniverse->currentScene == --pUniverse->sceneList.end())
			pUniverse->currentScene = pUniverse->sceneList.begin();
		else
			++pUniverse->currentScene;

		// setup the new scene
		pUniverse->setupCurrentScene();
		goto WAIT_KEY;
	case 32:// space
		pUniverse->currentScene = pUniverse->sceneList.begin();

		// setup the new scene
		pUniverse->setupCurrentScene();
		goto WAIT_KEY;
	case 'p':
		pUniverse->bPause = !pUniverse->bPause;
		break;
	// Single screen shot
	case 'C':
		pUniverse->postScreenCaptureFrames(1);
		pUniverse->context.getMessageStream()->write(Message::LEVEL_INFO, "Screen single shot capture");
		
		break;
	// Screen video
	case 'V':
		if (pUniverse->hasScreenCaptureFrames()) {
			pUniverse->postScreenCaptureFrames(0);
			pUniverse->context.getMessageStream()->write(Message::LEVEL_INFO, "Screen video capture STOP");
		}
		else {
			pUniverse->postScreenCaptureFrames(-1);
			pUniverse->context.getMessageStream()->write(Message::LEVEL_INFO, "Screen video capture START at %.1f FPS", pUniverse->getScreenCaptureFrameRate());
		}
		
		break;
	}

	if (pUniverse->currentScene != pUniverse->sceneList.end() && !(*pUniverse->currentScene)->isAsync())
		(*pUniverse->currentScene)->keyboardHandler(key, x, y);

	WAIT_KEY:
	pUniverse->key = key;
	pUniverse->evWaitKey.set(true);
}

void Universe::arrowKeyCallback(int key, int x, int y) {
	keyboardCallback(key, x, y);
}

//------------------------------------------------------------------------------

void Universe::setTitle(const char* title) {
	if (interrupted())
		return;

	{
		CriticalSectionWrapper csw(csOpenGL);
		if (title != NULL) {
			this->sTitle.assign(title);
			this->sTitle.append(" - ");
		}
		this->sTitle.append(desc.name);
	}
	bTitle = true;
}

void Universe::setSize(int width, int height) {
	reshapeCallback(width, height);	
	bSize = true;
}

Scene* Universe::createScene(const Scene::Desc& desc) {
	Scene::Ptr pScene = desc.create(*this);
	if (pScene == NULL)
		throw MsgUniverseSceneCreate(Message::LEVEL_CRIT, "Unable to create Scene");

	{
		CriticalSectionWrapper csw(csPhysX);
		// Set simulation parameters
		pScene->getNxScene()->setTiming((NxReal)this->maxTimeStep);
	}
	{
		CriticalSectionWrapper csw(csSceneList);
		sceneList.push_back(SceneList::Pair(pScene.get(), pScene));
	}

	return pScene.get();
}

void Universe::releaseScene(Scene& scene) {
	if (!sceneList.contains(&scene)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Universe::releaseScene(): Unable to find specified scene"	);
		return;
	}

	{
		CriticalSectionWrapper csw(csSceneList);
		scene.release();
		sceneList.erase(&scene);
	}
}

Universe::SceneList Universe::getSceneList() const {
	CriticalSectionWrapper csw(csSceneList);
	return sceneList; // copy constructor called before ~CriticalSectionWrapper()
}

//------------------------------------------------------------------------------

SecTmReal Universe::getRenderFrameRate() const {
	return desc.renderFPS;
}

void Universe::postScreenCaptureFrames(U32 numOfFrames) {
	this->recorderFrameCounter = numOfFrames; // TODO thread safety
	this->recorderFrameDrop = 0;
}

bool Universe::hasScreenCaptureFrames() const {
	return this->recorderFrameCounter > 1; // do not report single frame
}

SecTmReal Universe::getScreenCaptureFrameRate() const {
	return desc.renderFPS/(SEC_TM_REAL_ONE + desc.recorderFrameDrop);
}

void Universe::setScreenCaptureFrameDrop(U32 frameDrop) {
	desc.recorderFrameDrop = frameDrop;
}

int Universe::getWindowWidth() const {
	return desc.windowWidth;
}

int Universe::getWindowHeight() const {
	return desc.windowHeight;
}

//------------------------------------------------------------------------------
