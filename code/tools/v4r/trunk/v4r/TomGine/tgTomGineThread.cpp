
#include "tgTomGineThread.h"

using namespace TomGine;

bool threaddebug = false;

/** ThreadDrawing (OpenGL commands are only allowed in this thread) */
namespace TomGine {
void* ThreadDrawing(void* c) {
	tgTomGineThread *tt = (tgTomGineThread*) c;

	if (threaddebug)
		printf("ThreadDrawing: start\n");

	TomGine::tgEngine* render = new TomGine::tgEngine(tt->width, tt->height,
			10.0, 0.01, "TomGine", false, true);

	pthread_mutex_lock(&tt->dataMutex);
	tt->m_engine = render;
	bool stop = tt->stopTomGineThread;
	pthread_mutex_unlock(&tt->dataMutex);

	std::list<V4R::Event> eventlist;

	while (!stop) {
		if (threaddebug)
			printf("ThreadDrawing: wait\n");
		sem_wait(&tt->renderSem);

		pthread_mutex_lock(&tt->eventMutex);
		if (threaddebug)
			printf("ThreadDrawing: get events\n");
		eventlist.clear();
		while (!tt->m_eventlist.empty()) {
			eventlist.push_back(tt->m_eventlist.front());
			tt->m_eventlist.pop_front();
			sem_trywait(&tt->renderSem);
		}
		tt->m_eventlist.clear();
		pthread_mutex_unlock(&tt->eventMutex);

		pthread_mutex_lock(&tt->dataMutex);
		if (threaddebug)
			printf("ThreadDrawing: apply events\n");
		while (!eventlist.empty()) {
			if (threaddebug)
				printf("ThreadDrawing: event type: %d\n",
						eventlist.front().type);
			render->InputControl(eventlist.front());
			eventlist.pop_front();
		}

		//		pthread_mutex_lock(&tt->dataMutex);
		if (threaddebug)
			printf("ThreadDrawing: render C\n");
		tt->GL_Update(render);
		if (threaddebug)
			printf("ThreadDrawing: render D\n");
		tt->GL_Draw3D(render);
		if (threaddebug)
			printf("ThreadDrawing: render E\n");
		stop = tt->stopTomGineThread;
		pthread_mutex_unlock(&tt->dataMutex);
		if (threaddebug)
			printf("ThreadDrawing: swap\n");
		render->Swap();
		sem_post(&tt->renderFinishedSem);
	}

	pthread_mutex_lock(&tt->dataMutex);
	if (threaddebug)
		printf("ThreadDrawing: clean up A\n");
	tt->GL_Clear();
	if (threaddebug)
		printf("ThreadDrawing: clean up B\n");
	tt->m_engine = NULL;
	if (threaddebug)
		printf("ThreadDrawing: clean up C\n");
	pthread_mutex_unlock(&tt->dataMutex);
	if (threaddebug)
		printf("ThreadDrawing: clean up D\n");

	delete render;

	if (threaddebug)
		printf("ThreadDrawing: stop\n");

	pthread_mutex_lock(&tt->dataMutex);
	tt->renderingStopped = true;
	pthread_mutex_unlock(&tt->dataMutex);

	pthread_exit(NULL);
	//	return((void *)0);
}

void* ThreadEventHandling(void* c) {
	tgTomGineThread *tt = (tgTomGineThread*) c;

	if (threaddebug)
		printf("ThreadEventHandling: start\n");

	pthread_mutex_lock(&tt->dataMutex);
	bool stop = tt->stopTomGineThread;
	pthread_mutex_unlock(&tt->dataMutex);

	bool engineExists = false;

	V4R::Event event;
	while (!stop) {
		if (threaddebug)
			printf("ThreadEventHandling: loop\n");
		if (engineExists) {

			if (threaddebug)
				printf("ThreadEventHandling: Wait for event\n");
			tt->m_engine->WaitForEvent(event);

			pthread_mutex_lock(&tt->dataMutex);
			if (tt->KeyHandler(event))
				tt->stopTomGineThread = true;
			pthread_mutex_unlock(&tt->dataMutex);

			pthread_mutex_lock(&tt->eventMutex);
			tt->m_eventlist.push_back(event);
			sem_post(&tt->renderSem);
			pthread_mutex_unlock(&tt->eventMutex);

			if (threaddebug)
				printf("ThreadEventHandling: continue\n");

		} else {
			usleep(10000); // wait for render engine to start up
		}
		pthread_mutex_lock(&tt->dataMutex);
		stop = tt->stopTomGineThread;
		engineExists = tt->m_engine;
		pthread_mutex_unlock(&tt->dataMutex);
	}

	if (threaddebug)
		printf("ThreadEventHandling: stop\n");

	pthread_mutex_lock(&tt->dataMutex);
	tt->eventsStopped = true;
	pthread_mutex_unlock(&tt->dataMutex);

	pthread_exit(NULL);
	//	return((void *)0);
}
}

/********************** tgTomGineThread ************************/
tgTomGineThread::tgTomGineThread(int w, int h) :
	width(w), height(h), mode(1), stopTomGineThread(false),
			renderingStopped(false), eventsStopped(false), drawImage(false), loadImage(false),
			draw3D(true), drawPointCloud(true), drawVPointCloud(true),
			drawLabels(false), drawModels(false), drawNurbs(false),
			useProbLevel(false), showCoordinateFrame(false), clear(false) {
	camChanged = false;
	rotCenterChanged = false;
	rotCenter[0] = 0.;
	rotCenter[1] = 0.;
	rotCenter[2] = 0.;
	m_engine = NULL;
	sem_init(&renderSem, 0, 0);
	sem_init(&renderFinishedSem, 0, 0);
	pthread_mutex_init(&eventMutex, NULL);
	pthread_mutex_init(&dataMutex, NULL);
	pthread_create(&thread_gl, NULL, ThreadDrawing, this);
	pthread_create(&thread_event, NULL, ThreadEventHandling, this);
}

tgTomGineThread::~tgTomGineThread() {
	stopTomGineThread = true;
	sem_post(&renderSem);
	sem_post(&renderFinishedSem);

	if (threaddebug)
		printf("tgTomGineThread::~tgTomGineThread A\n");
	pthread_join(thread_gl, NULL);
	if (threaddebug)
		printf("tgTomGineThread::~tgTomGineThread B\n");
	pthread_join(thread_event, NULL);
	if (threaddebug)
		printf("tgTomGineThread::~tgTomGineThread C\n");

	sem_destroy(&renderSem);
	sem_destroy(&renderFinishedSem);
	pthread_mutex_destroy(&dataMutex);
	pthread_mutex_destroy(&eventMutex);
	if (threaddebug)
		printf("tgTomGineThread::~tgTomGineThread D\n");
}

void tgTomGineThread::GL_Clear() {
	for (unsigned i = 0; i < this->m_pointclouds.size(); i++)
		delete this->m_pointclouds[i];
	m_pointclouds.clear();
	for (unsigned i = 0; i < this->nurbsSurface.size(); i++)
		delete this->nurbsSurface[i];
	nurbsSurface.clear();
	nurbsSurfaceData.clear();

	points3D.clear();
	pointCols3D.clear();
	pointSize3D.clear();
	lines3D.clear();
	lineCols3D.clear();
	lineWidth3D.clear();
	vLabel.clear();
}

bool tgTomGineThread::KeyHandler(V4R::Event &event) {
	if (event.type == V4R::TMGL_Quit)
		return true;
	if (event.type == V4R::TMGL_Press) {
		if (event.input == V4R::TMGL_Escape)
			return true;
		else if (event.input == V4R::TMGL_c)
			clear = true;
		else if (event.input == V4R::TMGL_i)
			drawImage = !drawImage;
		else if (event.input == V4R::TMGL_p)
			drawPointCloud = !drawPointCloud;
		else if (event.input == V4R::TMGL_l)
			drawLabels = !drawLabels;
		else if (event.input == V4R::TMGL_m)
			drawModels = !drawModels;
		else if (event.input == V4R::TMGL_n)
			drawNurbs = !drawNurbs;
		else if (event.input == V4R::TMGL_u)
			useProbLevel = !useProbLevel;
		else if (event.input == V4R::TMGL_q)
			return true;
	}
	return false;
}

void tgTomGineThread::GL_DrawPointCloud() {

	glEnable(GL_POINT_SMOOTH);
	glPointSize(1.0);

	for (unsigned i = 0; i < this->m_pointclouds.size(); i++)
		this->m_pointclouds[i]->DrawColorPoints();

#ifdef DEBUG
	tgCheckError("[tgTomGineThread::DrawPointCloud]");
#endif
}

void tgTomGineThread::GL_DrawPoints3D() {
	glDisable(GL_LIGHTING);
	glEnable(GL_POINT_SMOOTH);

	glPointSize(1.0);

	glBegin(GL_POINTS);
	for (unsigned i = 0; i < points3D.size(); i++) {
		glColor3f(pointCols3D[i].x, pointCols3D[i].y, pointCols3D[i].z);
		glVertex3f(points3D[i].x, points3D[i].y, points3D[i].z);
	}
	glEnd();

#ifdef DEBUG
	tgCheckError("[tgTomGineThread::DrawPoints3D]");
#endif
}

void tgTomGineThread::GL_DrawLines3D() {
	glDisable(GL_LIGHTING);

	for (unsigned i = 0; i < lines3D.size(); i++) {
		if (useProbLevel) {
			if (lineWidth3D[i] > 0.95) {
				glLineWidth(lineWidth3D[i] * 5.0f);
				glBegin(GL_LINES);
				glColor3f(lineCols3D[i][2], lineCols3D[i][1], lineCols3D[i][0]);
				glVertex3f(lines3D[i].first[0], lines3D[i].first[1],
						lines3D[i].first[2]);
				glVertex3f(lines3D[i].second[0], lines3D[i].second[1],
						lines3D[i].second[2]);
				glEnd();
			}
		} else {
			glLineWidth(lineWidth3D[i] * 5.0f);
			glBegin(GL_LINES);
			glColor3f(lineCols3D[i][2], lineCols3D[i][1], lineCols3D[i][0]);
			glVertex3f(lines3D[i].first[0], lines3D[i].first[1],
					lines3D[i].first[2]);
			glVertex3f(lines3D[i].second[0], lines3D[i].second[1],
					lines3D[i].second[2]);
			glEnd();
		}
	}
#ifdef DEBUG
	tgCheckError("[tgTomGineThread::DrawLines3D]");
#endif
}

void tgTomGineThread::GL_DrawModels() {
	for (unsigned i = 0; i < this->m_models.size(); i++) {
		m_models[i]->Draw();
	}
#ifdef DEBUG
	tgCheckError("[tgTomGineThread::DrawModels] models");
#endif
}

void tgTomGineThread::GL_SyncNurbsData() {

	// add new nurbs
	while (this->nurbsSurface.size() < this->nurbsSurfaceData.size()) {
		unsigned id = this->nurbsSurface.size();
		TomGine::tgNurbsSurface *surf = new TomGine::tgNurbsSurface(
				this->nurbsSurfaceData[id]);
		this->nurbsSurface.push_back(surf);
		this->nurbsSurfaceData[id].sync = true;
	}
#ifdef DEBUG
	tgCheckError("[tgTomGineThread::SyncNurbsData] add new nurbs");
#endif
	// update out of sync nurbs
	for (unsigned i = 0; i < this->nurbsSurfaceData.size(); i++) {
		if (!this->nurbsSurfaceData[i].sync) {
			this->nurbsSurface[i]->Set(this->nurbsSurfaceData[i]);
			this->nurbsSurfaceData[i].sync = true;
		}
	}

}

void tgTomGineThread::GL_DrawNurbs() {
	for (unsigned i = 0; i < this->nurbsSurface.size(); i++) {
		this->nurbsSurface[i]->DrawFaces();
//		glPointSize(5.0);
//		glColor3f(0.0, 1.0, 0.0);
//		this->nurbsSurface[i]->DrawCPs();
	}
#ifdef DEBUG
	tgCheckError("[tgTomGineThread::DrawNurbs]");
#endif
}

void tgTomGineThread::GL_Update(TomGine::tgEngine *render) {
	glClearColor(0.0, 0.0, 0.0, 1.);

	if (clear) {
		GL_Clear();
		clear = false;
	}

	// set camera pose...
	if (camChanged) {
		unsigned w = unsigned(width);
		unsigned h = unsigned(height);
		render->SetCamera(intrinsic, w, h, extR, extT);
		camChanged = false;
	}

	// set center of rotation
	if (rotCenterChanged)
		render->SetCenterOfRotation(rotCenter[0], rotCenter[1], rotCenter[2]);

	// synchronize NURBS data
	if (this->drawNurbs)
		GL_SyncNurbsData();
}

void tgTomGineThread::GL_Draw3D(TomGine::tgEngine *render) {
	// set background image
	if (loadImage && !img.empty()){
		render->LoadBackgroundImage(img.data, img.cols, img.rows, GL_BGR, true);
		loadImage = false;
	}
	if(drawImage)
		render->DrawBackgroundImage();

	render->Activate3D();

	if (drawPointCloud)
		GL_DrawPointCloud();

	if (drawModels)
		GL_DrawModels();

	if (drawNurbs)
		GL_DrawNurbs();

	GL_DrawPoints3D();

	render->DrawCoordinates(0.2, 2.0);

//	render->PrintText2D("test", vec2(10,10), 20);

}

/***************************** PUBLIC *****************************/

void tgTomGineThread::SetCamera(cv::Mat &_intrinsic) {
	pthread_mutex_lock(&dataMutex);
	_intrinsic.copyTo(intrinsic);
	camChanged = true;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetCamera(cv::Mat &R, cv::Mat &t) {
	pthread_mutex_lock(&dataMutex);
	R.copyTo(extR);
	t.copyTo(extT);
	camChanged = true;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetRotationCenter(cv::Vec3d &_rotCenter) {
	pthread_mutex_lock(&dataMutex);
	rotCenter = _rotCenter;
	rotCenterChanged = true;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetImage(cv::Mat &_img) {
	if (_img.channels() != 3)
		return;

	pthread_mutex_lock(&dataMutex);
	_img.copyTo(img);
	loadImage = true;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::AddPoint3D(double x, double y, double z, uchar r,
		uchar g, uchar b, double size) {
	pthread_mutex_lock(&dataMutex);
	points3D.push_back(cv::Point3f((float) x, (float) y, (float) z));
	pointCols3D.push_back(
			cv::Point3f((float) r / 255., (float) g / 255., (float) b / 255.));
	pointSize3D.push_back((float) size);
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::AddLine3D(double x1, double y1, double z1,
								double x2, double y2, double z2,
								uchar r, uchar g, uchar b,
								float width) {
	pthread_mutex_lock(&dataMutex);
	lines3D.push_back(
			std::make_pair<cv::Vec3f, cv::Vec3f>(
					cv::Vec3f((float) x1, (float) y1, (float) z1),
					cv::Vec3f((float) x2, (float) y2, (float) z2)));
	lineCols3D.push_back(
			cv::Vec3f((float) r / 255., (float) g / 255., (float) b / 255.));
	lineWidth3D.push_back(width);

	pthread_mutex_unlock(&dataMutex);
}

int tgTomGineThread::AddModel(TomGine::tgModel *model) {
	pthread_mutex_lock(&dataMutex);
	this->m_models.push_back(model);
	int id = (this->m_models.size() - 1);
	pthread_mutex_unlock(&dataMutex);
	return id;
}

int tgTomGineThread::AddNurbsSurface(
		const TomGine::tgNurbsSurfacePatch &nurbsData) {
	pthread_mutex_lock(&dataMutex);
	this->nurbsSurfaceData.push_back(nurbsData);
	this->drawNurbs = true;
	int id = (this->nurbsSurfaceData.size() - 1);
	pthread_mutex_unlock(&dataMutex);
	return id;
}

int tgTomGineThread::AddPointCloud(const TomGine::tgModel &pcl) {
	tgModel* tg_cloud = new tgModel;
	tg_cloud->m_colorpoints = pcl.m_colorpoints;

	pthread_mutex_lock(&dataMutex);
	this->m_pointclouds.push_back(tg_cloud);
	int id = (this->m_pointclouds.size() - 1);
	pthread_mutex_unlock(&dataMutex);
	return id;
}

int tgTomGineThread::AddPointCloud(cv::Mat_<cv::Vec4f> cloud) {
	tgRGBValue color;
	tgModel* tg_cloud = new tgModel;

	for (unsigned i = 0; i < cloud.rows; i++) {
		for (unsigned j = 0; j < cloud.cols; j++) {
			TomGine::tgColorPoint cpt;
			cv::Vec4f &pt = cloud(i, j);
			color.float_value = pt[3];
			cpt.color[0] = color.Red;
			cpt.color[1] = color.Green;
			cpt.color[2] = color.Blue;
			cpt.pos = vec3(pt[0], pt[1], pt[2]);
			tg_cloud->m_colorpoints.push_back(cpt);
		}
	}
	pthread_mutex_lock(&dataMutex);
	this->m_pointclouds.push_back(tg_cloud);
	int id = (this->m_pointclouds.size() - 1);
	pthread_mutex_unlock(&dataMutex);
	return id;
}

void tgTomGineThread::SetModel(int id, TomGine::tgModel *model) {
	pthread_mutex_lock(&dataMutex);
	if(id<0 || id >= this->m_models.size()){
		pthread_mutex_unlock(&dataMutex);
		printf( "[tgTomGineThread::SetModel] Warning index out of bounds: %d.\n",
				id);
		return;
	}
	this->m_models[id] = model;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetNurbsSurface(int id,
		const TomGine::tgNurbsSurfacePatch &nurbsData) {
	pthread_mutex_lock(&dataMutex);
	if (id < 0 || id >= this->nurbsSurfaceData.size()) {
		pthread_mutex_unlock(&dataMutex);
		printf( "[tgTomGineThread::SetNurbsSurface] Warning index out of bounds: %d.\n",
				id);
		return;
	}
	this->nurbsSurfaceData[id] = nurbsData;
	this->nurbsSurfaceData[id].sync = false;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetPointCloud(int id, const TomGine::tgModel &pcl) {

	pthread_mutex_lock(&dataMutex);
	if (id < 0 || id >= this->m_pointclouds.size()) {
		pthread_mutex_unlock(&dataMutex);
		printf( "[tgTomGineThread::SetPointCloud] Warning index out of bounds: %d.\n",
				id);
		return;
	}
	pthread_mutex_unlock(&dataMutex);

	tgModel* tg_cloud = new tgModel;
	tg_cloud->m_colorpoints = pcl.m_colorpoints;

	pthread_mutex_lock(&dataMutex);
	delete this->m_pointclouds[id];
	this->m_pointclouds[id] = tg_cloud;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::SetPointCloud(int id, cv::Mat_<cv::Vec4f> cloud) {

	pthread_mutex_lock(&dataMutex);
	if (id < 0 || id >= this->m_pointclouds.size()) {
		pthread_mutex_unlock(&dataMutex);
		printf( "[tgTomGineThread::SetPointCloud] Warning index out of bounds: %d.\n",
				id);
		return;
	}
	pthread_mutex_unlock(&dataMutex);

	tgRGBValue color;
	tgModel* tg_cloud = new tgModel;

	for (unsigned i = 0; i < cloud.rows; i++) {
		for (unsigned j = 0; j < cloud.cols; j++) {
			TomGine::tgColorPoint cpt;
			cv::Vec4f &pt = cloud(i, j);
			color.float_value = pt[3];
			cpt.color[0] = color.Red;
			cpt.color[1] = color.Green;
			cpt.color[2] = color.Blue;
			cpt.pos = vec3(pt[0], pt[1], pt[2]);
			tg_cloud->m_colorpoints.push_back(cpt);
		}
	}
	pthread_mutex_lock(&dataMutex);
	delete this->m_pointclouds[id];
	this->m_pointclouds[id] = tg_cloud;
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::Update() {
	V4R::Event event;
	event.type = V4R::TMGL_None;
	pthread_mutex_lock(&eventMutex);
	m_eventlist.push_back(event);
	sem_post(&renderSem);
	pthread_mutex_unlock(&eventMutex);

	sem_wait(&renderFinishedSem);
}

void tgTomGineThread::Clear() {
	pthread_mutex_lock(&dataMutex);
	clear = true;
	pthread_mutex_unlock(&dataMutex);
}

bool tgTomGineThread::Stopped() {
	pthread_mutex_lock(&dataMutex);
	bool stopped = (this->renderingStopped && this->eventsStopped);
	pthread_mutex_unlock(&dataMutex);
	return stopped;
}

void tgTomGineThread::ClearPoints3D() {
	pthread_mutex_lock(&dataMutex);
	points3D.clear();
	pointCols3D.clear();
	pointSize3D.clear();
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearLines3D() {
	pthread_mutex_lock(&dataMutex);
	lines3D.clear();
	lineCols3D.clear();
	lineWidth3D.clear();
	pthread_mutex_unlock(&dataMutex);
}

void tgTomGineThread::ClearModels() {
	pthread_mutex_lock(&dataMutex);
	m_models.clear();
	pthread_mutex_unlock(&dataMutex);
}


