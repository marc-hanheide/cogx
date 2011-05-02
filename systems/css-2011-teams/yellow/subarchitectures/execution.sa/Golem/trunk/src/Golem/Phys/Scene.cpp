/** @file Scene.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/Quat.h>
#include <Golem/Tools/Stream.h>
#include <Golem/Phys/Scene.h>
#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Msg.h>
#include <Golem/Phys/NxCooking.h>
#include <Golem/Phys/NxStream.h>
#include <GL/gl.h>
#include <GL/glu.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Scene::Scene(Universe &universe) :
	universe(universe), context(universe.getContext())
{}

Scene::~Scene() {
	release();
}

bool Scene::initDebug() {
	CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
	
	// Debug data visualization
	const Real visualizationScaleInv = REAL_ONE/universe.visualizationScale;
	
	pNxPhysicsSDK->setParameter(NX_VISUALIZATION_SCALE, desc.draw.debug ? NxReal(visualizationScaleInv) : NxReal(0.0)); // secure use of draw.debug
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_WORLD_AXES, (NxReal)(1.0));
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_BODY_AXES, (NxReal)1.0);
	pNxPhysicsSDK->setParameter(NX_VISUALIZE_ACTOR_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_CONTACT_FORCE, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LOCAL_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_WORLD_AXES, (NxReal)1.0);
	//pNxPhysicsSDK->setParameter(NX_VISUALIZE_JOINT_LIMITS, (NxReal)1.0);

	return true;
}

bool Scene::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgSceneInvalidDesc(Message::LEVEL_CRIT, "Scene::create(): Invalid description");
	
	{
		CriticalSectionWrapper csw(universe.getCSOpenGL());
		this->desc = desc;

		viewPoint = desc.viewPointDflt;
		viewPoint *= REAL_ONE/universe.visualizationScale;
		viewDir = desc.viewDirDflt;

		viewUp.set(&desc.nxSceneDesc.gravity.x);
		viewUp.multiply(-REAL_ONE, viewUp);
		viewUp.normalise(); // normalized reverse gravity vector

		clearColor = desc.clearColor;
		ambientColor = desc.ambientColor;
		diffuseColor = desc.diffuseColor;
		specularColor = desc.specularColor;

		mouseButton = desc.mouseButton;
		mouseX = 0;
		mouseY = 0;
		motionHandler(0, 0);
	}

	boundsGroupMask = Bounds::GROUP_DEFAULT;
	
	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX

		pNxPhysicsSDK = universe.pNxPhysicsSDK;
		
		pNxScene = pNxPhysicsSDK->createScene(desc.nxSceneDesc);
		if (pNxScene == NULL)
			throw MsgSceneNxSceneCreate(Message::LEVEL_CRIT, "Scene::create(): Unable to create NxScene");

		NxMaterial* defaultMaterial = pNxScene->getMaterialFromIndex(0); 
		defaultMaterial->setRestitution((NxReal)desc.restitution);
		defaultMaterial->setStaticFriction((NxReal)desc.staticFriction);
		defaultMaterial->setDynamicFriction((NxReal)desc.dynamicFriction);
	}

	if (!initDebug()) // Access to PhysX
		throw MsgSceneDebugInit(Message::LEVEL_CRIT, "Scene::create(): Unable to initialize debugging");

	return true;
}

void Scene::release() {
	// remove objects explicitly because they can use other resources which have to be released manually
	while (!objectList.empty())
		releaseObject(*objectList.back());
	
	for (NxShapeDescMap::iterator i = nxShapeDescMap.begin(); i != nxShapeDescMap.end(); i++)
		releaseNxShapeDescResources(i->second); // Access to PhysX
	nxShapeDescMap.clear();
	boundsDescMap.clear();

	if (pNxScene != NULL) {
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		pNxPhysicsSDK->releaseScene(*pNxScene);
		pNxScene = NULL;
	}
}

//------------------------------------------------------------------------------

Scene::NxShapeDescPtr Scene::createNxShapeDesc(const BoundingPlane::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxPlaneShapeDesc *pNxShapeDesc = new NxPlaneShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	//pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	//pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	// Ignore NxSphereShape local pose (it is not used by Novodex)
	pNxShapeDesc->normal.set(&pDesc->normal.v1);
	pNxShapeDesc->d = (NxReal)pDesc->distance;
	
	return nxShapeDescPtr;
}

Bounds::Desc::Ptr Scene::createBoundsDesc(const NxPlaneShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingPlane::Desc *pBoundingPlaneDesc = new BoundingPlane::Desc;
	pBoundsDesc.reset(pBoundingPlaneDesc);

	//pDesc->localPose.M.getRowMajor(&pBoundingPlaneDesc->pose.R.m11);
	//pDesc->localPose.t.get(&pBoundingPlaneDesc->pose.p.v1);
	// Ignore NxSphereShape local pose (it is not used by Novodex)
	pBoundingPlaneDesc->normal.set(&pDesc->normal.x);
	pBoundingPlaneDesc->distance = (Real)pDesc->d;

	return pBoundsDesc;
}

Scene::NxShapeDescPtr Scene::createNxShapeDesc(const BoundingSphere::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxSphereShapeDesc *pNxShapeDesc = new NxSphereShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);

	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	pNxShapeDesc->radius = (NxReal)pDesc->radius;
	
	return nxShapeDescPtr;
}

Bounds::Desc::Ptr Scene::createBoundsDesc(const NxSphereShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingSphere::Desc *pBoundingSphereDesc = new BoundingSphere::Desc;
	pBoundsDesc.reset(pBoundingSphereDesc);

	pDesc->localPose.M.getRowMajor(&pBoundingSphereDesc->pose.R.m11);
	pDesc->localPose.t.get(&pBoundingSphereDesc->pose.p.v1);	
	pBoundingSphereDesc->radius = (Real)pDesc->radius;

	return pBoundsDesc;
}

Scene::NxShapeDescPtr Scene::createNxShapeDesc(const BoundingBox::Desc* pDesc) const {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxBoxShapeDesc *pNxShapeDesc = new NxBoxShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);
	
	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);
	pNxShapeDesc->dimensions.set(&pDesc->dimensions.v1);

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr Scene::createBoundsDesc(const NxBoxShapeDesc *pDesc) const {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingBox::Desc *pBoundingBoxDesc = new BoundingBox::Desc;
	pBoundsDesc.reset(pBoundingBoxDesc);

	pDesc->localPose.M.getRowMajor(&pBoundingBoxDesc->pose.R.m11);
	pDesc->localPose.t.get(&pBoundingBoxDesc->pose.p.v1);
	pDesc->dimensions.get(&pBoundingBoxDesc->dimensions.v1);

	return pBoundsDesc;
}

Scene::NxShapeDescPtr Scene::createNxShapeDesc(const BoundingCylinder::Desc* pDesc) {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;

	// Create corresponding triangle mesh
	TriangleMesh mesh;
	if (!pDesc->createTriangleMesh(mesh))
		return nxShapeDescPtr;

	// and BoundingConvexMesh description
	BoundingConvexMesh::Desc desc;
	desc.vertices = mesh.vertices;
	desc.triangles = mesh.triangles;
	desc.pose = pDesc->pose;
	desc.bCook = true;//false;
	
	nxShapeDescPtr = createNxShapeDesc(&desc);
	return nxShapeDescPtr;
}

void Scene::copyShapeDesc(const ::NxConvexMeshDesc& src, BoundingConvexMesh::Desc& dst) const {
	if ((NxU32)dst.vertices.size() != src.numVertices) {
		dst.vertices.resize(src.numVertices);
	}
	// TODO recognize FP precision
	for (U32 i = 0; i < (U32)src.numVertices; ++i)
		dst.vertices[i].set(&((const NxReal*)src.points)[3*i]);
	
	if ((NxU32)dst.triangles.size() != src.numTriangles) {
		dst.triangles.resize(src.numTriangles);
	}
	// TODO recognize index size
	for (U32 i = 0; i < (U32)src.numTriangles; ++i)
		dst.triangles[i].set(&((const NxU32*)src.triangles)[3*i]);

	dst.bCook = (src.flags & NX_CF_COMPUTE_CONVEX) > 0;
}

Scene::NxShapeDescPtr Scene::createNxShapeDesc(BoundingConvexMesh::Desc* pDesc) {
	ASSERT(pDesc != NULL)
	NxShapeDescPtr nxShapeDescPtr;
	NxConvexMeshDesc nxConvexMeshDesc;

	nxConvexMeshDesc.numVertices = (NxU32)pDesc->vertices.size();
	// For some reason Ageia is unable to handle double precision floating point vectors
	std::vector<NxVec3> nxVertices(nxConvexMeshDesc.numVertices);
	for (U32 i = 0; i < (U32)nxConvexMeshDesc.numVertices; ++i)
		pDesc->vertices[i].get(&nxVertices[i].x);
	nxConvexMeshDesc.points = &nxVertices.front();
	nxConvexMeshDesc.pointStrideBytes = (NxU32)sizeof(NxVec3);

	if (pDesc->bCook) {
		nxConvexMeshDesc.flags |= NX_CF_COMPUTE_CONVEX;
	}
	else {
		nxConvexMeshDesc.numTriangles = (NxU32)pDesc->triangles.size();
		nxConvexMeshDesc.triangles = &pDesc->triangles.front();
		nxConvexMeshDesc.triangleStrideBytes = (NxU32)sizeof(Triangle);
	}

	CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX

	::MemoryWriteBuffer writeBufffer;
	if (!::CookConvexMesh(nxConvexMeshDesc, writeBufffer)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::createNxShapeDesc(): Unable to cook the convex mesh");
		return nxShapeDescPtr;
	}
	
	NxConvexShapeDesc *pNxShapeDesc = new NxConvexShapeDesc;
	nxShapeDescPtr.reset(pNxShapeDesc);
	
	pNxShapeDesc->localPose.M.setRowMajor(&pDesc->pose.R.m11);
	pNxShapeDesc->localPose.t.set(&pDesc->pose.p.v1);

	::MemoryReadBuffer readBufffer(writeBufffer.data);
	pNxShapeDesc->meshData = pNxPhysicsSDK->createConvexMesh(readBufffer);
	if (pNxShapeDesc->meshData == NULL) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::createNxShapeDesc(): Unable to create the convex mesh"	);
		nxShapeDescPtr.release();
		return nxShapeDescPtr;
	}

	if (pDesc->bCook) {
		NxConvexMeshDesc nxConvexMeshDesc;
		if (!pNxShapeDesc->meshData->saveToDesc(nxConvexMeshDesc)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::createNxShapeDesc(): Unable to save Novodex convex mesh description");
			nxShapeDescPtr.release();
			return nxShapeDescPtr;
		}

		//const U32* pt = (const U32*)nxConvexMeshDesc.triangles;
		//for (size_t i = 0; i < nxConvexMeshDesc.numTriangles; i++)
		//	printf("(%i, %i, %i)\n", pt[3*i+0], pt[3*i+1], pt[3*i+2]);

		copyShapeDesc(nxConvexMeshDesc, *pDesc);
	}

	return nxShapeDescPtr;
}

Bounds::Desc::Ptr Scene::createBoundsDesc(const NxConvexShapeDesc *pDesc) {
	ASSERT(pDesc != NULL)
	Bounds::Desc::Ptr pBoundsDesc;
	BoundingConvexMesh::Desc *pBoundingConvexMeshDesc = new BoundingConvexMesh::Desc;
	pBoundsDesc.reset(pBoundingConvexMeshDesc);

	CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
	
	NxConvexMeshDesc nxConvexMeshDesc;
	if (pDesc->meshData == NULL || !pDesc->meshData->saveToDesc(nxConvexMeshDesc)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::createBoundsDesc(): Unable to save Novodex convex mesh description");
		pBoundsDesc.release();
		return pBoundsDesc;
	}

	copyShapeDesc(nxConvexMeshDesc, *pBoundingConvexMeshDesc);

	return pBoundsDesc;
}

void Scene::releaseNxShapeDescResources(NxShapeDescPtr& nxShapeDescPtr) {
	NxConvexShapeDesc *pNxConvexShapeDesc = dynamic_cast<NxConvexShapeDesc*>(nxShapeDescPtr.get());
	if (pNxConvexShapeDesc != NULL && pNxConvexShapeDesc->meshData != NULL) {
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		
		pNxPhysicsSDK->releaseConvexMesh(*pNxConvexShapeDesc->meshData);
	}
}

//------------------------------------------------------------------------------

void Scene::preprocess(SecTmReal elapsedTime) {
	CriticalSectionWrapper csw(csObjects);
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
		(*i)->preprocess(elapsedTime);
}

void Scene::postprocess(SecTmReal elapsedTime) {
	CriticalSectionWrapper csw(csObjects);
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
		(*i)->postprocess(elapsedTime);
}

//------------------------------------------------------------------------------

void Scene::initGL() {
	// Setup default render states
	::glClearColor(
		GLclampf(this->clearColor._rgba.r)/255.0f,
		GLclampf(this->clearColor._rgba.g)/255.0f,
		GLclampf(this->clearColor._rgba.b)/255.0f,
		GLclampf(this->clearColor._rgba.a)/255.0f
	);
	::glEnable(GL_DEPTH_TEST);
	::glEnable(GL_COLOR_MATERIAL);
	::glEnable(GL_CULL_FACE);

	// Setup lighting
	GLfloat ambientColor [] = {
		GLfloat(this->ambientColor._rgba.r)/255.0f,
		GLfloat(this->ambientColor._rgba.g)/255.0f,
		GLfloat(this->ambientColor._rgba.b)/255.0f,
		GLfloat(this->ambientColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
	GLfloat diffuseColor [] = {
		GLfloat(this->diffuseColor._rgba.r)/255.0f,
		GLfloat(this->diffuseColor._rgba.g)/255.0f,
		GLfloat(this->diffuseColor._rgba.b)/255.0f,
		GLfloat(this->diffuseColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
	GLfloat specularColor [] = {
		GLfloat(this->specularColor._rgba.r)/255.0f,
		GLfloat(this->specularColor._rgba.g)/255.0f,
		GLfloat(this->specularColor._rgba.b)/255.0f,
		GLfloat(this->specularColor._rgba.a)/255.0f
	};
	::glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
	GLfloat position[] = {100.0f, 100.0f, 400.0f, 1.0f};
	::glLightfv(GL_LIGHT0, GL_POSITION, position);
	::glEnable(GL_LIGHTING);
	::glEnable(GL_LIGHT0);
	
	::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	::glEnable(GL_BLEND);
}

void Scene::initCamera() {
	// Setup camera
	if (desc.glMat) {
		std::vector<double> mat(16);

		// intrinsic camera parameters
		::glMatrixMode(GL_PROJECTION);
		::glLoadMatrixf(desc.glMatIntrinsic);
		
		// extrinsic camera parameters
		::glMatrixMode(GL_MODELVIEW);
		::glLoadMatrixf(desc.glMatExtrinsic);
	}
	else {
		::glMatrixMode(GL_PROJECTION);
		::glLoadIdentity();
		::gluPerspective(
			GLdouble(60.0),
			GLdouble(universe.glAspectRatio),
			GLdouble(0.1/universe.visualizationScale),
			GLdouble(100.0/universe.visualizationScale)
		);
		::gluLookAt(
			GLdouble(viewPoint.v1), GLdouble(viewPoint.v2), GLdouble(viewPoint.v3),
			GLdouble(viewPoint.v1 + viewDir.v1), GLdouble(viewPoint.v2 + viewDir.v2), GLdouble(viewPoint.v3 + viewDir.v3),
			GLdouble(viewUp.v1), GLdouble(viewUp.v2), GLdouble(viewUp.v3)
		);
		::glMatrixMode(GL_MODELVIEW);
		::glLoadIdentity();
	}

	::glViewport(0, 0, universe.getWindowWidth(), universe.getWindowHeight());
}

void Scene::render() {
	// Clear buffers
	::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	{
		CriticalSectionWrapper csw(csObjects);
		for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
			(*i)->customRender();
	}
	
	initCamera();
	
	{
		CriticalSectionWrapper csw(csObjects);
		for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
			(*i)->render();
	}

	if (desc.draw.debug) // csOpenGL is always locked here
		nxDebugRenderer.renderData(*pNxScene->getDebugRenderable()); // always pNxScene != NULL and always called in the same thread as PhysX
}

//------------------------------------------------------------------------------

void Scene::mouseHandler(int button, int state, int x, int y) {
	this->mouseButton = button;
	this->mouseX = x;
	this->mouseY = y;

	CriticalSectionWrapper csw(csObjects);
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
		(*i)->mouseHandler(button, state, x, y);
}

void Scene::motionHandler(int x, int y) {
	if (mouseButton == desc.mouseButton) {
		int dx = mouseX - x;
		int dy = mouseY - y;

		viewDir.normalise();
		viewNormal.cross(viewDir, viewUp);

		Quat qx(REAL_PI*1.0*dx/180.0, viewUp);
		qx.multiply(viewDir, viewDir);
		Quat qy(REAL_PI*1.0*dy/180.0, viewNormal);
		qy.multiply(viewDir, viewDir);

		mouseX = x;
		mouseY = y;
	}

	CriticalSectionWrapper csw(csObjects);
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
		(*i)->motionHandler(x, y);
}

void Scene::keyboardHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 101:// arrow up
		viewPoint += viewDir*Real(0.2)/universe.visualizationScale;
		break;
	case 103:// arrow down
		viewPoint -= viewDir*Real(0.2)/universe.visualizationScale;
		break;
	case 100:// arrow left
		viewPoint -= viewNormal*Real(0.2)/universe.visualizationScale;
		break;
	case 102:// arrow right
		viewPoint += viewNormal*Real(0.2)/universe.visualizationScale;
		break;
	case 22:// ctrl v
		context.getMessageStream()->write(Message::LEVEL_INFO,
			"view point (%.3f, %.3f, %.3f), view dir (%.3f, %.3f, %.3f)",
			universe.visualizationScale*viewPoint.v1, universe.visualizationScale*viewPoint.v2, universe.visualizationScale*viewPoint.v3, viewDir.v1, viewDir.v2, viewDir.v3
		);
		break;
	case 'z':
		{
			CriticalSectionWrapper csw(universe.getCSOpenGL());
			desc.draw.solid = !desc.draw.solid;
		}
		break;
	case 'x':
		{
			CriticalSectionWrapper csw(universe.getCSOpenGL());
			desc.draw.wire = !desc.draw.wire;
		}
		break;
	case 'c':
		{
			CriticalSectionWrapper csw(universe.getCSOpenGL());
			desc.draw.shadow = !desc.draw.shadow;
		}
		break;
	case 'v':
		{
			CriticalSectionWrapper csw(universe.getCSOpenGL());
			desc.draw.debug = !desc.draw.debug;
		}
	// visualisation may be off in the next scene (global change)!
	case 104: case 105: case 32:// pgup, pgdown, space
		initDebug();
		break;
	}

	CriticalSectionWrapper csw(csObjects);
	for (ObjectList::const_iterator i = objectList.begin(); i != objectList.end(); i++)
		(*i)->keyboardHandler(key, x, y);
}

//------------------------------------------------------------------------------

/** Creates Novodex shape description from bounds description. */
NxShapeDesc* Scene::createNxShapeDesc(Bounds::Desc::Ptr pBoundsDesc) {
	// check if shape description has been already created
	NxShapeDescMap::iterator pos = nxShapeDescMap.find(pBoundsDesc.get());
	if (pos != nxShapeDescMap.end())
		return pos->second.get();

	// check if description is valid
	if (!pBoundsDesc->isValid())
		throw MsgSceneBoundsDescInvalidDesc(Message::LEVEL_CRIT, "Scene::createNxShapeDesc(): Invalid bounds description");

	NxShapeDescPtr nxShapeDescPtr;
	
	switch (pBoundsDesc->getType()) {
	case Bounds::TYPE_PLANE:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingPlane::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_SPHERE:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingSphere::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_CYLINDER:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingCylinder::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_BOX:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<const BoundingBox::Desc*>(pBoundsDesc.get()));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		nxShapeDescPtr = createNxShapeDesc(dynamic_cast<BoundingConvexMesh::Desc*>(pBoundsDesc.get())); // Access to PhysX
		break;
	default:
		ASSERT(false)
		break;
	}

	if (nxShapeDescPtr == NULL)
		throw MsgSceneBoundsDescCreate(Message::LEVEL_CRIT, "Scene::createNxShapeDesc(): Unable to create bounds description");

	nxShapeDescMap[pBoundsDesc.get()] = nxShapeDescPtr;
	boundsDescMap[nxShapeDescPtr.get()] = pBoundsDesc;

	return nxShapeDescPtr.get();
}

/** Releases the Novodex shape description. */
void Scene::releaseNxShapeDesc(NxShapeDesc& nxShapeDesc) {
	BoundsDescMap::iterator pos1 = boundsDescMap.find(&nxShapeDesc);
	if (pos1 == boundsDescMap.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::releaseNxShapeDesc(): Unable to to find the specified shape description");
		return;
	}

	NxShapeDescMap::iterator pos2 = nxShapeDescMap.find(pos1->second.get());
	releaseNxShapeDescResources(pos2->second); // Access to PhysX (for ConvexMesh)
	boundsDescMap.erase(pos1);
	nxShapeDescMap.erase(pos2);
}

/** Returns bounds description associated with the Novodex shape description. */
Bounds::Desc::Ptr Scene::createBoundsDesc(const NxShapeDesc &nxShapeDesc) {
	// check if bound description has been already created
	BoundsDescMap::const_iterator pos = boundsDescMap.find(&nxShapeDesc);
	if (pos != boundsDescMap.end())
		return pos->second;
	
	Bounds::Desc::Ptr pBoundsDesc;

	// check if description is valid
	if (!nxShapeDesc.isValid())
		throw MsgSceneNxShapeDescInvalidDesc(Message::LEVEL_CRIT, "Scene::createBoundsDesc(): invalid Novodex shape description");

	NxShapeDescPtr nxShapeDescPtr;

	if (nxShapeDesc.getType() == NX_SHAPE_PLANE) {
		const NxPlaneShapeDesc *pDescSrc = dynamic_cast<const NxPlaneShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxPlaneShapeDesc *pDescDst = new NxPlaneShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxPlaneShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_SPHERE) {
		const NxSphereShapeDesc *pDescSrc = dynamic_cast<const NxSphereShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxSphereShapeDesc *pDescDst = new NxSphereShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxSphereShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_BOX) {
		const NxBoxShapeDesc *pDescSrc = dynamic_cast<const NxBoxShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxBoxShapeDesc *pDescDst = new NxBoxShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxBoxShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst);
		}
	}
	else if (nxShapeDesc.getType() == NX_SHAPE_CONVEX) {
		const NxConvexShapeDesc *pDescSrc = dynamic_cast<const NxConvexShapeDesc*>(&nxShapeDesc);
		if (pDescSrc != NULL) {
			NxConvexShapeDesc *pDescDst = new NxConvexShapeDesc();
			nxShapeDescPtr.reset(pDescDst);
			::memcpy(pDescDst, pDescSrc, sizeof(NxConvexShapeDesc));
			pBoundsDesc = createBoundsDesc(pDescDst); // Access to PhysX
		}
	}
	else {
		ASSERT(false)
	}

	if (pBoundsDesc == NULL)
		throw MsgSceneNxShapeDescCreate(Message::LEVEL_CRIT, "Scene::createBoundsDesc(): failed to create bounds description");

	nxShapeDescMap[pBoundsDesc.get()] = nxShapeDescPtr;
	boundsDescMap[nxShapeDescPtr.get()] = pBoundsDesc;

	return pBoundsDesc;
}

/** Releases the bounds description and related Novodex shape description. */
void Scene::releaseBoundsDesc(Bounds::Desc::Ptr pBoundsDesc) {
	NxShapeDescMap::iterator pos1 = nxShapeDescMap.find(pBoundsDesc.get());
	if (pos1 == nxShapeDescMap.end()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::releaseBoundsDesc(): Unable to to find the specified bouds description");
		return;
	}
	
	BoundsDescMap::iterator pos2 = boundsDescMap.find(pos1->second.get());
	releaseNxShapeDescResources(pos1->second); // Access to PhysX
	nxShapeDescMap.erase(pos1);
	boundsDescMap.erase(pos2);
}

//------------------------------------------------------------------------------

Object* Scene::createObject(const Object::Desc& desc) {
	Object::Ptr pObject(desc.create(*this)); // Access to PhysX
	if (pObject == NULL)
		throw MsgSceneObjectCreate(Message::LEVEL_CRIT, "Scene::createObject(): Unable to create object");

	{
		CriticalSectionWrapper csw(csObjects);
		objectList.push_back(ObjectList::Pair(pObject.get(), pObject));
	}

	return pObject.get();
}

void Scene::releaseObject(Object& object) {
	if (!objectList.contains(&object)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Scene::releaseObject(): Unable to find specified object");
		return;
	}

	{
		CriticalSectionWrapper csw(csObjects);
		object.release(); // Access to PhysX
		objectList.erase(&object);
	}
}

const Scene::ObjectList& Scene::getObjectList() const {
//	CriticalSectionWrapper csw(csObjects);
	return objectList; // copy constructor called before ~CriticalSectionWrapper()
}

//------------------------------------------------------------------------------

/** Generates new bounds group mask. */
U32 Scene::createBoundsGroup() {
	U32 group = Bounds::GROUP_DEFAULT;
	
	do group <<= 1; while (group & boundsGroupMask);
	boundsGroupMask |= group;
	
	return group ? group : Bounds::GROUP_UNDEF;
}

/** Generates new bounds group mask. */
void Scene::removeBoundsGroup(U32 group) {
	boundsGroupMask &= ~group | Bounds::GROUP_DEFAULT;
}

U32 Scene::getBoundsGroupMask() const {
	return boundsGroupMask;
}

//------------------------------------------------------------------------------

void Scene::getDraw(Draw& draw) const {
	CriticalSectionWrapper csw(universe.getCSOpenGL());
	draw = desc.draw;
}

void Scene::setDraw(const Draw& draw) {
	CriticalSectionWrapper csw(universe.getCSOpenGL());
	desc.draw = draw;
}


void Scene::resetCamera() {
	CriticalSectionWrapper csw(universe.getCSOpenGL());
	desc.glMat = false;
}

void Scene::setCamera(const Vec3& viewPoint, const Vec3& viewDir) {
	CriticalSectionWrapper csw(universe.getCSOpenGL());
	this->viewPoint = viewPoint;
	this->viewDir = viewDir;
	desc.glMat = false;
}

void Scene::setCamera(const GLfloat glMatIntrinsic[], const GLfloat glMatExtrinsic[]) {
	CriticalSectionWrapper csw(universe.getCSOpenGL());
	std::copy(glMatIntrinsic, glMatIntrinsic + 16, desc.glMatIntrinsic);
	std::copy(glMatExtrinsic, glMatExtrinsic + 16, desc.glMatExtrinsic);
	desc.glMat = true;
}

//------------------------------------------------------------------------------

