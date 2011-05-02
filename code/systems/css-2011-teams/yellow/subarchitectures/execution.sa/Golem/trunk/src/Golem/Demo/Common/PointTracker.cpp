/** @file PointTracker.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/PointTracker.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool PointRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	const U32 size = desc.pPoints != NULL ? (U32)(desc.pPoints->size()) : U32(0);
	const U32 items = Math::clamp((U32)(desc.showFrac*size), (U32)0, (U32)size);
//	if (items <= 0)
//		return true;

	// reserve memory buffer
	reset();
	if (desc.positionShow)
		reservePoints(items);

	for (U32 i = 0; i < items; i++) {
		const PointFeature &feature = (*desc.pPoints)[i];// use permutation if necessary
		
		if (desc.positionShow) {
			addPoint(feature.position, desc.positionColour);
		}
	}

	return true;
}

//------------------------------------------------------------------------------

PointTracker::PointTracker(Embodiment &embodiment) :
	Base(embodiment),
	rand(context.getRandSeed())
{
	pPointRenderer.reset(new PointRenderer());
}

PointTracker::~PointTracker() {
}

bool PointTracker::create(const PointTracker::Desc& desc) {
	Base::create(desc); // throws

	pointsDensity = desc.pointsDensity;
	positionCov = desc.positionCov;
	//orientationCov = desc.orientationCov;
	planeBoundsDesc = desc.planeBoundsDesc;
	
	pointRendererDesc = desc.pointRendererDesc;
	pointRendererDesc.pPoints = &points;
	pointShow = desc.pointShow;
	
	timeStamp = SEC_TM_REAL_ZERO;
	points.clear();
	evWait.set(false);
	
	return true;
}

bool PointTracker::read(PointFeature::Seq &points, SecTmReal &timeStamp, MSecTmU32 timeOut) {
	if (!evWait.wait(timeOut))
		return false;

	CriticalSectionWrapper csw(cs);
	
	timeStamp = this->timeStamp;
	points = this->points;

	evWait.set(false);
	return true;
}

void PointTracker::postprocess(SecTmReal elapsedTime) {
	CriticalSectionWrapper csw(cs);
	
	if (!get(points, timeStamp))
		return;
	evWait.set(true);

	if (pointShow)
		pPointRenderer->create(pointRendererDesc);
}

void PointTracker::render() {
	// postprocess() and render() are always called from the same thread 
	CriticalSectionWrapper csw(cs);
	
	if (pointShow)
		pPointRenderer->render();
}

void PointTracker::keyboardHandler(unsigned char key, int x, int y) {
	if (key == 5) {// F5
		CriticalSectionWrapper csw(cs);
		
		pointShow = !pointShow;
		if (pointShow)
			pPointRenderer->create(pointRendererDesc);
	}
}

void PointTracker::setPointsDensity(Real pointsDensity) {
	CriticalSectionWrapper csw(cs);
	this->pointsDensity = pointsDensity;
}

void PointTracker::setPositionCov(const Mat33 &positionCov) {
	CriticalSectionWrapper csw(cs);
	this->positionCov = positionCov;
}

//------------------------------------------------------------------------------

VirtualPointTracker::VirtualPointTracker(Embodiment &embodiment) : PointTracker(embodiment) {
	actor = NULL;
	bInit = true;
}

VirtualPointTracker::~VirtualPointTracker() {
}

bool VirtualPointTracker::create(const VirtualPointTracker::Desc& desc) {
	PointTracker::create(desc); // throws

	actor = desc.actor;
	initPointFeatures();
	return true;
}

bool VirtualPointTracker::get(PointFeature::Seq &points, SecTmReal &timeStamp) {
	if (actor == NULL)
		return false;
	
	// object global pose
	const NxMat34 nxPose = actor->getNxActor()->getGlobalPose();
	Mat34 pose;
	nxPose.M.getRowMajor(&pose.R.m11);
	nxPose.t.get(&pose.p.v1);
	if (!bInit && this->pose.equals(pose, REAL_EPS))
		return false;
	this->pose = pose;
	
	// current time stamp
	timeStamp = context.getTimer().elapsed();

	// update points poses
	const U32 numOfPointFeatures = U32(pointsInit.size());
	points.resize(numOfPointFeatures);

	for (U32 i = 0; i < numOfPointFeatures; i++) {
		const PointFeature &f0 = pointsInit[i];
		PointFeature &f1 = points[i];
		
		pose.multiply(f1.position, f0.position);
	}

	bInit = false;

	return true;
}

void VirtualPointTracker::setActor(Actor *actor) {
	CriticalSectionWrapper csw(cs);
	this->actor = actor;
	initPointFeatures();
}

void VirtualPointTracker::setPointsDensity(Real pointsDensity) {
	CriticalSectionWrapper csw(cs);
	this->pointsDensity = pointsDensity;
	initPointFeatures();
}

void VirtualPointTracker::setPositionCov(const Mat33 &positionCov) {
	CriticalSectionWrapper csw(cs);
	this->positionCov = positionCov;
	initPointFeatures();
}

void VirtualPointTracker::initPointFeatures() {
	if (actor == NULL)
		return;

	Bounds::SeqPtr pBoundsSeq = actor->getGlobalBoundsSeq();

	Bounds::Ptr pBounds(BoundingBox::Desc().create());
	for (Bounds::Seq::const_iterator i = pBoundsSeq->begin(); i != pBoundsSeq->end(); i++)
		if (i == pBoundsSeq->begin()) {
			pBounds->set(**i);
			if ((**i).getType() == Bounds::TYPE_PLANE) {
				BoundingBox::Desc desc = planeBoundsDesc;
				desc.pose.multiply(desc.pose, pBounds->getPose());
				pBounds = desc.create();
			}
		}
		else {
			pBounds->combine(**i);
		}

	BoundingBox& boundingBox = static_cast<BoundingBox&>(*pBounds);
	
	Mat34 pose = actor->getPose();
	Mat34 invPose;
	invPose.setInverseRT(pose);
	boundingBox.multiplyPose(invPose, boundingBox.getPose());
	Bounds::multiplyPose(invPose, pBoundsSeq->begin(), pBoundsSeq->end());

	const Vec3 dimensions = boundingBox.getDimensions();
	const Real volume = dimensions.v1*dimensions.v2*dimensions.v3;
	const U32 numOfTrials = (U32)Math::round(getPointsDensity()*volume);

	// initialise points
	pointsInit.clear();
	for (U32 i = 0; i < numOfTrials; i++) {
		PointFeature feature;
		
		feature.position.set(
			rand.nextUniform(-dimensions.v1, +dimensions.v1),
			rand.nextUniform(-dimensions.v2, +dimensions.v2),
			rand.nextUniform(-dimensions.v3, +dimensions.v3)
		);
		boundingBox.getPose().multiply(feature.position, feature.position);
		if (!Bounds::intersect(pBoundsSeq->begin(), pBoundsSeq->end(), feature.position))
			continue;
			
		pointsInit.push_back(feature);
	}

	//context.getMessageStream()->write(Message::LEVEL_DEBUG,
	//	"{%f, %f, %f}, %d", dimensions.v1, dimensions.v2, dimensions.v3, numOfTrials
	//));
	//for (U32 j = 0; j < 8; j++) {
	//	PointFeature feature;
	//	feature.position = boundingBox.getEdges()[j];
	//	pointsInit.push_back(feature);
	//}
	
	this->pose = pose;
	bInit = true;
}

//------------------------------------------------------------------------------

VisionPointTracker::VisionPointTracker(Embodiment &embodiment) : PointTracker(embodiment) {
}

VisionPointTracker::~VisionPointTracker() {
}

bool VisionPointTracker::create(const VisionPointTracker::Desc& desc) {
	PointTracker::create(desc); // throws

	// TODO initialise vision device
	
	return true;
}

bool VisionPointTracker::get(PointFeature::Seq &points, SecTmReal &timeStamp) {
	// TODO read points poses
	return false;
	
	//return true;
}

//------------------------------------------------------------------------------
