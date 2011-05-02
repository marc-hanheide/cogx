/** @file Predictor.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include "Prediction/Predictor.h"
#include <Golem/Tools/Msg.h>
#include <Golem/Demo/Common/FTSensor.h>
#include <Golem/Demo/Common/PointTracker.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

#ifdef _PREDICTOR_PERFMON
U32 Predictor::cycles;
SecTmReal Predictor::tmTotal;
SecTmReal Predictor::tmRender;
SecTmReal Predictor::tmMeasure;
SecTmReal Predictor::tmEvaluateFinger;
SecTmReal Predictor::tmEvaluatePolyflap;
SecTmReal Predictor::tmPredictFinger;
SecTmReal Predictor::tmPredictPolyflap;

void Predictor::resetLog() {
	cycles = 0;
	tmTotal = SEC_TM_REAL_ZERO;
	tmRender = SEC_TM_REAL_ZERO;
	tmMeasure = SEC_TM_REAL_ZERO;
	tmEvaluateFinger = SEC_TM_REAL_ZERO;
	tmEvaluatePolyflap = SEC_TM_REAL_ZERO;
	tmPredictFinger = SEC_TM_REAL_ZERO;
	tmPredictPolyflap = SEC_TM_REAL_ZERO;
}

void Predictor::postLog(Context &context, const char *str) {
	context.getMessageStream()->write(Message::LEVEL_DEBUG,
		"%s: {tot = %.4f, ren = %.4f, msr = %.4f, evf = %.4f, evp = %.4f, prf = %.4f, prp = %.4f}",
		str,
		tmTotal/Real(cycles),
		tmRender/Real(cycles),
		tmMeasure/Real(cycles),
		tmEvaluateFinger/Real(cycles),
		tmEvaluatePolyflap/Real(cycles),
		tmPredictFinger/Real(cycles),
		tmPredictPolyflap/Real(cycles)
	);
}
#endif

//------------------------------------------------------------------------------

bool golem::operator < (const golem::Hypothesis::Ptr &left, const golem::Hypothesis::Ptr &right) {
	return left->cdf < right->cdf;
}

//------------------------------------------------------------------------------

bool HypothesisRenderer::create(const Desc &desc) {
	if (!PointRenderer::create(desc))
		return false;

	// if (desc.pHypothesis != NULL)
	// reserve memory buffer
	//if (desc.positionShow)
	//	reservePoints(items);

	return true;
}

//------------------------------------------------------------------------------

Predictor::Predictor(golem::Scene &scene) :
	Object(scene),
	rand(context.getRandSeed())
{
	pFingerSensRenderer.reset(new HypothesisRenderer());
	pPolyflapSensRendererDesc.reset(new HypothesisRenderer());
	pFingerPredRenderer.reset(new HypothesisRenderer());
	pPolyflapPredRendererDesc.reset(new HypothesisRenderer());
}

bool Predictor::create(const Predictor::Desc& desc) {
	Object::create(desc); // throws

	filePath = desc.filePath;
	
	fingerSensRendererDesc = desc.fingerSensRendererDesc;
	fingerSensRendererDesc.pPoints = &fingerSens.points;
	fingerSensRendererDesc.pHypothesis = &fingerSens;
	polyflapSensRendererDesc = desc.polyflapSensRendererDesc;
	polyflapSensRendererDesc.pPoints = &polyflapSens.points;
	polyflapSensRendererDesc.pHypothesis = &polyflapSens;
	
	fingerPredRendererDesc = desc.fingerPredRendererDesc;
	fingerPredRendererDesc.pPoints = &fingerPred.points;
	fingerPredRendererDesc.pHypothesis = &fingerPred;
	polyflapPredRendererDesc = desc.polyflapPredRendererDesc;
	polyflapPredRendererDesc.pPoints = &polyflapPred.points;
	polyflapPredRendererDesc.pHypothesis = &polyflapPred;
	
	objectsSensShow = desc.objectsSensShow;
	
	tmRunTime = SecTmReal(0.0);
	pArm = NULL;
	pAction.reset();
	pFingerFTSensor = NULL;
	pFingerTracker = NULL;
	pPlaygroundTracker = NULL;
	pPolyflapTracker = NULL;

	step = -1;
	
	// Load from file
	try {
		FileReadStream fs(filePath.c_str());
		data.load(fs);
	}
	catch (const MsgStream&) {
		context.getMessageStream()->write(Message::LEVEL_ERROR,
			"Predictor::create(): unable to load data from file %s", filePath.c_str()
		);
		this->data = desc.data;
	}

	polyflapSeq[0].resize(data.numOfHypotheses);
	polyflapSeq[1].resize(data.numOfHypotheses);
	polyflapSeq[2].resize(data.numOfHypotheses);
	for (U32 h = 0; h < data.numOfHypotheses; h++) {
		polyflapSeq[0][h].reset(new DynHypothesis());
		polyflapSeq[0][h]->points.reserve(data.numOfFeatures);
		polyflapSeq[1][h].reset(new DynHypothesis());
		polyflapSeq[1][h]->points.reserve(data.numOfFeatures);
		polyflapSeq[2][h].reset(new DynHypothesis());
		polyflapSeq[2][h]->points.reserve(data.numOfFeatures);
	}

	pPolyflapSeq[0] = &polyflapSeq[0];
	pPolyflapSeq[1] = &polyflapSeq[1];
	pPolyflapSeq[2] = &polyflapSeq[2];

	fingerSeq[0].resize(1);
	fingerSeq[1].resize(1);
	fingerSeq[2].resize(1);
	for (U32 h = 0; h < 1; h++) {
		fingerSeq[0][h].reset(new KinHypothesis());
		fingerSeq[0][h]->points.reserve(data.numOfFeatures);
		fingerSeq[1][h].reset(new KinHypothesis());
		fingerSeq[1][h]->points.reserve(data.numOfFeatures);
		fingerSeq[2][h].reset(new KinHypothesis());
		fingerSeq[2][h]->points.reserve(data.numOfFeatures);
	}

	pFingerSeq[0] = &fingerSeq[0];
	pFingerSeq[1] = &fingerSeq[1];
	pFingerSeq[2] = &fingerSeq[2];

	polyflapTmp.points.reserve(data.numOfHypotheses);
	sampleTmp.reset(new Hypothesis());	
	
#ifdef _PREDICTOR_PERFMON
	resetLog();
#endif

	return true;
}

void Predictor::release() {
	// Save to file
	if (!filePath.empty()) {
		FileWriteStream stream(filePath.c_str());
		data.store(stream);
	}
}

void Predictor::postprocess(SecTmReal elapsedTime) {
	//CriticalSectionWrapper csw(cs);
}

void Predictor::render() {
	// postprocess() and render() are always called from the same thread 
	CriticalSectionWrapper csw(cs);
	
	if (objectsSensShow) {
		pFingerSensRenderer->render();
		pPolyflapSensRendererDesc->render();
	}
	if (objectsPredShow) {
		pFingerPredRenderer->render();
		pPolyflapPredRendererDesc->render();
	}
}

void Predictor::keyboardHandler(unsigned char key, int x, int y) {
	if (key == 6) {// F6
		objectsSensShow = !objectsSensShow;
		setupRenderData(objectsSensShow, false);
	}
	if (key == 7) {// F7
		objectsPredShow = !objectsPredShow;
		setupRenderData(false, objectsPredShow);
	}
}

void Predictor::setupRenderData(bool objectsSensShow, bool objectsPredShow) {
	CriticalSectionWrapper csw(cs);
	
	if (objectsSensShow) {
		pFingerSensRenderer->create(fingerSensRendererDesc);
		pPolyflapSensRendererDesc->create(polyflapSensRendererDesc);
	}
	if (objectsPredShow) {
		pFingerPredRenderer->create(fingerPredRendererDesc);		
		pPolyflapPredRendererDesc->create(polyflapPredRendererDesc);
	}
}

//------------------------------------------------------------------------------

void Predictor::nextOrientation(Vec3 &v, Real length) const {
	Real magnitude;

	do {
		v.v1 = rand.nextUniform(-REAL_ONE, +REAL_ONE);
		v.v2 = rand.nextUniform(-REAL_ONE, +REAL_ONE);
		v.v3 = rand.nextUniform(-REAL_ONE, +REAL_ONE);
		magnitude = v.magnitude();
	} while (magnitude > REAL_ONE - REAL_EPS || magnitude < REAL_EPS);
	
	length /= magnitude;
	v.v1 *= length;
	v.v2 *= length;
	v.v3 *= length;
}

void Predictor::generate(Mat34 &trn, const Generator &generator, Real dl, Real da, SecTmReal dt) const {
	const Real lin = dl*generator.dirLinVar;
	const Real ang = da*generator.dirAngVar;

	Vec3 ap, bp;

	trn.R.fromAngleAxis(ang, generator.axis);
	trn.R.multiply(ap, Vec3(generator.dir.v1*lin, generator.dir.v2*lin, generator.dir.v3*lin));
	trn.R.multiply(bp, Vec3(-generator.anchor.v1, -generator.anchor.v2, -generator.anchor.v3));
	trn.p.add(bp, ap);
	trn.p.add(trn.p, generator.anchor);

//if (debug1)
//context.getMessageStream()->write("lin = %f, ang = %f", lin, ang));
}

void Predictor::transform(Generator &generator, const Mat34 &trn) const {
	trn.multiply(generator.anchor, generator.anchor);
	trn.R.multiply(generator.axis, generator.axis);
	trn.R.multiply(generator.dir, generator.dir);
}

Hypothesis *Predictor::sample(Hypothesis::Seq &seq) const {
	ASSERT(!seq.empty())
	sampleTmp->cdf = rand.nextUniform<Real>()*seq.back()->cdf;
	Hypothesis::Seq::iterator ptr = std::lower_bound(seq.begin(), seq.end(), sampleTmp);
	return ptr == seq.end() ? (ptr - 1)->get() : (ptr)->get();
}

Generator *Predictor::sample(Generator::Seq &seq) const {
	ASSERT(!seq.empty())
	Generator gen;
	gen.cdf = rand.nextUniform<Real>()*seq.back().cdf;
	Generator::Seq::iterator ptr = std::lower_bound(seq.begin(), seq.end(), gen);
	return ptr == seq.end() ? &*(ptr - 1) : &*(ptr);
}

void Predictor::setup(Hypothesis *hypothesis, Hypothesis::Seq &seq, const Data &data) const {
	Hypothesis *previous = sample(seq);
	
	hypothesis->previous = previous;
	hypothesis->t = previous->t + data.timeDelta;
	hypothesis->weight = previous->weight;
	//hypothesis->cdf = previous->cdf;
	
	hypothesis->trn.setId();
	//hypothesis->points.resize(previous->points.size());
}

void Predictor::cost(DynHypothesis *polyflap, const KinHypothesis *finger, const Data &data, bool bDynamic) const {
	ASSERT(polyflap != NULL && finger != NULL)

	const BoundingPlane &playgroundBounds = dynamic_cast<const BoundingPlane&>(*pPlaygroundBounds->front());
	const BoundingSphere &fingerBounds = dynamic_cast<const BoundingSphere&>(*finger->bounds->front());
	
	// Contact distance
	const Real meanDist = Real(1.0)/Math::pow(this->pPlaygroundTracker->getPointsDensity(), Real(1.0/3.0));
	const Real maxDist = polyflap->contact*meanDist;
	// Gravity vector
	NxVec3 nxGravity(0.0f, 0.0f, -9.81f);
	//getNxScene()->getGravity(nxGravity);// ooops, AGEIA guys forgot to do something here!
	const Vec3 gravity((Real)nxGravity.x, (Real)nxGravity.y, (Real)nxGravity.z);
	// Number of points to be considered
	const U32 numOfFeatures = (U32)polyflap->points.size();
	
	// Cost variables
	U32 penetrationCount = 0, potentialCount = 0, frictionCount = 0, momentumCount = 0;
	
	polyflap->penetrationCost = REAL_ZERO;
	polyflap->potentialCost = REAL_ZERO;
	polyflap->frictionCost = REAL_ZERO;
	polyflap->momentumCost = REAL_ZERO;

	const Hypothesis *p1 = polyflap->previous;
	const Hypothesis *p2 = p1 != NULL ? p1->previous : NULL;

//U32 pos = 0, neg = 0;
//Real cpos = REAL_ZERO, cneg = REAL_ZERO;

	for (U32 i = 0; i < numOfFeatures; i++) {
		const PointFeature &f0 = polyflap->points[i];
	
		// penetration cost
		if (fingerBounds.getPosition().distance(f0.position) < fingerBounds.getRadius() - meanDist) {
			polyflap->penetrationCost += REAL_ONE;
			penetrationCount++;
		}
		if (playgroundBounds.getNormal().dot(f0.position) < playgroundBounds.getDistance() - meanDist) {
			polyflap->penetrationCost += REAL_ONE;
			penetrationCount++;
		}
		
		//if (p1 == NULL || !bDynamic || i%10 != 0)
		if (p1 == NULL || !bDynamic)
			continue;

		const PointFeature &f1 = p1->points[i];
		Vec3 f01;
		f01.subtract(f1.position, f0.position);

		// potential cost
		//Real diff = f01.dot(gravity);
		//polyflap->potentialCost += diff;
		//if (diff >= REAL_ZERO) {
		//	pos++;
		//	cpos += diff;
		//}
		//else {
		//	neg++;
		//	cneg += diff;
		//}

		polyflap->potentialCost += f01.dot(gravity);
		potentialCount++;

		// friction cost
		if (playgroundBounds.getNormal().dot(f0.position) < playgroundBounds.getDistance() + maxDist &&
			playgroundBounds.getNormal().dot(f1.position) < playgroundBounds.getDistance() + maxDist)
		{
			polyflap->frictionCost += f01.magnitude();
			frictionCount++;
		}

		if (p2 == NULL)
			continue;

		const PointFeature &f2 = p2->points[i];
		Vec3 f12;
		f12.subtract(f2.position, f1.position);

		// momentum cost
		Vec3 dist;
		dist.subtract(f01, f12);
		polyflap->momentumCost += dist.magnitude(); // local, for global one must sum up vectors!
		momentumCount++;
	}

	if (penetrationCount > 0)
		polyflap->penetrationCost *= polyflap->penetration/Real(numOfFeatures);
	if (potentialCount > 0)
		polyflap->potentialCost *= polyflap->potential/Real(potentialCount);
	if (frictionCount > 0)
		polyflap->frictionCost *= polyflap->friction/Real(frictionCount);
	if (momentumCount > 0)
		polyflap->momentumCost *= polyflap->momentum/Real(momentumCount);

//if (debug1)
//context.getMessageStream()->write("pos = %d, cpos = %f, neg = %d, cneg = %f", pos, cpos, neg, cneg));	
//context.getMessageStream()->write(
//	"penetrationCount = %d, potentialCount = %d, frictionCount = %d, momentumCount = %d",
//	penetrationCount, potentialCount, frictionCount, momentumCount
//));	
}

void Predictor::read(KinHypothesis &finger, DynHypothesis &polyflap, SecTmReal tmBegin, MSecTmU32 timeOut) {
	SecTmReal timeStamp;
	
	// Finger
	if (!pFingerTracker->read(finger.points, timeStamp, timeOut)) {
		//context.getMessageStream()->write(Message::LEVEL_WARNING,
		//	"Predictor::measure(): finger tracker read timeout"
		//));
	}
	finger.t = std::max(SEC_TM_REAL_ZERO, timeStamp - tmBegin);
	finger.bounds = dynamic_cast<const VirtualPointTracker*>(pFingerTracker)->getActor()->getGlobalBoundsSeq();
	
	{
		CriticalSectionWrapper csw(pFingerTracker->getScene().getUniverse().getCSPhysX()); // Access to PhysX
		const NxMat34 nxPose = dynamic_cast<const VirtualPointTracker*>(pFingerTracker)->getActor()->getNxActor()->getGlobalPose();
		nxPose.M.getRowMajor(&finger.pose.R.m11);
		nxPose.t.get(&finger.pose.p.v1);
	}

	// Polyflap
	if (!pPolyflapTracker->read(polyflap.points, timeStamp, timeOut)) {
		//context.getMessageStream()->write(Message::LEVEL_WARNING,
		//	"Predictor::measure(): polyflap tracker read timeout"
		//));
	}
	polyflap.t = std::max(SEC_TM_REAL_ZERO, timeStamp - tmBegin);
}

//------------------------------------------------------------------------------

void Predictor::predictFinger(KinHypothesis *finger, const Data &data) const {
	ASSERT(finger->previous != NULL)
	const KinHypothesis *previous = static_cast<const KinHypothesis*>(finger->previous);
	const U32 numOfFeatures = (U32)previous->points.size();
	
	finger->points.resize(numOfFeatures);
	finger->bounds = previous->bounds;

	//golem::WorkspaceState::Seq::const_iterator next = search(
	//	data.fingerTrajectory.begin(),
	//	data.fingerTrajectory.end(),
	//	golem::WorkspaceState(finger->t)
	//);

	//if (next != data.fingerTrajectory.end()) {
	//	golem::WorkspaceState::Seq::const_iterator prev = search(
	//		data.fingerTrajectory.begin(),
	//		data.fingerTrajectory.end(),
	//		golem::WorkspaceState(previous->t)
	//	);

	//	if (next != prev) {
	//		// trnNext = trnDelta*trnPrev => trnDelta = trnNext*trnPrev^-1

	//		Mat34 trnNext;
	//		trnNext.p = next->p;
	//		trnNext.R.fromQuat(next->q);
	//		
	//		Mat34 trnPrevInv;
	//		trnPrevInv.p = prev->p;
	//		trnPrevInv.R.fromQuat(prev->q);
	//		if (trnPrevInv.setInverse(trnPrevInv))
	//			finger->trn.multiply(trnNext, trnPrevInv);
	//	}
	//}

	// HACK: begin
	if (fingerSensSeq.size() > this->step + 1) {
		// trnNext = trnDelta*trnPrev => trnDelta = trnNext*trnPrev^-1
		Mat34 inv;
		inv.setInverseRT(fingerSensSeq[this->step].pose);
		finger->trn.multiply(fingerSensSeq[this->step + 1].pose, inv);
	}
	// HACK: end

	Bounds::multiplyPose(finger->trn, finger->bounds->begin(), finger->bounds->end());
	for (U32 i = 0; i < numOfFeatures; i++)
		finger->trn.multiply(finger->points[i].position, previous->points[i].position);
}

void Predictor::predictPolyflap(DynHypothesis *polyflap, const KinHypothesis *finger, const Data &data) const {
	ASSERT(polyflap->previous != NULL)
	const DynHypothesis *previous = static_cast<const DynHypothesis*>(polyflap->previous);
	const U32 numOfFeatures = (U32)previous->points.size();
	
	// setup
	polyflap->generator = previous->generator;
	polyflap->points = previous->points;
	polyflap->penetration = previous->penetration;//rand.nextGaussian(previous->penetration, data.penetrationVar);
	polyflap->potential = previous->potential;//rand.nextGaussian(previous->potential, data.potentialVar);
	polyflap->friction = previous->friction;//rand.nextGaussian(previous->friction, data.frictionVar);
	polyflap->momentum = previous->momentum;//rand.nextGaussian(previous->momentum, data.momentumVar);
	polyflap->contact = previous->contact;//rand.nextGaussian(previous->contact, data.contactVar);
	
	polyflap->cost = previous->cost;

	// Transformation magnitudes
	const Real linMagnitude = std::max(data.linMagnitude, std::max(polyflap->trn.p.magnitude(), finger->trn.p.magnitude()));
	Real angle0, angle1;
	Vec3 axis;
	polyflap->trn.R.toAngleAxis(angle0, axis);
	previous->trn.R.toAngleAxis(angle1, axis);
	const Real angMagnitude = std::max(data.angMagnitude, std::max(angle0, angle1));

	// calculate static cost
	cost(polyflap, finger, data, false);

if (polyflap->penetrationCost > REAL_ZERO && !debugCollision) {
	debugCollision = true;
	//context.getMessageStream()->write("COLLISION"));
}

//if (debug0)
//context.getMessageStream()->write(
//	"{%f, %f, %f}, {%f, %f, %f}, {%f, %f, %f}",
//	polyflap->generator.anchor.v1, polyflap->generator.anchor.v2, polyflap->generator.anchor.v3,
//	polyflap->generator.axis.v1, polyflap->generator.axis.v2, polyflap->generator.axis.v3,
//	polyflap->generator.dir.v1, polyflap->generator.dir.v2, polyflap->generator.dir.v3
//));
//generate(polyflap->trn, polyflap->generator, Real(0.05), Real(0.5), data.timeDelta);
//for (U32 i = 0; i < numOfFeatures; i++)
//	polyflap->trn.multiply(polyflap->points[i].position, previous->points[i].position);
////cost(polyflap, finger, data);
//transform(polyflap->generator, polyflap->trn);
//return;

	polyflapTmp = *polyflap;
	Real linMagnitudeAvr = REAL_ZERO;
	Real angMagnitudeAvr = REAL_ZERO;
	for (U32 step = 0; step < data.numOfSteps; step++) {	
debug1 = debug0 && step == 0;

		Mat34 trn;

		//// generate random transformation given magnitude of translation and rotation
		//// translation
		//trn.p.set(Real(0.0), rand.nextGaussian(Real(0.0), linMagnitude*data.linMagnitude), Real(0.0));
		////nextOrientation(trn.p, rand.nextGaussian(Real(0.0), linMagnitude*data.linMagnitude));		
		//// rotation
		//Vec3 axis;
		//nextOrientation(axis);
		//Real angle = REAL_ZERO;
		////Real angle = rand.nextGaussian(Real(0.0), angMagnitude*data.angMagnitude);		
		//trn.R.fromAngleAxis(angle, axis);
		
		Real linMagnitudeTmp = rand.nextGaussian(linMagnitudeAvr, linMagnitude);
		Real angMagnitudeTmp = rand.nextGaussian(angMagnitudeAvr, angMagnitude);
		generate(trn, polyflapTmp.generator, linMagnitudeTmp, angMagnitudeTmp, data.timeDelta);
		polyflapTmp.trn.multiply(trn, polyflap->trn);

		// setup points
		for (U32 i = 0; i < numOfFeatures; i++)
			polyflapTmp.trn.multiply(polyflapTmp.points[i].position, previous->points[i].position);

		cost(&polyflapTmp, finger, data);
		
		if (
			polyflapTmp.penetrationCost > polyflap->penetrationCost
			)
			continue;
		if (
			polyflapTmp.penetrationCost + polyflapTmp.potentialCost + polyflapTmp.frictionCost + polyflapTmp.momentumCost >=
			polyflap->penetrationCost + polyflap->potentialCost + polyflap->frictionCost + polyflap->momentumCost
			)
			continue;

		linMagnitudeAvr += (linMagnitudeTmp - linMagnitudeAvr)/Math::sqrt(Real(data.numOfSteps));
		angMagnitudeAvr += (angMagnitudeTmp - angMagnitudeAvr)/Math::sqrt(Real(data.numOfSteps));
		
//if (debug0)
//context.getMessageStream()->write(
//	"penetration %f, potential %f, friction %f, momentum %f",
//	polyflap->penetrationCost, polyflap->potentialCost, polyflap->frictionCost, polyflap->momentumCost
//));
		transform(polyflapTmp.generator, trn);
		*polyflap = polyflapTmp;
	}

	polyflap->cost += polyflap->penetrationCost + polyflap->potentialCost + polyflap->frictionCost + polyflap->momentumCost;

//if (debug0)
//context.getMessageStream()->write(
//	"linMag = %f, angMag = %f, polyflapMag = %f, fingerMag = %f",
//	linMagnitude, angMagnitude, polyflap->trn.p.magnitude(), finger->trn.p.magnitude()
//));
//context.getMessageStream()->write(
//	"linmag = %f, weight = %f: penetration %f, potential %f, friction %f, momentum %f",
//	linMagnitude, polyflap->weight, polyflap->penetrationCost, polyflap->potentialCost, polyflap->frictionCost, polyflap->momentumCost
//));
}

void Predictor::evaluateFinger(KinHypothesis *finger, const KinHypothesis &sens, const Data &data) const {
	//finger->t = sens.t;
	finger->pose = sens.pose;
	finger->bounds = sens.bounds;
	finger->points = sens.points;
	finger->weight = REAL_ONE;
}

void Predictor::evaluatePolyflap(DynHypothesis *polyflap, const DynHypothesis &sens, const Data &data) const {
	// TODO more complex case for variable number of new points
	if (polyflap->points.size() != sens.points.size()) {
		polyflap->weight = REAL_ZERO;
		context.getMessageStream()->write(Message::LEVEL_ERROR,
			"Predictor::evaluatePolyflap(): variable number of points is not supported"
		);
		return;
	}

	const U32 numOfFeatures = (U32)polyflap->points.size();
	
	// Distant variable
	Real dist = REAL_ZERO;

	for (U32 i = 0; i < numOfFeatures; i++) {
		PointFeature &fp = polyflap->points[i];
		const PointFeature &fs = sens.points[i];

		dist += fp.position.distance(fs.position);
	}

	//polyflap->t = sens.t;	
	polyflap->weight = REAL_ONE/(data.featureDistFac + dist/Real(numOfFeatures));
}

void Predictor::updateFinger(Hypothesis::Seq &seq, Data &data) const {
	KinHypothesis *finger = static_cast<KinHypothesis*>(seq[0].get());
	finger->cdf = REAL_ONE;
	
	//golem::WorkspaceState wNew;
	//wNew.p.set(finger->pose.p);
	//wNew.q.fromMat33(finger->pose.R);
	//wNew.t = finger->t;

	//golem::WorkspaceState::Seq::iterator i = search(
	//	data.fingerTrajectory.begin(),
	//	data.fingerTrajectory.end(),
	//	golem::WorkspaceState(finger->t)
	//);
	//
	//if (i == data.fingerTrajectory.end()) {
	//	data.fingerTrajectory.insert(i, wNew);
	//}
	//else if (i->t > finger->t + data.timeDelta/SecTmReal(2.0)) {
	//	data.fingerTrajectory.insert(i, wNew);
	//}
	//else if (i->t < finger->t - data.timeDelta/SecTmReal(2.0)) {
	//	data.fingerTrajectory.insert(++i, wNew);
	//}
	//else {
	//	golem::WorkspaceState wUpdate;
	//	
	//	wUpdate.p.subtract(wNew.p, i->p);
	//	wUpdate.p.multiplyAdd(data.fingerTrajectoryUpdateFac, wUpdate.p, i->p);		
	//	wUpdate.q.slerp(i->q, wNew.q, data.fingerTrajectoryUpdateFac);
	//	wUpdate.t = i->t + data.fingerTrajectoryUpdateFac*(wNew.t - i->t);

	//	*i = wUpdate;
	//}
}

void Predictor::updatePolyflap(Hypothesis::Seq &seq, Data &data) const {
	const U32 numOfHypotheses = (U32)seq.size();
	
	Real cdf = REAL_ZERO;
	for (U32 h = 0; h < numOfHypotheses; h++) {
		DynHypothesis *polyflap = static_cast<DynHypothesis*>(seq[h].get());
		
		// update CDF
		polyflap->cdf = (cdf += polyflap->weight);
	}

	for (U32 h = 0; h < numOfHypotheses; h++) {
		DynHypothesis *polyflap = static_cast<DynHypothesis*>(seq[h].get());

		// update coefficients
		//data.penetrationMean += data.costMeanUpdateFac*(polyflap->penetration - data.penetrationMean);
		data.potentialMean += data.costMeanUpdateFac*(polyflap->potential - data.potentialMean);
		data.frictionMean += data.costMeanUpdateFac*(polyflap->friction - data.frictionMean);
		data.momentumMean += data.costMeanUpdateFac*(polyflap->momentum - data.momentumMean);
		//data.contactMean += data.costMeanUpdateFac*(polyflap->contact - data.contactMean);
	}

	// update variances
	//data.penetrationVar *= data.costVarUpdateFac;
	data.potentialVar *= data.costVarUpdateFac;
	data.frictionVar *= data.costVarUpdateFac;
	data.momentumVar *= data.costVarUpdateFac;
	//data.contactVar *= data.costVarUpdateFac;
}

//------------------------------------------------------------------------------

bool Predictor::init(U32 mode) {
	if (pArm == NULL || pAction == NULL || pFingerFTSensor == NULL || pFingerTracker == NULL || pPlaygroundTracker == NULL || pPolyflapTracker == NULL)
		return false;

	const bool bOffline = (mode & Scenario::MODE_OFFLINE) > 0;	
	const bool bRecording = mode == Scenario::MODE_OFFLINE;
	const bool bPlayback = bOffline && !bRecording;
	
	// read sensory data
	if (!bPlayback)
		read(fingerSens, polyflapSens, SEC_TM_REAL_ZERO);
	setupRenderData(objectsSensShow, objectsPredShow);

	pPlaygroundBounds = dynamic_cast<const VirtualPointTracker*>(pPlaygroundTracker)->getActor()->getGlobalBoundsSeq();
	
	return true;
}

void Predictor::run(U32 mode) {
#ifdef _PREDICTOR_PERFMON
	PerfTimer &timer = *context.getTimer();
	SecTmReal tm;
	#define PERFMON_BEGIN(tm) tm = timer.elapsed();
	#define PERFMON_END(tm, tmVar) tmVar += timer.elapsed() - tm;
#else
	#define PERFMON_BEGIN(tm)
	#define PERFMON_END(tm, tmVar)
#endif

debugCollision = false;
	
	const SecTmReal tmBegin = context.getTimer().elapsed();
	const U32 steps = (U32)Math::ceil(tmRunTime/data.timeDelta);
	
	const bool bOffline = (mode & Scenario::MODE_OFFLINE) > 0;
	const bool bFiltering = (mode & Scenario::MODE_FILTERING) > 0;
	const bool bSimulation = (mode & Scenario::MODE_SIMULATION) > 0;	
	const bool bRecording = mode == Scenario::MODE_OFFLINE;
	const bool bPlayback = bOffline && !bRecording;
	
	if (bRecording) {
		fingerSensSeq.resize(steps);
		polyflapSensSeq.resize(steps);
	}
	
	this->step = -1;
	for (U32 s = 0; s < steps; s++) {
		const SecTmReal tmLoopBegin = context.getTimer().elapsed();		
		this->step = s;

		const bool bInit = s == 0;
		const bool bSens = bInit || !bSimulation;
		
		if (bSens && !bPlayback) {
			PERFMON_BEGIN(tm)
			// read sensory data
			read(fingerSens, polyflapSens, tmBegin, 0);
			PERFMON_END(tm, tmMeasure)
		}
		
		if (bRecording) {
			fingerSensSeq[s] = fingerSens;
			polyflapSensSeq[s] = polyflapSens;
		}
		
		if (bPlayback) {
			fingerSens = fingerSensSeq[s];
			polyflapSens = polyflapSensSeq[s];
		}
		
		if (bSens) {
			PERFMON_BEGIN(tm)
			setupRenderData(objectsSensShow, false);
			PERFMON_END(tm, tmRender)
		}
		
		if (!bRecording) {
			const U32 numOfGenerators = (U32)generators.size();
			const U32 numOfFingerHypotheses = (U32)(*pFingerSeq[0]).size();
			const U32 numOfPolyflapHypotheses = (U32)(*pPolyflapSeq[0]).size();
			
			if (bInit) {			
				Real cdf = REAL_ZERO;
				for (U32 g = 0; g < numOfGenerators; g++) {
					Generator &gen = generators[g];
					gen.cdf = (cdf += gen.weight);
				}

				for (U32 h = 0; h < numOfFingerHypotheses; h++) {
					KinHypothesis *finger = static_cast<KinHypothesis*>((*pFingerSeq[1])[h].get());
					
					finger->t = fingerSens.t;
					
					finger->previous = NULL;
					finger->weight = REAL_ONE/Real(numOfFingerHypotheses);
					finger->cdf = (REAL_ONE + h)/Real(numOfFingerHypotheses);
					finger->points = fingerSens.points;
					finger->trn.setId();
					
					finger->pose.setId();
					finger->bounds = fingerSens.bounds;
				}

				for (U32 h = 0; h < numOfPolyflapHypotheses; h++) {
					DynHypothesis *polyflap = static_cast<DynHypothesis*>((*pPolyflapSeq[1])[h].get());

					polyflap->t = polyflapSens.t;

					polyflap->previous = NULL;
					polyflap->weight = REAL_ONE/Real(numOfPolyflapHypotheses);
					polyflap->cdf = (REAL_ONE + h)/Real(numOfPolyflapHypotheses);
					polyflap->points = polyflapSens.points;
					polyflap->trn.setId();
					
					polyflap->generator = *sample(generators);

					polyflap->penetration = data.penetrationMean;
					polyflap->potential = data.potentialMean;
					polyflap->friction = data.frictionMean;
					polyflap->momentum = data.momentumMean;
					polyflap->contact = data.contactMean;

					polyflap->cost = REAL_ZERO;
				}
			}
			else {
				if (bFiltering) {
					PERFMON_BEGIN(tm)
					for (U32 h = 0; h < numOfFingerHypotheses; h++) {
						KinHypothesis *finger = static_cast<KinHypothesis*>((*pFingerSeq[0])[h].get());
						evaluateFinger(finger, fingerSens, data);
					}
					updateFinger((*pFingerSeq[0]), data);
					PERFMON_END(tm, tmEvaluateFinger)

					PERFMON_BEGIN(tm)
					for (U32 h = 0; h < numOfPolyflapHypotheses; h++) {
						DynHypothesis *polyflap = static_cast<DynHypothesis*>((*pPolyflapSeq[0])[h].get());
						evaluatePolyflap(polyflap, polyflapSens, data);
					}
					updatePolyflap((*pPolyflapSeq[0]), data);
					PERFMON_END(tm, tmEvaluatePolyflap)
				}
				else{
					//Real min = numeric_const<Real>::MAX, max = numeric_const<Real>::MIN, mean = REAL_ZERO;
					Real k = Real(1.0), T = Real(1.0);
					Real cdf = REAL_ZERO;
					for (U32 h = 0; h < numOfPolyflapHypotheses; h++) {
						DynHypothesis *polyflap = static_cast<DynHypothesis*>((*pPolyflapSeq[0])[h].get());
						
						//polyflap->weight = Math::exp(-polyflap->cost/(k*T));
						polyflap->weight = Math::exp(-(polyflap->penetrationCost + polyflap->potentialCost + polyflap->frictionCost + polyflap->momentumCost)/(k*T));
						//if (min > polyflap->weight)
						//	min = polyflap->weight;
						//if (max < polyflap->weight)
						//	max = polyflap->weight;
						//mean += polyflap->weight;
						
						// update CDF
						polyflap->cdf = (cdf += polyflap->weight);
					}
					//context.getMessageStream()->write(
					//	"min = %f, max = %f, mean = %f", min, max, mean/Real(numOfPolyflapHypotheses)
					//));
				}

				// rotate pointers
				Math::rotate(pFingerSeq[2], pFingerSeq[1], pFingerSeq[0]);
				Math::rotate(pPolyflapSeq[2], pPolyflapSeq[1], pPolyflapSeq[0]);
			}

			// Predict finger hypotheses
			PERFMON_BEGIN(tm)
			KinHypothesis *fingerPred = NULL;
			for (U32 h = 0; h < numOfFingerHypotheses; h++) {
				KinHypothesis *finger = static_cast<KinHypothesis*>((*pFingerSeq[0])[h].get());
				setup(finger, (*pFingerSeq[1]), data);
				predictFinger(finger, data);

				// in prediction mode modify weights here

				if (fingerPred == NULL || fingerPred->weight < finger->weight)
					fingerPred = finger;
			}
			PERFMON_END(tm, tmPredictFinger)

			// Predict sensor hypotheses
			PERFMON_BEGIN(tm)
//U32 ahi = 0, alo = 0;
			DynHypothesis *polyflapPred = NULL, *polyflapCost = NULL;
			for (U32 h = 0; h < numOfPolyflapHypotheses; h++) {
				KinHypothesis *finger = static_cast<KinHypothesis*>(sample((*pFingerSeq[0])));

debug0 = h == 0;
				DynHypothesis *polyflap = static_cast<DynHypothesis*>((*pPolyflapSeq[0])[h].get());
				setup(polyflap, (*pPolyflapSeq[1]), data);
				predictPolyflap(polyflap, finger, data);

				if (!debugCollision) polyflap->generator = *sample(generators);

				// in prediction mode modify weights here (according to computed cost)

				//polyflap->weight *= polyflap->cost;
				if (polyflapPred == NULL || polyflapPred->weight < polyflap->weight)
					polyflapPred = polyflap;
				if (polyflapCost == NULL || polyflapCost->cost > polyflap->cost)
					polyflapCost = polyflap;
//if (polyflap->generator.dirAngVar < 0.01) alo++; else ahi++;
			}
			PERFMON_END(tm, tmPredictPolyflap)

			{
				CriticalSectionWrapper csw(cs);
				
				this->fingerPred = *fingerPred;
				this->polyflapPred = *polyflapPred;
			}
			PERFMON_BEGIN(tm)
			setupRenderData(false, objectsPredShow);
			PERFMON_END(tm, tmRender)

			//context.getMessageStream()->write(Message::LEVEL_DEBUG,
			//	"a{%d, %d}, c{%.5f, %.5f}, w{%.5f, %.5f}, pe{%.5f, %.5f}, po{%.5f, %.5f}, fr{%.5f, %.5f}, mo{%.5f, %.5f}",
			//	alo, ahi,
			//	polyflapPred->cost, polyflapCost->cost,
			//	polyflapPred->weight, polyflapCost->weight,
			//	polyflapPred->penetrationCost, polyflapCost->penetrationCost,
			//	polyflapPred->potentialCost, polyflapCost->potentialCost,
			//	polyflapPred->frictionCost, polyflapCost->frictionCost,
			//	polyflapPred->momentumCost, polyflapCost->momentumCost
			//));
		}

		const SecTmReal tmLoopEnd = context.getTimer().elapsed();
		const SecTmReal tmWait = tmLoopBegin + data.timeDelta - tmLoopEnd;

		if (tmWait > SEC_TM_REAL_ZERO)
			PerfTimer::sleep(tmWait);
		else if (!bOffline || bRecording) {
			context.getMessageStream()->write(Message::LEVEL_WARNING,
				"Predictor::run(): delta time sync skew %f [sec]", -tmWait
			);
		}

PerfTimer::sleep(SecTmReal(0.01));

#ifdef _PREDICTOR_PERFMON
		cycles++;
		tmTotal += tmLoopEnd - tmLoopBegin;
#endif
	}

#ifdef _PREDICTOR_PERFMON
	postLog(context, "Predictor::run(): ");
#endif

#undef PERFMON_BEGIN
#undef PERFMON_END
}

void Predictor::cleanup() {
	{
		CriticalSectionWrapper csw(cs);
		
		fingerSens.points.clear();
		polyflapSens.points.clear();
		fingerPred.points.clear();
		polyflapPred.points.clear();
	}

	setupRenderData(objectsSensShow, objectsPredShow);
}

//------------------------------------------------------------------------------
