/** @file Predictor.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_PREDICTOR_H_
#define _GOLEM_DEMO_PREDICTOR_H_

//------------------------------------------------------------------------------

#include "Scenario.h"
#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/Finger.h>
#include <Golem/Demo/Common/FTSensor.h>
#include <Golem/Demo/Common/PointTracker.h>
#include <algorithm>

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _PREDICTOR_PERFMON

//------------------------------------------------------------------------------

/** Hypothesis of an object state */
class Hypothesis {
public:
	/** Hypothesis pointer */
	typedef shared_ptr<Hypothesis> Ptr;
	/** Hypothesis array */
	typedef std::vector<Hypothesis::Ptr> Seq;

	/** time stamp */
	SecTmReal t;

	/** Previous hypothesis */
	Hypothesis *previous;
	/** Sample weight */
	Real weight;
	/** Cumulative distribution function value */
	Real cdf;

	/** PointFeature representation of the body */
	PointFeature::Seq points;
	/** Relative pose transformation */
	Mat34 trn;

	Hypothesis() {
		previous = NULL;
	}

	/**	Assignment operator. */
	inline const Hypothesis& operator = (const Hypothesis &h) {
		t = h.t;
		
		previous = h.previous;		
		weight = h.weight;
		cdf = h.cdf;
		
		points = h.points;
		trn = h.trn;

		return *this;
	}
	
	/** Sample CDF comparator */
	friend bool operator < (const golem::Hypothesis::Ptr &left, const golem::Hypothesis::Ptr &right);
};


/** Sample CDF comparator */
bool operator < (const golem::Hypothesis::Ptr &left, const golem::Hypothesis::Ptr &right);


/** Object kinematic-state hypothesis */
class KinHypothesis : public Hypothesis {
public:
	/** Finger pose */
	Mat34 pose;
	/** Bounds representation of the finger */
	golem::Bounds::SeqPtr bounds;
	
	/**	Assignment operator. */
	inline const KinHypothesis& operator = (const KinHypothesis &h) {
		Hypothesis::operator = (h);
		
		pose = h.pose;
		bounds = h.bounds != NULL ? Bounds::clone(h.bounds->begin(), h.bounds->end()) : h.bounds;

		return *this;
	}
};


/** Object dynamic-state hypothesis */
class DynHypothesis : public Hypothesis {
public:
	/** Generator */
	Generator generator;

	/** Penetration cost constant */
	Real penetration;
	/** Potential cost constant */
	Real potential;
	/** Friction cost constant */
	Real friction;
	/** Momentum cost constant */
	Real momentum;
	/** Contact cost constant */
	Real contact;
	
	/** Penetration energy */
	Real penetrationCost;
	/** Potential energy */
	Real potentialCost;
	/** Friction */
	Real frictionCost;
	/** Momentum */
	Real momentumCost;
	/** Contact */
	Real contactCost;
	
	/** Total cost */
	Real cost;
	
	/**	Assignment operator. */
	inline const DynHypothesis& operator = (const DynHypothesis &h) {
		Hypothesis::operator = (h);
		
		generator = h.generator;
		
		penetration = h.penetration;
		potential = h.potential;
		friction = h.friction;
		momentum = h.momentum;
		contact = h.contact;
		
		penetrationCost = h.penetrationCost;
		potentialCost = h.potentialCost;
		frictionCost = h.frictionCost;
		momentumCost = h.momentumCost;
		contactCost = h.contactCost;
		
		cost = h.cost;

		return *this;
	}
};

//------------------------------------------------------------------------------

/** Renders object data */
class HypothesisRenderer : public golem::PointRenderer {
public:
	typedef shared_ptr<HypothesisRenderer> Ptr;

	/** Renderer description */
	class Desc : public golem::PointRenderer::Desc {
	public:
		/** */
		const Hypothesis *pHypothesis;
		
		/** Position point colour */
		//Vec3 positionColour;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			PointRenderer::Desc::setToDefault();

			pHypothesis = NULL;
			showFrac = Real(0.1);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!PointRenderer::Desc::isValid())
				return false;
			//if (pHypothesis == NULL)
			//	return false;
			
			return true;
		}
	};

	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

class Predictor : public golem::Object {
public:
	typedef shared_ptr<Predictor> Ptr;

	/** Predictor data */
	class Data : public Serializable {
	public:
		/** Simulation time delta step */
		SecTmReal timeDelta;
		
		/** Number of hypotheses */
		U32 numOfHypotheses;
		/** Number of points */
		U32 numOfFeatures;
		/** Number of steps */
		U32 numOfSteps;

		/** Finger trajectory */
		//golem::WorkspaceState::Seq fingerTrajectory;
		
		/** Finger trajectory update */
		//Real fingerTrajectoryUpdateFac;
		
		/** Linear transformation magnitude */
		Real linMagnitude;
		/** Angular transformation magnitude */
		Real angMagnitude;
		
		/** Penetration cost constant mean and variance */
		Real penetrationMean;
		Real penetrationVar;
		/** Potential cost constant mean and variance */
		Real potentialMean;
		Real potentialVar;
		/** Friction cost constant mean and variance */
		Real frictionMean;
		Real frictionVar;
		/** Momentum cost constant mean and variance */
		Real momentumMean;
		Real momentumVar;
		/** Contact cost constant mean and variance */
		Real contactMean;
		Real contactVar;
		
		/** Meanicients update factor specifies learning rate */
		Real costMeanUpdateFac;
		/** Variations update factor  */
		Real costVarUpdateFac;
		
		/** PointFeature distance factor  */
		Real featureDistFac;
		
		/** Constructs Collision description object */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			timeDelta = SecTmReal(0.1);
			numOfHypotheses = 1000;
			numOfFeatures = 5000;
			numOfSteps = 10;
			
			//fingerTrajectory.clear();
			//fingerTrajectoryUpdateFac = Real(0.1);
			
			linMagnitude = Real(0.005);
			angMagnitude = Real(0.1);

			penetrationMean = Real(1000.0);
			penetrationVar = Real(1.0);
			potentialMean = Real(10.0);
			potentialVar = Real(0.1);
			frictionMean = Real(50.0);
			frictionVar = Real(0.1);
			momentumMean = Real(1.0);
			momentumVar = Real(0.1);
			contactMean = Real(2.0);
			contactVar = Real(0.2);
			costMeanUpdateFac = Real(0.01);
			costVarUpdateFac = Real(0.999);

			featureDistFac = Real(0.005);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (timeDelta <= REAL_ZERO)
				return false;
			if (numOfHypotheses <= 0)
				return false;
			if (numOfFeatures <= 0)
				return false;
			if (numOfSteps <= 0)
				return false;

			//if (fingerTrajectoryUpdateFac <= REAL_ZERO)
			//	return false;
			
			if (linMagnitude <= REAL_ZERO || angMagnitude <= REAL_ZERO)
				return false;
			
			if (penetrationMean <= REAL_ZERO || penetrationVar <= REAL_ZERO)
				return false;
			if (potentialMean <= REAL_ZERO || potentialVar <= REAL_ZERO)
				return false;
			if (frictionMean <= REAL_ZERO || frictionVar <= REAL_ZERO)
				return false;
			if (momentumMean <= REAL_ZERO || momentumVar <= REAL_ZERO)
				return false;
			if (contactMean <= REAL_ZERO || contactVar <= REAL_ZERO)
				return false;
			if (costMeanUpdateFac <= REAL_ZERO || costVarUpdateFac <= REAL_ZERO)
				return false;
			
			if (featureDistFac <= REAL_ZERO)
				return false;
			
			return true;
		}

		/** Loads data from the specified stream */
		virtual void load(const Stream &stream) {
			stream >> timeDelta >> numOfHypotheses >> numOfFeatures >> numOfSteps;

			//U32 points;
			//stream.read(points);
			//if (points > 0) {
			//	fingerTrajectory.resize(points);
			//	stream.read(&fingerTrajectory.front(), points);
			//}
			//stream.read(fingerTrajectoryUpdateFac);

			stream >> linMagnitude >> angMagnitude >> penetrationMean >> penetrationVar >> potentialMean >> potentialVar
				>> frictionMean >> frictionVar >> momentumMean >> momentumVar >> contactMean >> contactVar
				>> costMeanUpdateFac >> costVarUpdateFac >> featureDistFac;
		}
		
		/** Stores data to the specified stream */
		virtual void store(Stream &stream) const {
			stream << timeDelta << numOfHypotheses << numOfFeatures << numOfSteps;

			//U32 points = (U32)fingerTrajectory.size();
			//stream.write(points);
			//if (points > 0) {
			//	stream.write(&fingerTrajectory.front(), points);
			//}
			//stream.write(fingerTrajectoryUpdateFac);
		
			stream << linMagnitude << angMagnitude << penetrationMean << penetrationVar << potentialMean << potentialVar
				<< frictionMean << frictionVar << momentumMean << momentumVar << contactMean << contactVar
				<< costMeanUpdateFac << costVarUpdateFac << featureDistFac;
		}
	};

	/** Object description */
	class Desc : public golem::Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Predictor, Object::Ptr, Scene&)

	public:
		/** File path */
		std::string filePath;
		
		/** Objects sensory state renderers descriptions */
		HypothesisRenderer::Desc fingerSensRendererDesc, polyflapSensRendererDesc;
		/** Objects sensory state renderer state */
		bool objectsSensShow;
		/** Objects predicted state renderers descriptions */
		HypothesisRenderer::Desc fingerPredRendererDesc, polyflapPredRendererDesc;
		/** Objects predicted state renderer state */
		bool objectsPredShow;
		
		/** Predictor data */
		Data data;

		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Object::Desc::setToDefault();

			filePath = "predictor.dat";

			fingerSensRendererDesc.setToDefault();
			fingerSensRendererDesc.positionColour = RGBA::YELLOW;
			polyflapSensRendererDesc.setToDefault();
			polyflapSensRendererDesc.positionColour = RGBA::YELLOW;
			objectsSensShow = true;
			fingerPredRendererDesc.setToDefault();
			fingerPredRendererDesc.positionColour = RGBA::WHITE;
			polyflapPredRendererDesc.setToDefault();
			polyflapPredRendererDesc.positionColour = RGBA::WHITE;
			objectsPredShow = true;

			data.setToDefault();
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			
			if (!fingerSensRendererDesc.isValid() || !polyflapSensRendererDesc.isValid())
				return false;
			if (!fingerPredRendererDesc.isValid() || !polyflapPredRendererDesc.isValid())
				return false;
			
			if (!data.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** File path */
	std::string filePath;

	/** Objects sensory state renderers */
	HypothesisRenderer::Ptr pFingerSensRenderer, pPolyflapSensRendererDesc;
	/** Objects sensory state renderers descriptions */
	HypothesisRenderer::Desc fingerSensRendererDesc, polyflapSensRendererDesc;
	/** Objects sensory state renderer state */
	bool objectsSensShow;
	/** Objects predicted state renderers */
	HypothesisRenderer::Ptr pFingerPredRenderer, pPolyflapPredRendererDesc;
	/** Objects predicted state renderers descriptions */
	HypothesisRenderer::Desc fingerPredRendererDesc, polyflapPredRendererDesc;
	/** Objects predicted state renderer state */
	bool objectsPredShow;
	
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Critical section */
	CriticalSection cs;

	/** Running time */
	SecTmReal tmRunTime;
	/** Arm */
	golem::Arm *pArm;
	/** Action */
	golem::shared_ptr<golem::GenWorkspaceState::Seq> pAction;
	/** Generators */
	Generator::Seq generators;
	/** Finger FT sensor */
	golem::FTSensor *pFingerFTSensor;
	/** Finger tracker */
	golem::PointTracker *pFingerTracker;
	/** Playground tracker */
	golem::PointTracker *pPlaygroundTracker;
	Bounds::SeqPtr pPlaygroundBounds;
	/** Polyflap tracker */
	golem::PointTracker *pPolyflapTracker;
	
	/** Predictor data */
	Data data;

	/** */
	KinHypothesis fingerSens, fingerPred;
	Hypothesis::Seq fingerSeq[3];
	Hypothesis::Seq *pFingerSeq[3];
	std::vector<KinHypothesis> fingerSensSeq;
	/** */
	DynHypothesis polyflapSens, polyflapPred;
	Hypothesis::Seq polyflapSeq[3];
	Hypothesis::Seq *pPolyflapSeq[3];
	std::vector<DynHypothesis> polyflapSensSeq;

	U32 step; // HACK

	mutable DynHypothesis polyflapTmp;
	mutable Hypothesis::Ptr sampleTmp;

	mutable bool debug0, debug1, debugCollision;

	/** Binary search */
	template <class I, class T> I search(I begin, I end, const T &val) const {
		if (begin == end)
			return begin;

		I upper = std::lower_bound(begin, end, val);
		if (upper == begin)
			return begin;
		if (upper == end)
			return --end;
		I lower = upper - 1;

		return Math::abs(lower->t - val.t) < Math::abs(upper->t - val.t) ? lower : upper;
	}

	/** Setups render data */
	void setupRenderData(bool objectsSensShow, bool objectsPredShow);
	
	/** Generates unit vector with random oriantation */
	void nextOrientation(Vec3 &v, Real length = REAL_ONE) const;
	
	/** Generates transformation from generator */
	void generate(Mat34 &trn, const Generator &generator, Real dl, Real da, SecTmReal dt) const;
	
	/** Transforms generator */
	void transform(Generator &generator, const Mat34 &trn) const;
	
	/** Samples hypothesis */
	Hypothesis *sample(Hypothesis::Seq &seq) const;
	
	/** Samples generator */
	Generator *sample(Generator::Seq &seq) const;
	
	/** Samples and sets up a new hypothesis */
	void setup(Hypothesis *hypothesis, Hypothesis::Seq &seq, const Data &data) const;
	
	/** Computes cost of the polyflap configuration */
	void cost(DynHypothesis *polyflap, const KinHypothesis *finger, const Data &data, bool bDynamic = true) const;
	
	/** Synchronises hypotheses with sensors */
	void read(KinHypothesis &finger, DynHypothesis &polyflap, SecTmReal tmBegin, MSecTmU32 timeOut = MSEC_TM_U32_INF);

	/** Predict the next finger hypothesis (motion model) */
	void predictFinger(KinHypothesis *finger, const Data &data) const;
	
	/** Predict the next polyflap hypothesis (motion model) */
	void predictPolyflap(DynHypothesis *polyflap, const KinHypothesis *finger, const Data &data) const;
	
	/** Evaluates hypothesis (sensor model) */
	void evaluateFinger(KinHypothesis *finger, const KinHypothesis &sens, const Data &data) const;
	
	/** Evaluates hypothesis (sensor model) */
	void evaluatePolyflap(DynHypothesis *polyflap, const DynHypothesis &sens, const Data &data) const;
	
	/** Update prediction parameters */
	void updateFinger(Hypothesis::Seq &seq, Data &data) const;

	/** Update prediction parameters */
	void updatePolyflap(Hypothesis::Seq &seq, Data &data) const;


	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates Predictor from description. */
	bool create(const Predictor::Desc& desc);
	
	/** Releases resources */
	virtual void release();
	
	/** Objects can be constructed only in the Scene context. */
	Predictor(golem::Scene &scene);

public:
public:
#ifdef _PREDICTOR_PERFMON
	static U32 cycles;
	static SecTmReal tmTotal;
	static SecTmReal tmRender;
	static SecTmReal tmMeasure;
	static SecTmReal tmEvaluateFinger;
	static SecTmReal tmEvaluatePolyflap;
	static SecTmReal tmPredictFinger;
	static SecTmReal tmPredictPolyflap;

	static void resetLog();
	static void postLog(Context &context, const char *str);
#endif
	
	/** Running time */
	void setRunTime(SecTmReal tmRunTime) {
		this->tmRunTime = tmRunTime;
	}

	/** Arm */
	void setArm(golem::Arm *pArm) {
		this->pArm = pArm;
	}

	/** Action */
	void setAction(const golem::shared_ptr<golem::GenWorkspaceState::Seq> &pAction) {
		this->pAction = pAction;
	}

	/** Generators */
	void setGenerators(const Generator::Seq &generators) {
		this->generators = generators;
	}

	/** Finger FT sensor */
	void setFingerFTSensor(golem::FTSensor *pFingerFTSensor) {
		this->pFingerFTSensor = pFingerFTSensor;
	}

	/** Finger tracker */
	void setFingerTracker(golem::PointTracker *pFingerTracker) {
		this->pFingerTracker = pFingerTracker;
	}

	/** Playground tracker */
	void setPlaygroundTracker(golem::PointTracker *pPlaygroundTracker) {
		this->pPlaygroundTracker = pPlaygroundTracker;
	}
	
	/** Polyflap tracker */
	void setPolyflapTracker(golem::PointTracker *pPolyflapTracker) {
		this->pPolyflapTracker = pPolyflapTracker;
	}
	
	/** Init prediction session */
	virtual bool init(U32 mode);
	
	/** Run prediction session */
	virtual void run(U32 mode);
	
	/** Cleanup prediction session */
	virtual void cleanup();
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DEMO_PREDICTOR_H_*/
