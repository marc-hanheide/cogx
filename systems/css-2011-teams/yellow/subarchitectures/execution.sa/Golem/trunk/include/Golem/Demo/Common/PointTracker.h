/** @file PointTracker.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_POINTTRACKER_H_
#define _GOLEM_DEMO_COMMON_POINTTRACKER_H_

//------------------------------------------------------------------------------

#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Volumetric point feature */
struct PointFeature {
	typedef std::vector<PointFeature> Seq;

	/** position */
	Vec3 position;

	PointFeature() {
	}

	PointFeature(const Vec3 &position) : position(position) {
	}
};

//------------------------------------------------------------------------------

class PointTracker;

/** Renders point features */
class PointRenderer : public DebugRenderer {
public:
	typedef shared_ptr<PointRenderer> Ptr;

	/** Renderer description */
	class Desc {
	public:
		/** PointFeatures to render */
		const PointFeature::Seq *pPoints;

		/** Position point colour */
		RGBA positionColour;
		/** Display position point */
		bool positionShow;
		/** PointFeature show fraction */
		Real showFrac;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			pPoints = NULL;
			positionColour = RGBA::YELLOW;
			positionShow = true;
			showFrac = Real(1.0);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			//if (pPoints == NULL)
			//	return false;
			if (showFrac < REAL_ZERO || showFrac > REAL_ONE)
				return false;
			
			return true;
		}
	};

	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** PointTracker
*/
class PointTracker : public Sensor<PointFeature::Seq> {
public:
	typedef shared_ptr<PointTracker> Ptr;
	typedef Sensor<PointFeature::Seq> Base;

	/** Object description */
	class Desc : public Base::Desc {
	public:
		/** PointFeatures density */
		Real pointsDensity;
		/** PointFeature position noise covariance matrix */
		Mat33 positionCov;
		///** PointFeature orientation noise covariance matrix */
		//Mat33 orientationCov;
		/** Plane bounds */
		BoundingBox::Desc planeBoundsDesc;
		
		/** Point feature renderer description */
		PointRenderer::Desc pointRendererDesc;
		/** Point feature renderer state */
		bool pointShow;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();
			
			pointsDensity = Real(0.5e3*0.5e3*0.5e3);
			positionCov.setZero();
			//orientationCov.setZero();
			planeBoundsDesc.setToDefault();

			pointRendererDesc.setToDefault();
			pointShow = false;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;

			if (pointsDensity <= REAL_ZERO)
				return false;
			if (!positionCov.isFinite())
				return false;
			//if (!orientationCov.isFinite())
			//	return false;
			if (!planeBoundsDesc.isValid())
				return false;

			if (!pointRendererDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** PointFeatures density */
	Real pointsDensity;
	/** PointFeature position noise covariance matrix */
	Mat33 positionCov;
	///** PointFeature orientation noise covariance matrix */
	//Mat33 orientationCov;
	/** Plane bounds description */
	BoundingBox::Desc planeBoundsDesc;
	
	/** Point feature renderer */
	PointRenderer::Ptr pPointRenderer;
	PointRenderer::Desc pointRendererDesc;
	bool pointShow;
	Mat34 pose;

	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Current points set */
	PointFeature::Seq points;
	SecTmReal timeStamp;
	Event evWait;
	/** Critical section */
	CriticalSection cs;

	/** Reads the current sensory data */
	virtual bool get(PointFeature::Seq &points, SecTmReal &timeStamp) = 0;
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const PointTracker::Desc& desc);
	
	/** Constructor */
	PointTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~PointTracker();

	/** Reads the current sensory data */
	virtual bool read(PointFeature::Seq &points, SecTmReal &timeStamp, MSecTmU32 timeOut = MSEC_TM_U32_INF);
	
	/** PointFeatures density */
	virtual inline Real getPointsDensity() const {
		return pointsDensity;
	}
	
	/** PointFeatures density */
	virtual void setPointsDensity(Real pointsDensity);
	
	/** PointFeature position noise covariance matrix */
	virtual const Mat33 getPositionCov() const {
		return positionCov;
	}

	/** PointFeature position noise covariance matrix */
	virtual void setPositionCov(const Mat33 &positionCov);
};

//------------------------------------------------------------------------------

/** VirtualPointTracker
*/
class VirtualPointTracker : public PointTracker {
public:
	typedef shared_ptr<VirtualPointTracker> Ptr;

	/** Object description */
	class Desc : public PointTracker::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(VirtualPointTracker, Channel::Ptr, Embodiment&)

	public:
		/** Target Actor to be tracked */
		Actor *actor;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PointTracker::Desc::setToDefault();
			
			name = "VirtualPointTracker";
			actor = NULL;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PointTracker::Desc::isValid())
				return false;

			//if (actor == NULL)
			//	return false;
			
			return true;
		}
	};

protected:
	/** Target Actor to be tracked */
	Actor *actor;
	/** Initial points set */
	PointFeature::Seq pointsInit;
	/** Initialisation falg */
	bool bInit;
	
	/** Reads the current sensory data */
	virtual bool get(PointFeature::Seq &points, SecTmReal &timeStamp);
	
	/** Initialises points */
	void initPointFeatures();
	
	/** Creates object from description. */
	bool create(const VirtualPointTracker::Desc& desc);
	
	/** Constructor */
	VirtualPointTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~VirtualPointTracker();

	/** Target Actor to be tracked */
	virtual Actor *getActor() const {
		return actor;
	}

	/** Target Actor to be tracked */
	virtual void setActor(Actor *actor);

	/** PointFeatures density */
	virtual void setPointsDensity(Real pointsDensity);
	
	/** PointFeature position noise covariance matrix */
	virtual void setPositionCov(const Mat33 &positionCov);
};

//------------------------------------------------------------------------------

/** VirtualPointTracker
*/
class VisionPointTracker : public PointTracker {
	typedef shared_ptr<VisionPointTracker> Ptr;

public:
	/** Object description */
	class Desc : public PointTracker::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(VisionPointTracker, Channel::Ptr, Embodiment&)

	public:

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PointTracker::Desc::setToDefault();
			
			name = "VisionPointTracker";
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PointTracker::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Reads the current sensory data */
	virtual bool get(PointFeature::Seq &points, SecTmReal &timeStamp);
	
	/** Creates object from description. */
	bool create(const VisionPointTracker::Desc& desc);
	
	/** Constructor */
	VisionPointTracker(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~VisionPointTracker();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_POINTTRACKER_H_*/
