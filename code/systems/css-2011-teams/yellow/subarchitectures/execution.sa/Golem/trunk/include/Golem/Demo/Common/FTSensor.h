/** @file FTSensor.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_FTSENSOR_H_
#define _GOLEM_DEMO_COMMON_FTSENSOR_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Wrench.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/DigitalFilter.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class FTSensor;

/** Force/torque sensor debug renderer */
class FTSensorRenderer : public DebugRenderer {
public:
	typedef shared_ptr<FTSensorRenderer> Ptr;

	FTSensor &ftSensor;

	/** Renderer description */
	class Desc {
	public:
		/** Force vector colour */
		RGBA forceColour;
		/** Force vector scale */
		Real forceScale;
		/** Display force vector */
		bool forceShow;
		/** Torque axes scale */
		Real torqueScale;
		/** Display torque axes */
		bool torqueShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			forceColour = RGBA::WHITE;
			forceScale = Real(1.0);
			forceShow = true;
			torqueScale = Real(1.0);
			torqueShow = true;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (forceScale < REAL_ZERO || torqueScale < REAL_ZERO)
				return false;
			
			return true;
		}
	};

	FTSensorRenderer(FTSensor &ftSensor);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** FTSensor
*/
class FTSensor : public Sensor<Wrench> {
public:
	typedef shared_ptr<FTSensor> Ptr;
	typedef Sensor<Wrench> Base;

	/** Object description */
	class Desc : public Base::Desc {
	public:
		/** Joint wrench gain coefficient */
		Wrench wrenchGain;
		
		/** Force/torque sensor renderer description */
		FTSensorRenderer::Desc ftSensorRendererDesc;
		/** Force/torque sensor renderer state */
		bool ftSensorShow;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();
			
			wrenchGain.set(Real(1.0), Real(1.0), Real(1.0), Real(1.0), Real(1.0), Real(1.0));
			ftSensorRendererDesc.setToDefault();
			ftSensorShow = true;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;
			
			if (!wrenchGain.isFinite())
				return false;
			if (!ftSensorRendererDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Joint wrench gain coefficient */
	Wrench wrenchGain;
	
	/** Surface point feature renderer */
	FTSensorRenderer::Ptr pFTSensorRenderer;
	FTSensorRenderer::Desc ftSensorRendererDesc;
	bool ftSensorShow;
	
	/** Current wrench */
	Wrench wrench;
	SecTmReal timeStamp;
	Event evWait;
	/** Critical section */
	CriticalSection cs;

	/** Reads the current sensory data */
	virtual bool get(Wrench &wrench, SecTmReal &timeStamp) = 0;
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const FTSensor::Desc& desc);
	
	/** Constructor */
	FTSensor(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~FTSensor();

	/** Reads the current sensory data */
	virtual bool read(Wrench &wrench, SecTmReal &timeStamp, MSecTmU32 timeOut = MSEC_TM_U32_INF);

	/** Joint wrench gain coefficient */
	const Wrench &getWrenchGain() const {
		return wrenchGain;
	}

	/** Joint wrench gain coefficient */
	void setWrenchGain(const Wrench &wrenchGain);
};

//------------------------------------------------------------------------------

/** VirtualFTSensor
*/
class VirtualFTSensor : public FTSensor {
	typedef shared_ptr<VirtualFTSensor> Ptr;

public:
	/** Object description */
	class Desc : public FTSensor::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(VirtualFTSensor, Channel::Ptr, Embodiment&)

	public:
		/** Wrench is measured at the specifid configuration space coordinates */
		NxJoint *joint;
		/** Fast filter description */
		golem::FIIRFilter::Desc fastFilterDesc;
		/** Slow filter description */
		golem::FIIRFilter::Desc slowFilterDesc;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			FTSensor::Desc::setToDefault();
			
			name = "VirtualFTSensor";

			joint = NULL;
			
			const U32 order = 4;
			const Real k = Real(0.0001);
			
			fastFilterDesc.feedforwardCoeffs.clear();
			fastFilterDesc.feedforwardCoeffs.resize(order, REAL_ONE/Real(order));
			
			slowFilterDesc.feedforwardCoeffs.clear();
			slowFilterDesc.feedforwardCoeffs.resize(order, k/Real(order));
			
			slowFilterDesc.feedbackCoeffs.clear();
			slowFilterDesc.feedbackCoeffs.resize(2);
			slowFilterDesc.feedbackCoeffs[0] = (REAL_ONE);
			slowFilterDesc.feedbackCoeffs[1] = -(REAL_ONE - k);
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!FTSensor::Desc::isValid())
				return false;

			//if (joint == NULL)
			//	return false;
			if (!fastFilterDesc.isValid() || !slowFilterDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** A joint used for wrench measurements */
	NxJoint *joint;
	/** Fast filters */
	golem::FIIRFilter fastFilter[6];
	/** Slow filters */
	golem::FIIRFilter slowFilter[6];

	/** Resets filters */
	void resetsFilters();
	
	/** Reads the current sensory data */
	virtual bool get(Wrench &wrench, SecTmReal &timeStamp);
	
	/** Creates object from description. */
	bool create(const VirtualFTSensor::Desc& desc);
	
	/** Constructor */
	VirtualFTSensor(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~VirtualFTSensor();

	/** Returns a joint used for wrench measurements */
	virtual const NxJoint *getJoint() const {
		return joint;
	}

	/** Sets a joint used for wrench measurements */
	virtual void setJoint(NxJoint *joint);
};

//------------------------------------------------------------------------------

/** VirtualFTSensor
*/
class Nano17FTSensor : public FTSensor {
	typedef shared_ptr<Nano17FTSensor> Ptr;

public:
	/** Object description */
	class Desc : public FTSensor::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Nano17FTSensor, Channel::Ptr, Embodiment&)

	public:
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			FTSensor::Desc::setToDefault();
			
			name = "Nano17FTSensor";
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!FTSensor::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Reads the current sensory data */
	virtual bool get(Wrench &wrench, SecTmReal &timeStamp);
	
	/** Creates object from description. */
	bool create(const Nano17FTSensor::Desc& desc);
	
	/** Constructor */
	Nano17FTSensor(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~Nano17FTSensor();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_FTSENSOR_H_*/
