/** @file Finger.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_FINGER_H_
#define _GOLEM_DEMO_COMMON_FINGER_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Wrench.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Ctrl/ReacPlanner.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Class realising reactive arm movement control. */
class WrenchTransmitter : public Transmitter {
public:
	typedef shared_ptr<WrenchTransmitter> Ptr;
	friend class Desc;

	/** WrenchTransmitter description */
	class Desc : public Transmitter::Desc {
	private:
		MemoryWriteStream buffer;
	
	public:
		/** Universe pointer */
		golem::Universe *pUniverse;

		/** Scene description of a virtual actuator. */
		NxSceneDesc nxSceneDesc;
		/** Arm controller actor description. */
		NxActorDesc nxActorDesc;
		/** Arm controller joint description. */
		NxD6JointDesc nxJointDesc;
		
		/** Arm reference pose bounds description */
		BoundingBox::Desc referencePoseBoundsDesc;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Creates Transmitter given the description object. 
		* @return		pointer to the Transmitter, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Transmitter::Ptr create(golem::Arm &arm) const {
			WrenchTransmitter *pWrenchTransmitter = new WrenchTransmitter(arm);
			Transmitter::Ptr pTransmitter(pWrenchTransmitter);

			if (!pWrenchTransmitter->create(*this))
				pTransmitter.release();

			return pTransmitter;
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Transmitter::Desc::setToDefault();
			
			pUniverse = NULL;

			buffer.resetWrite();	// reset buffer

			// Scene description of a virtual actuator.
			nxSceneDesc.setToDefault();
			nxSceneDesc.gravity.set(0.0f, 0.0f, -9.81f);

			// Arm controller actor description
			nxActorDesc.setToDefault();
			
			buffer.write(NxBodyDesc());
			NxBodyDesc* pNxBodyDesc = (NxBodyDesc*)buffer.get().back();
			pNxBodyDesc->angularDamping = (NxReal)0.9;
			pNxBodyDesc->linearDamping = (NxReal)0.9;
			nxActorDesc.body = pNxBodyDesc;
			nxActorDesc.density = (NxReal)1.0;
			
			buffer.write(NxSphereShapeDesc());
			NxSphereShapeDesc &nxShapeDesc = *(NxSphereShapeDesc*)buffer.get().back();
			nxShapeDesc.radius = (NxReal)1.0;
			nxActorDesc.shapes.push_back(&nxShapeDesc);
			
			// Arm controller joint description
			const Real actuatorValue = Real(1.0);
			const Real actuatorSpring = Real(0.0);
			const Real actuatorDamping = Real(0.0);
			const Real actuatorRestitution = Real(0.0);

			nxJointDesc.setToDefault();
			
			nxJointDesc.xMotion = NX_D6JOINT_MOTION_LIMITED;
			nxJointDesc.yMotion = NX_D6JOINT_MOTION_LIMITED;//NX_D6JOINT_MOTION_LOCKED
			nxJointDesc.zMotion = NX_D6JOINT_MOTION_LIMITED;//NX_D6JOINT_MOTION_LOCKED
			nxJointDesc.linearLimit.value = (NxReal)actuatorValue;
			nxJointDesc.linearLimit.spring = (NxReal)actuatorSpring;
			nxJointDesc.linearLimit.damping = (NxReal)actuatorDamping;
			nxJointDesc.linearLimit.restitution = (NxReal)actuatorRestitution;

			nxJointDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
			nxJointDesc.swing1Limit.value = (NxReal)actuatorValue;
			nxJointDesc.swing1Limit.spring = (NxReal)actuatorSpring;
			nxJointDesc.swing1Limit.damping = (NxReal)actuatorDamping;
			nxJointDesc.swing1Limit.restitution = (NxReal)actuatorRestitution;
			
			nxJointDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
			nxJointDesc.swing2Limit.value = (NxReal)actuatorValue;
			nxJointDesc.swing2Limit.spring = (NxReal)actuatorSpring;
			nxJointDesc.swing2Limit.damping = (NxReal)actuatorDamping;
			nxJointDesc.swing2Limit.restitution = (NxReal)actuatorRestitution;
			
			nxJointDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;
			nxJointDesc.twistLimit.low.value =  (NxReal)-actuatorValue;
			nxJointDesc.twistLimit.low.spring = (NxReal)actuatorSpring;
			nxJointDesc.twistLimit.low.damping = (NxReal)actuatorDamping;
			nxJointDesc.twistLimit.low.restitution = (NxReal)actuatorRestitution;
			nxJointDesc.twistLimit.high.value = (NxReal)actuatorValue;
			nxJointDesc.twistLimit.high.spring = (NxReal)actuatorSpring;
			nxJointDesc.twistLimit.high.damping = (NxReal)actuatorDamping;
			nxJointDesc.twistLimit.high.restitution = (NxReal)actuatorRestitution;
		
			referencePoseBoundsDesc.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Transmitter::Desc::isValid())
				return false;

			if (pUniverse == NULL)
				return false;
			if (!nxSceneDesc.isValid() || !nxActorDesc.isValid()/* || !nxJointDesc.isValid()*/)
				return false;
			
			if (!referencePoseBoundsDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Universe */
	golem::Universe *pUniverse;
	/** Novodex Scene. */
	NxScene* pNxScene;
	/** Novodex Actor. */
	NxActor* pNxActor;
	/** Arm reference pose bounds */
	Bounds::Ptr referencePoseBounds;

	/** Creates WrenchTransmitter from the description. 
	* @param desc		WrenchTransmitter description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** WrenchTransmitter constructor */
	WrenchTransmitter(golem::Arm &arm);

public:
	/** Sends control signal vector in user-defined format. */
	virtual bool set(const void *data);

	/** Receives the arm control signal. */
	virtual bool get(Signal &signal) const;
	
	/** Arm reference pose bounds */
	virtual const BoundingBox* getReferencePoseBounds() const {
		return static_cast<const BoundingBox*>(referencePoseBounds.get());
	}
};

//------------------------------------------------------------------------------

/** Renders finger's actuator data */
class FingerRenderer : public DebugRenderer {
protected:
	/** Tracker */
	const golem::ReacPlanner &reacPlanner;

public:
	typedef shared_ptr<FingerRenderer> Ptr;

	/** Renderer description */
	class Desc {
	public:
		/** Reference pose bounds colour */
		RGBA boundsColour;
		/** Display reference pose bounds */
		bool boundsShow;
		/** Init pose axes size */
		Vec3 initPoseSize;
		/** Display init pose axes */
		bool initPoseShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			boundsColour = RGBA::GREEN;
			boundsShow = true;
			initPoseSize.set(Real(0.1));
			initPoseShow = false;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!initPoseSize.isFinite())
				return false;
			
			return true;
		}
	};

	FingerRenderer(const golem::ReacPlanner &reacPlanner);
	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Finger
*/
class Finger : public golem::Effector<golem::GenConfigspaceState> {
public:
	typedef Effector<golem::GenConfigspaceState> Base;

	/** Object description */
	class Desc : public Base::Desc {
	protected:
		MemoryWriteStream buffer;

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Finger, Channel::Ptr, Embodiment&)

	public:
		/** Finger controller description */
		PhysReacPlanner::Desc fingerCtrlDesc;
		/** Finger actor description.
			Finger body is assumed to be "anchored" at the origin of the controller tool frame along Y axis.
			Use Finger::Desc::setFingerPose() to change the default orientation.
		*/
		Actor::Desc fingerActorDesc;
		/** Finger joint description.
			Joint axis is assumed to be oriented along Y axis of the controller tool frame.
			Use Finger::Desc::setFingerPose() to change the default orientation.
		*/
		NxD6JointDesc fingerJointDesc;
		
		/** Finger renderer description. */
		FingerRenderer::Desc fingerRendererDesc;
		/** Finger renderer state. */
		bool fingerRendererShow;
		
		/** Constructs description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();
			
			this->name = "Finger";

			// Finger controller description
			fingerCtrlDesc.setToDefault();

			// Replace the default transmitter with WrenchTransmitter
			WrenchTransmitter::Desc* pWrenchTransmitterDesc = new WrenchTransmitter::Desc();
			fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc.reset(pWrenchTransmitterDesc);

			// Disable signal synchronization
			fingerCtrlDesc.pReacPlannerDesc->signalSync = false;
			
			// Initial transmitter pose influence the finger pose
			pWrenchTransmitterDesc->initSignal.type = golem::Transmitter::Signal::TYPE_WORKSPACE;
			pWrenchTransmitterDesc->initSignal.gws.pos.R.rotX(-REAL_PI_2);
			pWrenchTransmitterDesc->initSignal.gws.pos.p.set(Real(0.0), Real(0.25), Real(0.075 + 0.015));
			
			// Movement boundaries
			pWrenchTransmitterDesc->referencePoseBoundsDesc.dimensions.set(Real(0.2), Real(0.17), Real(0.075));
			pWrenchTransmitterDesc->referencePoseBoundsDesc.pose.p.set(Real(0.0), Real(0.25), Real(0.075 + 0.015));
			
			// setup finger actor
			Mat34 pose;
			pose.setId();
			setFinger((Real)0.015, (Real)0.1, (Real)0.002, pose);
			
			// setup finger joint
			//setJoint(Real(0.02), Real(0.01), Real(0.02), Real(0.01), Real(0.0), Real(0.0));
			setJoint(Real(0.0), Real(0.0), Real(0.02), Real(0.01), Real(0.0), Real(0.0));
			
			// Finger's debug data renderer description
			fingerRendererDesc.setToDefault();
			
			fingerRendererShow = true;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;

			if (!fingerCtrlDesc.isValid() || !fingerActorDesc.isValid()/* || !fingerJointDesc.isValid()*/)
				return false;
			if (!fingerRendererDesc.isValid())
				return false;
			
			return true;
		}

		/** Creates the finger actor */
		virtual void setFinger(Real tipRadius, Real fingerLength, Real fingerRadius, const Mat34 &pose) {
			buffer.resetWrite();	// reset buffer

			// Finger actor description.
			// Finger body is assumed to be "anchored" at the origin of the controller tool frame along Y axis.
			// Use Finger::Desc::setFingerPose() to change the default orientation.
			fingerActorDesc.setToDefault();
			
			buffer.write(NxBodyDesc());
			NxBodyDesc* pNxBodyDesc = (NxBodyDesc*)buffer.get().back();
			pNxBodyDesc->angularDamping = (NxReal)0.9;
			pNxBodyDesc->linearDamping = (NxReal)0.9;
			fingerActorDesc.nxActorDesc.body = pNxBodyDesc;
			fingerActorDesc.nxActorDesc.density = (NxReal)10.0;
			fingerActorDesc.nxActorDesc.globalPose.t.set(&pose.p.v1);
			fingerActorDesc.nxActorDesc.globalPose.t.y += NxReal(fingerLength); // Y axis
			fingerActorDesc.nxActorDesc.globalPose.M.setRowMajor(&pose.R.m11);

			buffer.write(NxSphereShapeDesc());
			NxSphereShapeDesc &nxTipShapeDesc = *(NxSphereShapeDesc*)buffer.get().back();
			nxTipShapeDesc.radius = (NxReal)tipRadius;
			nxTipShapeDesc.localPose.t.y = (NxReal)0.0;
			fingerActorDesc.nxActorDesc.shapes.push_back(&nxTipShapeDesc);
			
			buffer.write(NxBoxShapeDesc());
			NxBoxShapeDesc &nxFingerShapeDesc = *(NxBoxShapeDesc*)buffer.get().back();
			nxFingerShapeDesc.dimensions.set((NxReal)fingerRadius, NxReal(fingerLength)/NxReal(2.0), (NxReal)fingerRadius);
			nxFingerShapeDesc.localPose.t.y = -NxReal(fingerLength)/NxReal(2.0); // Y axis
			fingerActorDesc.nxActorDesc.shapes.push_back(&nxFingerShapeDesc);
		}

		/** Creates the finger joint */
		virtual void setJoint(Real linSpring, Real angSpring, Real linDamping, Real angDamping, Real linRestitution, Real angRestitution) {
			// Finger joint description.
			// Joint axis is assumed to be oriented along Y axis of the controller tool frame.
			// Use Finger::Desc::setFingerPose() to change the default orientation.
			//const Real linValue = Real(0.001);
			//const Real angValue = Real(0.001);
			
			const Real linValue = Real(0.0);
			const Real angValue = Real(0.0);

			fingerJointDesc.setToDefault();

			fingerJointDesc.xMotion = NX_D6JOINT_MOTION_LOCKED;
			fingerJointDesc.yMotion = NX_D6JOINT_MOTION_LIMITED;
			//fingerJointDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
			fingerJointDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;
			fingerJointDesc.linearLimit.value = (NxReal)linValue;
			fingerJointDesc.linearLimit.spring = (NxReal)linSpring;
			fingerJointDesc.linearLimit.damping = (NxReal)linDamping;
			fingerJointDesc.linearLimit.restitution = (NxReal)linRestitution;

			fingerJointDesc.swing1Motion = NX_D6JOINT_MOTION_LIMITED;
			fingerJointDesc.swing1Limit.value = (NxReal)angValue;
			fingerJointDesc.swing1Limit.spring = (NxReal)angSpring;
			fingerJointDesc.swing1Limit.damping = (NxReal)angDamping;
			fingerJointDesc.swing1Limit.restitution = (NxReal)angRestitution;
			
			fingerJointDesc.swing2Motion = NX_D6JOINT_MOTION_LIMITED;
			fingerJointDesc.swing2Limit.value = (NxReal)angValue;
			fingerJointDesc.swing2Limit.spring = (NxReal)angSpring;
			fingerJointDesc.swing2Limit.damping = (NxReal)angDamping;
			fingerJointDesc.swing2Limit.restitution = (NxReal)angRestitution;
			
			fingerJointDesc.twistMotion = NX_D6JOINT_MOTION_LIMITED;
			fingerJointDesc.twistLimit.low.value =  (NxReal)-angValue;
			fingerJointDesc.twistLimit.low.spring = (NxReal)angSpring;
			fingerJointDesc.twistLimit.low.damping = (NxReal)angDamping;
			fingerJointDesc.twistLimit.low.restitution = (NxReal)angRestitution;
			fingerJointDesc.twistLimit.high.value = (NxReal)angValue;
			fingerJointDesc.twistLimit.high.spring = (NxReal)angSpring;
			fingerJointDesc.twistLimit.high.damping = (NxReal)angDamping;
			fingerJointDesc.twistLimit.high.restitution = (NxReal)angRestitution;
		}
	};

protected:
	/** Finger actor description */
	Actor::Desc fingerActorDesc;
	/** Finger joint description */
	NxD6JointDesc fingerJointDesc;
	
	/** Finger controller */
	PhysReacPlanner* pFingerCtrl;
	/** Finger actor */
	Actor *pFingerActor;
	/** Finger joint */
	NxD6Joint *pFingerJoint;
	
	/** Finger renderer */
	FingerRenderer::Ptr pFingerRenderer;
	/** Finger renderer description. */
	FingerRenderer::Desc fingerRendererDesc;
	/** Finger renderer state */
	bool fingerRendererShow;
	
	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(SecTmReal elapsedTime);
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const Desc& desc);
	
	/** Releases resources */
	virtual void release();
		
	/** Constructor */
	Finger(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~Finger();

	/** Writes the effector data */
	virtual bool write(const golem::GenConfigspaceState &data, SecTmReal timeStamp) {
		return pFingerCtrl->getReacPlanner().send(data);
	}
	
	/** Writes the effector data */
	virtual bool write(const golem::GenWorkspaceState &data, SecTmReal timeStamp) {
		return pFingerCtrl->getReacPlanner().send(data);
	}
	
	/** Writes the effector data in user defined format */
	virtual bool write(const void* data, SecTmReal timeStamp) {
		return pFingerCtrl->getReacPlanner().send(data);
	}
	
	/** Finger controller */
	virtual const PhysReacPlanner &getFingerCtrl() const {
		return *pFingerCtrl;
	}
	virtual PhysReacPlanner &getFingerCtrl() {
		return *pFingerCtrl;
	}
	
	/** Finger actor */
	virtual const Actor &getFingerActor() const {
		return *pFingerActor;
	}
	virtual Actor &getFingerActor() {
		return *pFingerActor;
	}

	/** Finger joint */
	virtual const NxJoint &getFingerJoint() const {
		return *pFingerJoint;
	}
	virtual NxJoint &getFingerJoint() {
		return *pFingerJoint;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_FINGER_H_*/
