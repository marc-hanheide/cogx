/** @file Robot.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_ROBOT_H_
#define _GOLEM_DEMO_ROBOT_H_

//------------------------------------------------------------------------------

#include <Golem/Demo/Common/Finger.h>
#include <Golem/Demo/Common/RigidBodyTracker.h>
#include <Golem/Demo/Common/Retina.h>
#include <Golem/Demo/Common/Recognition.h>
#include <set>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

typedef golem::VirtualRigidBodyTracker RetinaObject;
typedef std::set<RetinaObject*> RetinaObjectList;

class Robot : public golem::Embodiment {
public:
	
	/** Object description */
	class Desc : public golem::Embodiment::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Robot, Object::Ptr, Scene&)

	public:
		/** Finger description */
		Finger::Desc fingerDesc;
		/** Control signal gain */
		Wrench ctrlGain;
		
		/** Retina description */
		golem::Retina::Desc retinaDesc;
		/** Finger retina object */
		RetinaObject::Desc fingerRetinaObjectDesc;
		
		/** Recognition description */
		golem::Recognition::Desc recognitionDesc;
		
		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Embodiment::Desc::setToDefault();
			
			const Real shift = Real(0.015);
			const Vec3 dimensions(Real(0.15), Real(0.15), Real(0.075));
			const Vec3 centre(Real(0.0), Real(0.25), dimensions.v3);
						
			// Replace the default transmitter with WrenchTransmitter
			fingerDesc.setToDefault();
			golem::WrenchTransmitter::Desc* pWrenchTransmitterDesc = dynamic_cast<golem::WrenchTransmitter::Desc*>(
				fingerDesc.fingerCtrlDesc.pReacPlannerDesc->pTransmitterDesc.get()
			);
			
			// turn off planner and collision detection
			fingerDesc.fingerCtrlDesc.pPlannerDesc->pHeuristicDesc->collisionDetection = false;
			
			// turn off renderables
			fingerDesc.fingerCtrlDesc.optimisedPathExShow = false;
			fingerDesc.fingerCtrlDesc.poseShow = false;
			fingerDesc.fingerCtrlDesc.pathShow = false;
			fingerDesc.fingerRendererShow = false;
			
			// Initial transmitter pose influence the finger pose
			pWrenchTransmitterDesc->initSignal.type = golem::Transmitter::Signal::TYPE_WORKSPACE;
			pWrenchTransmitterDesc->initSignal.gws.pos.R.rotX(-REAL_PI_2);
			pWrenchTransmitterDesc->initSignal.gws.pos.p.set(centre.v1, centre.v2, centre.v3 + shift);
			
			// Movement boundaries
			pWrenchTransmitterDesc->referencePoseBoundsDesc.dimensions = dimensions;
			pWrenchTransmitterDesc->referencePoseBoundsDesc.pose.p.set(centre.v1, centre.v2, centre.v3 + shift);

			// Control signal gain setup
			ctrlGain.set(Real(2.0e-2), Real(2.0e-1), Real(2.0e-2), Real(2.0e-1), Real(2.0e-1), Real(2.0e-1));
		
			// Retina
			retinaDesc.setToDefault();
			retinaDesc.retinaPose.p.set(centre.v1, centre.v2, centre.v3 - shift);
			Vec3 receptiveFieldGrid(Real(0.0025), Real(0.0025), Real(0.0025));
			Vec3 retinaDimensions(Real(2.0)*dimensions.v1, Real(2.0)*dimensions.v2, Real(2.0)*dimensions.v3 + shift);
			retinaDesc.setReceptiveFieldNum(retinaDimensions, receptiveFieldGrid);
			retinaDesc.volume = true;
			retinaDesc.retinaRendererDesc.retinaShow = true;

			// Finger retina object
			fingerRetinaObjectDesc.setToDefault();
			fingerRetinaObjectDesc.rigidBodyRendererDesc.bodyColour = RGBA::WHITE;

			// Recognition algorithm
			recognitionDesc.setToDefault();
			recognitionDesc.realisationRendererDesc.numOfRealisations = 1;
			const U32 numOfRotations = 8;


			// layer #1
			recognitionDesc.addDefaultLayer();
			recognitionDesc.layersDesc.back()->addGaborFilter(numOfRotations, 1.0);

			// layer #2
			recognitionDesc.addDefaultLayer();
			recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			recognitionDesc.layersDesc.back()->scale = Real(0.5);

			// layer #3
			recognitionDesc.addDefaultLayer();
			recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			recognitionDesc.layersDesc.back()->scale = Real(0.5);

			// layer #4
			//recognitionDesc.addDefaultLayer();
			//recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			//recognitionDesc.layersDesc.back()->scale = Real(0.5);

			//recognitionDesc.addDefaultLayer();
			//recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			//recognitionDesc.layersDesc.back()->scale = Real(0.5);

			//recognitionDesc.addDefaultLayer();
			//recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			//recognitionDesc.layersDesc.back()->scale = Real(0.5);

			//recognitionDesc.addDefaultLayer();
			//recognitionDesc.layersDesc.back()->add2PartsSymm(numOfRotations, Real(0.25), 0.5);
			//recognitionDesc.layersDesc.back()->scale = Real(0.5);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Embodiment::Desc::isValid())
				return false;
			
			if (!fingerDesc.isValid() || !ctrlGain.isFinite() || !retinaDesc.isValid())
				return false;
			
			return true;
		}
	};

protected:
	/** Finger */
	golem::Finger *pFinger;
	/** Control signal gain */
	Wrench ctrlGain;

	/** Retina */
	golem::Retina* pRetina;
	/** Retina objects */
	RetinaObjectList retinaObjectList;

	/** Recognition */
	golem::Recognition* pRecognition;

	int button, state, x, y, dx, dy, dz;

	void mouseHandler(int button, int state, int x, int y);

	void motionHandler(int x, int y);

	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** (Pre)processing function called BEFORE every physics simulation step and before randering. */
	virtual void preprocess(SecTmReal elapsedTime);
	
	/** Creates object from description. */
	bool create(const Robot::Desc& desc) {
		if (!Embodiment::create(desc))
			return false;
		
		pFinger = dynamic_cast<golem::Finger*>(createChannel(desc.fingerDesc));
		if (pFinger == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create finger");
		ctrlGain = desc.ctrlGain;

		pRetina = dynamic_cast<golem::Retina*>(createChannel(desc.retinaDesc));
		if (pRetina == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create retina filter");
		
		RetinaObject::Desc fingerRetinaObjectDesc = desc.fingerRetinaObjectDesc;
		fingerRetinaObjectDesc.actor = &pFinger->getFingerActor();
		RetinaObject* pRetinaObject = createRetinaObject(fingerRetinaObjectDesc);
		if (pRetinaObject == NULL)
			throw Message(Message::LEVEL_ERROR, "Robot::create(): Unable to create finger retina object");
		
		pRecognition = dynamic_cast<golem::Recognition*>(createChannel(desc.recognitionDesc));
		if (pRecognition == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Unable to create recognition filter");
		
		return true;
	}
	
	/** Releases resources */
	virtual void release() {
		Embodiment::release();
	}
	
	/** Objects can be constructed only in the Scene context. */
	Robot(golem::Scene &scene);

	virtual ~Robot() {
	}

public:
	/** Finger */
	const golem::Finger* getFinger() const {
		return pFinger;
	}
	golem::Finger* getFinger() {
		return pFinger;
	}
	
	/** Retina filter */
	const golem::Retina* getRetina() const {
		return pRetina;
	}
	golem::Retina* getRetina() {
		return pRetina;
	}
	
	/** Recognition filter */
	const golem::Recognition* getRecognition() const {
		return pRecognition;
	}
	golem::Recognition* getRecognition() {
		return pRecognition;
	}
	
	/** Creates Object from the Object description. */
	virtual RetinaObject* createRetinaObject(const RetinaObject::Desc& desc);

	/** Releases the retina object. */
	virtual void releaseRetinaObject(RetinaObject& retinaObject);

	/** Returns collection of retina objects */
	virtual const RetinaObjectList& getRetinaObjectList() const {
		return retinaObjectList;
	}

	/** Process data */
	bool read(golem::RecognitionOut &recognitionOut, golem::RetinaOut &retinaOut, SecTmReal &timeStamp);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_ROBOT_H_*/
