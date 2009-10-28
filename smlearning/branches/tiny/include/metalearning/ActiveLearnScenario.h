/** @file ActiveLearnScenario.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

// #pragma once
#ifndef _SMLEARNING_ACTIVELEARNSCENARIO_H_
#define _SMLEARNING_ACTIVELEARNSCENARIO_H_

//------------------------------------------------------------------------------

#include <Golem/Application.h>
#include <Golem/PhysReacPlanner.h>
#include <Golem/Katana.h>
#include <Golem/Simulator.h>
#include <metalearning/Scenario.h>
#include <Golem/Rand.h>
// #include <Push/Data.h>
// #include <Push/Generator.h>

//------------------------------------------------------------------------------

namespace smlearning {

/** Image index */
class ImageIndex {
public:
        typedef std::vector<ImageIndex> Seq;

        /** Video index */
        int videoIndex;
        /** Image index */
        int imageIndex;
        /** Image time stamp */
        golem::SecTmReal timeStamp;
};

//------------------------------------------------------------------------------

/** Learning data format */
class LearningData : public golem::Serializable {
public:
	/** Data chunk */
	class Chunk {
	public:
		typedef std::vector<Chunk> Seq;
		
		/** Do nothing */
		Chunk() {
		}
		
		/** Data chunk time stamp */
		golem::SecTmReal timeStamp;
		
		/** Arm state - (joint) dynamic configuration */
		golem::GenConfigspaceState armState;
		/** End-effector GLOBAL pose */
		golem::Mat34 effectorPose;
		/** Object GLOBAL pose */
		golem::Mat34 objectPose;
		
// 		/** FT sensor data */
// 		golem::Wrench ftsData;
		/** Index to the camera image or video */
		ImageIndex imageIndex;
	};

	/** (Dynamic) Effector bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::effectorPose */
	golem::Bounds::Seq effector;
	/** (Dynamic) Object bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::objectPose */
	golem::Bounds::Seq object;
	/** (Static) Obstacles bounds in GLOBAL coordinates (usually ground plane) */
	golem::Bounds::Seq obstacles;
	
	/** Time-dependent data */
	Chunk::Seq data;

	/** Record validity */
	//bool bArmState;
	//bool bEffectorPose;
	//bool bObjectPose;
	//bool bFtsData;
	//bool bImageIndex;
	//bool bEffector;
	//bool bObject;
	//bool bObstacles;

	/** Load from stream */
	virtual bool load(const golem::Stream &stream);
	/** Save to stream */
	virtual bool store(golem::Stream &stream) const;
	/** Reset to default (empty)*/
	void setToDefault() {
		effector.clear();
		object.clear();
		obstacles.clear();
		data.clear();
		//bArmState = false;
		//bEffectorPose = false;
		//bObjectPose = false;
		//bFtsData = false;
		//bImageIndex = false;
		//bEffector = false;
		//bObject = false;
		//bObstacles = false;
	}
	/** Check if the data is valid */
	bool isValid() const {
		if (!data.empty()) // must not be empty
			return false;
		//if (bEffector && effector.empty())
		//	return false;
		//if (bObject && object.empty())
		//	return false;
		//if (bObstacles && obstacles.empty())
		//	return false;

		return true;
	}
};



//------------------------------------------------------------------------------

/** ActiveLearnScenario class */
class ActiveLearnScenario : public golem::Object, public smlearning::Scenario {
public:	
	/** Just Interrupted */
	class Interrupted {};
	
	/** Object description */
	class Desc : public golem::Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC(ActiveLearnScenario, golem::Object::Ptr, golem::Scene)

	public:
		/** Arm */
		golem::PhysReacPlanner::Desc armDesc;
		/** Effector bounds group */
		golem::U32 effectorGroup;
		/** Finger */
		golem::Bounds::Desc::Seq fingerDesc;
		/** Local end-effector reference pose */
		golem::WorkspaceCoord referencePose;
		/** Global end-effector home pose */
		golem::WorkspaceCoord homePose;
		/** Motion effects stabilization time period */
		golem::SecTmReal speriod;
		/** 2D motion constraint */
		bool motion2D;
// 		/** ActiveLearnScenario trials */
// 		Trial::Seq trials;
// 		/** Trial data path */
// 		DataPath dataPath;
		/** Random number generator */
		golem::RealRand rand;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::Object::Desc::setToDefault();
			
			// default arm description
			armDesc.setToDefault();
			armDesc.pArmDesc.reset(new golem::KatSimArm::Desc);
			armDesc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = golem::Real(1.0)*golem::REAL_PI; // last joint of Katana
			// Effector group
			effectorGroup = 0x4;
			// finger setup
			fingerDesc.clear();
			// end-effector reference pose
			referencePose.setId();
			// end-effector home pose
			homePose.R.rotX(-0.5*golem::REAL_PI); // end-effector pointing downwards
			homePose.p.set(golem::Real(0.0), golem::Real(0.1), golem::Real(0.1));
			// Other stuff
			motion2D = false;
			speriod = 1.0;
// 			// experimental setup
// 			trials.clear();
// 			// experiment data path
// 			dataPath.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!golem::Object::Desc::isValid())
				return false;
			if (!armDesc.isValid() || !referencePose.isFinite() || !homePose.isFinite())
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = fingerDesc.begin(); i != fingerDesc.end(); i++)
				if (!(*i)->isValid())
					return false;
// 			if (trials.empty() || !dataPath.isValid())
// 				return false;
// 			for (Trial::Seq::const_iterator i = trials.begin(); i != trials.end(); i++)
// 				if (!i->isValid())
// 					return false;
			
			return true;
		}
	};

	/** Run experiment */
	void run(int argc, char* argv[]);

protected:
	/** Description */
	Desc desc;
	/** Arm */
	golem::PhysReacPlanner* arm;
	/** End-effector */
	golem::JointActor* effector;
	/** End-effector bounds */
	golem::Bounds::Seq effectorBounds;
	/** Object */
	golem::Actor* object;
	/** Obstacles */
	golem::Actor* obstacles;
	/** Trial data */
	LearningData learningData;
	/** Time */
	golem::SecTmReal trialTime;
	Sequence currentSeq;
	FeatureVector currentMotorCommandVector;
	FeatureVector currentFeatureVector;

	/** Creator */
	golem::Creator creator;
	/** Synchronization objects */
	golem::CriticalSection cs;
	golem::Event ev;
	volatile bool bStart, bStop, bRec;

	/** (Post)processing function called AFTER every physics simulation step and before randering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** Creates ActiveLearnScenario from description. */
	bool create(const ActiveLearnScenario::Desc& desc);
	/** Releases resources */
	virtual void release();
	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene);
};

/** Reads/writes ActiveLearnScenario description from/to a given context */
bool XMLData(ActiveLearnScenario::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public golem::Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /* _SMLEARNING_ACTIVELEARNSCENARIO_H_ */
