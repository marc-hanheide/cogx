/** @file DEKinematics.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_DEKINEMATICS_H_
#define _GOLEM_CTRL_DEKINEMATICS_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kinematics.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Approximate inverse kinematic problem solver using differential evolution. */
class DEKinematics : public Kinematics {
public:
	typedef shared_ptr<DEKinematics> Ptr;
	friend class Desc;

	/** Joint pose generator interface */
	class Generator {
	public:
		typedef shared_ptr<Generator> Ptr;
	
	protected:
		/** Generator of pseudo random numbers */
		Rand rand;

	public:
		virtual void next(ConfigspaceCoord &cc, const ConfigspaceCoord &min, const ConfigspaceCoord &max, U32 numOfJoints) const = 0;

		inline void setRandSeed(const RandSeed &randSeed) {
			rand.setRandSeed(randSeed);
		}
	};
	
	/** Uniform joint pose generator */
	class UniformGenerator : public Generator {
	public:
		virtual void next(ConfigspaceCoord &cc, const ConfigspaceCoord &min, const ConfigspaceCoord &max, U32 numOfJoints) const {
			for (U32 i = 0; i < numOfJoints; i++)
				cc[i] = rand.nextUniform(min[i], max[i]);
		}
	};
	
	/** Gaussian joint pose generator */
	class GaussianGenerator : public Generator {
	protected:
		const Real contraction;
	
	public:
		GaussianGenerator(Real contraction) : contraction(contraction) {}
		
		virtual void next(ConfigspaceCoord &cc, const ConfigspaceCoord &min, const ConfigspaceCoord &max, U32 numOfJoints) const {
			for (U32 i = 0; i < numOfJoints; i++) {
				const Real width = max[i] - min[i];
				const Real mean = min[i] + Real(0.5)*width;
				const Real stddev = contraction*width;
				
				Real &val = cc[i];
				do
					val = rand.nextGaussian<golem::Real>(mean, stddev);
				while (min[i] > val || max[i] < val);
			}
		}
	};
	
	/** Kinematics description */
	class Desc : public Kinematics::Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
	
		/** waypoint population size */
		U32 populationSize;
		/** number of generations */
		U32 numOfGenerations;
		/** vector difference factor */
		Real diffFac;
		/** crossover probability */
		Real crossProb;
		/** goal distance factor */
		Real distGoalFac;
		/** previous pose distance factor */
		Real distPrevFac;
		/** pose vs velocity computation factor */
		Real posVelFac;
		/** joint position/velocity generator */
		Generator::Ptr generator;
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(DEKinematics, Kinematics::Ptr, Heuristic&)
		
		/** Constructs the description object. */
		Desc() {
			setToDefault();
		}
		
		/** Sets the parameters to the default values */
		void setToDefault() {
			Kinematics::Desc::setToDefault();

			populationSize = 100;
			numOfGenerations = 20000;
			diffFac = Real(0.5);
			crossProb = Real(0.7);
			distGoalFac = Real(0.97); // 0.95 [0, 1]
			distPrevFac = Real(0.8); // 0.5 [0, 1]
			posVelFac = Real(0.47);
			generator.reset(new UniformGenerator);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Kinematics::Desc::isValid())
				return false;

			if (populationSize <= 4 || numOfGenerations <= 0)
				return false;
			if (diffFac <= REAL_ZERO)
				return false;
			if (crossProb < REAL_ZERO || crossProb > REAL_ONE)
				return false;
			if (distGoalFac < REAL_ZERO || distGoalFac > REAL_ONE)
				return false;
			if (distPrevFac < REAL_ZERO || distPrevFac > REAL_ONE)
				return false;
			if (posVelFac < REAL_ZERO || posVelFac > REAL_ONE)
				return false;
			if (generator == NULL)
				return false;
			
			return true;
		}
	};

protected:
	/** waypoint population size */
	U32 populationSize;
	/** number of generations */
	U32 numOfGenerations;
	/** vector difference factor */
	Real diffFac;
	/** crossover probability */
	Real crossProb;
	/** goal distance factor */
	Real distGoalFac;
	/** previous pose distance factor */
	Real distPrevFac;
	/** pose vs velocity computation factor */
	Real posVelFac;
	
	/** joint position/velocity generator */
	Generator::Ptr generator;
	
	Waypoint::Seq posPopulation;
	bool bInitPosPopulation;
	
	GenWaypoint::Seq velPopulation;
	bool bInitVelPopulation;

	/** Joint coords zero */
	ConfigspaceCoord zero;

	virtual Real costPos(const ConfigspaceCoord &jCurr, const ConfigspaceCoord &cPrev, const Waypoint &wCurr, const Waypoint &wGoal) const;
	
	virtual Real costVel(const ConfigspaceCoord &jVelCurr, const ConfigspaceCoord &jVelPrev, const Twist &wVelCurr, const Twist &wVelGoal) const;
	
	virtual bool collides(const Waypoint &w) const;
	
	virtual bool find(U32 &solution, GenWaypoint::Seq &population, const GenConfigspaceCoord &gj, const GenWorkspaceCoord &gw, const Generator &generator, SecTmReal runningTime) const;
	
	virtual bool find(U32 &solution, Waypoint::Seq &population, const ConfigspaceCoord &jc, const WorkspaceCoord &wc, const Generator &generator, SecTmReal runningTime) const;
	
	/** Creates Kinematics from the description. 
	* @param desc		Kinematics description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Kinematics constructor */
	DEKinematics(golem::Heuristic &heuristic);

public:
	/** Generalised inverse kinematics solver */
	virtual bool find(GenConfigspaceCoord &gj, const GenWorkspaceCoord &gw, SecTmReal runningTime = SEC_TM_REAL_MAX);
	
	/** Inverse kinematics solver */
	virtual bool find(ConfigspaceCoord &jc, const WorkspaceCoord &wc, SecTmReal runningTime = SEC_TM_REAL_MAX);

	/** Returns waypoint population */
	virtual void setPosPopulation(const Waypoint::Seq &population);

	/** Returns waypoint population */
	virtual const Waypoint::Seq &getPosPopulation() const;
	
	/** Returns waypoint population */
	virtual void setVelPopulation(const GenWaypoint::Seq &population);

	/** Returns waypoint population */
	virtual const GenWaypoint::Seq &getVelPopulation() const;
	
	/** waypoint population size */
	virtual void setPopulationSize(U32 populationSize) {
		this->populationSize = populationSize;
	}

	/** waypoint population size */
	virtual U32 getPopulationSize() const {
		return populationSize;
	}

	/** number of generations */
	virtual void setNumOfGenerations(U32 numOfGenerations) {
		this->numOfGenerations = numOfGenerations;
	}
	
	/** number of generations */
	virtual U32 getNumOfGenerations() const {
		return numOfGenerations;
	}

	/** vector difference factor */
	virtual void setDiffFac(Real diffFac) {
		this->diffFac = diffFac;
	}

	/** vector difference factor */
	virtual Real getDiffFac() const {
		return diffFac;
	}

	/** crossover probability */
	virtual void setCrossProb(Real crossProb) {
		this->crossProb = crossProb;
	}

	/** crossover probability */
	virtual Real getCrossProb() const {
		return crossProb;
	}

	/** goal distance factor */
	virtual void setDistGoalFac(Real distGoalFac) {
		this->distGoalFac = distGoalFac;
	}

	/** goal distance factor */
	virtual Real getDistGoalFac() const {
		return distGoalFac;
	}

	/** pose vs velocity computation factor */
	virtual void setPosVelFac(Real posVelFac) {
		this->posVelFac = posVelFac;
	}

	/** pose vs velocity computation factor */
	virtual Real getPosVelFac() const {
		return posVelFac;
	}

	/** joint position/velocity generator */
	virtual void setGenerator(Generator::Ptr generator) {
		this->generator = generator;
	}

	/** joint position/velocity generator */
	virtual Generator::Ptr getGenerator() const {
		return generator;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DEKINEMATICS_H_*/
