/** @file Transmitter.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_TRANSMITTER_H_
#define _GOLEM_CTRL_TRANSMITTER_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Performance monitor */
//#define _TRANSMITTER_PERFMON

//------------------------------------------------------------------------------

/** Motor command transmitter. */
class Transmitter {
public:
	typedef shared_ptr<Transmitter> Ptr;
	/** Motor command signal */
	class Signal {
	public:
		/** Signal type */
		enum Type {
			/** configuration space */
			TYPE_JOINTSPACE,
			/** workspace */
			TYPE_WORKSPACE,
		};

		/** Signal type */
		Type type;
		
		/** Signal data */
		GenConfigspaceState gjs;
		GenWorkspaceState gws;

		Signal() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_JOINTSPACE;

			gjs.set(GenCoord(REAL_ZERO, REAL_ZERO, REAL_ZERO));
			gjs.t = SEC_TM_REAL_ZERO;
			
			gws.setZero();
			gws.t = SEC_TM_REAL_ZERO;
		}

		/** Checks if the signal is valid. */
		bool isValid() const {
			if (type != TYPE_JOINTSPACE && type != TYPE_WORKSPACE)
				return false;

			if (!gjs.isFinite() || !Math::isFinite(gjs.t))
				return false;
			if (!gws.isFinite() || !Math::isFinite(gws.t))
				return false;

			return true;
		}
	};

	/** Transmitter description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		/** Initial control signal */
		Signal initSignal;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			initSignal.setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!initSignal.isValid())
				return false;

			return true;
		}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Transmitter, Transmitter::Ptr, Arm&)
	};

protected:
	/** Arm */
	golem::Arm &arm;
	/** Context object */
	golem::Context &context;

	/** Control signal */
	Signal signal, initSignal;
	mutable CriticalSection cs;
	
	/** Creates Transmitter from the description. 
	* @param desc		Transmitter description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Transmitter constructor */
	Transmitter(golem::Arm &arm);

public:
	/** Descructor */
	virtual ~Transmitter();

	/** Sets control signal vector in configuration space coordinates. */
	virtual bool set(const GenConfigspaceState &data);
	
	/** Sets control signal vector in workspace coordinates. */
	virtual bool set(const GenWorkspaceState &data);
	
	/** Sets control signal vector in user-defined format. */
	virtual bool set(const void *data);
	
	/** Gets the arm control signal. */
	virtual bool get(Signal &signal) const;

	/** Gets the arm initial control signal. */
	inline const Signal& getInitSignal() const {
		return initSignal;
	}

	/** Sets the arm initial control signal. */
	inline void setInitSignal(const Signal& signal) {
		initSignal = signal;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_TRANSMITTER_H_*/
