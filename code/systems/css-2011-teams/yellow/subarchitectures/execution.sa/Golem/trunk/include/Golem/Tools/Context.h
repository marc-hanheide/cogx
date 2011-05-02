/** @file Context.h
 * 
 * Context class is a collection of application helper objects.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_CONTEXT_H_
#define _GOLEM_TOOLS_CONTEXT_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/MessageStream.h>
#include <Golem/Math/Rand.h>
#include <Golem/Tools/Parallels.h>
#include <Golem/Defs/Desc.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Context object is a collection of shared object */
class Context {
public:
	typedef shared_ptr<Context> Ptr;
	friend class Desc;

	/** Context description */
	class Desc {
	public:
		/** Random generator seed */
		RandSeed randSeed;
		/** Message Stream */
		MessageStream::Desc::Ptr messageStreamDesc;
		/** Parallels thread joint time out */
		MSecTmU32 threadTimeOut;
		/** Number of threads in Parallels */
		U32 threadParallels;

		/** Constructs context description. */
		Desc() {
			setToDefault();
		}
		
		/** Virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			messageStreamDesc.reset(new BufferedStream::Desc);
			threadTimeOut = 5000;
			threadParallels = 0; // inactive by default
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (messageStreamDesc == NULL || !messageStreamDesc->isValid())
				return false;
			if (threadTimeOut <= 0)
				return false;

			return true;
		}
		
		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC0(Context, Context::Ptr)
	};
	
protected:
	/** Precision/performance timer */
	PerfTimer timer;
	/** Random generator seed */
	RandSeed randSeed;
	/** Message stream */
	MessageStream::Ptr messages;
	/** Parallels thread joint time out */
	MSecTmU32 threadTimeOut;
	/** Parallels */
	shared_ptr<Parallels> parallels;
	
	/** Creates context from description */
	bool create(const Desc &desc);

	/** Default constructror */
	Context();

public:
	/** Virtual destructor */
	virtual ~Context();
	
	/** Initialise context in the current module */
	void initModule();

	/** Returns pointer to timer */
	inline const PerfTimer& getTimer() const {
		return timer;
	}

	/** Returns reference to randseed */
	inline const RandSeed& getRandSeed() const {
		return randSeed;
	}

	/** Returns pointer to message stream */
	inline const MessageStream* getMessageStream() const {
		return messages.get();
	}
	inline MessageStream* getMessageStream() {
		return messages.get();
	}

	/** Returns pointer to parallels */
	inline const Parallels* getParallels() const {
		return parallels.get();
	}
	inline Parallels* getParallels() {
		return parallels.get();
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_CONTEXT_H_*/
