/** @file Embodiment.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_EMBODIMENT_H_
#define _GOLEM_DEMO_COMMON_EMBODIMENT_H_

//------------------------------------------------------------------------------

#include <Golem/Phys/Object.h>
#include <Golem/Math/Collection.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Embodiment;

/** Sensorimotor communication channel
*/
class Channel : public Object {
public:
	typedef shared_ptr<Channel> Ptr;
	
	/** Channel description */
	class Desc : public Object::Desc  {
		friend class Embodiment;

	private:
		mutable Embodiment *pEmbodiment;
		mutable Channel::Ptr pChannel;
	
	protected:
		/** Creates/initialises the object. */
		virtual Channel::Ptr create(Embodiment &embodiment) const = 0;
	
		/** Creates/initialises the object in the Scene context. */
		virtual Object::Ptr create(Scene &scene) const {
			if (pEmbodiment != NULL)
				pChannel = create(*pEmbodiment);
			return static_pointer_cast<Object::Ptr>(pChannel);
		}
	
	public:
		/** Channel name */
		std::string name;

		/** Constructs description object */
		Desc() : pEmbodiment(NULL) {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			name = "";
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Embodiment object */
	Embodiment &embodiment;
	/** Channel name */
	std::string name;

	/** Creates object from description. */
	bool create(const Channel::Desc& desc);
	
	/** Releases resources */
	virtual void release();
	
	/** Constructor */
	Channel(Embodiment &embodiment);
	
public:
	/** Destructor is inaccesible */
	virtual ~Channel();

	/** Channel name */
	virtual const char *getName() const {
		return name.c_str();
	}

	/** Embodiment object */
	inline const Embodiment& getEmbodiment() const {
		return embodiment;
	}
	
	/** Embodiment object */
	inline Embodiment& getEmbodiment() {
		return embodiment;
	}
};

//------------------------------------------------------------------------------

/** Embodiment
*/
class Embodiment : public Object {
	typedef shared_ptr<Embodiment> Ptr;
	friend class Channel;

public:
	typedef PrivateList<Channel*, Channel::Ptr> ChannelList;
	
	/** Embodiment description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Embodiment, Object::Ptr, Scene&)

	public:
		/** Embodiment name */
		const char *name;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();
		
			name = "Embodiment";
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;

			return true;
		}
	};

protected:
	/** Embodiment name */
	const char *name;
	/** Channels collection */
	ChannelList channelList;

	/** Creates object from description. */
	bool create(const Embodiment::Desc& desc);
	
	/** Releases resources */
	void release();
	
	/** Constructor */
	Embodiment(Scene &scene);

public:
	/** Destructor is inaccesible */
	virtual ~Embodiment();

	/** Embodiment name */
	virtual const char *getName() const {
		return name;
	}

	/** Creates Channel from the description. */
	virtual Channel *createChannel(const Channel::Desc &desc);

	/** Releases the Channel. */
	virtual void releaseChannel(Channel &channel);

	/** Returns collection of Channels */
	virtual const ChannelList& getChannelList() const {
		return channelList;
	}
};

//------------------------------------------------------------------------------

/** Sensor
*/
template <typename DATA>
class Sensor : public Channel {
public:
	typedef shared_ptr<Sensor> Ptr;

	/** Sensor description */
	class Desc : public Channel::Desc {
	public:
	};

protected:
	/** Constructor */
	Sensor(Embodiment &embodiment) : Channel(embodiment) {
	}

public:
	/** Reads the current sensory data */
	virtual bool read(DATA &data, SecTmReal &timeStamp, MSecTmU32 timeOut = MSEC_TM_U32_INF) = 0;
};

//------------------------------------------------------------------------------

/** Effector
*/
template <typename DATA>
class Effector : public Channel {
public:
	typedef shared_ptr<Effector> Ptr;

	/** Effector description */
	class Desc : public Channel::Desc {
	public:
	};

protected:
	/** Constructor */
	Effector(Embodiment &embodiment) : Channel(embodiment) {
	}

public:
	/** Writes the effector data */
	virtual bool write(const DATA &data, SecTmReal timeStamp) = 0;
};

//------------------------------------------------------------------------------

/** Filter
*/
template <typename INPUT, typename OUTPUT>
class Filter : public Channel {
public:
	typedef shared_ptr<Filter> Ptr;

	/** Filter description */
	class Desc : public Channel::Desc {
	public:
	};

protected:
	/** Constructor */
	Filter(Embodiment &embodiment) : Channel(embodiment) {
	}

public:
	/** Process data */
	virtual bool process(OUTPUT &out, const INPUT &inp, MSecTmU32 timeOut = MSEC_TM_U32_INF) = 0;
};

//------------------------------------------------------------------------------

/** Learner realises INPUT->OUTPUT mapping approximation. 
*/
template <typename INPUT, typename OUTPUT>
class Learner : public Channel {
public:
	typedef shared_ptr<Learner> Ptr;

	/** Learner description */
	class Desc : public Channel::Desc {
	public:
	};

protected:
	/** Constructor */
	Learner(Embodiment &embodiment) : Channel(embodiment) {
	}

public:
	/** Updates INP -> OUT pair */
	virtual bool set(const OUTPUT &out, const INPUT &inp, MSecTmU32 timeOut = MSEC_TM_U32_INF) = 0;

	/** Finds INP -> OUT */
	virtual bool get(OUTPUT &out, const INPUT &inp, MSecTmU32 timeOut = MSEC_TM_U32_INF) const = 0;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_EMBODIMENT_H_*/
