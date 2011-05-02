/** @file MessageStream.h
 * 
 * Message Stream.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_MESSAGESTREAM_H_
#define _GOLEM_TOOLS_MESSAGESTREAM_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Message.h>
#include <Golem/Math/Queue.h>
#include <Golem/Sys/Thread.h>
#include <Golem/Defs/Desc.h>
#include <iostream>
#include <stdio.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Message filter */
class MessageFilter {
public:
	typedef shared_ptr<MessageFilter> Ptr;

	virtual ~MessageFilter() {}

	/** Returns true if accepted, false otherwise */
	virtual bool operator () (SecTmReal time, U32 thread, Message::Level level, Message::Code code) const = 0;
};

/** Message stream with message filtering */
class MessageStream {
public:
	typedef shared_ptr<MessageStream> Ptr;

	/** Message logger description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Message filter, accept all messages if NULL */
		MessageFilter::Ptr filter;

		/** Constructs Logger description. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			filter.reset();
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			return true;
		}
		
		/** Creates Logger from the description. */
		virtual MessageStream::Ptr create() const = 0;
	};

protected:
	/** Message filter */
	MessageFilter::Ptr filter;

	/** Creates object from description */
	bool create(const Desc &desc);

	/** Constructor */
	MessageStream();

public:
	/** Virtual destructor */
	virtual ~MessageStream() {}
	
	/** Writes given message to the stream bypassing filter */
	virtual void write(Message::Ptr message) = 0;
	
	/** Constructs a message and writes to the stream */
	virtual void write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, va_list argptr);

	/** Constructs a message and writes to the stream */
	virtual void write(const char* format, ...);
	
	/** Constructs a message and writes to the stream */
	virtual void write(Message::Level level, const char* format, ...);
	
	/** Constructs a message and writes to the stream */
	virtual void write(Message::Level level, Message::Code code, const char* format, ...);
};

//------------------------------------------------------------------------------

/** Accepts messages with level equal or higher than specified */
class LevelFilter: public MessageFilter {
private:
	const Message::Level _level;

public:
	LevelFilter(Message::Level level = Message::LEVEL_INFO) : _level(level) {
	}
	
	virtual bool operator () (SecTmReal time, U32 thread, Message::Level level, Message::Code code) const {
		return level >= _level || level == Message::LEVEL_UNDEF;
	}
};

/** Output buffered stream */
class BufferedStream : public MessageStream, public Runnable {
public:
	friend class Desc;

	/** Message logger description */
	class Desc : public MessageStream::Desc {
	public:
		/** ANSI-C stream */
		FILE* cstr;
		/** C++ stream */
		std::ostream* cppstr;
		/** Message queue size */
		U32 queueSize;
		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Working thread time out */
		MSecTmU32 threadTimeOut;

		/** Constructs Logger description. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			MessageStream::Desc::setToDefault();
			filter.reset(new LevelFilter);
			cstr = stdout;// print on console
			cppstr = NULL;// &std::cout;
			queueSize = 1000;
			threadPriority = Thread::NORMAL;
			threadTimeOut = 30000; // 30 sec
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!MessageStream::Desc::isValid())
				return false;
			if ((cstr == NULL && cppstr == NULL) || queueSize <= 0 || threadTimeOut <= 0)
				return false;
			return true;
		}

		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC0(BufferedStream, MessageStream::Ptr)
	};

protected:
	FILE* cstr;
	std::ostream* cppstr;
	golem::queue<Message::Ptr> messages;
	MSecTmU32 threadTimeOut;
	Thread thread;
	Event evQueue;
	CriticalSection csQueue;
	CriticalSection csStream;
	bool terminate;

	/** Reads stream */
	virtual void run();

	/** Creates object from description */
	bool create(const Desc &desc);

public:
	/** Virtual destructor */
	virtual ~BufferedStream();

	/** Writes given message to the stream bypassing filter */
	virtual void write(Message::Ptr message);

	/** Output stream */
	void setCStream(FILE* cstr);

	/** Output stream */
	void setCppStream(std::ostream* cppstr);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_TOOLS_MESSAGESTREAM_H_*/
