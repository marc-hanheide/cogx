/** @file Recorder.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_RECORDER_H_
#define _GOLEM_PHYS_RECORDER_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Context.h>
#include <Golem/Phys/Object.h>
#include <list>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** OpenGL screen capture */
class Recorder : protected Runnable {
public:
	typedef shared_ptr<Recorder> Ptr;

	class Desc {
	public:
		/** File name */
		std::string fileName;
		/** File extension */
		std::string fileExt;
		/** Leading zeros */
		bool leadingZeros;
		/** Encoder string */
		std::string encoderString;

		/** Buffer length */
		U32 bufferLen;
		/** Inter-thread signalling time out */
		MSecTmU32 threadTimeOut;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}

		/** Creates Recorder from the description. */
		CREATE_FROM_OBJECT_DESC1(Recorder, Recorder::Ptr, Context&)
		
		/** Sets the parameters to the default values. */
		void setToDefault() {
			fileName = "screen";
			fileExt = ".png";
			leadingZeros = true;
			encoderString = "image/png";
			
			bufferLen = 100;
			threadTimeOut = 5000; //[msec]
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (fileName.empty() || fileExt.empty() || encoderString.empty())
				return false;
			if (bufferLen < 1)
				return false;
			
			return true;
		}
	};

protected:
	class Frame {
	public:
		typedef std::list<Frame> Seq;

		int x;
		int y;
		int width;
		int height;
		size_t size;
		shared_ptr<U32, arr_cnt<U32> > data;

		Frame() {
		}
		Frame(int x, int y, int width, int height) : x(x), y(y), width(width), height(height), size((size_t)width*height), data(new U32 [size]) {
			if (data != NULL) ::memset(data.get(), 0, size);
		}
	};

	/** Context */
	golem::Context &context;
	
	/** File name */
	std::string fileName;
	/** File extension */
	std::string fileExt;
	/** Leading zeros */
	bool leadingZeros;
	/** Encoder string */
	std::string encoderString;
	/** File counter */
	U32 fileCount;

	/** Buffer length */
	U32 bufferLen;
	/** Frame buffer */
	Frame::Seq frameSeq;

	CriticalSection cs;
	Event ev;
	Thread thread;
	volatile bool bTerminate;
	MSecTmU32 threadTimeOut;

	/** captures current screen */
	void save(const Frame& frame);
	
	virtual void run();
	
	/** Constructs Recorder class without initialisation */
	Recorder(golem::Context& context);

	/** Creates/initialises the Recorder */
	bool create(const Desc& desc);
	
public:
	virtual ~Recorder();

	/** captures current screen */
	void capture(int x, int y, int width, int height);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_RECORDER_H_*/
