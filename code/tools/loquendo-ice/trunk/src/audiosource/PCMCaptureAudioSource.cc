// ----------------------------------------------------------------------------
// Copyright (C) 2010-2011 DFKI GmbH Talking Robots 
// Miroslav Janicek (miroslav.janicek@dfki.de) 
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// ----------------------------------------------------------------------------

#include "PCMCaptureAudioSource.h"

#include <log4cxx/logger.h>

#include <iostream>

using namespace std;
using namespace log4cxx;

//------------------------------------------------------------------------------
// CALLBACK FUNCTIONS

// start recording
static int captureStart(lasrxHandleType inst, void * data) {
	PCMCaptureAudioSource * as = (PCMCaptureAudioSource *) data;
	LOG4CXX_TRACE(as->logger, "capture start");
//	as->start();
	return LASRX_RETCODE_OK;
/*
	if (rc < 0) {
//		cerr << tty::red << "* Unable to start PCM capture: " << snd_strerror(rc) << tty::dcol << endl;
		return LASRX_RETCODE_AUDIO;
	}
	else {
		return LASRX_RETCODE_OK;
	}
*/
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// stop recording
static int captureStop(lasrxHandleType inst, void * data) {
	PCMCaptureAudioSource * as = (PCMCaptureAudioSource *) data;
	LOG4CXX_TRACE(as->logger, "capture stop");
//	as->stop();
	return LASRX_RETCODE_OK;
/*
	if (capture) {
		delete capture;
		capture = NULL;
	}
	PCMCapture * capture = (PCMCapture *) hnd;
	pa_simple_drain(capture->getHandle(), NULL);
*/
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// get samples
static int captureProcessing(lasrxHandleType inst, void * data) {
	PCMCaptureAudioSource * as = (PCMCaptureAudioSource *) data;

//	int epd_flag;
//	lasrxGetEpdMode(inst, &epd_flag);  // XXX: what's this for?

	if (as->getCapture() == NULL) {
		LOG4CXX_ERROR(as->logger, "capture processing: PCM capture uninitialised");
		return LASRX_RETCODE_AUDIO;
	}

	int rc;
	if ((rc = as->getCapture()->read()) < 0) {
		LOG4CXX_ERROR(as->logger, "capture processing: error reading audio samples");
		return LASRX_RETCODE_AUDIO;
	}

	if (lasrxAudioStore(inst, as->getCapture()->getBuffer(), as->getCapture()->getBufferSize(), LASRX_STOREMODE_DONTWAIT) != LASRX_RETCODE_OK) {
		return LASRX_RETCODE_AUDIO;
	}
	else {
//		paudio->samples_delivered += NUM_SAMPLES_BLOCK;
		return LASRX_RETCODE_OK;
	}

}

//------------------------------------------------------------------------------
// AUDIO SOURCE WRAPPER

PCMCaptureAudioSource::PCMCaptureAudioSource(LoggerPtr logger_, const std::string & description_,
		pa_sample_format_t format_, unsigned int rate_)
: logger(logger_), capture(NULL), description(description_), format(format_), rate(rate_)
{
	LOG4CXX_TRACE(logger, "constructing");
}


PCMCaptureAudioSource::~PCMCaptureAudioSource()
{
	stop();
	unregisterSource();  // TODO: check for errors
	if (capture) {
		delete capture;
	}
	LOG4CXX_TRACE(logger, "destroyed");
}

void
PCMCaptureAudioSource::start()
{
	if (capture == NULL) {
		// TODO: how to do this elegantly?
		capture = new PCMCapture(Logger::getLogger(logger->getParent()->getName() + ".capture"),
				description, format, rate);
	}
}

void
PCMCaptureAudioSource::stop()
{
	LOG4CXX_TRACE(logger, "stopping");
	if (capture) {
		delete capture;
		capture = NULL;
	}
}

// Connect the audio source channel to Loquendo ASR instance.
lasrxResultType
PCMCaptureAudioSource::registerSource(void * instance)
{
	LOG4CXX_INFO(logger, "registering the audio source with Loquendo");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioStart(instance, captureStart)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioStop(instance, captureStop)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioProcessing(instance, captureProcessing)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioDataPointer(instance, (void *) this)) != LASRX_RETCODE_OK) {
		return rc;
	}
	inst = instance;
	return rc;
}

// Disconnect the audio source channel from Loquendo ASR instance.
lasrxResultType
PCMCaptureAudioSource::unregisterSource()
{
	LOG4CXX_INFO(logger, "unregistering the audio source from Loquendo");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioStart(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioStop(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioProcessing(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioDataPointer(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	inst = NULL;
	return rc;
}
