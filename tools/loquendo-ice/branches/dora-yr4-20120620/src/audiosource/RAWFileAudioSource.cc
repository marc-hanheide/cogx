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

#include "RAWFileAudioSource.h"

#include <log4cxx/logger.h>

#include <iostream>

using namespace std;
using namespace log4cxx;

#define BUFFER_SIZE  64

//------------------------------------------------------------------------------
// CALLBACK FUNCTIONS

// start recording
static int
captureStart(lasrxHandleType inst, void * data)
{
	RAWFileAudioSource * as = (RAWFileAudioSource *) data;
	LOG4CXX_TRACE(as->logger, "capture start");
	return LASRX_RETCODE_OK;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// stop recording
static int
captureStop(lasrxHandleType inst, void * data)
{
	RAWFileAudioSource * as = (RAWFileAudioSource *) data;
	LOG4CXX_TRACE(as->logger, "capture stop");
	return LASRX_RETCODE_OK;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// get samples
static int
captureProcessing(lasrxHandleType inst, void * data)
{
	RAWFileAudioSource * as = (RAWFileAudioSource *) data;

	// TODO: check that this is correct!
	size_t nr = as->buffer_size;
	as->file.read((char *) as->buffer, as->buffer_size);

/*
	if (!as->file.good()) {
		LOG4CXX_DEBUG(as->logger, "we've probably reached EOF");
		return LASRX_RETCODE_AUDIO;
	}
*/

	if (!as->file.good()) {
		nr = 0;
		LOG4CXX_DEBUG(as->logger, "captureProcessing: we've probably reached EOF");
		return LASRX_RETCODE_AUDIO;
	}

	if (lasrxAudioStore(inst, as->buffer, nr, LASRX_STOREMODE_DONTWAIT) != LASRX_RETCODE_OK) {
		return LASRX_RETCODE_AUDIO;
	}
	else {
//		paudio->samples_delivered += NUM_SAMPLES_BLOCK;
		return LASRX_RETCODE_OK;
	}

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// check whether the audio source is stopped
static int
isStopped(lasrxHandleType inst, void * data)
{
	RAWFileAudioSource * as = (RAWFileAudioSource *) data;

	if (!as->file.good()) {
		LOG4CXX_DEBUG(as->logger, "isStopped: we've probably reached EOF");
		return LASRX_TRUE;
	}
	else {
		LOG4CXX_DEBUG(as->logger, "isStopped: so far good");
		return LASRX_FALSE;
	}
}

//------------------------------------------------------------------------------
// AUDIO SOURCE WRAPPER

RAWFileAudioSource::RAWFileAudioSource(LoggerPtr logger_, const std::string & path_)
: logger(logger_), buffer_size(BUFFER_SIZE), path(path_)
{
	LOG4CXX_TRACE(logger, "constructing the RAW file reader");

	buffer = new uint8_t[buffer_size];
	for (int i = 0; i < (int) buffer_size; i++) {
		buffer[i] = 0;
	}

	LOG4CXX_TRACE(logger, "opening " << path);
	file.open(path.c_str(), ios::in|ios::binary);

	if (!file.is_open()) {
		LOG4CXX_ERROR(logger, "failed to open \"" << path << "\"");
		throw "Failed to open \"" + path + "\"";
	}
}


RAWFileAudioSource::~RAWFileAudioSource()
{
	stop();
	unregisterSource();  // TODO: check for errors

	file.close();
	delete[] buffer;
	LOG4CXX_TRACE(logger, "destroyed");
}

void
RAWFileAudioSource::start()
{
	LOG4CXX_WARN(logger, "unimplemented: start()");
}

void
RAWFileAudioSource::stop()
{
	LOG4CXX_WARN(logger, "unimplemented: stop()");
}

// Connect the audio source channel to Loquendo ASR instance.
lasrxResultType
RAWFileAudioSource::registerSource(void * instance)
{
	LOG4CXX_INFO(logger, "registering the audio source with Loquendo");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioStart(instance, captureStart)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioStop(instance, captureStop)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioIsStopped(instance, isStopped)) != LASRX_RETCODE_OK) {
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
RAWFileAudioSource::unregisterSource()
{
	LOG4CXX_INFO(logger, "unregistering the audio source from Loquendo");
	lasrxResultType rc = LASRX_RETCODE_OK;
	if ((rc = lasrxSetAudioStart(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioStop(inst, NULL)) != LASRX_RETCODE_OK) {
		return rc;
	}
	if ((rc = lasrxSetAudioIsStopped(inst, NULL)) != LASRX_RETCODE_OK) {
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
