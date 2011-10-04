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

#include <cstdlib>
#include <cstring>

extern "C" {
#include <fcntl.h>
#include <unistd.h>
}

#include "PCMCapture.h"
#include <pulse/error.h>
#include <pulse/channelmap.h>

#include <log4cxx/logger.h>

using namespace std;
using namespace log4cxx;

#define BUFFER_SIZE  64

PCMCapture::PCMCapture(LoggerPtr logger_, const string & description, pa_sample_format_t format,
		unsigned int rate)
: handle(NULL), buffer_size(BUFFER_SIZE), logger(logger_)
{
	LOG4CXX_TRACE(logger, "initialising PCM capture with format=" << pa_sample_format_to_string(format) << ", rate=" << rate);

	pa_sample_spec sfmt;
	sfmt.format = format;
	sfmt.rate = rate;
	sfmt.channels = 1;

	pa_channel_map monoMap;

	int error;

	if (!(handle = pa_simple_new(NULL, description.c_str(), PA_STREAM_RECORD, NULL, "voice recording",
			&sfmt, pa_channel_map_init_mono(&monoMap), NULL, &error))) {
		LOG4CXX_ERROR(logger, "pa_simple_new() error: " << pa_strerror(error));
		throw "Error in pa_simple_new";  // TODO: do this properly
	}

	buffer = new uint8_t[buffer_size];

	LOG4CXX_TRACE(logger, "PCM capture initialised");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

PCMCapture::~PCMCapture()
{
	LOG4CXX_TRACE(logger, "closing PCM capture");
	if (handle) {
		// XXX flush always complains about "Bad state"
//		int error;
//		LOG4CXX_TRACE(logger, "flushing");
//		pa_simple_flush(handle, &error);
//		LOG4CXX_TRACE(logger, "flush result = " << pa_strerror(error));
		LOG4CXX_TRACE(logger, "freeing the handle");
		pa_simple_free(handle);
	}
	delete[] buffer;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int
PCMCapture::read()
{
	int error;
	int rc;

	if ((rc = pa_simple_read(handle, buffer, buffer_size, &error)) < 0) {
		LOG4CXX_ERROR(logger, "pa_simple_read() error: " << pa_strerror(error));
	}
	return rc;
}
