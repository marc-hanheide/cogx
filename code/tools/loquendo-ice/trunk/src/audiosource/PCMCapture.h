#ifndef PCMCAPTURE_H__
#define PCMCAPTURE_H__  1

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

#include <pulse/simple.h>
#include <log4cxx/logger.h>

#include <string>
#include <iostream>

class PCMCapture {
public:
	PCMCapture(log4cxx::LoggerPtr logger, const std::string & description,
			pa_sample_format_t format, unsigned int rate);
	virtual ~PCMCapture();

	inline void * getBuffer() { return (void *) buffer; }
	inline size_t getBufferSize() { return buffer_size; }
	inline pa_simple * getHandle() { return handle; }

	int read();

private:
	pa_simple * handle;
	size_t buffer_size;
	uint8_t * buffer;
	log4cxx::LoggerPtr logger;
};

#endif
