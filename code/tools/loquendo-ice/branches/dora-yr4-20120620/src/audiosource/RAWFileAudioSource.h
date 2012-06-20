#ifndef RAWFILEAUDIOSOURCE_H__
#define RAWFILEAUDIOSOURCE_H__  1

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

#include <cstdlib>  // required by LoqASR.h
#include <LoqASR.h>

#include <string>
#include <iostream>
#include <fstream>

#include <pulse/simple.h>
#include <log4cxx/logger.h>

#include "AudioSource.h"

class RAWFileAudioSource : public AudioSource {

public:
	RAWFileAudioSource(log4cxx::LoggerPtr logger, const std::string & path);
	virtual ~RAWFileAudioSource();

	virtual void start();
	virtual void stop();

	virtual lasrxResultType registerSource(void * instance);
	virtual lasrxResultType unregisterSource();

	log4cxx::LoggerPtr logger;

	size_t buffer_size;
	uint8_t * buffer;
	std::ifstream file;

private:
	std::string path;
};

#endif
