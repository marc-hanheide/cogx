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

#include "PCMCapture.h"
#include "TtyUtils.h"

#include <cstdlib>

extern "C" {
#include <signal.h>
}

#include <iostream>
#include <fstream>

static bool recording;

void
stopRecording(int sigNum);

int
main(int argc, char ** argv)
{
	if (argc != 2) {
		std::cout << "Usage: " << argv[0] << " OUTPUT-FILE" << std::endl;
		return EXIT_FAILURE;
	}

	PCMCapture * capture = new PCMCapture(PA_SAMPLE_ALAW, 8000);

	std::fstream file(argv[1], std::ios::out|std::ios::binary|std::ios::trunc);

	recording = true;
	signal(SIGINT, stopRecording);
	std::cerr << "recording started..." << std::endl;
	for (long i = 0; recording; i++) {
		int rc = capture->read();

		if (rc < 0) {
			std::cerr << tty::red << "error at " << i << std::endl;
			break;
		}
		else {
			file.write((const char *) capture->getBuffer(), capture->getBufferSize());
		}
		i++;
	}
	std::cerr << "recording stopped." << std::endl;
	file.close();
	delete capture;

	return EXIT_SUCCESS;
}

void
stopRecording(int sigNum)
{
//	std::cerr << "Ctrl-C detected" << std::endl;
	recording = false;
}
