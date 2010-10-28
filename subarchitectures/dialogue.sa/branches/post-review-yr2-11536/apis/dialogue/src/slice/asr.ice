#ifndef ASR_ICE
#define ASR_ICE

// ===================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
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
// ===================================================================

// ===================================================================
// MODULE: de.dfki.lt.tr.dialogue.slice.asr
//
// Defines the base data structures and interfaces for integrating automatic
// speech recognition (ASR) systems into the architecture. System-specific
// data structures may be found in the corresponding Slice files.
//
// This Slice module is used with the dialogue API v6.0
//
// Authors:		Miroslav Janicek  <miroslav.janicek@dfki.de>
//
// ===================================================================

module de {
module dfki {
module lt {
module tr {
module dialogue {
module slice {
module asr {

	// Base class for recognition results.
	class RecognitionResult {
	};

	// Structure used to indicate recognition failure, e.g. voice has been
	// detected, but no speech was recognised in the signal.
	class NoRecognitionResult extends RecognitionResult {
	};

	// A class providing a callback function for receiving recognition
	// results from an ASR server.
	interface ResultListener {
		void receiveRecognitionResult(RecognitionResult res);
		void unregisteredFromServer(string reason);
	};

};
};
};
};
};
};
};

#endif
