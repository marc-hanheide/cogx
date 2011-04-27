#ifndef ASR_LOQUENDO_ICE
#define ASR_LOQUENDO_ICE

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
// MODULE: de.dfki.lt.tr.dialogue.slice.asr.loquendo
//
// Defines the interface to the Loquendo ASR server and the recognition
// result data structures.
//
// Dependencies:
// - de.dfki.lt.tr.dialogue.slice.asr, defined in [ ./asr.ice ]
//
// This Slice module is used with the dialogue API v6.0
//
// Authors:		Miroslav Janicek  <miroslav.janicek@dfki.de>
//
// ===================================================================

#include <Ice/Identity.ice>
#include <asr.ice>

module de {
module dfki {
module lt {
module tr {
module dialogue {
module slice {
module asr {
module loquendo {

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// DATA STRUCTURES

	class WordHypothesis {
		string word;
		string lang;
		float acousticScore;
		float confidence;
		int startFrame;
		int endFrame;
	};

	sequence<WordHypothesis> WordHypothesisSeq;

	class Hypothesis {
		string str;
		WordHypothesisSeq words;
		float acousticScore;
		float confidence;
		string roName;
		string roRule;
	};

	sequence<Hypothesis> HypothesisSeq;

	enum RejectionFlag {
		RecognitionFalse,
		RecognitionNomatch
	};

	class NBestList extends RecognitionResult {
		float snr;  // signal-to-noise ratio
		RejectionFlag rejectionAdvice;
		string roName;
		string roRule;
		int speechStartFrame;
		int speechEndFrame;
		HypothesisSeq hypos;
	};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// EXCEPTIONS

	exception LoquendoException {
		string function;
		int returnCode;
		string errorMessage;
	};

	exception ServerException {
		string message;
	};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// INTERFACES

	enum RecogniserState {
		Stopped,
		Running,
		Error
	};

	interface Recogniser {

		void setSessionFile(string filename);
//			throws FileReadErrorException, RecogniserRunningException;
		void setGrammarFile(string filename);
//			throws FileReadErrorException, RecogniserRunningException;

		void start()
			throws LoquendoException;
		void stop();
		RecogniserState getState();

		void addListener(Ice::Identity ident);
//		bool isConnected();
	};

};
};
};
};
};
};
};
};

#endif
