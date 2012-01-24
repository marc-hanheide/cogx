#ifndef ASR_LOQUENDO_ICE
#define ASR_LOQUENDO_ICE

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

// ----------------------------------------------------------------------------
// Defines the interface to the Loquendo ASR server and the recognition
// result data structures.
//
// Authors:		Miroslav Janicek  <miroslav.janicek@dfki.de>
// ----------------------------------------------------------------------------

#include <Ice/Identity.ice>

module de {
module dfki {
module lt {
module tr {
module dialogue {
module asr {
module loquendo {

	const string RELEASE = "1.2";

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

module time {

	// a translation of the C timeval structure into Ice
	class TimeVal {
		long sec;
		int usec;
	};

};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

module result {

	// Base class for recognition results.
	class RecognitionResult {
	};

	// Structure used to indicate recognition failure, e.g. voice has been
	// detected, but no speech was recognised in the signal.
	class NoRecognitionResult extends RecognitionResult {
	};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// N-BEST LISTS

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
		time::TimeVal timeAnchor;
		float snr;  // signal-to-noise ratio
		RejectionFlag rejectionAdvice;
		string roName;
		string roRule;
		int speechStartFrame;
		int speechEndFrame;
		HypothesisSeq hypos;
	};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// WORD LATTICES

	class Node {
		int id;  // node identifier
		float time;  // time from start of utterance (in sec)
		float frame;  // number of frames from start of utterance
	};

	sequence<Node> NodeSeq;

	class Link {
		int id;  // link identifier
		int start;  // start node number
		int end;  // end node number
		string word;  // word
		float confidence;  // acoustic confidence of the link
		float searchScore;  // tracing info (?)
		float sequenceScore;  // tracing info (?)
		float acoustic;  // acoustic likelihood of the link
	};

	sequence<Link> LinkSeq;

	class WordLattice extends RecognitionResult {
		time::TimeVal timeAnchor;
		NodeSeq nodes;
		LinkSeq links;
	};

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

	// AUDIO

	class AudioSource {
	};

	class PulseAudioPCMCapture extends AudioSource {
	};

	class RAWFile extends AudioSource {
		string path;
	};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	// INTERFACES

	interface Server {

		void setAudioSource(AudioSource as)
				throws ServerException;

		void setGrammarFile(string filename)
				throws LoquendoException;

		void addClient(Ice::Identity ident);

		void start()
				throws LoquendoException;
		void stop();
		void shutdown();

		time::TimeVal getCurrentTime();
		string getCWD();
		string getStatusString();
		Ice::IdentitySeq getClients();
	};

	interface Client {
		void onStart();
		void onStop();
		void onAudioSourceChange(AudioSource as);
		void onEndOfStream();
		void onGrammarChange(string filename);
		void onRecognitionResult(result::RecognitionResult res);
		void onUnregistrationFromServer(string reason);
	};

};
};
};
};
};
};
};

#endif
