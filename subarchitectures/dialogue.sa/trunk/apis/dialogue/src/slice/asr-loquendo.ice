#ifndef ASR_LOQUENDO_ICE
#define ASR_LOQUENDO_ICE

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
