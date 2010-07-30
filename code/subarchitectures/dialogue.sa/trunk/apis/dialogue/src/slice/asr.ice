#ifndef ASR_ICE
#define ASR_ICE

module de {
module dfki {
module lt {
module tr {
module dialogue {
module slice {
module asr {

	class RecognitionResult {
	};

	class NoRecognitionResult extends RecognitionResult {
	};

	interface ResultListener {
		void receiveRecognitionResult(RecognitionResult res);
	};

};
};
};
};
};
};
};

#endif
