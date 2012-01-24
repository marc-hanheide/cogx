#ifndef ASR_CLARIFY_ICE
#define ASR_CLARIFY_ICE

#include <cast/slice/CDL.ice>
#include <beliefs.ice>
#include <dialogue.ice>
#include <intentions.ice>

module de {
module dfki {
module lt { 
module tr { 
module dialogue {
module slice {
module asr {

class UnclarifiedPhonString {
	PhonString ps;
};

class UnclarifiedPossibleInterpretedIntentions {
	de::dfki::lt::tr::beliefs::slice::intentions::PossibleInterpretedIntentions pii;
	string phonStringWordList;
	float asrConfidence;
};


};
};
};

}; 
}; 
}; 
}; 

#endif