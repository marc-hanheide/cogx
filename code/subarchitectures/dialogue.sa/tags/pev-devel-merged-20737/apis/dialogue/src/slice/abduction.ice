#ifndef ABDUCTION_ICE
#define ABDUCTION_ICE

#include <lf.ice>
#include <beliefs.ice>
#include <ref.ice>
#include <time.ice>
#include <abducer.ice>

module de {
module dfki {
module lt { 
module tr { 
module dialogue { 
module slice { 
module interpret {

	class InterpretationRequest {
		de::dfki::lt::tr::infer::abducer::lang::ModalisedAtom goal;
	};

	["java:type:java.util.ArrayList<String>"] sequence<string> stringSeq;

	dictionary<string, de::dfki::lt::tr::dialogue::ref::ConstraintSeq> StringToConstraintSeqDict;

	class Interpretation {
		lf::LogicalForm lform;
		time::Interval ival;
		de::dfki::lt::tr::infer::abducer::proof::ProofWithCostSeq proofs;
		stringSeq ungroundedNoms;
//		de::dfki::lt::tr::dialogue::ref::ResolutionRequestSeq rrs;
//		StringToConstraintSeqDict cts;
	};

//	class UngroundedIntentionalContent extends de::dfki::lt::tr::beliefs::slice::intentions::IntentionalContent {
//		VarConstraints varctrs;
//	};

//	["java:type:java.util.ArrayList<UngroundedIntentionalContent>"] sequence<UngroundedIntentionalContent> UngroundedIntentionalContentSeq;

//	class UngroundedIntention {
//		UngroundedIntentionalContentSeq alt;
//
//		lf::LogicalForm lform;
//		time::Interval ival;
//	};

};
};
}; 
}; 
}; 
}; 
};

#endif