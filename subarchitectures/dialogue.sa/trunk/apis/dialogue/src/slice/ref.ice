#ifndef REF_ICE
#define REF_ICE

#include <beliefs.ice>
#include <time.ice>

module de {
module dfki {
module lt { 
module tr { 
module dialogue { 
module ref { 

	class Constraint {
		string feature;
		string value;
	};

	["java:type:java.util.ArrayList<Constraint>"] sequence<Constraint> ConstraintSeq;

	class ResolutionRequest {
		string nom;
		string sort;
		ConstraintSeq constraints;
		de::dfki::lt::tr::dialogue::slice::time::Interval ival;
	};

	["java:type:java.util.ArrayList<ResolutionRequest>"] sequence<ResolutionRequest> ResolutionRequestSeq;

	class EpistemicReferenceHypothesis {
		de::dfki::lt::tr::beliefs::slice::epstatus::EpistemicStatus epst;
		de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula referent;
//		ConstraintSeq scorerConstraints;
		double score;
	};

	["java:type:java.util.ArrayList<EpistemicReferenceHypothesis>"] sequence<EpistemicReferenceHypothesis> EpistemicReferenceHypothesisSeq;

	class ResolutionResult {
		string nom;
		EpistemicReferenceHypothesisSeq hypos;
	};

};
};
};
};
};
};

#endif
