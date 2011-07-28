   
#include <beliefs_cast.ice>

#ifndef BELIEF_COGX_ICE
#define BELIEF_COGX_ICE 

 
module eu {
module cogx {

module mln {

module slice {
sequence<string> FactSeq;
sequence<double> FactProbSeq;

class MLNState {
	FactSeq facts;
	FactProbSeq probs;
};
};
};

module beliefs {
module slice {


class PerceptBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class GroundedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class SharedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class PresupposedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class AssertedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

}; 
// end slice
}; 
// end beliefs
}; 
// end cogx
}; 
// end eu


#endif



