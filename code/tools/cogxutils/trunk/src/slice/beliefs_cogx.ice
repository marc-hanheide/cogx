   
#include <beliefs_cast.ice>

#ifndef BELIEF_COGX_ICE
#define BELIEF_COGX_ICE 

 
module eu {
module cogx {

module mln {

module slice {

struct MLNFact {
	string type;
	string estatus;
	string key;
	string id;
	string atom;
	double prob;
};

sequence<MLNFact> MLNFactSeq;

class MLNState {
	MLNFactSeq facts;
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

// GeorgeY4 belief subclasses

class PrivateBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class AssumedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class MergedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class VerifiedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
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



