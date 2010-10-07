   
#include <beliefs_cast.ice>

#ifndef BELIEF_COGX_ICE
#define BELIEF_COGX_ICE 

 
module eu {
module cogx {
module beliefs {
module slice {

class PerceptBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class GroundedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
};

class SharedBelief extends de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBelief {
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



