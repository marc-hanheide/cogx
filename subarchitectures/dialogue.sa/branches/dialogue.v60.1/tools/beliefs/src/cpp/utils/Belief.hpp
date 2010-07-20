#include <beliefs.hpp>

namespace dfkibeliefs = de::dfki::lt::tr::beliefs::slice;

namespace beliefs {
  class Belief : public dfkibeliefs::sitbeliefs::dBelief {

  };

  typedef  ::IceInternal::Handle<Belief> BeliefPtr;
 

}
