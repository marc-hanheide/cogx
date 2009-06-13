#include "BindingScoreUtils.hpp"

namespace Binding {
using namespace std;

std::ostream&
operator<<(std::ostream& _out, const BindingData::BindingScore& _score)
{
  _out << "comparable(" << _score.comparable << "), ";
  _out << "mismatch(" << _score.mismatch << "), ";
  _out << "relationMismatch(" << _score.relationMismatch << "), ";
//  _out << "existing_binding_match(" << _score.existing_binding_match << "), ";
  _out << "matches(" << _score.matches << "), ";
  _out << "relationMatches(" << _score.relationMatches << "), ";
  _out << "sticky(" << _score.sticky << "),";
  _out << "sH(" << _score.salienceHeuristics << "), ";
  _out << "v(" << _score.proxyUpdatesWhenThisComputed << "), ";
  _out << "(" << _score.proxyID << "["<< _score.proxyFeatureSignature << "]" << " vs "<< _score.unionID<< ")";
  return _out;
}


BindingData::BindingScorePtr
defaultBindingScore()
{
  BindingData::BindingScorePtr scoreP = new BindingData::BindingScore;
  scoreP->comparable = false;
  scoreP->mismatch = false;
  //  score.existing_binding_match = false;
  scoreP->matches = 0;
  scoreP->relationMismatch = false;
  scoreP->relationMatches = 0;
  scoreP->sticky = false;
  scoreP->salienceHeuristics = 1.0E13;
  scoreP->proxyUpdatesWhenThisComputed = -1;
  scoreP->proxyID = "";
  scoreP->unionID = "";
  return scoreP;
}

BindingData::BindingScorePtr
defaultThresholdScore() {
  BindingData::BindingScorePtr scoreP(defaultBindingScore());
  scoreP->comparable = true;
  scoreP->sticky = true;
  return scoreP;
}

bool
operator<(const BindingData::BindingScore& _score1,
	  const BindingData::BindingScore& _score2)
{
  if(       _score1.comparable != _score2.comparable) {
    return  _score1.comparable >  _score2.comparable; // comparable is better
  } else if(_score1.mismatch   != _score2.mismatch) {
    return  _score1.mismatch   <  _score2.mismatch;  // no mismatch is better
  } else if(_score1.relationMismatch   != _score2.relationMismatch) {
    return  _score1.relationMismatch   <  _score2.relationMismatch; // no mismatch w.r.t relations is better
  } else if(_score1.matches != _score2.matches) {
    return  _score1.matches >  _score2.matches; // more matches is better
  } else if(_score1.relationMatches != _score2.relationMatches) {
    return  _score1.relationMatches >  _score2.relationMatches; // more matches w.r.t relations is better
  } else if(_score1.sticky != _score2.sticky) {
#ifdef EXTRANDEBUG
    if(_score1.sticky) {
      assert(_score1.comparable);
      assert(!_score1.mismatch);
      assert(!_score1.relationMismatch);
      assert(_score1.matches == 0);
      assert(_score1.relationMatches == 0);
    }
    if(_score2.sticky) {
      assert(_score2.comparable);
      assert(!_score2.mismatch);
      assert(!_score2.relationMismatch);
      assert(_score2.matches == 0);
      assert(_score2.relationMatches == 0);
    }
#endif // NDEBUG
    return  _score1.sticky >  _score2.sticky; // more matches w.r.t stickiness is better
  }


  return false; // i.e. the scores are identical
}

boost::logic::tribool
tribool_cast(BindingData::TriBool _tri) {
  switch(_tri) {
  case BindingData::TRUETB:
    return true;
    break;
  case BindingData::FALSETB:
    return false;
    break;
  case BindingData::INDETERMINATETB:
    return boost::logic::indeterminate;
    break;
  }
  throw Binding::BindingException("Error in BindingData::TriBool -> boost::tribool conversion");
}

BindingData::TriBool
tribool_cast(boost::logic::tribool _tri) {
  switch(_tri.value) {
  case boost::logic::tribool::true_value:
    return BindingData::TRUETB;
    break;
  case boost::logic::tribool::false_value:
    return BindingData::FALSETB;
    break;
  case  boost::logic::tribool::indeterminate_value:
    return BindingData::INDETERMINATETB;
    break;
  }
  throw Binding::BindingException("Error in boost::tribool -> BindingData::TriBool conversion");
}

std::string&
triboolToString(boost::logic::tribool _tri) {
  switch(_tri.value) {
  case boost::logic::tribool::true_value:
    static string _true("true");
    return _true;
    break;
  case boost::logic::tribool::false_value:
    static string _false("false");
    return _false;
    break;
  case  boost::logic::tribool::indeterminate_value:
    static string _indeterminate("indeterminate");
    return _indeterminate;
    break;
  }
  throw Binding::BindingException("Error in boost::tribool -> BindingData::TriBool conversion");
}

std::string
scoreToString(const BindingData::BindingScore& _score) {
  std::stringstream str;
  str << _score;
  return str.str();
}


/**
bool operator>(const BindingData::BindingScore& _score1,
	       const BindingData::BindingScore& _score2)
{
  return _score2 < _score1;
}

bool operator==(const BindingData::BindingScore& _score1,
		const BindingData::BindingScore& _score2)
{
  return !(_score1 < _score2) && !(_score2 < _score1) &&
    _score1.salienceHeuristics ==
    _score2.salienceHeuristics &&
    _score1.proxyUpdatesWhenThisComputed ==
    _score2.proxyUpdatesWhenThisComputed &&
    _score1.proxyFeatureSignature ==
    _score2.proxyFeatureSignature;
;
}

bool operator!=(const BindingData::BindingScore& _score1,
		       const BindingData::BindingScore& _score2)
{
  return !(_score1 == _score2);
}
*/

} // namespace Binding


