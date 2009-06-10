#include "BindingScoreUtils.hpp"

namespace Binding {
using namespace std;

std::ostream& 
operator<<(std::ostream& _out, const BindingData::BindingScore& _score)
{
  _out << "comparable(" << _score.m_comparable << "), ";
  _out << "mismatch(" << _score.m_mismatch << "), ";
  _out << "relationMismatch(" << _score.m_relationMismatch << "), ";
//  _out << "existing_binding_match(" << _score.m_existing_binding_match << "), ";
  _out << "matches(" << _score.m_matches << "), ";
  _out << "relationMatches(" << _score.m_relationMatches << "), ";
  _out << "sticky(" << _score.m_sticky << "),";
  _out << "sH(" << _score.m_salienceHeuristics << "), ";
  _out << "v(" << _score.m_proxyUpdatesWhenThisComputed << "), ";
  _out << "(" << _score.m_proxyID << "["<< _score.m_proxyFeatureSignature << "]" << " vs "<< _score.m_unionID<< ")";
  return _out;
}

BindingData::BindingScore 
defaultBindingScore() 
{
  BindingData::BindingScore score;
  score.m_comparable = false;
  score.m_mismatch = false;
  //  score.m_existing_binding_match = false;
  score.m_matches = 0;
  score.m_relationMismatch = false;
  score.m_relationMatches = 0;
  score.m_sticky = false;
  score.m_salienceHeuristics = 1.0E13;
  score.m_proxyUpdatesWhenThisComputed = -1;
  score.m_proxyID = "";
  score.m_unionID = "";
  return score;
}

BindingData::BindingScore  
defaultThresholdScore() {
  BindingData::BindingScore score(defaultBindingScore());
  score.m_comparable = true;
  score.m_sticky = true;
  return score;
}

bool 
operator<(const BindingData::BindingScore& _score1, 
	  const BindingData::BindingScore& _score2)
{
  if(       _score1.m_comparable != _score2.m_comparable) {
    return  _score1.m_comparable >  _score2.m_comparable; // comparable is better
  } else if(_score1.m_mismatch   != _score2.m_mismatch) {
    return  _score1.m_mismatch   <  _score2.m_mismatch;  // no mismatch is better
  } else if(_score1.m_relationMismatch   != _score2.m_relationMismatch) {
    return  _score1.m_relationMismatch   <  _score2.m_relationMismatch; // no mismatch w.r.t relations is better
  } else if(_score1.m_matches != _score2.m_matches) {
    return  _score1.m_matches >  _score2.m_matches; // more matches is better
  } else if(_score1.m_relationMatches != _score2.m_relationMatches) {
    return  _score1.m_relationMatches >  _score2.m_relationMatches; // more matches w.r.t relations is better
  } else if(_score1.m_sticky != _score2.m_sticky) {
#ifdef EXTRANDEBUG
    if(_score1.m_sticky) {
      assert(_score1.m_comparable);
      assert(!_score1.m_mismatch);
      assert(!_score1.m_relationMismatch);
      assert(_score1.m_matches == 0);
      assert(_score1.m_relationMatches == 0);
    }
    if(_score2.m_sticky) {
      assert(_score2.m_comparable);
      assert(!_score2.m_mismatch);
      assert(!_score2.m_relationMismatch);
      assert(_score2.m_matches == 0);
      assert(_score2.m_relationMatches == 0);
    }
#endif // NDEBUG
    return  _score1.m_sticky >  _score2.m_sticky; // more matches w.r.t stickiness is better
  } 

  
  return false; // i.e. the scores are identical  
}

boost::logic::tribool 
tribool_cast(cast::cdl::TriBool _tri) {
  switch(_tri) {
  case cast::cdl::triTrue:
    return true;
    break;
  case cast::cdl::triFalse:
    return false;
    break;
  case cast::cdl::triIndeterminate:
    return boost::logic::indeterminate;
    break;
  }
  throw Binding::BindingException("Error in cast::cdl::TriBool -> boost::tribool conversion");
}

cast::cdl::TriBool
tribool_cast(boost::logic::tribool _tri) {
  switch(_tri.value) {
  case boost::logic::tribool::true_value:
    return cast::cdl::triTrue;
    break;
  case boost::logic::tribool::false_value:
    return cast::cdl::triFalse;
    break;
  case  boost::logic::tribool::indeterminate_value:
    return cast::cdl::triIndeterminate;
    break;
  }
  throw Binding::BindingException("Error in boost::tribool -> cast::cdl::TriBool conversion");
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
  throw Binding::BindingException("Error in boost::tribool -> cast::cdl::TriBool conversion");
}

std::string
scoreToString(const BindingData::BindingScore& _score) {
  std::stringstream str;
  str << _score;
  return str.str();
}


bool operator>(const BindingData::BindingScore& _score1, 
	       const BindingData::BindingScore& _score2) 
{
  return _score2 < _score1;
}

bool operator==(const BindingData::BindingScore& _score1, 
		const BindingData::BindingScore& _score2)
{
  return !(_score1 < _score2) && !(_score2 < _score1) && 
    _score1.m_salienceHeuristics == 
    _score2.m_salienceHeuristics &&
    _score1.m_proxyUpdatesWhenThisComputed ==
    _score2.m_proxyUpdatesWhenThisComputed &&
    _score1.m_proxyFeatureSignature ==
    _score2.m_proxyFeatureSignature;
;
}

bool operator!=(const BindingData::BindingScore& _score1, 
		       const BindingData::BindingScore& _score2)
{
  return !(_score1 == _score2);
}


} // namespace Binding


