#include "PlanningUtils.hpp"
using namespace Planner;


bool operator== (const Fact & _f1, 
		 const Fact & _f2) {

  if (strcmp(_f1.name,_f2.name) != 0 ||
      _f1.arguments.length() != _f2.arguments.length()) {
    return false;
  }

  // are the arguments the same?
  for (unsigned int i = 0; i < _f1.arguments.length(); i++) {
    if (strcmp(_f1.arguments[i],_f2.arguments[i]) != 0) {
      return false;
    }
  }

  // are the values the same?
  return (strcmp(_f1.value,_f2.value) == 0);

}








std::ostream & 
operator<<(std::ostream &_stream, 
	   const Planner::Fact &_fact) {

  if (_fact.modality == K_MODALITY) {
    _stream<<"(K ";
  }

  _stream<<"(";
  _stream<<_fact.name;
  _stream<<" ";
  for (unsigned int i = 0; i <  _fact.arguments.length(); ++i) {
    _stream<<_fact.arguments[i];
    _stream<<(" ");
  }
  _stream<<": ";
  _stream<<_fact.value;
  _stream<<")";

  if (_fact.modality == K_MODALITY) {
    _stream<<")";
  }

  return _stream;
}

std::ostream & 
operator<<(std::ostream &_stream, 
	   const Planner::ObjectDeclaration &_decl) {
  _stream<<"(" << _decl.name<< " - " << _decl.type << ")";
  return _stream;
}


std::ostream & 
operator<<(std::ostream &_stream, 
	   const TemporaryPlanningState &_state) {
  _stream<<"TemporaryPlanningState:"<<endl;

  _stream <<"\nObjects:"<<endl;
  for(TemporaryPlanningState::ObjectDeclarationSet::const_iterator os = _state.m_objectList.begin(); 
      os != _state.m_objectList.end(); ++os) {    
      _stream <<*os<<endl;
  }

  _stream <<"\nFacts:"<<endl;
  for(TemporaryPlanningState::FactSet::const_iterator fs = _state.m_factList.begin(); 
      fs != _state.m_factList.end(); ++fs) {          
      _stream <<*fs<<endl;
  }
  return _stream;
}

