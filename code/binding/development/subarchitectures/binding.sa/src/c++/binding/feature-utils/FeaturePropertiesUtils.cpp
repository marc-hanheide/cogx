#include "FeaturePropertiesUtils.hpp"
#include "binding/utils/BindingUtils.hpp"
#include <iostream>
#include "binding/ontology/BindingFeatureOntology.hpp"
using namespace std;

namespace Binding {
  
ostream& 
operator<<(ostream& _out,const FeatureProperties& _p)
{
  _out << PRINTNAMED(_p.m_isInvariant) << endl;
  _out << PRINTNAMED(_p.m_isSimplex) << endl;
  _out << PRINTNAMED(_p.m_isDemanded) << endl;
//  _out << PRINTNAMED(_p.m_isImplemented) << endl;
  _out << BOOST_PP_STRINGIZE(_p.m_comparableInternally) << ": ";
  print_set(_out,_p.m_comparableInternally);
  _out << BOOST_PP_STRINGIZE(_p.m_comparableExternally) << ": ";
  print_set(_out,_p.m_comparableExternally);
  _out << endl;
  return _out;
}


//const string&
//type2Name_temp(const type_info& _info) {
//  return BindingFeatureOntology::construct().featureName(_info);
//}

} // namespace binding
