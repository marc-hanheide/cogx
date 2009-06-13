#include "FeaturePropertiesUtils.hpp"
#include "binding/utils/BindingUtils.hpp"
#include <iostream>
#include "binding/ontology/BindingFeatureOntology.hpp"
using namespace std;

namespace Binding {
  
ostream& 
operator<<(ostream& _out,const FeatureProperties& _p)
{
  _out << PRINTNAMED(_p.isInvariant) << endl;
  _out << PRINTNAMED(_p.isSimplex) << endl;
  _out << PRINTNAMED(_p.isDemanded) << endl;
//  _out << PRINTNAMED(_p.isImplemented) << endl;
  _out << BOOST_PP_STRINGIZE(_p.comparableInternally) << ": ";
  print_set(_out,_p.comparableInternally);
  _out << BOOST_PP_STRINGIZE(_p.comparableExternally) << ": ";
  print_set(_out,_p.comparableExternally);
  _out << endl;
  return _out;
}


//const string&
//type2Name_temp(const type_info& _info) {
//  return BindingFeatureOntology::construct().featureName(_info);
//}

} // namespace binding
