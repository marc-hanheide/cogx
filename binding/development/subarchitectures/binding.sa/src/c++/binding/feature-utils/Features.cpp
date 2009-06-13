#include "Features.hpp"
#include "binding/abstr/AbstractMonitor.hpp"

namespace Binding {
//AbstractFeature::AbstractFeature(const cdl::WorkingMemoryID& _id)
// : workingemoryID(_id) { }

std::ostream& 
operator<<(std::ostream& _out,const AbstractFeature& _f)
{
  _out << _f.toString(AbstractFeature::only_feature);
  return _out;
}

} // namespace Binding
