#ifndef BINDING_SALIENCE_HELPER_H_
#define BINDING_SALIENCE_HELPER_H_

#include <ostream>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>
#include "binding/feature-utils/FeatureHelper.hpp"
#include "binding/feature-utils/AbstractInternalComparator.hpp"

namespace Binding {

class SalienceVsSalienceComparator : public ReflexiveInternalComparator<BindingFeatures::Salience> 
{
public:
  SalienceVsSalienceComparator() : 
    ReflexiveInternalComparator<BindingFeatures::Salience>() {}
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const;
};

class SalienceVsCreationTimeComparator : public SymmetricInternalComparator<BindingFeatures::Salience,BindingFeatures::CreationTime> 
{
public:
  SalienceVsCreationTimeComparator() : 
    SymmetricInternalComparator<BindingFeatures::Salience,BindingFeatures::CreationTime>() {}
  /// now only returns indeterminate.
  virtual boost::tribool compare(const AbstractFeature& _proxyFeature, 
				 const AbstractFeature& _unionFeature) const {return boost::indeterminate;}
};

class SalienceHelper : public FeatureHelper<BindingFeatures::Salience> {
public:
  SalienceHelper();
  virtual std::ostream& print(std::ostream& _out, const AbstractFeature& _feat) const;
  virtual bool operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const;
};

bool operator<(const FrameworkBasics::BALTTime& _time1, const FrameworkBasics::BALTTime& _time2);
bool operator==(const FrameworkBasics::BALTTime& _time1, const FrameworkBasics::BALTTime& _time2);
std::ostream& operator<<(std::ostream& _out, const FrameworkBasics::BALTTime& _time);
std::ostream& operator<<(std::ostream& _out, const BindingFeaturesCommon::TimeSeq& _time);
bool operator==(const BindingFeaturesCommon::TimeSeq& _time1, const BindingFeaturesCommon::TimeSeq& _time2);
std::ostream& operator<<(std::ostream& _out, const BindingFeaturesCommon::TimeSeq& _time);
bool operator==(const BindingFeaturesCommon::TimeSeq& _time1, const BindingFeaturesCommon::TimeSeq& _time2);
bool start_time_leq(const BindingFeaturesCommon::StartTime& _time1, const BindingFeaturesCommon::StartTime& _time2);
bool end_time_leq(const BindingFeaturesCommon::EndTime& _time1, const BindingFeaturesCommon::EndTime& _time2);
bool start_end_time_leq(const BindingFeaturesCommon::StartTime& _time1, const BindingFeaturesCommon::EndTime& _time2);
bool starts_before(const BindingFeatures::Salience& _salience1, const BindingFeatures::Salience& _salience2);
bool ends_after(const BindingFeatures::Salience& _salience1, const BindingFeatures::Salience& _salience2);
bool overlap(const BindingFeatures::Salience& _salience1, const BindingFeatures::Salience& _salience2);
/// a simplified way to create a BALTTime from a double
FrameworkBasics::BALTTime baltTime(double _t);
/// translates BALTtime into a double
double baltTime(const FrameworkBasics::BALTTime& _t);
/// returns a start time in the infinite past (for open intervals)
BindingFeaturesCommon::StartTime infinitePast();
/// creates a start time at the specified instant
BindingFeaturesCommon::StartTime startTime(const FrameworkBasics::BALTTime& _t);
/// returns an end time in the infinite past (for open intervals)
BindingFeaturesCommon::EndTime infiniteFuture();
/// creates an end time at the specified instant
BindingFeaturesCommon::EndTime endTime(const FrameworkBasics::BALTTime& _t);
BindingFeaturesCommon::EndTime diff(const BindingFeaturesCommon::StartTime& _time1, const BindingFeaturesCommon::EndTime& _time2);



} // namespace Binding

#endif // BINDING_SALIENCE_HELPER_H_
