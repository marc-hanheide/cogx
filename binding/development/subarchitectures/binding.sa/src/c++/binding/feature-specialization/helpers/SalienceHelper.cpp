#include "SalienceHelper.hpp"
#include "binding/utils/BindingUtils.hpp"


namespace Binding {
using namespace std; 
using namespace boost; 
using namespace BindingFeatures; 

boost::tribool 
SalienceVsSalienceComparator::compare(const AbstractFeature& _proxyFeature, 
				      const AbstractFeature& _unionFeature) const {
  const BindingFeatures::Salience& proxySalience(getIDLFeature(_proxyFeature));
  const BindingFeatures::Salience& unionSalience(getIDLFeature(_unionFeature));    
  
  if(overlap(proxySalience,unionSalience))
    return true;
  return indeterminate;
}

SalienceHelper::SalienceHelper() 
{
  FeatureProperties prop; 
  prop.m_isInvariant   = false;
  prop.m_isSimplex     = true;
  prop.m_isDemanded    = false;
  setProperties(prop);
}

ostream& 
SalienceHelper::print(ostream& _out, const AbstractFeature& _feat) const 
{
  const Salience& salience(extract(_feat));
  assert(salience.m_start.m_t.length() <= 1);
  assert(salience.m_end.m_t.length() <= 1);      
  assert((salience.m_start.m_t.length() == 0 || salience.m_end.m_t.length() == 0) ||
	 (salience.m_start.m_t[0] < salience.m_end.m_t[0]) ||
	 (salience.m_start.m_t == salience.m_end.m_t));

  if(salience.m_start.m_t == salience.m_end.m_t)
    _out << "[" << salience.m_start.m_t << "]";
  else
    _out << "[" << salience.m_start.m_t << "," << salience.m_end.m_t << "]";
  return _out;
}

bool 
SalienceHelper::operatorLessImpl(const AbstractFeature& _feat1, const AbstractFeature& _feat2) const 
{
  const Salience& salience1 (extract(_feat1));
  const Salience& salience2 (extract(_feat2));
  if(!(salience1.m_start.m_t == salience2.m_start.m_t)) {
    // an infinitely old start time
    if(salience1.m_start.m_t.length() == 0)
      return true;
    // an infinitely old start time in second salience, is not less 
    if(salience2.m_start.m_t.length() == 0)
      return false;
    // return just the times compared
    //    assert(salience1.m_start.m_t.length() == 1 && salience2.m_start.m_t.length() == 1);
    return salience1.m_start.m_t[0] < salience2.m_start.m_t[0];
  }
  // an infinitely old end time
  if(salience1.m_end.m_t.length() == 0)
    return false;
  // an infinitely old end time in second salience
  if(salience2.m_end.m_t.length() == 0)
    return true;
  // return just the times compared
  //  assert(salience1.m_end.m_t.length() == 1 && salience2.m_end.m_t.length() == 1);
  return salience1.m_end.m_t[0] < salience2.m_end.m_t[0];  
}



bool operator<(const FrameworkBasics::BALTTime& _time1, const FrameworkBasics::BALTTime& _time2) 
{
  if(      _time1.m_s  != _time2.m_s)
    return _time1.m_s  <  _time2.m_s;
  return   _time1.m_us <  _time2.m_us;
}

bool operator==(const FrameworkBasics::BALTTime& _time1, const FrameworkBasics::BALTTime& _time2) 
{
  return _time1.m_s == _time2.m_s && _time1.m_us == _time2.m_us;
}


ostream& 
operator<<(ostream& _out, const FrameworkBasics::BALTTime& _time)
{
  _out << _time.m_s << "." << _time.m_us;
  return _out;
}

ostream&
operator<<(ostream& _out, const BindingFeaturesCommon::TimeSeq& _time)
{
  if(_time.length() == 0) {
    _out << "oo";
  } else 
    _out << _time[0];
  return _out;
}

bool
operator==(const BindingFeaturesCommon::TimeSeq& _time1, const BindingFeaturesCommon::TimeSeq& _time2)
{

  if(_time1.length() != _time2.length() )
    return false;
  if(_time1.length() == 0 && _time2.length() == 0) 
    return true;
  //assert(_time1.length() == 1 && _time2.length() == 1);
  return _time1[0] == _time2[0];
}


bool
start_time_leq(const BindingFeaturesCommon::StartTime& _time1, 
	       const BindingFeaturesCommon::StartTime& _time2)
{
  if(_time1.m_t.length() == 0 && _time2.m_t.length() == 0)
    return false;
  if(_time1.m_t.length() < _time2.m_t.length())
    return true;
  if(_time1.m_t.length() > _time2.m_t.length())
    return false;
  //  assert(_time1.m_t.length() == 1 && _time2.m_t.length() == 1);
  return _time1.m_t[0] < _time2.m_t[0] || _time1.m_t[0] == _time2.m_t[0];
}

bool
end_time_leq(const BindingFeaturesCommon::EndTime& _time1, 
	     const BindingFeaturesCommon::EndTime& _time2)
{
  if(_time1.m_t.length() == 0 && _time2.m_t.length() == 0)
    return false;
  if(_time2.m_t.length() < _time1.m_t.length())
    return true;
  if(_time2.m_t.length() > _time1.m_t.length())
    return false;
  assert(_time1.m_t.length() == 1 && _time2.m_t.length() == 1);
  return _time1.m_t[0] < _time2.m_t[0] || _time1.m_t[0] == _time2.m_t[0];

}

// leq or equal...
bool
start_end_time_leq(const BindingFeaturesCommon::StartTime& _time1, // start_time 
		   const BindingFeaturesCommon::EndTime& _time2) // end_time
{
  if(_time1.m_t.length() == 0 && _time2.m_t.length() == 0)
    return true; // infinitely old vs. infinite future
  if(_time1.m_t.length() < _time2.m_t.length())
    return true;
  if(_time2.m_t.length() < _time1.m_t.length())
    return true;
  //  assert(_time1.m_t.length() == 1 && _time2.m_t.length() == 1);
  return _time1.m_t[0] < _time2.m_t[0] || _time1.m_t[0] == _time2.m_t[0];
}

bool
starts_before(const Salience& _salience1, 
	      const Salience& _salience2)
{
  return start_time_leq(_salience1.m_start,_salience2.m_start);
}

bool
ends_after(const Salience& _salience1, 
	   const Salience& _salience2)
{
  return end_time_leq(_salience1.m_end,_salience2.m_end);
}

bool 
overlap(const Salience& _salience1, 
	const Salience& _salience2)
{
  if(_salience1.m_start.m_t.length() == 0 && _salience2.m_start.m_t.length() == 0) // open start
    return true;
  if(_salience1.m_end.m_t.length() == 0 && _salience2.m_end.m_t.length() == 0) // open ends (i.e. from some time until present)
    return true;  
  if(starts_before(_salience1,_salience2)) {
    if(start_end_time_leq(_salience2.m_start,_salience1.m_end)) // overlap
      return true;
  }
  if(starts_before(_salience2,_salience1)) {
    if(start_end_time_leq(_salience1.m_start,_salience2.m_end)) // overlap
      return true;
  }
  return false;

}

FrameworkBasics::BALTTime 
baltTime(double _t)
{
  FrameworkBasics::BALTTime ret; 
  ret.m_s=static_cast<int>(_t);
  ret.m_us=static_cast<int>((_t-ret.m_s)*1.0E6);
  return ret;
}

double 
baltTime(const FrameworkBasics::BALTTime& _t)
{
  double ret = _t.m_s + static_cast<double>(_t.m_us) / 1.0E6;
  return ret;
}

BindingFeaturesCommon::StartTime 
infinitePast()
{
  BindingFeaturesCommon::StartTime ret; 
  ret.m_t.length(0); 
  return ret;
}

BindingFeaturesCommon::StartTime 
startTime(const FrameworkBasics::BALTTime& _t)
{
  BindingFeaturesCommon::StartTime ret; 
  ret.m_t.length(1); 
  ret.m_t[0]=_t; 
  return ret;
}

BindingFeaturesCommon::EndTime 
infiniteFuture()
{
  BindingFeaturesCommon::EndTime ret; 
  ret.m_t.length(0); 
  return ret;
}

BindingFeaturesCommon::EndTime 
endTime(const FrameworkBasics::BALTTime& _t)
{
  BindingFeaturesCommon::EndTime ret; 
  ret.m_t.length(1); 
  ret.m_t[0]=_t; 
  return ret;
}

BindingFeaturesCommon::EndTime
diff(const BindingFeaturesCommon::StartTime& _time1, 
     const BindingFeaturesCommon::EndTime& _time2) {
  if(_time1.m_t.length() == 0)
    return infiniteFuture();
  if(_time2.m_t.length() == 0)
    return infiniteFuture();
  return endTime(baltTime(baltTime(_time2.m_t[0]) - baltTime(_time1.m_t[0])));
}

} //namespace Binding
