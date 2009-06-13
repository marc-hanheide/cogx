#ifndef BINDING_EXPERIMENTAL_UTILS_H_
#define BINDING_EXPERIMENTAL_UTILS_H_

#include <ostream>
#include <string>
#include <BindingFeaturesCommon.hpp>
#include <binding/idl/BindingFeatures.hh>

namespace Binding {



inline 
bool 
operator==(const BindingFeaturesCommon::ReferenceFrame& _r1,const BindingFeaturesCommon::ReferenceFrame& _r2){
  return std::string(_r1.coordinateSystemID) == std::string(_r2.coordinateSystemID);
}

inline 
bool 
operator<(const BindingFeaturesCommon::ReferenceFrame& _r1,const BindingFeaturesCommon::ReferenceFrame& _r2){
  return std::string(_r1.coordinateSystemID) < std::string(_r2.coordinateSystemID);
}

inline bool operator!=(const BindingFeaturesCommon::ReferenceFrame& _r1,const BindingFeaturesCommon::ReferenceFrame& _r2){
  return !(_r1 == _r2);
}

inline 
bool 
operator<(const BindingFeaturesCommon::Vector2& _v1,const BindingFeaturesCommon::Vector2& _v2){
  if(_v1.x != _v2.x)
    return _v1.x < _v2.x;
  if(_v1.y != _v2.y)
    return _v1.y < _v2.y;
  return false;
}

inline 
bool 
operator==(const BindingFeaturesCommon::Vector2& _v1,const BindingFeaturesCommon::Vector2& _v2){
  if(_v1.x == _v2.x && 
     _v1.y == _v2.y)
    return true;
  return false;
}

inline bool operator!=(const BindingFeaturesCommon::Vector2& _v1,const BindingFeaturesCommon::Vector2& _v2) {
  return !(_v1 == _v2);
}


inline 
bool 
operator<(const BindingFeaturesCommon::Line2& _l1,const BindingFeaturesCommon::Line2& _l2){
  if(_l1.point1 != _l2.point1)
    return _l1.point1 < _l2.point1;
  if(_l1.point2 != _l2.point2)
    return _l1.point2 < _l2.point2;
  return false;
}


inline
std::ostream&
operator<<(std::ostream& _out, const BindingFeaturesCommon::Vector2& _v) {
  _out << "(" << _v.x << "," << _v.y << ")";
  return _out;
}

inline
std::ostream&
operator<<(std::ostream& _out, const BindingFeaturesCommon::Line2& _l) {
  _out << _l.point1 << "->" << _l.point2;
  return _out;
}

inline
std::ostream&
operator<<(std::ostream& _out, const BindingFeaturesCommon::ReferenceFrame& _f) {
  _out << _f.coordinateSystemID; 
  return _out;
}



} // namespace Binding

#endif // BINDING_EXPERIMENTAL_UTILS_H_
