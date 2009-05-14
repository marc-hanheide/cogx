// This file is generated by omniidl (C++ backend) - omniORB_4_1. Do not edit.

#include "VisualAttributes.hh"

OMNI_USING_NAMESPACE(omni)

static const char* _0RL_dyn_library_version = omniORB_4_1_dyn;

static ::CORBA::TypeCode::_Tracker _0RL_tcTrack(__FILE__);

static const char* _0RL_enumMember_VisualAttributes_mColour[] = { "RED", "ORANGE", "YELLOW", "GREEN", "BLUE", "INDIGO", "VIOLET", "PINK", "BLACK", "WHITE", "BROWN", "GREY", "PURPLE", "UNKNOWN_COLOUR" };
static CORBA::TypeCode_ptr _0RL_tc_VisualAttributes_mColour = CORBA::TypeCode::PR_enum_tc("IDL:VisualAttributes/Colour:1.0", "Colour", _0RL_enumMember_VisualAttributes_mColour, 14, &_0RL_tcTrack);
#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace VisualAttributes { 
  const ::CORBA::TypeCode_ptr _tc_Colour = _0RL_tc_VisualAttributes_mColour;
} 
#else
const ::CORBA::TypeCode_ptr VisualAttributes::_tc_Colour = _0RL_tc_VisualAttributes_mColour;
#endif

static CORBA::PR_structMember _0RL_structmember_VisualAttributes_mColourWithConfidence[] = {
  {"m_colour", _0RL_tc_VisualAttributes_mColour},
  {"m_confidence", CORBA::TypeCode::PR_float_tc()}
};

#ifdef _0RL_tc_VisualAttributes_mColourWithConfidence
#  undef _0RL_tc_VisualAttributes_mColourWithConfidence
#endif
static CORBA::TypeCode_ptr _0RL_tc_VisualAttributes_mColourWithConfidence = CORBA::TypeCode::PR_struct_tc("IDL:VisualAttributes/ColourWithConfidence:1.0", "ColourWithConfidence", _0RL_structmember_VisualAttributes_mColourWithConfidence, 2, &_0RL_tcTrack);

#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace VisualAttributes { 
  const ::CORBA::TypeCode_ptr _tc_ColourWithConfidence = _0RL_tc_VisualAttributes_mColourWithConfidence;
} 
#else
const ::CORBA::TypeCode_ptr VisualAttributes::_tc_ColourWithConfidence = _0RL_tc_VisualAttributes_mColourWithConfidence;
#endif


static const char* _0RL_enumMember_VisualAttributes_mShape[] = { "SQUARE", "CUBE", "TRIANGLE", "PYRAMID", "CIRCLE", "SPHERE", "CYLINDER", "UNKNOWN_SHAPE" };
static CORBA::TypeCode_ptr _0RL_tc_VisualAttributes_mShape = CORBA::TypeCode::PR_enum_tc("IDL:VisualAttributes/Shape:1.0", "Shape", _0RL_enumMember_VisualAttributes_mShape, 8, &_0RL_tcTrack);
#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace VisualAttributes { 
  const ::CORBA::TypeCode_ptr _tc_Shape = _0RL_tc_VisualAttributes_mShape;
} 
#else
const ::CORBA::TypeCode_ptr VisualAttributes::_tc_Shape = _0RL_tc_VisualAttributes_mShape;
#endif

static const char* _0RL_enumMember_VisualAttributes_mSize[] = { "TINY", "SMALL", "BIG", "LARGE", "HUGE", "UNKNOW_SIZE" };
static CORBA::TypeCode_ptr _0RL_tc_VisualAttributes_mSize = CORBA::TypeCode::PR_enum_tc("IDL:VisualAttributes/Size:1.0", "Size", _0RL_enumMember_VisualAttributes_mSize, 6, &_0RL_tcTrack);
#if defined(HAS_Cplusplus_Namespace) && defined(_MSC_VER)
// MSVC++ does not give the constant external linkage otherwise.
namespace VisualAttributes { 
  const ::CORBA::TypeCode_ptr _tc_Size = _0RL_tc_VisualAttributes_mSize;
} 
#else
const ::CORBA::TypeCode_ptr VisualAttributes::_tc_Size = _0RL_tc_VisualAttributes_mSize;
#endif

static void _0RL_VisualAttributes_mColour_marshal_fn(cdrStream& _s, void* _v)
{
  VisualAttributes::Colour* _p = (VisualAttributes::Colour*)_v;
  *_p >>= _s;
}
static void _0RL_VisualAttributes_mColour_unmarshal_fn(cdrStream& _s, void*& _v)
{
  VisualAttributes::Colour* _p = (VisualAttributes::Colour*)_v;
  *_p <<= _s;
}

void operator<<=(::CORBA::Any& _a, VisualAttributes::Colour _s)
{
  _a.PR_insert(_0RL_tc_VisualAttributes_mColour,
               _0RL_VisualAttributes_mColour_marshal_fn,
               &_s);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, VisualAttributes::Colour& _s)
{
  return _a.PR_extract(_0RL_tc_VisualAttributes_mColour,
                       _0RL_VisualAttributes_mColour_unmarshal_fn,
                       &_s);
}

static void _0RL_VisualAttributes_mColourWithConfidence_marshal_fn(cdrStream& _s, void* _v)
{
  VisualAttributes::ColourWithConfidence* _p = (VisualAttributes::ColourWithConfidence*)_v;
  *_p >>= _s;
}
static void _0RL_VisualAttributes_mColourWithConfidence_unmarshal_fn(cdrStream& _s, void*& _v)
{
  VisualAttributes::ColourWithConfidence* _p = new VisualAttributes::ColourWithConfidence;
  *_p <<= _s;
  _v = _p;
}
static void _0RL_VisualAttributes_mColourWithConfidence_destructor_fn(void* _v)
{
  VisualAttributes::ColourWithConfidence* _p = (VisualAttributes::ColourWithConfidence*)_v;
  delete _p;
}

void operator<<=(::CORBA::Any& _a, const VisualAttributes::ColourWithConfidence& _s)
{
  VisualAttributes::ColourWithConfidence* _p = new VisualAttributes::ColourWithConfidence(_s);
  _a.PR_insert(_0RL_tc_VisualAttributes_mColourWithConfidence,
               _0RL_VisualAttributes_mColourWithConfidence_marshal_fn,
               _0RL_VisualAttributes_mColourWithConfidence_destructor_fn,
               _p);
}
void operator<<=(::CORBA::Any& _a, VisualAttributes::ColourWithConfidence* _sp)
{
  _a.PR_insert(_0RL_tc_VisualAttributes_mColourWithConfidence,
               _0RL_VisualAttributes_mColourWithConfidence_marshal_fn,
               _0RL_VisualAttributes_mColourWithConfidence_destructor_fn,
               _sp);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, VisualAttributes::ColourWithConfidence*& _sp)
{
  return _a >>= (const VisualAttributes::ColourWithConfidence*&) _sp;
}
::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, const VisualAttributes::ColourWithConfidence*& _sp)
{
  void* _v;
  if (_a.PR_extract(_0RL_tc_VisualAttributes_mColourWithConfidence,
                    _0RL_VisualAttributes_mColourWithConfidence_unmarshal_fn,
                    _0RL_VisualAttributes_mColourWithConfidence_marshal_fn,
                    _0RL_VisualAttributes_mColourWithConfidence_destructor_fn,
                    _v)) {
    _sp = (const VisualAttributes::ColourWithConfidence*)_v;
    return 1;
  }
  return 0;
}

static void _0RL_VisualAttributes_mShape_marshal_fn(cdrStream& _s, void* _v)
{
  VisualAttributes::Shape* _p = (VisualAttributes::Shape*)_v;
  *_p >>= _s;
}
static void _0RL_VisualAttributes_mShape_unmarshal_fn(cdrStream& _s, void*& _v)
{
  VisualAttributes::Shape* _p = (VisualAttributes::Shape*)_v;
  *_p <<= _s;
}

void operator<<=(::CORBA::Any& _a, VisualAttributes::Shape _s)
{
  _a.PR_insert(_0RL_tc_VisualAttributes_mShape,
               _0RL_VisualAttributes_mShape_marshal_fn,
               &_s);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, VisualAttributes::Shape& _s)
{
  return _a.PR_extract(_0RL_tc_VisualAttributes_mShape,
                       _0RL_VisualAttributes_mShape_unmarshal_fn,
                       &_s);
}

static void _0RL_VisualAttributes_mSize_marshal_fn(cdrStream& _s, void* _v)
{
  VisualAttributes::Size* _p = (VisualAttributes::Size*)_v;
  *_p >>= _s;
}
static void _0RL_VisualAttributes_mSize_unmarshal_fn(cdrStream& _s, void*& _v)
{
  VisualAttributes::Size* _p = (VisualAttributes::Size*)_v;
  *_p <<= _s;
}

void operator<<=(::CORBA::Any& _a, VisualAttributes::Size _s)
{
  _a.PR_insert(_0RL_tc_VisualAttributes_mSize,
               _0RL_VisualAttributes_mSize_marshal_fn,
               &_s);
}

::CORBA::Boolean operator>>=(const ::CORBA::Any& _a, VisualAttributes::Size& _s)
{
  return _a.PR_extract(_0RL_tc_VisualAttributes_mSize,
                       _0RL_VisualAttributes_mSize_unmarshal_fn,
                       &_s);
}

