/** @file Constants.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Defs/Constants.h>
#include <limits.h>
#include <float.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const numeric_const<F32>::FConst numeric_const<F32>::IEEE_POS_INF = {0x7f800000};
const numeric_const<F32>::FConst numeric_const<F32>::IEEE_NEG_INF = {0xff800000};
const F32 numeric_const<F32>::MIN = FLT_MIN;
const F32 numeric_const<F32>::MAX = FLT_MAX;
const F32 numeric_const<F32>::POS_INF = numeric_const<F32>::IEEE_POS_INF.f32;
const F32 numeric_const<F32>::NEG_INF = numeric_const<F32>::IEEE_NEG_INF.f32;
const F32 numeric_const<F32>::INF = numeric_const<F32>::IEEE_POS_INF.f32;
const F32 numeric_const<F32>::EPS = FLT_EPSILON;

const numeric_const<F64>::FConst numeric_const<F64>::IEEE_POS_INF = {0x7ff0000000000000ULL};
const numeric_const<F64>::FConst numeric_const<F64>::IEEE_NEG_INF = {0xfff0000000000000ULL};
const F64 numeric_const<F64>::MIN = DBL_MIN;
const F64 numeric_const<F64>::MAX = DBL_MAX;
const F64 numeric_const<F64>::POS_INF = numeric_const<F64>::IEEE_POS_INF.f64;
const F64 numeric_const<F64>::NEG_INF = numeric_const<F64>::IEEE_NEG_INF.f64;
const F64 numeric_const<F64>::INF = numeric_const<F64>::IEEE_POS_INF.f64;
const F64 numeric_const<F64>::EPS = DBL_EPSILON;

//------------------------------------------------------------------------------
