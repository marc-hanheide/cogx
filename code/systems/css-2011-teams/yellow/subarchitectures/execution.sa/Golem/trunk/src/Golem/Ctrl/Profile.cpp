/** @file Profile.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Profile.h>
#include <Golem/Ctrl/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Profile::Profile() {
}

bool Profile::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgProfileInvalidDesc(Message::LEVEL_CRIT, "Profile::create(): Invalid description");

	distance.pProfile = this;
	velocity.pProfile = this;
	acceleration.pProfile = this;

	begin = desc.begin;
	end = desc.end;
	
	root = desc.root;
	extremum = desc.extremum;
	derivative = desc.derivative;

	return true;
}

//------------------------------------------------------------------------------

Real Profile::getLength() const {
	return getDistance(end) - getDistance(begin);
}

bool Profile::getDistanceInverse(Real &t, Real distance) const {
	return root->findSingle(t, distance, this->distance, begin, end);
}

void Profile::getDistanceExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = begin;
	tmax = end;
	min = getDistance(begin);
	max = getDistance(end);
}

Real Profile::getVelocity(Real t) const {
	return derivative->findFirst(t, distance);
}

bool Profile::getVelocityInverse(Seq &t, Real velocity) const {
	return root->findMultiple(t, velocity, this->velocity, begin, end);
}

void Profile::getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	extremum->findGlobal(tmin, min, velocity, begin, end, true);
	extremum->findGlobal(tmax, max, velocity, begin, end, false);
}

Real Profile::getAcceleration(Real t) const {
	return derivative->findSecond(t, distance);
}

bool Profile::getAccelerationInverse(Seq &t, Real acceleration) const {
	return root->findMultiple(t, acceleration, this->acceleration, begin, end);
}

void Profile::getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	extremum->findGlobal(tmin, min, acceleration, begin, end, true);
	extremum->findGlobal(tmax, max, acceleration, begin, end, false);
}

//------------------------------------------------------------------------------

bool Polynomial1::create(const Desc& desc) {
	Profile::create(desc); // throws

	length = desc.length;
	return true;
}

//------------------------------------------------------------------------------

bool Polynomial4::create(const Desc& desc) {
	Profile::create(desc); // throws
	
	a[0] = desc.a[0];
	a[1] = desc.a[1];
	a[2] = desc.a[2];
	a[3] = desc.a[3];
	return true;
}

void Polynomial4::scaleDuration(Real duration) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	set2dvdv(
		t0,
		t0 + duration,
		Polynomial4::getDistance(t0), Polynomial4::getVelocity(t0),
		Polynomial4::getDistance(t1), Polynomial4::getVelocity(t1)
	);
}

void Polynomial4::scaleLength(Real length) {
	const Real t0 = getBegin();
	const Real t1 = getEnd();
	const Real d0 = Polynomial4::getDistance(t0);
	set2dvdv(
		t0,
		t1,
		d0, Polynomial4::getVelocity(t0),
		d0 + length, Polynomial4::getVelocity(t1)
	);
}

Real Polynomial4::getDistance(Real t) const {
	t -= getBegin();
	return a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t;
}

Real Polynomial4::getVelocity(Real t) const {
	t -= getBegin();
	return a[1] + Real(2.0)*a[2]*t + Real(3.0)*a[3]*t*t;
}

void Polynomial4::getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial4::getVelocity(tmin);
	max = Polynomial4::getVelocity(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
	
	if (Math::equals(a[3], REAL_ZERO, REAL_EPS))
		return;
	Real t = - a[2]/(Real(3.0)*a[3]) + getBegin();
	if (t < getBegin() || t > getEnd())
		return;

	if (a[3] > Real(0.0)) {
		tmin = t;
		min = Polynomial4::getVelocity(tmin);
	}
	else {
		tmax = t;
		max = Polynomial4::getVelocity(tmax);
	}
}

Real Polynomial4::getAcceleration(Real t) const {
	t -= getBegin();
	return Real(2.0)*a[2] + Real(6.0)*a[3]*t;
}

void Polynomial4::getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const {
	tmin = getBegin();
	tmax = getEnd();
	min = Polynomial4::getAcceleration(tmin);
	max = Polynomial4::getAcceleration(tmax);
	if (min > max) {
		std::swap(tmin, tmax);
		std::swap(min, max);
	}
}

void Polynomial4::set2dvdv(Real t0, Real t1, Real d0, Real v0, Real d1, Real v1) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	
	Real t10 = t1 - t0, t0d = t0*t0, t1d = t1*t1;

	Real mn1 = (d1 - d0)/t10;
	Real m1 = v0 - mn1, n1 = v1 - mn1;

	Real mn2 = (t1d - t0d)/t10;
	Real m2 = Real(2.0)*t0 - mn2, n2 = Real(2.0)*t1 - mn2;
	
	Real mn3 = (t1d*t1 - t0d*t0)/t10;
	Real m3 = Real(3.0)*t0d - mn3, n3 = Real(3.0)*t1d - mn3;

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0*t0;
	a[0] = d0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set2dvav(Real t0, Real t1, Real d0, Real v0, Real a0, Real v1) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	
	Real t10 = t1 - t0;
	Real t0d = t0*t0, t1d = t1*t1;

	a[3] = ((v1 - v0) - a0*t10)/(Real(3.0)*(t1d - t0d) - Real(6.0)*t0*t10);
	a[2] = Real(0.5)*a0 - Real(3.0)*a[3]*t0;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0d;
	a[0] = d0 - a[1]*t0 - a[2]*t0d - a[3]*t0d*t0;
}

void Polynomial4::set3dvdd(Real t0, Real t1, Real t2, Real d0, Real v0, Real d1, Real d2) {
	setBegin(t0);
	setEnd(t1);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	
	Real t0d, t1d, t2d;

	Real m1 = d1 - d0 + v0*(t0 - t1), n1 = d2 - d0 + v0*(t0 - t2);
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	Real m2 = t1d + t0d - Real(2.0)*t0*t1, n2 = t2d + t0d - Real(2.0)*t0*t2;
	
	t1d *= t1; t2d *= t2;
	Real m3 = t1d + t0d*(Real(2.0)*t0 - Real(3.0)*t1), n3 = t2d + t0d*(Real(2.0)*t0 - Real(3.0)*t2);

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v0 - Real(2.0)*a[2]*t0 - Real(3.0)*a[3]*t0*t0;
	a[0] = d0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set3dddv(Real t0, Real t1, Real t2, Real d0, Real d1, Real d2, Real v2) {
	setBegin(t1);
	setEnd(t2);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	
	Real t0d, t1d, t2d;

	Real m1 = d0 - d2 + v2*(t2 - t0), n1 = d1 - d2 + v2*(t2 - t1);
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2;
	Real m2 = t0d + t2d - Real(2.0)*t2*t0, n2 = t1d + t2d - Real(2.0)*t2*t1;
	
	t0d *= t0; t1d *= t1;
	Real m3 = t0d + t2d*(Real(2.0)*t2 - Real(3.0)*t0), n3 = t1d + t2d*(Real(2.0)*t2 - Real(3.0)*t1);

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = v2 - Real(2.0)*a[2]*t2 - Real(3.0)*a[3]*t2*t2;
	a[0] = d0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

void Polynomial4::set4dddd(Real t0, Real t1, Real t2, Real t3, Real d0, Real d1, Real d2, Real d3) {
	setBegin(t1);
	setEnd(t2);
	t0 -= getBegin();
	t1 -= getBegin();
	t2 -= getBegin();
	t3 -= getBegin();
	
	Real t0d, t1d, t2d, t3p;
	Real t10 = t1 - t0, t20 = t2 - t0, t30 = t3 - t0;
	
	Real mn1 = (d1 - d0)/t10;
	Real m1 = (d2 - d0)/t20 - mn1, n1 = (d3 - d0)/t30 - mn1;
	
	t0d = t0*t0; t1d = t1*t1; t2d = t2*t2; t3p = t3*t3;
	Real mn2 = (t1d - t0d)/t10;
	Real m2 = (t2d - t0d)/t20 - mn2, n2 = (t3p - t0d)/t30 - mn2;
	
	t0d *= t0; t1d *= t1; t2d *= t2; t3p *= t3;
	Real mn3 = (t1d - t0d)/t10;
	Real m3 = (t2d - t0d)/t20 - mn3, n3 = (t3p - t0d)/t30 - mn3;

	a[3] = (n1*m2 - m1*n2)/(n3*m2 - m3*n2);
	a[2] = (m1 - a[3]*m3)/m2;
	a[1] = mn1 - a[2]*mn2 - a[3]*mn3;
	a[0] = d0 - a[1]*t0 - a[2]*t0*t0 - a[3]*t0*t0*t0;
}

//------------------------------------------------------------------------------

