/** @file Twist.h
 * 
 * Implementation of twists.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_TWIST_H_
#define SYSTEM_TWIST_H_

//------------------------------------------------------------------------------

#include "system/Mat33.h"

//------------------------------------------------------------------------------

namespace msk {

//------------------------------------------------------------------------------
class Mat34;
class Twist;

extern void TwistToMat34(Mat34& m, const Twist& t, Real theta);
extern void Mat34ToTwist(Twist& t, Real& theta, const Mat34& m);
extern void AdjointTransform(Twist& dst, const Twist& t, const Mat34& m);
extern void AdjointInverseTransform(Twist& dst, const Twist& t, const Mat34& m);
extern void AdjointTransposedTransform(Twist& dst, const Twist& t, const Mat34& m);

/** Twist coordinates of rigid body transformation (generator of exponential mapping: se(3) -> SE(3)).
*/
class Twist {
	friend class Mat34;
	
	friend void TwistToMat34(class Mat34& m, const Twist& t, Real theta);
	friend void Mat34ToTwist(Twist& t, Real& theta, const class Mat34& m);
	friend void AdjointTransform(Twist& dst, const Twist& t, const class Mat34& m);
	friend void AdjointInverseTransform(Twist& dst, const Twist& t, const class Mat34& m);
	friend void AdjointTransposedTransform(Twist& dst, const Twist& t, const class Mat34& m);

protected:
	/** linear component */
	Vec3 v;
	/** angular component */
	Vec3 w;

public:
	/** Default constructor does not do any initialisation.
	*/
	inline Twist() {}

	/** Initialise with the all parameters unrolled.
	*/
	inline Twist(Real v1, Real v2, Real v3, Real w1, Real w2, Real w3) :
		v(v1, v2, v3), w(w1, w2, w3)
	{}

	/** Initialise with v, w.
	*/
	inline Twist(const Vec3& v, const Vec3& w) : v(v), w(w) {}

	/** Creates twist from screw motion.
	*/
	inline Twist(Real h, const Vec3& anchor, const Vec3& axis) {
		fromScrew(h, anchor, axis);
	}

	/** Creates twist from homogeneous matrix.
	*	@param	m	homogeneous matrix to extract twist from.
	*/
	inline Twist(Real& theta, const class Mat34 &m) {
		msk::Mat34ToTwist(*this, theta, m);
	}

	/** Copy constructor.
	*/
	inline Twist(const Twist& t) : v(t.v), w(t.w) {}

	/** Set the members of the twist.
	*/
	inline void set(Real v1, Real v2, Real v3, Real w1, Real w2, Real w3) {
		this->v.set(v1, v2, v3);
		this->w.set(w1, w2, w3);
	}

	/** Set the members of the twist
	*/
	inline void set(const Vec3& v, const Vec3& w) {
		this->v.set(v);
		this->w.set(w);
	}

	/** Sets linear component v.
	*/
	inline void setV(Real v1, Real v2, Real v3) {
		this->v.set(v1, v2, v3);
	}

	/** Sets linear component v.
	*/
	inline void setV(const Vec3& v) {
		this->v.set(v);
	}

	/** Sets angular component w.
	*/
	inline void setW(Real w1, Real w2, Real w3) {
		this->w.set(w1, w2, w3);
	}

	/** Sets angular component w.
	*/
	inline void setW(const Vec3& w) {
		this->w.set(w);
	}

	/** Set the members of the quaterion from array
	*/
	inline void set(const F32 t[]) {
		v.set(&t[0]);
		w.set(&t[3]);
	}

	/** Set the members of the quaterion from array
	*/
	inline void set(const F64 t[]) {
		v.set(&t[0]);
		w.set(&t[3]);
	}

	/** sets the twist to zero
	*/
	inline void setZero() {
		v.setZero();
		w.setZero();
	}

	/** Set twist to the identity transformation
	*/
	inline void setId() {
		setZero();
	}

	inline void get(F32 t[]) const {
		v.get(&t[0]);
		w.get(&t[3]);
	}

	inline void get(F64 t[]) const {
		v.get(&t[0]);
		w.get(&t[3]);
	}

	/** Returns linear component v.
	*/
	inline const Vec3& getV() const {
		return v;
	}
	inline Vec3& getV() {
		return v;
	}

	/** Returns angular component w.
	*/
	inline const Vec3& getW() const {
		return w;
	}
	inline Vec3& getW() {
		return w;
	}

	/** Creates twist from homogeneous matrix.
	*/
	inline void fromMat34(Real& theta, const class Mat34& m) {
		msk::Mat34ToTwist(*this, theta, m);
	}

	/** Creates homogeneous matrix from twist.
	*/
	inline void toMat34(class Mat34& m, Real theta) const {
		msk::TwistToMat34(m, *this, theta);
	}

	/** Creates twist from a screw motion.
	*/
	inline void fromScrew(Real h, const Vec3& anchor, const Vec3& axis) {
		if (Math::isFinite(h)) {
			// Rotation and translation
			// [ v, w ] = [ -axis x anchor + h axis = anchor x axis + h axis, axis ]
			v.cross(anchor, axis);
			w.multiply(h, axis); // temp
			v.add(v, w);
			w = axis;
		}
		else {
			// Pure translation
			// [ v, w ] = [ axis, 0 ]
			v = axis;
			w.setZero();
		}
	}

	/** Creates screw motion from twist.
	*/
	inline void toScrew(Vec3& anchor, Vec3& axis) const {
		getAxis(anchor, axis);
	}

	/** Returns pitch of the corresponding screw motion.
	*/
	inline Real getPitch() const {
		return w.isZero() ? REAL_PINF : w.dot(v) / w.magnitudeSquared();
	}

	/** Returns magnitude of the corresponding screw motion.
	*/
	inline Real getMagnitude() const {
		return w.isZero() ? v.magnitude() : w.magnitude();
	}

	/** Returns anchor and axis of the corresponding screw motion.
	*/
	inline void getAxis(Vec3& anchor, Vec3& axis) const {
		if (w.isZero()) {
			anchor.setZero();
			axis = v;
		}
		else {
			anchor.cross(w, v);
			anchor.multiply(REAL_ONE / w.magnitudeSquared(), anchor);
			axis = w;
		}
	}

	/** Adjoint transformation.
	*/
	inline void adjointTransform(const Twist& t, const class Mat34& m) {
		msk::AdjointTransform(*this, t, m);
	}

	/** Inverse adjoint transformation.
	*/
	inline void adjointInverseTransform(const Twist& t, const class Mat34& m) {
		msk::AdjointInverseTransform(*this, t, m);
	}

	/** Transposed adjoint transformation.
	*/
	inline void adjointTransposedTransform(const Twist& t, const class Mat34& m) {
		msk::AdjointTransposedTransform(*this, t, m);
	}

	/** maps to the closest unit twist.
	*/
	inline Real normalise() {
		return w.isZero() ? v.normalise() : w.normalise();
	}

	/** Test if the twist is the identity transformation.
	*/
	inline bool isIdentityTransformation() const {
		return v.isZero() && w.isZero();
	}

	/** this = a + b
	*/
	inline void add(const Twist& a, const Twist& b) {
		v.add(a.v, b.v);
		w.add(a.w, b.w);
	}

	/** this = a - b
	*/
	inline void subtract(const Twist& a, const Twist& b) {
		v.subtract(a.v, b.v);
		w.subtract(a.w, b.w);
	}

	/** this = s * a;
	*/
	inline void multiply(Real s, const Twist& a) {
		v.multiply(s, a.v);
		w.multiply(s, a.w);
	}

	/** this = b * a;
	*/
	inline void arrayMultiply(const Twist& a, const Twist& b) {
		v.arrayMultiply(a.v, b.v);
		w.arrayMultiply(a.w, b.w);
	}

	/** this = s * a + b;
	*/
	inline void multiplyAdd(Real s, const Twist& a, const Twist& b) {
		v.multiplyAdd(s, a.v, b.v);
		w.multiplyAdd(s, a.w, b.w);
	}

	/** tests for exact zero twist
	*/
	inline bool isZero() const {
		return !v.isZero() || !w.isZero();
	}

	/** tests for exact Id twist
	*/
	inline bool isId() const {
		return isZero();
	}

	/** tests for finite twist
	*/
	inline bool isFinite() const {
		return v.isFinite() && w.isFinite();
	}

	/**	Assignment operator.
	*/
	inline Twist& operator = (const Twist& t) {
		v = t.v;	w = t.w;
		return *this;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*SYSTEM_TWIST_H_*/
