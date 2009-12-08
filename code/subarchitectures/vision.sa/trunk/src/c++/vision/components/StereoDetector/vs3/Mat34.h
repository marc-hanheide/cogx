/** @file Mat34.h
 * 
 * Mathematical routines.
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 *
 * @version 1.0
 *
 */

#ifndef SYSTEM_MAT34_H_
#define SYSTEM_MAT34_H_

//------------------------------------------------------------------------------

#include "system/Wrench.h"

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

/** Homogeneous representation of SE(3) rigid body transformations.
*/
class Mat34 {
public:
	/** rotation matrix	*/
	Mat33 R;
	/** translation	*/
	Vec3 p;

	/** Default constructor does not do any initialisation.
	*/
	inline Mat34() {
	}

	/** Creates matrix from rotation matrix and translation vector
	*/
	inline Mat34(const Mat33& rot, const Vec3& trn) : R(rot), p(trn) {
	}

	/** Creates matrix from twist coordinates
	*/
	inline Mat34(const Twist& t, Real theta) {
		msk::TwistToMat34(*this, t, theta);
	}

	/** Copy constructor.
	*/
	inline Mat34(const Mat34& m) {
		R = m.R;	p = m.p;
	}

	/** set the matrix given a column matrix
	*/
	inline void setColumn44(const F32 m[]) {
		R.setColumn44(m);
		p.v1 = (Real)m[12];
		p.v2 = (Real)m[13];
		p.v3 = (Real)m[14];
	}

	/** set the matrix given a column matrix
	*/
	inline void setColumn44(const F64 m[]) {
		R.setColumn44(m);
		p.v1 = (Real)m[12];
		p.v2 = (Real)m[13];
		p.v3 = (Real)m[14];
	}

	/** retrieve the matrix in a column format
	*/
	inline void getColumn44(F32 m[]) const {
		R.getColumn44(m);
		m[12] = (F32)p.v1;
		m[13] = (F32)p.v2;
		m[14] = (F32)p.v3;
		m[3] = m[7] = m[11] = ZERO_F32;
		m[15] = ONE_F32;
	}

	/** retrieve the matrix in a column format
	*/
	inline void getColumn44(F64 m[]) const {
		R.getColumn44(m);
		m[12] = (F64)p.v1;
		m[13] = (F64)p.v2;
		m[14] = (F64)p.v3;
		m[3] = m[7] = m[11] = ZERO_F64;
		m[15] = ONE_F64;
	}

	/** set the matrix given a row matrix.
	*/
	inline void setRow44(const F32 m[]) {
		R.setRow44(m);
		p.v1 = (Real)m[3];
		p.v2 = (Real)m[7];
		p.v3 = (Real)m[11];
	}

	/** set the matrix given a row matrix.
	*/
	inline void setRow44(const F64 m[]) {
		R.setRow44(m);
		p.v1 = (Real)m[3];
		p.v2 = (Real)m[7];
		p.v3 = (Real)m[11];
	}

	/** retrieve the matrix in a row format.
	*/
	inline void getRow44(F32 m[]) const {
		R.getRow44(m);
		m[3] = (F32)p.v1;
		m[7] = (F32)p.v2;
		m[11] = (F32)p.v3;
		m[12] = m[13] = m[14] = ZERO_F32;
		m[15] = ONE_F32;
	}

	/** retrieve the matrix in a row format.
	*/
	inline void getRow44(F64 m[]) const {
		R.getRow44(m);
		m[3] = (F64)p.v1;
		m[7] = (F64)p.v2;
		m[11] = (F64)p.v3;
		m[12] = m[13] = m[14] = ZERO_F64;
		m[15] = ONE_F64;
	}

	inline void setZero() {
		R.setZero();
		p.setZero();
	}

	inline void setId() {
		R.setId();
		p.setZero();
	}

	/** Creates matrix from twist coordinates
	*/
	inline void fromTwist(const Twist& t, Real theta) {
		msk::TwistToMat34(*this, t, theta);
	}

	/** Creates matrix from wrench
	*/
	inline void fromWrench(const Wrench& w, Real magnitude) {
		msk::TwistToMat34(*this, w, magnitude);
	}

	/** Creates twist from matrix
	*/
	inline void toTwist(Twist& t, Real& theta) const {
		msk::Mat34ToTwist(t, theta, *this);
	}

	/** Creates wrench from matrix
	*/
	inline void toWrench(Wrench& w, Real& magnitude) const {
		msk::Mat34ToTwist(w, magnitude, *this);
	}
	
	/** Adjoint transformation.
	*/
	inline void adjointTransform(Twist& dst, const Twist& t) const {
		msk::AdjointTransform(dst, t, *this);
	}

	/** Inverse adjoint transformation.
	*/
	inline void adjointInverseTransform(Twist& dst, const Twist& t) const {
		msk::AdjointInverseTransform(dst, t, *this);
	}

	/** Transposed adjoint transformation.
	*/
	inline void adjointTransposedTransform(Twist& dst, const Twist& t) const {
		msk::AdjointTransposedTransform(dst, t, *this);
	}

	/** Creates special Euklidean SE(3) matrix from twist (wedge operator ^).
	*/
	inline void twistToSpecial(const Twist &twist) {
		R.axisToSkew(twist.w);
		p = twist.v;
	}

	/** Creates twist from special Euklidean SE(3) matrix (vee operator v).
	*/
	inline void specialToTwist(Twist &twist) const {
		R.skewToAxis(twist.w);
		twist.v = p;
	}

	/** Returns true for identity matrix
	*/
	inline bool isIdentity() const {
		return R.isIdentity() && p.isZero();
	}

	/** Returns true if all elems are finite.
	*/
	inline bool isFinite() const {
		return R.isFinite() && p.isFinite();
	}

	/** Assigns inverse to m: [ inv(R) , inv(R) * -p ].
	Returns false if singular (i.e. if no inverse exists), setting dest to identity.
	*/
	inline bool getInverse(Mat34& m) const {
		bool bRet = R.getInverse(m.R);
		m.p.multiply(-REAL_ONE, p);
		m.R.multiply(m.p, m.p); 
		return bRet;
	}

	/** Same as #getInverse(), for orthonormal m: [ RT , RT * -p ].
	*/
	inline bool getInverseRT(Mat34& m) const {
		m.R.setTransposed(R);
		m.p.multiply(-REAL_ONE, p);
		m.R.multiply(m.p, m.p); 
		return true;
	}

	/** a = this * b
	*/
	inline void multiply(Vec3& a, const Vec3& b) const {
		// a = R * b + p;
		R.multiply(a, b);
		a.add(a, p);
	}

	/** this = a * b: [aR, ap] * [bR, bp] = [aR * bR, aR * bp + ap].
	*/
	inline void multiply(const Mat34& a, const Mat34& b) {
		Vec3 tmp;

		a.R.multiply(tmp, b.p);
		p.add(tmp, a.p);
		R.multiply(a.R, b.R);
	}

	/** a = inverse(this) * b, assumes M is rotation matrix
	*/
	inline void multiplyByInverseRT(Vec3& a, const Vec3& b) const {
		//a = RT * b - RT * p = RT * (b - p)
		a.subtract(b, p);
		R.multiplyByTranspose(a, a);
	}

	/**	Assignment operator.
	*/
	inline const Mat34& operator = (const Mat34 &m) {
		R = m.R;	p = m.p;
		return *this;
	}

	/** operator wrapper for multiply
	*/
	inline Vec3 operator * (const Vec3& a) const {
		Vec3 tmp;
		multiply(tmp, a);
		return tmp;
	}

	/** operator wrapper for multiply
	*/
	inline Mat34 operator * (const Mat34& b) const {
		Mat34 a;
		a.multiply(*this, b);
		return a;
	}

	/** operator wrapper for multiplyByInverseRT
	*/
	inline Vec3 operator % (const Vec3& a) const {
		Vec3 tmp;
		multiplyByInverseRT(tmp, a);
		return tmp;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*SYSTEM_MAT34_H_*/
