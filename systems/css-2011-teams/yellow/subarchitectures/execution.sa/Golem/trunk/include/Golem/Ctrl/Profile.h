/** @file Profile.h
 * 
 * Trajectory profile interface and library.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_PROFILE_H_
#define _GOLEM_CTRL_PROFILE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Root.h>
#include <Golem/Math/Extremum.h>
#include <Golem/Math/Derivative.h>
#include <Golem/Defs/Desc.h>

namespace golem {

//------------------------------------------------------------------------------

/** Abstract class representing trajectory profiles of moving objects. */
class Profile {
public:
	typedef shared_ptr<Profile> Ptr;
	typedef Root<Real>::Seq Seq;

	/** Profile description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Beginning in seconds */
		Real begin;
		/** End in seconds */
		Real end;
		
		/** Root finder */
		Root<Real>::Ptr root;
		/** Extremum finder */
		Extremum<Real>::Ptr extremum;
		/** Derivative finder */
		Derivative<Real>::Ptr derivative;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Sets the profile parameters to the default values */
		virtual void setToDefault() {
			begin = Real(0.0);
			end = Real(1.0);

			root.reset(new RootSecant<Real>);
			extremum.reset(new ExtremumGold<Real>);
			derivative.reset(new DerivativeSecant<Real>);
		}

		/** Checks if the profile description is valid. */
		virtual bool isValid() const {
			if (root == NULL || extremum == NULL || derivative == NULL)
				return false;

			return true;
		}
		
		/** Creates Profile given the description object. 
		* @return		pointer to Profile, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Profile::Ptr create() const = 0;
	};

private:
	struct Distance : public Function<Real> {
		const Profile *pProfile;
		virtual Real get(Real x) const {
			return pProfile->getDistance(x);
		}
	};

	struct Velocity : public Function<Real> {
		const Profile *pProfile;
		virtual Real get(Real x) const {
			return pProfile->getVelocity(x);
		}
	};

	struct Acceleration : public Function<Real> {
		const Profile *pProfile;
		virtual Real get(Real x) const {
			return pProfile->getAcceleration(x);
		}
	};

	/** Beginning in seconds */
	Real begin;
	/** End in seconds */
	Real end;

	/** Root finder */
	Root<Real>::Ptr root;
	/** Extremum finder */
	Extremum<Real>::Ptr extremum;
	/** Derivative finder */
	Derivative<Real>::Ptr derivative;
	
	Distance distance;
	Velocity velocity;
	Acceleration acceleration;

public:
	Profile();

	/** Descructor */
	virtual ~Profile() {}

	/** Creates Profile from the description. 
	* @param desc	Profile description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);
	
	/** Returns trajectory profile beginning. */
	inline Real getBegin() const {
		return begin;
	}

	/** Sets trajectory profile beginning. */
	inline void setBegin(Real begin) {
		this->begin = begin;
	}
	
	/** Returns trajectory profile end. */
	inline Real getEnd() const {
		return end;
	}

	/** Sets trajectory profile end. */
	inline void setEnd(Real end) {
		this->end = end;
	}
	
	/** Returns trajectory duration. */
	inline Real getDuration() const {
		return end - begin;
	}

	/** Re-scales the profile with the specified duration time. */
	virtual void scaleDuration(Real duration = Real(1.0)) = 0;

	/** Returns trajectory profile length. */
	virtual Real getLength() const;

	/** Scales the profile with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = Real(1.0)) = 0;
	
	/** Distance profile f:time -> distance */
	virtual Real getDistance(Real t) const = 0;

	/** Inverse trajectory profile f:distance -> time.
	*	There can exist no more than one solution (travelled distance cannot decrease).
	*/
	virtual bool getDistanceInverse(Real &t, Real distance) const;

	/** Profile minima and maximum */
	virtual void getDistanceExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
	
	/** Velocity profile f:time -> velocity */
	virtual Real getVelocity(Real t) const;

	/** Inverse trajectory profile f:velocity -> time.
	*	There can exist more than one solution.
	*/
	virtual bool getVelocityInverse(Seq &t, Real velocity) const;

	/** Velocity minimum and maximum */
	virtual void getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Acceleration profile f:time -> acceleration */
	virtual Real getAcceleration(Real t) const;

	/** Inverse trajectory profile f:acceleration -> time.
	*	There can exist more than one solution.
	*/
	virtual bool getAccelerationInverse(Seq &t, Real acceleration) const;

	/** Velocity minimum and maximum */
	virtual void getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
};

//------------------------------------------------------------------------------

/** Constant velocity profile. 
 */ 
class Polynomial1 : public Profile {
public:
	/** Profile description */
	class Desc : public Profile::Desc {
	public:
		/** Length */
		Real length;

		Desc() {
			Desc::setToDefault();
		}

		/** Sets the profile parameters to the default values */
		virtual void setToDefault() {
			Profile::Desc::setToDefault();

			length = Real(1.0);
		}

		/** Checks if the profile description is valid. */
		virtual bool isValid() const {
			if (!Profile::Desc::isValid())
				return false;
			if (length <= REAL_ZERO)
				return false;

			return true;
		}
		
		/** Sets the profile parameters to the default values */
		CREATE_FROM_OBJECT_DESC0(Polynomial1, Profile::Ptr)
	};

protected:
	/** Length */
	Real length;

public:
	/** Creates a default profile of unit length and unit duration */
	bool create(const Desc& desc);
	
	/** Re-scales the profile with the specified duration time. */
	virtual void scaleDuration(Real duration = REAL_ONE) {
		setEnd(getBegin() + duration);
	}
	
	/** Scales the profile with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = REAL_ONE) {
		this->length = length;
	}

	/** Profile minimum and maximum */
	virtual Real getDistance(Real t) const {
		return t*length/getDuration();
	}
	
	/** Velocity profile f:time -> velocity */
	virtual Real getVelocity(Real t) const {
		return length/getDuration();
	}

	/** Acceleration profile f:time -> acceleration */
	virtual Real getAcceleration(Real t) const {
		return Real(0.0);
	}
};

//------------------------------------------------------------------------------

/** 3rd-degree polynomial profile. 
 * <p>
 * f(t) = a3*t<sup>3</sup> + a2*t<sup>2</sup> + a1*t + a0
 * <p>
 * A polynomial f() within a single window can be unambiguously specified by 
 * positions/distances (d0, d1) and velocities (v0, v1) on its boundaries 
 * at the time t0 and t1 suitably, i.e. there are 4 equations:
 * <p>
 * f(t0) = d0
 * <p>
 * f'(t0) = v0
 * <p>
 * f(t1) = d1
 * <p>
 * f'(t1) = v1
 * <p>
 */ 
class Polynomial4 : public Profile {
public:
	/** Profile description */
	class Desc : public Profile::Desc {
	public:
		/** Polynomial coefficients */
		Real a[4];
		
		Desc() {
			Desc::setToDefault();
		}
		
		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the profile parameters to the default values */
		virtual void setToDefault() {
			Profile::Desc::setToDefault();

			// The profile of a unit duration and a unit length (for {begin, end} == {0, 1}):
			// f(t) = 3*t^2*(1 - 2/3*t) = 3*t^2 - 2*t^3; f(0) = 0, f(1) = 1
			a[0] = Real(0.0);
			a[1] = Real(0.0);
			a[2] = Real(3.0);
			a[3] = Real(-2.0);
		}

		/** Checks if the profile description is valid. */
		virtual bool isValid() const {
			if (!Profile::Desc::isValid())
				return false;
			if (a[3] == REAL_ZERO)
				return false;

			return true;
		}
		
		/** Sets the profile parameters to the default values */
		CREATE_FROM_OBJECT_DESC0(Polynomial4, Profile::Ptr)
	};

protected:
	/** Polynomial coefficients */
	Real a[4];
	
public:
	/** Creates a default profile of unit length and unit duration */
	bool create(const Desc& desc);
	
	/** Re-scales the profile with the specified duration time. */
	virtual void scaleDuration(Real duration = REAL_ONE);
	
	/** Scales the profile with the specified length (total travelled distance) */
	virtual void scaleLength(Real length = REAL_ONE);

	/** Distance profile and maximum */
	virtual Real getDistance(Real t) const;
	
	/** Velocity profile f:time -> velocity */
	virtual Real getVelocity(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getVelocityExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;
	
	/** Acceleration profile f:time -> acceleration */
	virtual Real getAcceleration(Real t) const;

	/** Velocity minimum and maximum */
	virtual void getAccelerationExtrema(Real &tmin, Real &tmax, Real &min, Real &max) const;

	/** Creates profile from 2 waypoints.
	* Created profile begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably distance, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param d0		travelled distance at waypoint #0
	* @param v0		velocity at waypoint #0
	* @param d1		travelled distance at waypoint #1
	* @param v1		velocity at waypoint #1
	*/
	void set2dvdv(Real t0, Real t1, Real d0, Real v0, Real d1, Real v1);

	/** Creates profile from 2 waypoints.
	* Created profile begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably distance, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param d0		travelled distance at waypoint #0
	* @param v0		velocity at waypoint #0
	* @param a0		acceleration at waypoint #1
	* @param v1		velocity at waypoint #1
	*/
	void set2dvav(Real t0, Real t1, Real d0, Real v0, Real a0, Real v1);

	/** Creates profile from 3 waypoints.
	* Created profile begins at t0 and ends at t1.
	* Symbols d, v, a stand for suitably distance, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param d0		travelled distance at waypoint #0
	* @param v0		velocity at waypoint #0
	* @param d1		travelled distance at waypoint #1
	* @param d2		travelled distance at waypoint #2
	*/
	void set3dvdd(Real t0, Real t1, Real t2, Real d0, Real v0, Real d1, Real d2);
	
	/** Creates profile from 3 waypoints.
	* Created profile begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably distance, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param d0		travelled distance at waypoint #0
	* @param d1		travelled distance at waypoint #1
	* @param d2		travelled distance at waypoint #2
	* @param v2		velocity at waypoint #2
	*/
	void set3dddv(Real t0, Real t1, Real t2, Real d0, Real d1, Real d2, Real v2);

	/** Creates profile from 4 waypoints.
	* Created profile begins at t1 and ends at t2.
	* Symbols d, v, a stand for suitably distance, velocity, acceleration.
	*
	* @param t0		waypoint #0 [sec]
	* @param t1		waypoint #1 [sec]
	* @param t2		waypoint #2 [sec]
	* @param t3		waypoint #3 [sec]
	* @param d0		travelled distance at waypoint #0
	* @param d1		travelled distance at waypoint #1
	* @param d2		travelled distance at waypoint #2
	* @param d3		travelled distance at waypoint #3
	*/
	void set4dddd(Real t0, Real t1, Real t2, Real t3, Real d0, Real d1, Real d2, Real d3);

	/** Returns polynomial coefficients */
	void getCoeffs(F32 *a) const {
		a[0] = F32(this->a[0]);
		a[1] = F32(this->a[1]);
		a[2] = F32(this->a[2]);
		a[3] = F32(this->a[3]);
	}

	/** Returns polynomial coefficients */
	void getCoeffs(F64 *a) const {
		a[0] = F64(this->a[0]);
		a[1] = F64(this->a[1]);
		a[2] = F64(this->a[2]);
		a[3] = F64(this->a[3]);
	}

	/** Returns polynomial coefficients */
	const Real *getCoeffs() const {
		return a;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_PROFILE_H_*/
