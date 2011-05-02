/** @file Sample.h
 * 
 * Sample class.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_SAMPLE_H_
#define _GOLEM_MATH_SAMPLE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <functional>
#include <cmath>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Pointer dereferencing template */
struct Ref1 {
	template <typename Type, typename Ptr> static inline Type& get(Ptr& ptr) {
		return ptr; // nothing to do
	}
};
/** Pointer to pointer dereferencing template */
struct Ref2 {
	template <typename Type, typename Ptr> static inline Type& get(Ptr& ptr) {
		return *ptr;
	}
};

/** Cumulative density function sample */
template <typename Real> class Sample {
public:
	/** Cumulative density value comparator */
	template <typename Ref> struct Cmpr {
		template <typename Ptr> inline bool operator () (Ptr& l, Real cdf) const {
			return Ref::template get<const Sample>(l).cdf < cdf;
		}
		template <typename Ptr> inline bool operator () (Real cdf, Ptr& r) const {
			return cdf < Ref::template get<const Sample>(r).cdf;
		}
		template <typename Ptr> inline bool operator () (Ptr& l, Ptr& r) const {
			return Ref::template get<const Sample>(l).cdf < Ref::template get<const Sample>(r).cdf;
		}
	};

	/** Weight */
	Real weight;
	/** Cumulative density */
	Real cdf;

	/** No initialisation */
	inline Sample() {
	}

	/** Sets the sample weight; invalidates cdf if it is not specified */
	inline Sample(Real weight, Real cdf = -numeric_const<Real>::ONE) {
		set(weight, cdf);
	}

	/** Sets the parameters to the default values */
	inline void setToDefault() {
		set(numeric_const<Real>::ONE);
	}

	/** Sets the sample weight; invalidates cdf if it is not specified */
	inline void set(Real weight, Real cdf = -numeric_const<Real>::ONE) {
		this->weight = weight;
		this->cdf = cdf;
	}

	/** Sets the parameters to the default values */
	template <typename Ref, typename Seq> static inline void setToDefault(Seq& seq) {
		for (typename Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			Ref::template get<Sample>(*i).setToDefault();
	}

	/** Sets the sample weights to the specified value; invalidates cdf */
	template <typename Ref, typename Seq> static inline void set(Real weight, Seq& seq) {
		for (typename Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			Ref::template get<Sample>(*i).set(weight);
	}

	/** Checks if sample is valid. */
	bool isValid() const {
		if (weight < numeric_const<Real>::ZERO || cdf < numeric_const<Real>::ZERO)
			return false;
		
		return true;
	}
	
	/** Checks if the sample have identical data. */
	template <typename Ref1, typename Ref2> inline bool equals(const Sample& s) const {
		return weight == Ref1::template get<const Sample>(s).weight && cdf == Ref2::template get<const Sample>(s).cdf;
	}
	
	/** Checks if sample sequence is valid. */
	template <typename Ref, typename Seq> static inline bool isValid(const Seq& seq) {
		if (seq.empty() || Ref::template get<const Sample>(*(seq.end() - 1)).cdf <= numeric_const<Real>::ZERO)
			return false;
		for (typename Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
			if (!Ref::template get<const Sample>(*i).isValid())
				return false;
		
		return true;
	}
	
	/** Normalisation constant */
	template <typename Ref, typename Seq> static inline Real getNorm(const Seq& seq) {
		if (seq.empty())
			return numeric_const<Real>::ZERO;

		const Real cdf = Ref::template get<const Sample>(*(seq.end() - 1)).cdf;
		if (cdf <= numeric_const<Real>::ZERO)
			return numeric_const<Real>::ZERO;
		
		return numeric_const<Real>::ONE/cdf;
	}

	/** Normalises set of samples using samples weights */
	template <typename Ref, typename Seq> static inline bool normalise(Seq& seq) {
		if (seq.empty())
			return false;
		
		Real cdf = numeric_const<Real>::ZERO;
		for (typename Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
			Sample& sample = Ref::template get<Sample>(*i);
			sample.cdf = (cdf += sample.weight);
		}

		if (cdf > numeric_const<Real>::ZERO)
			return true;

		return uniform<Ref, Seq>(seq);
	}

	/** Normalises set of samples to 1 */
	template <typename Ref, typename Seq> static inline bool uniform(Seq& seq) {
		if (seq.empty())
			return false;
		
		const Real delta = numeric_const<Real>::ONE/seq.size();
		size_t j = 0;
		for (typename Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
			Sample& sample = Ref::template get<Sample>(*i);
			sample.weight = delta;
			sample.cdf = delta*++j;
		}

		return true;
	}

	/** Draws a sample from set of samples */
	template <typename Ref, typename Ptr, typename Seq, typename Rand> static inline Ptr sample(Seq& seq, Rand& rand) {
		if (seq.empty())
			return seq.end();

		const Real cdf = Ref::template get<const Sample>(*(seq.end() - 1)).cdf;
		if (cdf <= numeric_const<Real>::ZERO)
			return seq.end();

		const Real s = rand.template nextUniform<Real>()*cdf;
		const Ptr ptr = std::lower_bound(seq.begin(), seq.end(), s, Cmpr<Ref>());
		
		return ptr == seq.end() ? (seq.end() - 1) : ptr;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_SAMPLE_H_*/
