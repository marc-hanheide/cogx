/** @file DigitalFilter.h
 * 
 * Digital filter interface and library.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_DIGITALFILTER_H_
#define _GOLEM_DEMO_COMMON_DIGITALFILTER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Mat34.h>
#include <vector>

namespace golem {

//------------------------------------------------------------------------------

/** DigitalFilter. */
class DigitalFilter {
public:
	typedef shared_ptr<DigitalFilter> Ptr;

	/** Filter description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates Filter given the description object. 
		* @return		pointer to the Filter, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual DigitalFilter::Ptr create() const = 0;
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			return true;
		}
	};

public:
	/** Descructor */
	virtual ~DigitalFilter() {}

	/** Creates Filter from the description. 
	* @param desc	Filter description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const DigitalFilter::Desc& desc);
	
	/** Resets the filter. */
	virtual void reset() = 0;
	
	/** Updates the filter. */
	virtual void input(Real inputValue) = 0;

	/** Retuns the value of the filter. */
	virtual Real output() const = 0;
};

//------------------------------------------------------------------------------

/** Finite/infinite impulse response filter implementation.
*/
class FIIRFilter : public DigitalFilter {
public:
	/** FIIRFilter description */
	class Desc : public DigitalFilter::Desc {
	public:
		/** Feedforward coefficients */
		std::vector<Real> feedforwardCoeffs;
		/** Feedback coefficients */
		std::vector<Real> feedbackCoeffs;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Creates FIIRFilter given the description object. 
		* @return		pointer to the FIIRFilter, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual DigitalFilter::Ptr create() const {
			FIIRFilter *pFIIRFilter = new FIIRFilter();
			DigitalFilter::Ptr pFilter(pFIIRFilter);

			if (!pFIIRFilter->create(*this))
				pFilter.release();

			return pFilter;
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			DigitalFilter::Desc::setToDefault();
			
			// the moving average FIR filter
			feedforwardCoeffs.clear();
			feedforwardCoeffs.resize(2, REAL_ONE/Real(2));
			feedbackCoeffs.clear();
			feedbackCoeffs.resize(1, REAL_ONE);
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!DigitalFilter::Desc::isValid())
				return false;
			
			if (feedforwardCoeffs.size() < 1)
				return false;
			if (feedbackCoeffs.size() < 1 || Math::equals(feedbackCoeffs[0], REAL_ZERO, REAL_EPS))
				return false;

			return true;
		}
	};

protected:
	/** Feedforward coefficients */
	std::vector<Real> feedforwardCoeffs;
	/** Feedback coefficients */
	std::vector<Real> feedbackCoeffs;

	bool bInit;
	std::vector<Real> outputValues;
	std::vector<Real> inputValues;

public:
	/** Creates FIIRFilter from the description. */
	bool create(const FIIRFilter::Desc& desc);
	
	/** Resets the filter. */
	virtual void reset();
	
	/** Updates the filter. */
	virtual void input(Real inputValue);

	/** Retuns the value of the filter. */
	virtual Real output() const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_DIGITALFILTER_H_*/
