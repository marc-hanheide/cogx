/** @file ImageFilter.h
 * 
 * Linear filter defined on the image grid.
 *
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_IMAGEFILTER_H_
#define _GOLEM_DEMO_COMMON_IMAGEFILTER_H_

//------------------------------------------------------------------------------

#include <Golem/Demo/Common/Image.h>
#include <Golem/Demo/Common/Rotations.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Image filter response */
struct Response {
	/** response sequence */
	typedef std::vector<Response> Seq;
	
	/** Filter response comparator */
	struct FilterCmpr : public std::binary_function<Response, Response, bool> {
		inline bool operator () (const Response &l, const Response &r) const {
			return l.filter > r.filter;
		}
	};
	
	/** Envelop response comparator */
	struct EnvelopCmpr : public std::binary_function<Response, Response, bool> {
		inline bool operator () (const Response &l, const Response &r) const {
			return l.envelop > r.envelop;
		}
	};
	
	/** Filter response */
	Real filter;
	/** Envelop response */
	Real envelop;
};

//------------------------------------------------------------------------------

/** Generic linear filter defined on the image grid.
*	Default Gaussian filter envelop:
*	envelop(x, y) = C * exp(-(x^2/sigma.x^2 + y^2/sigma.y^2)/2)
*	C = 1/(2PI * sigma.x * sigma.y)
*/
class ImageFilter2 {
public:
	/** Class pointer */
	typedef shared_ptr<ImageFilter2> Ptr;
	/** Pointer sequence */
	typedef std::vector<Ptr> Seq;

	/** 1D Index type */
	typedef golem::Image2::Index Index;
	/** 2D Index type */
	typedef golem::Image2::Idx2 Idx2;
	/** Filter data array */
	typedef Arr2<Index, Real> Data;
	/** Filter data array sequence */
	typedef std::vector<Data> DataSeq;

	/** Filter description */
	class Desc {
	public:
		/** Description pointer */
		typedef shared_ptr<Desc> Ptr;
		/** Pointer sequence */
		typedef std::vector<Ptr> Seq;
		
		/** Filter radius in voxels */
		Idx2 radius;
		/** Number of filter rotations */
		U32 numOfRotations;
		/** Filter envelop symmetry */
		bool envelopSymmetry;
		/** Filter templates flag */
		bool templates;

		/** Sigma */
		Vec2 sigma;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates object given the description. 
		* @return		pointer to the object, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual ImageFilter2::Ptr create(golem::Embodiment &embodiment) const = 0;
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			radius.set(4);
			numOfRotations = 8;
			envelopSymmetry = true;
			templates = true;

			sigma.set(Real(0.5));
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!radius.isPositive() && !radius.isZero())
				return false;
			if (numOfRotations <= 0)
				return false;
			if (!sigma.isPositive())
				return false;

			return true;
		}
	};

protected:
	/** Embodiment */
	golem::Embodiment &embodiment;
	/** Context object */
	golem::Context &context;
	
	/** Filter radius and diameter in voxels */
	Idx2 radius, diameter;
	/** Filter envelop symmetry */
	bool envelopSymmetry;
	/** Templates flag */
	bool templates;
	
	/** Number of filter rotations */
	U32 numOfRotations;
	/** Rotation delta  */
	Real rotationDelta;
	
	/** Sigma, sigmaFac */
	Vec2 sigma, sigmaFac;
	/** C */
	Real C;

	/** Filter envelopes */
	DataSeq envelopes;
	/** Filter carriers (with envelopes) */
	DataSeq carriers;
	/** Envelopes weights */
	std::vector<Real> weights;
	
	/** Templates setup */
	virtual bool createTemplates();
	
	/** Returns filter envelope at the specified location in the canonical coordinates of the filter */
	virtual inline Real getEnvelop(const Vec2& location) const {
		return C*Math::exp(
			sigmaFac.v1*location.v1*location.v1 +
			sigmaFac.v2*location.v2*location.v2
		);
	}

	/** Returns filter carrier at the specified location in the canonical coordinates of the filter */
	virtual Real getCarrier(const Vec2& location) const = 0;

	/** Returns filter carrier at the specified location and symmetry axis */
	virtual Real getCarrier(const Vec2& location, const Vec2& axis) const = 0;

	/** Creates Filter from the description. 
	* @param desc	Filter description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const ImageFilter2::Desc& desc);
	
	/** ImageFilter2 constructor */
	ImageFilter2(golem::Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~ImageFilter2() {}

	/** Returns maximum (normalised) filter response at the specified image location */
	void response(Response::Seq& response, const Idx2& location, const Image2& image) const;

	/** Returns the default filter radius */
	inline const Idx2 &getRadius() const {
		return radius;
	}
	/** Returns the default filter diameter */
	inline const Idx2 &getDiameter() const {
		return diameter;
	}
	
	/** Returns filter envelopes array for all rotations */
	inline const ImageFilter2::DataSeq &getEnvelopes() const {
		return envelopes;
	}
	/** Returns filter carriers array for all rotations */
	inline const ImageFilter2::DataSeq &getCarriers() const {
		return carriers;
	}
	
	/** Filter rotations */
	inline U32 getNumOfRotations() const {
		return numOfRotations;
	}

	/** Filter rotations */
	inline Real getRotation(U32 rotationIndex) const {
		return (rotationDelta*rotationIndex);
	}
	
	/** Filter rotations */
	inline Real getRotationInv(U32 rotationIndex) const {
		return (-rotationDelta*rotationIndex);
	}
};

//------------------------------------------------------------------------------

/** Gabor filter implementation
*	G(p) = envelop(p) * carrier(p)
*	p = {x, y}^T
*	envelop(x, y) = C * exp(-(x^2/sigmax^2 + y^2/sigmay^2)/2)
*	C = 1/((2PI)^(3/2) * sigma.x * sigma.y)
*	carrier(x, y) = cos((axis.x*x + axis.y*y)*2PI/lambda + phi)
*/
class GaborFilter2 : public ImageFilter2 {
public:
	/** Filter description */
	class Desc : public ImageFilter2::Desc {
	public:
		/** Lambda */
		Real lambda;
		/** Phi */
		Real phi;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates Filter given the description object. 
		* @return		pointer to the Filter, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual ImageFilter2::Ptr create(golem::Embodiment &embodiment) const {
			GaborFilter2 *pGaborFilter = new GaborFilter2(embodiment);
			ImageFilter2::Ptr pFilter(pGaborFilter);

			if (!pGaborFilter->create(*this))
				pFilter.release();

			return pFilter;
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ImageFilter2::Desc::setToDefault();
			
			lambda = Real(10.0);
			phi = - REAL_PI_2; // odd filter
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!ImageFilter2::Desc::isValid())
				return false;

			if (lambda <= REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	/** Lambda and phi */
	Real lambda, phi;
	/** D */
	Real D;

	/** Returns filter carrier at the specified location in the canonical coordinates of the filter */
	virtual inline Real getCarrier(const Vec2& location) const {
		return Math::cos(D*location.v2 - phi); // Y-axis
	}

	/** Returns filter carrier at the specified location and symmetry axis */
	virtual inline Real getCarrier(const Vec2& location, const Vec2& axis) const {
		return Math::cos(D*location.dot(axis) - phi);
	}

	/** Creates Filter from the description. */
	bool create(const GaborFilter2::Desc& desc);

	/** GaborFilter2 constructor */
	GaborFilter2(golem::Embodiment &embodiment);

public:
	/** Returns Sigma */
	inline const Vec2 &getSigma() const {
		return sigma;
	}

	/** Returns Lambda */
	inline Real getLambda() const {
		return lambda;
	}

	/** Returns Phi */
	inline Real getPhi() const {
		return phi;
	}
};

//------------------------------------------------------------------------------

/** Generic linear filter defined on the image grid.
*	Default Gaussian filter envelop:
*	envelop(x, y, z) = C * exp(-(x^2/sigma.x^2 + y^2/sigma.y^2 + z^2/sigma.z^2)/2)
*	C = 1/((2PI)^(3/2) * sigma.x * sigma.y * sigma.z)
*/
class ImageFilter3 {
public:
	/** Class pointer */
	typedef shared_ptr<ImageFilter3> Ptr;
	/** Pointer sequence */
	typedef std::vector<Ptr> Seq;

	/** 1D Index type */
	typedef golem::Image3::Index Index;
	/** 3D Index type */
	typedef golem::Image3::Idx3 Idx3;
	/** Filter data array */
	typedef Arr3<Index, Real> Data;
	/** Filter data array sequence */
	typedef std::vector<Data> DataSeq;

	/** Filter description */
	class Desc {
	public:
		/** Description pointer */
		typedef shared_ptr<Desc> Ptr;
		/** Pointer sequence */
		typedef std::vector<Ptr> Seq;
		
		/** Filter rotations set */
		Rotations::Desc::Ptr rotationsDesc;
		/** Filter radius in voxels */
		Idx3 radius;
		/** Filter envelop symmetry */
		bool envelopSymmetry;
		/** Filter templates flag */
		bool templates;

		/** Sigma */
		Vec3 sigma;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates object given the description. 
		* @return		pointer to the object, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual ImageFilter3::Ptr create(golem::Embodiment &embodiment) const = 0;
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			radius.set(4);
			rotationsDesc.reset(new Rotations::Desc());
			envelopSymmetry = true;
			templates = true;

			sigma.set(Real(1.0));
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!radius.isPositive() && !radius.isZero())
				return false;
			if (rotationsDesc == NULL || !rotationsDesc->isValid())
				return false;
			if (!sigma.isPositive())
				return false;

			return true;
		}
	};

protected:
	/** Embodiment */
	golem::Embodiment &embodiment;
	/** Context object */
	golem::Context &context;
	
	/** Filter radius and diameter in voxels */
	Idx3 radius, diameter;
	/** Filter rotations set */
	Rotations::Ptr rotations;
	/** Number of filter rotations */
	U32 numOfRotations;
	/** Filter envelop symmetry */
	bool envelopSymmetry;
	/** Filter carrier degeneracy */
	bool carrierDegeneracy;
	/** Templates flag */
	bool templates;
	
	/** Sigma, sigmaFac */
	Vec3 sigma, sigmaFac;
	/** C */
	Real C;

	/** Filter envelopes */
	DataSeq envelopes;
	/** Filter carriers (with envelopes) */
	DataSeq carriers;
	/** Envelopes weights */
	std::vector<Real> weights;
	
	/** Templates setup */
	virtual bool createTemplates();
	
	/** Returns filter envelope at the specified location in the canonical coordinates of the filter */
	virtual inline Real getEnvelop(const Vec3& location) const {
		return C*Math::exp(
			sigmaFac.v1*location.v1*location.v1 +
			sigmaFac.v2*location.v2*location.v2 +
			sigmaFac.v3*location.v3*location.v3
		);
	}

	/** Returns filter carrier at the specified location in the canonical coordinates of the filter */
	virtual Real getCarrier(const Vec3& location) const = 0;

	/** Returns filter carrier at the specified location and symmetry axis */
	virtual Real getCarrier(const Vec3& location, const Vec3& axis) const = 0;

	/** Creates Filter from the description. 
	* @param desc	Filter description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const ImageFilter3::Desc& desc);
	
	/** ImageFilter3 constructor */
	ImageFilter3(golem::Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~ImageFilter3() {}

	/** Returns maximum (normalised) filter response at the specified image location */
	void response(Response::Seq& response, const Idx3& location, const Image3& image) const;

	/** Returns the default filter radius */
	inline const Idx3 &getRadius() const {
		return radius;
	}

	/** Returns the default filter diameter */
	inline const Idx3 &getDiameter() const {
		return diameter;
	}

	/** Filter symmetry axis for identity rotation */
	inline const Vec3& getSymmetryAxis() const {
		return rotations->getSymmetryAxis();
	}
	
	/** Filter rotations */
	inline const golem::RotationSeq &getRotations() const {
		return rotations->getRotations();
	}
	
	/** Returns filter envelopes array for all rotations */
	inline const ImageFilter3::DataSeq &getEnvelopes() const {
		return envelopes;
	}

	/** Returns filter carriers array for all rotations */
	inline const ImageFilter3::DataSeq &getCarriers() const {
		return carriers;
	}
};

//------------------------------------------------------------------------------

/** Gabor filter implementation
*	G(p) = envelop(p) * carrier(p)
*	p = {x, y, z}^T
*	envelop(x, y, z) = C * exp(-(x^2/sigmax^2 + y^2/sigmay^2 + z^2/sigmaz^2)/2)
*	C = 1/((2PI)^(3/2) * sigma.x * sigma.y * sigma.z)
*	carrier(x, y, z) = cos((axis.x*x + axis.y*y + axis.z*z)*2PI/lambda + phi)
*/
class GaborFilter3 : public ImageFilter3 {
public:
	/** Filter description */
	class Desc : public ImageFilter3::Desc {
	public:
		/** Lambda */
		Real lambda;
		/** Phi */
		Real phi;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates Filter given the description object. 
		* @return		pointer to the Filter, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual ImageFilter3::Ptr create(golem::Embodiment &embodiment) const {
			GaborFilter3 *pGaborFilter = new GaborFilter3(embodiment);
			ImageFilter3::Ptr pFilter(pGaborFilter);

			if (!pGaborFilter->create(*this))
				pFilter.release();

			return pFilter;
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ImageFilter3::Desc::setToDefault();
			
			lambda = Real(1.0);
			phi = - REAL_PI_2; // odd filter
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!ImageFilter3::Desc::isValid())
				return false;

			if (lambda <= REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	/** Lambda and phi */
	Real lambda, phi;
	/** D */
	Real D;

	/** Returns filter carrier at the specified location in the canonical coordinates of the filter */
	virtual inline Real getCarrier(const Vec3& location) const {
		return Math::cos(D*location.v3 - phi); // Z-axis
	}

	/** Returns filter carrier at the specified location and symmetry axis */
	virtual inline Real getCarrier(const Vec3& location, const Vec3& axis) const {
		return Math::cos(D*location.dot(axis) - phi);
	}

	/** Creates Filter from the description. */
	bool create(const GaborFilter3::Desc& desc);

	/** GaborFilter3 constructor */
	GaborFilter3(golem::Embodiment &embodiment);

public:
	/** Returns Sigma */
	inline const Vec3 &getSigma() const {
		return sigma;
	}

	/** Returns Lambda */
	inline Real getLambda() const {
		return lambda;
	}

	/** Returns Phi */
	inline Real getPhi() const {
		return phi;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_IMAGEFILTER_H_*/
