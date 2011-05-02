/** @file Rotations.h
 * 
 * Linear filter defined on the image grid.
 *
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_ROTATIONS_H_
#define _GOLEM_DEMO_COMMON_ROTATIONS_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat34.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Rand.h>
#include <Golem/Tools/Stream.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Rotation */
typedef Quat Rotation;
typedef std::vector<Rotation> RotationSeq;

/** Rigid object rotations. */
class Rotations {
public:
	/** Class pointer */
	typedef shared_ptr<Rotations> Ptr;

	/** Rotations description */
	class Desc {
	public:
		/** Description pointer */
		typedef shared_ptr<Desc> Ptr;
		
		/** Symmetry axis is the axis of the highest object symmetry
		*   (or some other arbitrary axis if an object has no rotation symmetry)
		*/
		Vec3 symmetryAxis;
		/** Rotation degeneracy around symmetry axis */
		bool degeneracy;
		/** Maximum number of rotations */
		U32 numOfRotations;
		
		/** Rotations file name */
		std::string rotationsPath;

		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}
		
		/** Creates object given the description. 
		* @return		pointer to the object, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		virtual Rotations::Ptr create(golem::Embodiment &embodiment) const {
			Rotations::Ptr pRotations(new Rotations(embodiment));

			if (!pRotations->create(*this))
				pRotations.release();

			return pRotations;
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			symmetryAxis.set(REAL_ZERO, REAL_ZERO, REAL_ONE); // Z-axis
			degeneracy = true;
			numOfRotations = 60;
			rotationsPath = "rotations.dat";
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (numOfRotations < 1 || !Math::equals(symmetryAxis.magnitude(), REAL_ONE, REAL_EPS))
				return false;

			return true;
		}
	};

protected:
	/** Orientations file header */
	class Header : public Serializable {
	public:
		static const std::string NAME;
		std::string name;
		Vec3 symmetryAxis;
		bool degeneracy;
		U32 items;

		virtual void load(const Stream &stream) {
			stream.read(&name, 255);
			stream.read(symmetryAxis);
			stream.read(degeneracy);
			stream.read(items);
		}
		virtual void store(Stream &stream) const {
			stream.write(&name, 255);
			stream.write(symmetryAxis);
			stream.write(degeneracy);
			stream.write(items);
		}
		virtual void setToDefault() {
			name = NAME;
		}
		virtual bool isValid() const {
			if (name != NAME)
				return false;

			return true;
		}
		size_t getSize() const {
			return name.size() + 1 + sizeof(bool) + sizeof(Vec3) + sizeof(U32);
		}
	};
	
	/** Embodiment */
	golem::Embodiment &embodiment;
	/** Context object */
	golem::Context &context;
	
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Symmetry axis for identity rotation */
	Vec3 symmetryAxis;
	/** Rotation degeneracy around symmetry axis */
	bool degeneracy;
	/** Object rotations */
	RotationSeq rotations;

	/** Random normalised tuple */
	virtual void next(Real *t, U32 n) const;
	
	/** Get rotation */
	bool get(Quat &q, const Vec3& a) const;
	
	/** Get axis */
	bool get(Vec3& a, const Quat &q) const;
	
	/** Generate rotations and axes */
	virtual void generate(U32 numOfRotations);
	
	/** Loads all data from a file */
	bool load(const std::string& rotationsPath);
	
	/** Writes all data to a file */
	bool store(const std::string& rotationsPath) const;
	
	/** Creates class from the description. 
	* @param desc	class description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Rotations::Desc& desc);
	
	/** Rotations constructor */
	Rotations(golem::Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~Rotations() {}

	/** Symmetry axis for identity rotation */
	inline const Vec3& getSymmetryAxis() const {
		return symmetryAxis;
	}
	
	/** Rotation degeneracy around symmetry axis */
	inline bool isDegenerated() const {
		return degeneracy;
	}

	/** Object rotations */
	inline const RotationSeq &getRotations() {
		return rotations;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_ROTATIONS_H_*/
