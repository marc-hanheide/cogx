/** @file Image.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_IMAGE_H_
#define _GOLEM_DEMO_COMMON_IMAGE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat23.h>
#include <Golem/Math/Mat34.h>
#include <Golem/Math/Rand.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Demo/Common/ArrN.h>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** 2D lattice */
template <typename Index, typename Element>
class Lattice2 : public Arr2<Index, Element> {
public:
	typedef Arr2<Index, Element> Base;
	typedef Lattice2<Index, Element> This;
	typedef typename Base::Idx2 Idx2;
	
	/** Lattice description */
	class Desc {
	public:
		/** Lattice dimensions */
		Idx2 dimensions;
		/** Lattice grid */
		Vec2 grid;
		/** Lattice pose */
		Mat23 pose;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			dimensions.set(0);
			grid.set(REAL_ONE);
			pose.setId();
		}
		
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!dimensions.isPositive() || !grid.isPositive() || !pose.isFinite())
				return false;

			return true;
		}
	};

protected:
	/** Lattice grid */
	Vec2 grid;
	/** Lattice pose */
	Mat23 pose;

public:
	Lattice2() {
	}

	Lattice2(const Desc &desc) {
		(void)create(desc);
	}
	
	/** Creates lattice */
	bool create(const Desc &desc) {
		if (!desc.isValid())
			return false;
		
		grid = desc.grid;
		pose = desc.pose;
		return Base::create(desc.dimensions);
	}
	
	/** Computes element position from 2D indices in local frame */
	inline void getPositionLocal(Vec2 &position, const Idx2 &idx) const {
		position.set(grid.v1*idx.n1, grid.v2*idx.n2);
	}
	/** Computes element position from 2D indices in global frame */
	inline void getPositionGlobal(Vec2 &position, const Idx2 &idx) const {
		getLocalPosition(position, idx);
		pose.multiply(position, position);
	}
	
	/** Finds nearest 2D index for a given position in local frame */
	inline void findIndexLocal(Idx2& idx, const Vec2 &position) const {
		idx.n1 = Math::clamp((Index)Math::round(position.v1/grid.v1), this->begin().n1, this->end().n1 - 1);
		idx.n2 = Math::clamp((Index)Math::round(position.v2/grid.v2), this->begin().n2, this->end().n2 - 1);
	}
	/** Finds nearest 2D index for a given position in global frame */
	inline void findIndexGlobal(Idx2& idx, const Vec2 &position) const {
		Vec2 positionInv;
		pose.multiplyByInverseRT(positionInv, position);
		findIndexLocal(idx, positionInv);
	}
	
	/** Finds index range for a given ball in local frame */
	inline void findRangeLocal(Idx2& begin, Idx2& end, const Vec2 &centre, Real diameter) const {
		begin.n1 = Math::clamp((Index)Math::floor( (centre.v1 - diameter)/grid.v1 ), this->begin().n1, this->end().n1);
		end.n1 = Math::clamp((Index)Math::ceil((centre.v1 + diameter)/grid.v1) + 1, this->begin().n1, this->end().n1);
		begin.n2 = Math::clamp((Index)Math::floor((centre.v2 - diameter)/grid.v2), this->begin().n2, this->end().n2);
		end.n2 = Math::clamp((Index)Math::ceil((centre.v2 + diameter)/grid.v2) + 1, this->begin().n2, this->end().n2);
	}
	/** Finds index range for a given ball in global frame */
	inline void findRangeGlobal(Idx2& begin, Idx2& end, const Vec2 &centre, Real diameter) const {
		Vec2 centreInv;
		pose.multiplyByInverseRT(centreInv, centre);
		findRangeLocal(begin, end, centreInv, diameter);
	}
	
	/** Returns lattice grid */
	inline const Vec2 &getGrid() const {
		return grid;
	}
	/** Sets lattice grid */
	inline void setGrid(const Vec2 &grid) {
		this->grid = grid;
	}
	
	/** Returns lattice pose */
	inline const Mat23 &getPose() const {
		return pose;
	}
	/** Sets lattice pose */
	inline void setPose(const Mat23 &pose) {
		this->pose = pose;
	}

	/**	Deep copy. */
	inline bool clone(const Lattice2 &lattice) {
		if (!Base::clone(lattice))
			return false;

		grid = lattice.grid;
		pose = lattice.pose;
		return true;
	}
};

//------------------------------------------------------------------------------

/** 2D image */
class Image2 : public Lattice2<I32, U8> {
public:
	typedef Lattice2<Index, Element> Base;
	typedef Image2 This;
	typedef Base::Idx2 Idx2;
	/** Image2 sequence */
	typedef std::vector<Image2> Seq;

	/** Element min */
	static const Element ElementMin;
	/** Element min */
	static const Element ElementMax;
	
protected:
	/** Finds intersection limits for a given image. */
	void getLimits(Vec2 &min, Vec2 &max, const Mat23 &trn, const Image2 &image) const;

public:
	Image2() {
	}

	Image2(const Base::Desc &desc) {
		(void)Base::create(desc);
	}
	
	/** Finds intersection range with a given image. */
	bool getIntersection(Idx2 &begin, Idx2 &end, const Image2 &image) const;

	/** Sets intersection with a given image */
	bool setIntersection(const Image2 &image);
	
	/** Interpolates image voxels within a given range */
	bool interpolateNearestNeighbour(const Idx2 &begin, const Idx2 &end, const Image2 &image);

	/**	Deep copy. */
	inline bool clone(const Image2 &image) {
		if (!Base::clone(image))
			return false;

		return true;
	}
};

//------------------------------------------------------------------------------

/** Lattice3 */
template <typename Index, typename Element>
class Lattice3 : public Arr3<I32, U8> {
public:
	typedef Arr3<Index, Element> Base;
	typedef Lattice3<Index, Element> This;
	typedef typename Base::Idx3 Idx3;

	/** Lattice description */
	class Desc {
	public:
		/** Lattice dimensions */
		Idx3 dimensions;
		/** Lattice grid */
		Vec3 grid;
		/** Lattice pose */
		Mat34 pose;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			dimensions.set(0);
			grid.set(REAL_ONE);
			pose.setId();
		}
		
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!dimensions.isPositive() || !grid.isPositive() || !pose.isFinite())
				return false;

			return true;
		}
	};

protected:
	/** Lattice grid */
	Vec3 grid;
	/** Lattice pose */
	Mat34 pose;

public:
	Lattice3() {
	}

	Lattice3(const Desc &desc) {
		(void)create(desc);
	}
	
	/** Creates lattice */
	bool create(const Desc &desc) {
		if (!desc.isValid())
			return false;
		
		grid = desc.grid;
		pose = desc.pose;
		return Base::create(desc.dimensions);
	}
	
	/** Computes element position from 3D indices */
	inline void getPosition(Vec3 &position, const Idx3 &idx) const {
		position.set(grid.v1*idx.n1, grid.v2*idx.n2, grid.v3*idx.n3);
		pose.multiply(position, position);
	}
	
	/** Computes element position from a linear index */
	inline void getPosition(Vec3 &position, Index i) const {
		Idx3 idx;
		getIndex(idx, i); // retrieve 3D indices
		getPosition(position, idx);
	}
	
	/** Returns lattice grid */
	inline const Vec3 &getGrid() const {
		return grid;
	}

	/** Sets lattice grid */
	inline void setGrid(const Vec3 &grid) {
		this->grid = grid;
	}
	
	/** Returns lattice pose */
	inline const Mat34 &getPose() const {
		return pose;
	}

	/** Sets lattice pose */
	inline void setPose(const Mat34 &pose) {
		this->pose = pose;
	}

	/**	Deep copy. */
	inline bool clone(const Lattice3 &lattice) {
		if (!Base::clone(lattice))
			return false;

		grid = lattice.grid;
		pose = lattice.pose;
		return true;
	}
};

//------------------------------------------------------------------------------

/** Element occupancy coefficient type must not allow for values smaller than zero */

/** Image3 */
class Image3 : public Lattice3<I32, U8> {
public:
	typedef Lattice3<Index, Element> Base;
	typedef Image3 This;
	typedef Base::Idx3 Idx3;
	/** Image3 sequence */
	typedef std::vector<Image3> Seq;

	/** Element min */
	static const Element ElementMin;
	/** Element min */
	static const Element ElementMax;
	
protected:
	/** Finds intersection limits for a given image. */
	void getLimits(Vec3 &min, Vec3 &max, const Mat34 &trn, const Image3 &image) const;

public:
	Image3() {
	}

	Image3(const Desc &desc) {
		(void)Base::create(desc);
	}
	
	/** Finds intersection range with a given image. */
	bool getIntersection(Idx3 &begin, Idx3 &end, const Image3 &image) const;

	/** Sets intersection with a given image */
	bool setIntersection(const Image3 &image);

	/** Interpolates image voxels within a given range */
	bool interpolateNearestNeighbour(const Idx3 &begin, const Idx3 &end, const Image3 &image);
	
	/**	Deep copy. */
	inline bool clone(const Image3 &image) {
		if (!Base::clone(image))
			return false;

		return true;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_IMAGE_H_*/
