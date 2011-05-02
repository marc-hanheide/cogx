/** @file ArrN.h
 * 
 * Generic N-dimensional array template.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_ARRN_H_
#define _GOLEM_DEMO_COMMON_ARRN_H_

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Math.h>
#include <memory.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Generic N-dimensional array template */
template <typename _Index, typename _Element>
class ArrN {
public:
	/** Index type */
	typedef _Index Index;
	/** Element type */
	typedef _Element Element;
	typedef ArrN<Index, Element> This;

	/** Array dimensions */
	struct Dimension {
		Index size;
		Index ptr;
	};

protected:
	Index n, size;
	golem::shared_ptr< Dimension, golem::arr_cnt<Dimension> > dimensions;
	golem::shared_ptr< Element, golem::arr_cnt<Element> > elements;

public:
	ArrN() {
	}

	//ArrN(const This& arr) {
	//	This::operator = (arr);
	//}

	ArrN(const Index *dim, Index n) {
		(void)create(dim, n);
	}

	/** Creates array */
	bool create(const Index *dim, Index n) {
		this->n = n;
		dimensions.reset(new Dimension [n]);
		if (dimensions == NULL) {
			release();
			return false;
		}

		for (Index i = 0; i < n; i++) {
			dimensions[i].size = dim[i];
			dimensions[i].ptr = i == 0 ? 0 : i == 1 ? dimensions[i - 1].size : dimensions[i - 1].size*dimensions[i - 1].ptr;
		}

		this->size = dimensions[n - 1].size * dimensions[n - 1].ptr;
		elements.reset(new Element [size]);
		if (elements == NULL) {
			release();
			return false;
		}

		return true;
	}

	/** Release data */
	void release() {
		n = 0;
		size = 0;
		dimensions.reset();
		elements.reset();
	}

	/** Fills array with a specified element */
	inline void fill(const Element &element) {
		for (Index i = 0; i < size; i++)
			elements[i] = element;
	}
	
	/** Fills array with zeros */
	inline void zero() {
		::memset(elements.get(), 0, size*sizeof(Element));
	}
	
	/** Copies elements from a specified array */
	inline void copy(const This &arr) {
		Index size = std::min(this->size, arr.size);
		for (Index i = 0; i < size; i++)
			elements[i] = arr.elements[i];
	}
	
	/** Checks if array is empty */
	inline bool isEmpty() const {
		return elements.get() == NULL;
	}

	/** Returns the number of dimensions of the array */
	inline Index getNumOfDimensions() const {
		return n;
	}

	/** Returns a constant pointer to the dimension array */
	inline const Dimension* getDimensions() const {
		return dimensions.get();
	}

	/** Returns the number of elements of the array */
	inline Index getNumOfElements() const {
		return size;
	}

	/** Returns a pointer to the elements array */
	inline const Element* getElements() const {
		return elements.get();
	}
	inline Element* getElements() {
		return elements.get();
	}

	/** Returns a reference to an element of the array from ND indices. */
	inline const Element &operator [] (const Index *idx) const {
		return elements[getIndex(idx)];
	}
	inline Element &operator [] (const Index *idx) {
		return elements[getIndex(idx)];
	}

	/** Returns a reference to an element of the array from 1D index. */
	inline const Element &operator [] (Index i) const {
		return elements[i];
	}
	inline Element &operator [] (Index i) {
		return elements[i];
	}

	/**	Clone (deep copy). */
	inline bool clone(const This &arr) {
		if (arr.isEmpty())
			return false;

		n = arr.n;
		dimensions.reset(new Dimension [n]);
		if (dimensions == NULL)
			return false;
		for (Index i = 0; i < n; i++)
			dimensions[i] = arr.dimensions[i];
		
		size = arr.size;
		elements.reset(new Element [size]);
		if (elements == NULL)
			return false;
		for (Index i = 0; i < size; i++)
			elements[i] = arr.elements[i];
		
		return true;
	}

	/** Calculates 1D index from ND indices  */
	inline Index getIndex(const Index *idx) const {
		Index i = idx[0];
		
		for (Index j = 1; j < n; j++)
			i += dimensions[j].ptr*idx[j];
	
		return i;
	}

	/** Calculates ND indices from 1D index  */
	inline void getIndex(Index *idx, Index i) const {
		Index j = n - 1;
		do {
			idx[j] = i/dimensions[j].ptr;
			i -= idx[j]*dimensions[j].ptr;
		} while (j-- > 0);
	}
};

//------------------------------------------------------------------------------

/** Array index */
template <typename _Index>
class Idx2 {
public:
	/** Index type */
	typedef _Index Index;
	
	Index n1, n2;

	Idx2() {}
	Idx2(Index n) : n1(n), n2(n) {}
	Idx2(Index n1, Index n2) : n1(n1), n2(n2) {}
	
	/** sets indices */
	inline void set(Index n) {
		this->n1 = n;
		this->n2 = n;
	}
	inline void set(Index n1, Index n2) {
		this->n1 = n1;
		this->n2 = n2;
	}
	
	/** sets indices to minimum value */
	inline void setMin(const Idx2& idx1, const Idx2& idx2) {
		this->n1 = std::min(idx1.n1, idx2.n1);
		this->n2 = std::min(idx1.n2, idx2.n2);
	}
	
	/** sets indices to maximum value */
	inline void setMax(const Idx2& idx1, const Idx2& idx2) {
		this->n1 = std::max(idx1.n1, idx2.n1);
		this->n2 = std::max(idx1.n2, idx2.n2);
	}
	
	/** tests for zero index */
	inline bool isZero() const {
		return n1 == 0 && n2 == 0;
	}

	/** tests for positive index */
	inline bool isPositive() const {
		return n1 > 0 && n2 > 0;
	}

	/** Returns a reference to the array indices. */
	inline const Idx2 &operator [] (Index i) const {
		return (&n1)[i];
	}
	inline Idx2 &operator [] (Index i) {
		return (&n1)[i];
	}

	/** true if all the members are smaller. */
	inline bool operator < (const Idx2& idx) const {
		return (n1 < idx.n1) && (n2 < idx.n2);
	}
	inline bool operator < (Index index) const {
		return (n1 < index) && (n2 < index);
	}

	/** true if all the members are larger. */
	inline bool operator > (const Idx2& idx) const {
		return (n1 > idx.n1) && (n2 > idx.n2);
	}
	inline bool operator > (Index index) const {
		return (n1 > index) && (n2 > index);
	}

	/** returns true if the two indices are equal. */
	inline bool operator == (const Idx2& idx) const {
		return (n1 == idx.n1) && (n2 == idx.n2);
	}
	inline bool operator == (Index index) const {
		return (n1 == index) && (n2 == index);
	}

	/** returns true if the two indices are unequal. */
	inline bool operator != (const Idx2& idx) const {
		return (n1 != idx.n1) || (n2 != idx.n2);
	}
	inline bool operator != (Index index) const {
		return (n1 != index) || (n2 != index);
	}
};

/** Generic 2-dimensional array template */
template <typename Index, typename Element>
class Arr2 : public ArrN<Index, Element> {
public:
	/** 2D Index */
	typedef golem::Idx2<Index> Idx2;
	typedef ArrN<Index, Element> Base;
	typedef Arr2<Index, Element> This;

protected:
	Idx2 ibegin, iend;

public:
	Arr2() {
	}

	//Arr2(const This& arr) {
	//	This::operator = (arr);
	//}

	Arr2(const Idx2 &dim) {
		(void)create(dim);
	}

	/** Creates array */
	bool create(const Idx2 &dim) {
		this->ibegin.set(0);
		this->iend = dim;
		const Index tmp[2] = {dim.n1, dim.n2};
		return Base::create(tmp, 2);
	}

	/** Creates array */
	bool create(Index n1, Index n2) {
		this->ibegin.set(0);
		this->iend.set(n1, n2);
		const Index tmp[2] = {n1, n2};
		return Base::create(tmp, 2);
	}

	/** Returns array beginning */
	inline const Idx2& begin() const {
		return ibegin;
	}
	
	/** Returns array end */
	inline const Idx2& end() const {
		return iend;
	}
	
	/** Returns array dimensions */
	inline const Idx2& getDimensions() const {
		return iend;
	}
	
	/** Fills array with a specified element */
	inline void fill(const Idx2 &begin, const Idx2 &end, const Element &element) {
		Idx2 i;
		for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
			for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
				(*this)(i) = element;
	}
	
	/** Copies elements from a specified array */
	inline void copy(const Idx2 &begin, const Idx2 &end, const This &arr) {
		Idx2 i;
		for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
			for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
				(*this)(i) = arr(i);
	}
	
	/** Copies elements from a specified array */
	inline void copy(const Idx2 &begin1, const Idx2 &begin2, const Idx2 &end, const This &arr) {
		Idx2 i, j;
		for (i.n1 = begin1.n1, j.n1 = begin2.n1; i.n1 < end.n1; i.n1++, j.n1++)
			for (i.n2 = begin1.n2, j.n2 = begin2.n2; i.n2 < end.n2; i.n2++, j.n2++)
				(*this)(i) = arr(j);
	}
	
	/** Returns a reference to an element of the array. */
	inline const Element &operator () (const Idx2 &idx) const {
		return this->elements[getIndex(idx)];
	}
	inline Element &operator () (const Idx2 &idx) {
		return this->elements[getIndex(idx)];
	}

	/**	Deep copy. */
	inline bool clone(const This &arr) {
		if (!Base::clone(arr))
			return false;
		ibegin = arr.ibegin;
		iend = arr.iend;
		return true;
	}

	/** Calculates 1D index from 2D indices  */
	inline Index getIndex(const Idx2 &idx) const {
		return idx.n1 + iend.n1*idx.n2;
	}

	/** Calculates 1D index from 2D indices  */
	inline Index getIndex(Index n1, Index n2) const {
		return n1 + iend.n1*n2;
	}

	/** Calculates 2D indices from 1D index  */
	inline void getIndex(Idx2 &idx, Index i) const {
		idx.n2 = i/iend.n1;
		i -= iend.n1*idx.n2;
		idx.n1 = i;
	}
};

//------------------------------------------------------------------------------

/** Array index */
template <typename _Index>
class Idx3 {
public:
	/** Index type */
	typedef _Index Index;
	
	Index n1, n2, n3;

	Idx3() {}
	Idx3(Index n) : n1(n), n2(n), n3(n) {}
	Idx3(Index n1, Index n2, Index n3) : n1(n1), n2(n2), n3(n3) {}

	/** sets indices */
	inline void set(Index n) {
		this->n1 = n;
		this->n2 = n;
		this->n3 = n;
	}
	inline void set(Index n1, Index n2, Index n3) {
		this->n1 = n1;
		this->n2 = n2;
		this->n3 = n3;
	}
	
	/** sets indices to minimum value */
	inline void setMin(const Idx3& idx1, const Idx3& idx2) {
		this->n1 = std::min(idx1.n1, idx2.n1);
		this->n2 = std::min(idx1.n2, idx2.n2);
		this->n3 = std::min(idx1.n3, idx2.n3);
	}
	
	/** sets indices to maximum value */
	inline void setMax(const Idx3& idx1, const Idx3& idx2) {
		this->n1 = std::max(idx1.n1, idx2.n1);
		this->n2 = std::max(idx1.n2, idx2.n2);
		this->n3 = std::max(idx1.n3, idx2.n3);
	}
	
	/** tests for zero index */
	inline bool isZero() const {
		return n1 == 0 && n2 == 0 && n3 == 0;
	}

	/** tests for positive index */
	inline bool isPositive() const {
		return n1 > 0 && n2 > 0 && n3 > 0;
	}

	/** Returns a reference to the array indices. */
	inline const Index &operator [] (Index i) const {
		return (&n1)[i];
	}
	inline Index &operator [] (Index i) {
		return (&n1)[i];
	}

	/** true if all the members are smaller. */
	inline bool operator < (const Idx3& idx) const {
		return (n1 < idx.n1) && (n2 < idx.n2) && (n3 < idx.n3);
	}
	inline bool operator < (Index index) const {
		return (n1 < index) && (n2 < index) && (n3 < index);
	}

	/** true if all the members are larger. */
	inline bool operator > (const Idx3& idx) const {
		return (n1 > idx.n1) && (n2 > idx.n2) && (n3 > idx.n3);
	}
	inline bool operator > (Index index) const {
		return (n1 > index) && (n2 > index) && (n3 > index);
	}

	/** returns true if the two indices are equal. */
	inline bool operator == (const Idx3& idx) const {
		return (n1 == idx.n1) && (n2 == idx.n2) && (n3 == idx.n3);
	}
	inline bool operator == (Index index) const {
		return (n1 == index) && (n2 == index) && (n3 == index);
	}

	/** returns true if the two indices are unequal. */
	inline bool operator != (const Idx3& idx) const {
		return (n1 != idx.n1) || (n2 != idx.n2) || (n3 != idx.n3);
	}
	inline bool operator != (Index index) const {
		return (n1 != index) || (n2 != index) || (n3 != index);
	}
};

/** Generic 3-dimensional array template */
template <typename Index, typename Element>
class Arr3 : public ArrN<Index, Element> {
public:
	/** 3D Index */
	typedef golem::Idx3<Index> Idx3;
	typedef ArrN<Index, Element> Base;
	typedef Arr3<Index, Element> This;

protected:
	Idx3 ibegin, iend;
	Index n12;

public:
	Arr3() {
	}

	//Arr3(const This& arr) {
	//	This::operator = (arr);
	//}

	Arr3(const Idx3 &dim) {
		(void)create(dim);
	}

	/** Creates array */
	bool create(const Idx3 &dim) {
		this->ibegin.set(0);
		this->iend = dim;
		this->n12 = dim.n1*dim.n2;
		const Index tmp[3] = {dim.n1, dim.n2, dim.n3};
		return Base::create(tmp, 3);
	}
	
	/** Creates array */
	bool create(Index n1, Index n2, Index n3) {
		this->ibegin.set(0);
		this->iend.set(n1, n2, n3);
		this->n12 = n1*n2;
		const Index tmp[3] = {n1, n2, n3};
		return Base::create(tmp, 3);
	}
	
	/** Returns array beginning */
	inline const Idx3& begin() const {
		return ibegin;
	}
	
	/** Returns array end */
	inline const Idx3& end() const {
		return iend;
	}
	
	/** Returns array dimensions */
	inline const Idx3& getDimensions() const {
		return iend;
	}
	
	/** Fills array with a specified element */
	inline void fill(const Idx3 &begin, const Idx3 &end, const Element &element) {
		Idx3 i;
		for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
			for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
				for (i.n3 = begin.n3; i.n3 < end.n3; i.n3++)
					(*this)(i) = element;
	}
	
	/** Copies elements from a specified array */
	inline void copy(const Idx3 &begin, const Idx3 &end, const This &arr) {
		Idx3 i;
		for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
			for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
				for (i.n3 = begin.n3; i.n3 < end.n3; i.n3++)
					(*this)(i) = arr(i);
	}
	
	/** Copies elements from a specified array */
	inline void copy(const Idx3 &begin1, const Idx3 &begin2, const Idx3 &end, const This &arr) {
		Idx3 i, j;
		for (i.n1 = begin1.n1, j.n1 = begin2.n1; i.n1 < end.n1; i.n1++, j.n1++)
			for (i.n2 = begin1.n2, j.n2 = begin2.n2; i.n2 < end.n2; i.n2++, j.n2++)
				for (i.n3 = begin1.n3, j.n3 = begin2.n3; i.n3 < end.n3; i.n3++, j.n3++)
					(*this)(i) = arr(j);
	}
	
	/** Returns a reference to an element of the array. */
	inline const Element &operator () (const Idx3 &idx) const {
		return this->elements[getIndex(idx)];
	}
	inline Element &operator () (const Idx3 &idx) {
		return this->elements[getIndex(idx)];
	}

	/**	Deep copy. */
	inline bool clone(const This &arr) {
		if (!Base::clone(arr))
			return false;
		ibegin = arr.ibegin;
		iend = arr.iend;
		n12 = arr.n12;
		return true;
	}

	/** Calculates 1D index from 3D indices  */
	inline Index getIndex(const Idx3 &idx) const {
		return idx.n1 + iend.n1*idx.n2 + n12*idx.n3;
	}

	/** Calculates 1D index from 3D indices  */
	inline Index getIndex(Index n1, Index n2, Index n3) const {
		return n1 + iend.n1*n2 + n12*n3;
	}

	/** Calculates 3D indices from 1D index  */
	inline void getIndex(Idx3 &idx, Index i) const {
		idx.n3 = i/n12;
		i -= n12*idx.n3;
		idx.n2 = i/iend.n1;
		i -= iend.n1*idx.n2;
		idx.n1 = i;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_ARRN_H_*/
