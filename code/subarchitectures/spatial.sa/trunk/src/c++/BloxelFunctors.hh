#ifndef BLOXELFUNCTORS_H
#define BLOXELFUNCTORS_H

#include "FunctorTypes.hh"
#include "GridMapData.hh" // for back-compatibility
#include <iostream>
#include <algorithm>

namespace SpatialGridMap { //temporary namespace for now

/**
 Query functor. The query functor is passed by reference to
 SpatialGridMaps query methods, the functor should contain an 
 void operator()(MapData & data, double size, double xPos, double yPos, double zPos)
 returning the value for the new bloxel.
 @param data the current data
 @param size size of the bloxel on the z-axis
 @param [x,y]Pos world coordinates of bloxel
 @param zPos the middle of the bloxel on the z-axis
 @return the supposed value of the bloxel

 Functor should also contain 
 FunctorType type()

 This defines how bloxels are modified before the functor is applied.

 Types available:
 Read - Iterates over bloxel reading each one, READ ONLY!
 (these functors should not take a reference, writing to data will cause wierd results)
 Replace - Replace affected area with one large bloxel and write to it
 Iterate - Iterates over bloxel writing new values to them,
 creates new bloxels at edges
 Split - Splits bloxel into many smaller and iterates over them,
 alowing for fine scaled changes

 */

template<typename MapData>
class BloxelFalse {
private:
public:
	static FunctorType type() {
		return Read;
	}
	BloxelFalse() {
	}
	;
	inline bool operator()(const MapData &data) const {
		return false;
	}
	;
};

/**
 BloxelMax query functor, will find the maximum value in query region
 Use getResult() to obtain result
 Use reset() to reset for a new query
 */
template<typename MapData>
class BloxelMax {
private:
	bool first;
	MapData result;
public:
	static FunctorType type() {
		return Read;
	}
	BloxelMax() :
		first(true) {
	}
	;
	inline void operator()(MapData data, double size, double xPos, double yPos,
			double zPos) {
		if (first) {
			result = data;
			first = false;
		} else {
			result = (result < data) ? data : result;
		}
	}
	;
	MapData getResult() const {
		return result;
	}
	;
	void reset() {
		first = true;
	}
	;
};

/**
 BloxelSum query functor, will find the sum of values times
 their bloxel height.
 Use getResult() to obtain result
 Use reset() to reset for a new query
 NOTE: You must multiply by the column base are to get the full integral!
 */
template<typename MapData>
class BloxelSum {
private:
	MapData result;
public:
	static FunctorType type() {
		return Read;
	}
	BloxelSum() :
		result() {
	}
	;
	inline void operator()(MapData data, double size, double xPos, double yPos,
			double zPos) {
		result = result + data * size;
	}
	;
	MapData getResult() const {
		return result;
	}
	;
	void reset() {
		result = MapData();
	}
	;
};

/**
 BloxelEquals modifier functor, will set all bloxels to the value used in constructor
 */
template<typename MapData>
class BloxelEquals {
private:
	MapData value;
public:
	static FunctorType type() {
		return Replace;
	}
	BloxelEquals(MapData value) :
		value(value) {
	}
	;
	inline MapData operator()(MapData & data, double size, double xPos,
			double yPos, double zPos) const {
		data = value;
	}
	;
};

/**
 BloxelAdd modifier functor, will add given value to all bloxels in region
 */
template<typename MapData>
class BloxelAdd {
private:
	MapData value;
public:
	static FunctorType type() {
		return Iterate;
	}
	BloxelAdd(MapData value) :
		value(value) {
	}
	;
	inline MapData operator()(MapData & data, double size, double xPos,
			double yPos, double zPos) const {
		data += value;
	}
	;
};

template<typename MapData>
class MakeObstacle {
public:
	static FunctorType type() {
		return Replace;
	}
	inline void operator()(MapData &data, double size, double xPos, double yPos,
			double zPos) const {
		data.occupancy = OCCUPIED;
	}
	;
};

template<typename MapData>
class MakeFree {
public:
	static FunctorType type() {
		return Replace;
	}
	inline void operator()(MapData &data, double size, double xPos, double yPos,
			double zPos) const {
		data.occupancy = FREE;
	}
	;
};

template<typename MapData>
class isRegionFree {
private:
	bool result;
public:
	static FunctorType type() {
		return Read;
	}
	isRegionFree() {
		result = true;
	}
	;
	bool getResult() {
		return result;
	}
	;
	inline void operator()(const MapData &data, double size, double xPos,
			double yPos, double zPos) {
		if (data.occupancy == OCCUPIED)
			result = false;
	}
	;
};

/**
 Obtacle detection functors, to be used with cone queries
 bool operator()(MapData data) returns true if obstacle
 */

/**
 Threshold obstacle decision functor, will return true iff a bloxel value
 is greater than the value provided in the constructor (using operator<)
 */
template<typename MapData>
class BloxelThreshold {
private:
	MapData threshold;
public:
	BloxelThreshold(MapData thr) :
		threshold(thr) {
	}
	;
	inline bool operator()(MapData data) const {
		return threshold < data;
	}
	;
};

template<typename MapData>
class isObstacle {
public:
	isObstacle() {
	}
	;
	inline bool operator()(MapData data) const {
		return data.occupancy == OCCUPIED ? true : false;
	}
	;
};

}
;

#endif
