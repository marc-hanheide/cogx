#ifndef GRIDDATAFUNCTORS_H
#define GRIDDATAFUNCTORS_H

#include "FunctorTypes.hh"
#include "GridMapData.hh"
#include "SpatialGridMap.hh"
#include <iostream>
#include <algorithm>

namespace SpatialGridMap {
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

//
// --- Read functors ---
//

/**
 BloxelMax query functor, will find the maximum pdf value in query region
 Use getResult() to obtain result
 Use reset() to reset for a new query
 */
class GDProbMax {
private:
  bool first;
  double result;
public:
  static FunctorType type() {
    return Read;
  }
  GDProbMax() {
    reset();
  }
  ;
  inline void operator()(const GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    if (first) {
      result = data.pdf;
      first = false;
    } else {
      result = result < data.pdf ? data.pdf : result;
    }
  }
  ;
  double getResult() const {
    return result;
  }
  ;
  void reset() {
    first = true;
  }
  ;
};

/**
 BloxelSum query functor, will find the sum of pdf value times
 their bloxel height.
 Use getResult() to obtain result
 Use reset() to reset for a new query
 NOTE: You must multiply by the column base are to get the full integral!
 */
class GDProbSum {
private:
  double result;
public:
  static FunctorType type() {
    return Read;
  }
  GDProbSum() {
    reset();
  }
  ;
  inline void operator()(const GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    result = result + data.pdf * size;
  }
  ;
  double getResult() const {
    return result;
  }
  ;
  void reset() {
    result = 0;
  }
  ;
};

/**
 Calculates the denominator in measurment update rile
 Note that since we can only query on cone bloxels in a loop and not
 cone bloxels + all other bloxels, the denominator is first the sum of all bloxel
 probabilities and here we substract the bit that falls under the view cone. The
 normal formula is
 sum over all bloxels
 denominator += bloxel.prob * (1 - (bloxel_in_cone == true ? sensordetectionProb : 0 ))
 end sum
 denominator += pOut
 return denominator
 */
class GDMeasUpdateGetDenominator {
private:
  double sensorDetectionProb, denominator;
public:
  static FunctorType type() {
    return Read;
  }
  GDMeasUpdateGetDenominator(double sensorDetectionProb, double probDensitySum) :
    sensorDetectionProb(sensorDetectionProb), denominator(probDensitySum) {
  }
  ;
  inline void operator()(const GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    denominator -= data.pdf * size * sensorDetectionProb;
  }
  ;
  double getResult() const {
    return denominator;
  }
  ;
};

/**
 After running GDMUpdateGetDenominator and calculating denominator this is where we set the pdf
 update rule
 bloxel.prob += (bloxel.prob * (1 - (bloxel_in_cone == true ? sensordetectionProb : 0 ))) / denominator
 */
class GDUnsuccessfulMeasUpdate {
private:
  double denominator, sensorDetectionProb;
public:
  static FunctorType type() {
    return Split;
  }
  GDUnsuccessfulMeasUpdate(double denominator, double sensorDetectionProb) :
    denominator(denominator), sensorDetectionProb(sensorDetectionProb) {
  }
  ;
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    data.pdf -= data.pdf * (sensorDetectionProb / denominator);
  }
  ;
};

/**
 
 Checks if region is obstacle free
 */
class GDIsFree {
private:
  bool result;
public:
  static FunctorType type() {
    return Read;
  }
  GDIsFree() {
    reset();
  }
  ;
  inline void operator()(const GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    result = result && !data.occupancy == OCCUPIED;
  }
  ;
  double getResult() const {
    return result;
  }
  ;
  void reset() {
    result = true;
  }
  ;
};

//
// ---- Modifier functors ---
//

/**
 Set modifier functor, will set all bloxels to the value used in constructor
 */
class GDSet {
private:
  GridMapData value;
public:
  static FunctorType type() {
    return Replace;
  }
  GDSet(GridMapData value) :
    value(value) {
  }
  ;
  void setValue(GridMapData value) {
    this->value = value;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data = value;
  }
  ;
};

/**
 Set probability modifier functor, will set all bloxels to the probability value used in constructor
 */
class GDProbSet {
private:
  double value;
public:
  static FunctorType type() {
    return Replace;
  }
  GDProbSet(double value) :
    value(value) {
  }
  ;
  void setValue(double value) {
    this->value = value;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.pdf = value;
  }
  ;
};

/**
 Sets bloxel pdfs to constant value,
 but keeps track of total probability mass, and ignores bloxels
 that are OCCUPIED
 */
class GDProbInit {
private:
  double value;
  double total;
public:
  static FunctorType type() {
    return Iterate;
  }
  GDProbInit(double value) :
    value(value), total(0.0) {
  }
  ;
  double getTotal() {
    return total;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    if (data.occupancy != OCCUPIED) {
      total += (value - data.pdf) * size;
      data.pdf = value;
    }
  }
  ;
};

/**
 Probability scaling functor. Multiplies the pdf in each bloxel by a global scalar.
 Useful for normalizing the map.*/
class GDProbScale {
private:
  double factor;
public:
  static FunctorType type() {
    return Iterate;
  }
  GDProbScale(double factor) :
    factor(factor) {
  }
  ;
  void setFactor(double factor) {
    this->factor = factor;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.pdf *= factor;
  }
  ;
};

/**
 Add probability functor will add given probability to all bloxels in region
 */
class GDProbAdd {
private:
  double value;
public:
  static FunctorType type() {
    return Iterate;
  }
  GDProbAdd(double value) :
    value(value) {
  }
  ;
  void setValue(double value) {
    this->value = value;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.pdf += value;
  }
  ;
};

class GDMakeObstacle {
public:
  static FunctorType type() {
    return Iterate;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.occupancy = OCCUPIED;
  }
  ;
};

class GDMakeFree {
public:
  static bool type() {
    return Iterate;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.occupancy = FREE;
  }
  ;
};

class GDMakeUnknown {
public:
  static bool type() {
    return Iterate;
  }
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) const {
    data.occupancy = UNKNOWN;
  }
  ;
};

// This is mostly for testing of split functionality
class GDSplitZMult {
private:
  double value;
public:
  static FunctorType type() {
    return Split;
  }
  GDSplitZMult(double value) :
    value(value) {
  }
  ;
  inline void operator()(GridMapData & data, double size, double xPos,
      double yPos, double zPos) {
    data.pdf = value * zPos;
  }
  ;
  void setValue(double value) {
    this->value = value;
  }
};

/**
 Obtacle detection functors, to be used with cone queries
 bool operator()(MapData data) returns true if obstacle
 */

class GDIsObstacle {
public:
  static FunctorType type() {
    return Read;
  }
  inline bool operator()(const GridMapData & data) const {
    return data.occupancy == OCCUPIED;
  }
  ;
};

class GDObstacleOrUnknown {
public:
  static FunctorType type() {
    return Read;
  }
  inline bool operator()(const GridMapData & data) const {
    return data.occupancy == OCCUPIED || data.occupancy == UNKNOWN;
  }
  ;
};

inline double normalizePDF(GridMap<GridMapData> & map, double pTarget,
    double currentWeight = 0.0) {
  if (currentWeight <= 0.0) {
    GDProbSum sumFunctor;
    map.universalQuery(sumFunctor);
    currentWeight = sumFunctor.getResult();
  }
  //	currentWeight += pOut;
  double scaleFactor = pTarget / currentWeight;
  GDProbScale scaleFunctor(scaleFactor);
  map.universalQuery(scaleFunctor, false);
  //	pOut = pOut * (1/currentWeight);

  //TODO: This check can be removed
  GDProbSum sumFunctor;
  map.universalQuery(sumFunctor);
  currentWeight = sumFunctor.getResult();
  cout << "normalized to: " << currentWeight << endl;
  return scaleFactor;
}

}
; // end of namespace

#endif
