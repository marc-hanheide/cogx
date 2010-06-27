#ifndef DENSITY_SAMPLING_HPP
#define DENSITY_SAMPLING_HPP

#include <vector>
#include "Math.hpp"
#include "RelationEvaluation.hpp"
#include <Navigation/LocalGridMap.hh>

using namespace std;
using namespace cogx::Math;

// Represents an axis-aligned equidistant-sampled box shape, whose position 
// is relative to the position (but not the orientation) of a base object
// (object1). The points represent samples of the distribution of a specific
// spatial relation between object1 and another object, object2.
// For each spatial sample point are maintained the values of the relation,
// if object2 were in that position, and there is one value for each of a
// finite set of orientations of object1 and object2.

// Exactly which orientations can be specified externally. For example,
// if the pose of object1 is actually known, there needs be only one.

// Two clouds A and B can be composited together; this presumes that A.object2 
// and B.object1 are the same object, and that the set of
// orientations of A.object2 and B.object1 are also the same.
// Chaining the cloud gives rise to a new cloud C, with
// C.object1 = A.object1
// C.orientations1 = A.orientations1
// C.object2 = B.object2
// C.orientations2 = B.orientations2
// C.samplingInterval = smallest common prime factor of A.samplingInterval and
//	B.samplingInterval
// C.sampleOffset = A.sampleOffset + B.sampleOffset
// 
// The values of each sample point of C are acquired by "convolving" 
// A and B; i.e., B is placed in every point in A and each of B's points weighted
// by the A point is put into C.

// Finally, a cloud can be KDE-sampled into a grid. This grid should have a cell size
// such that the sampling interval of the cloud is an integer multiple of it, or 
// aliasing effects will distort the result (more than necessary).

namespace spatial {

class SampleCloud
{
  public:
    SampleCloud(const spatial::Object *o1, const spatial::Object *o2, 
	Vector3 offset,
	double intervalQuantum,
	int intervalMultiplier, //Set to 0 if all extents are 0 (i.e. single sample)
	int xExt,
	int yExt,
	int zExt,
	const vector<Matrix33> &orientations1,
	const vector<Matrix33> &orientations2);
    SampleCloud() : object1(0), object2(0) {}
    SampleCloud(const SampleCloud &a);
    ~SampleCloud() { delete object1; delete object2; };
    SampleCloud &operator=(const SampleCloud &a);

    void compute(spatial::SpatialRelationType rel);

    SampleCloud composit(const SampleCloud &B) const;

    void KernelDensityEstimation2D(Cure::LocalGridMap<double> &outMap,
	Vector3 cloudCenter, double kernelWidthFactor, double &total, double baseValue);

    vector<double> values;
  private:

    spatial::Object *object1;
    spatial::Object *object2;

    Vector3 sampleOffset;
    double sampleIntervalQuantum;
    int sampleIntervalMultiplier;
    int xExtent;
    int yExtent;
    int zExtent;
    vector<Matrix33> object1Orientations;
    vector<Matrix33> object2Orientations;

};

void 
sampleBinaryRelationRecursively(const vector<spatial::SpatialRelationType> &relations,
      const std::vector<spatial::Object *> &objects,
    int currentLevel, Cure::LocalGridMap<double> &map, 
    int sampleNumberTarget, int orientationQuantization, double kernelWidthFactor,
    double &total,
    const std::vector<Vector3> &triangle = std::vector<Vector3>(), double baseOnness = 1.0);

void sampleBinaryRelationSystematically(const std::vector <SpatialRelationType> &relations,
    const std::vector<spatial::Object *> &objects,
    Cure::LocalGridMap<double> &outMap,
    int sampleNumberTarget, int orientationQuantization, double kernelWidthFactor,
    double &total, double baseValue);
};
#endif

