#ifndef DENSITY_SAMPLING_HPP
#define DENSITY_SAMPLING_HPP

#include <vector>
#include "Math.hpp"
#include "RelationEvaluation.hpp"
#include <Navigation/LocalGridMap.hh>
#include "SpatialGridMap.hh"
#include "GridMapData.hh"

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

class SampleCloud;

ostream &operator<<(ostream &o, const SampleCloud &c);
istream &operator>>(istream &o, SampleCloud &c);

class SampleCloud {
public:
	friend class DensitySampler;
	SampleCloud(const spatial::Object *o1, const spatial::Object *o2,
			spatial::SpatialRelationType rel, Vector3 offset,
			double intervalQuantum,
			int intervalMultiplier, //Set to 0 if all extents are 0 (i.e. single sample)
			int xExt, int yExt, int zExt, const vector<Matrix33> &orientations1,
			const vector<Matrix33> &orientations2);
	SampleCloud() :
		object1(0), object2(0) {
	}
	SampleCloud(const SampleCloud &a);
	~SampleCloud() {
		delete object1;
		delete object2;
	}
	;
	SampleCloud &operator=(const SampleCloud &a);

	friend ostream &spatial::operator<<(ostream &o, const SampleCloud &c);
	friend istream &spatial::operator>>(istream &o, SampleCloud &c);

	void compute(RelationEvaluator &evaluator);

	SampleCloud composit(const SampleCloud &B) const;

	void compact();
	void makePointCloud(Vector3 &center, double &interval, int &xExt, int &yExt,
			int &zExt, vector<double> &weights) const;

	void KernelDensityEstimation2D(Cure::LocalGridMap<double> &outMap,
			Vector3 cloudCenter, double kernelWidthFactor, double &total,
			double baseValue);

	vector<double> values;
private:

	spatial::Object *object1;
	spatial::Object *object2;
	spatial::SpatialRelationType rel;

	Vector3 sampleOffset;
	double sampleIntervalQuantum;
	double kernelRadius;
	int sampleIntervalMultiplier;
	int xExtent;
	int yExtent;
	int zExtent;
	vector<Matrix33> object1Orientations;
	vector<Matrix33> object2Orientations;

};

struct SampleCloudContainer {
	SampleCloud *cloud;
	string obj1Label;
	string obj2Label;
};

class DensitySampler {
public:
	DensitySampler(RelationEvaluator *evaluator) :
		m_evaluator(evaluator), m_orientationQuantization(4), m_sampleNumberTarget(
				5000), m_kernelWidthFactor(1.5) {
	}
	;
	~DensitySampler() {
		for (vector<SampleCloudContainer>::iterator it = m_sampleClouds.begin(); it
				!= m_sampleClouds.end(); it++) {
			delete it->cloud;
		}
	}

	void
	sampleBinaryRelationRecursively(
			const vector<spatial::SpatialRelationType> &relations, const std::vector<
					spatial::Object *> &objects, int currentLevel, Cure::LocalGridMap<
					double> &map, double &total, const std::vector<Vector3> &triangle =
					std::vector<Vector3>(), double baseOnness = 1.0);

	void
	sampleBinaryRelationSystematically(
			const std::vector<SpatialRelationType> &relations, const std::vector<
					spatial::Object *> &objects, const std::vector<string> &objectLabels,
			double cellSize, SampleCloud &outCloud);

	double kernelDensityEstimation3D(SpatialGridMap::GridMap<
			SpatialGridMap::GridMapData> &map,
			const vector<cogx::Math::Vector3> &centers, double interval, int xExtent,
			int yExtent, int zExtent, const vector<double> &values,
			double baseMultiplier, double totalWeight, const Cure::LocalGridMap<
					unsigned char> *lgm = 0);

	void setOrientationQuantization(int q) {
		m_orientationQuantization = q;
	}
	void setSampleNumberTarget(int n) {
		m_sampleNumberTarget = n;
	}
	void setKernelWidthFactor(double f) {
		m_kernelWidthFactor = f;
	}
	double getKernelWidthFactor() const {
		return m_kernelWidthFactor;
	}

	SampleCloud *
	tryLoadCloudFromFile(const string &supportObjectLabel,
			const string &onObjectLabel, SpatialRelationType type);
	void
	writeCloudToFile(const SampleCloud *cloud, const string &supportObjectLabel,
			const string &onObjectLabel, SpatialRelationType type);

	bool tryLoadOrientationsFromFile(const string &label);
	void writeOrientationsToFile(const string &label);

protected:
	RelationEvaluator * m_evaluator;

	SampleCloud *
	createRelativeSampleCloud(SpatialRelationType relationType, Object *o1,
			Object *o2, const vector<Matrix33> &orientations1,
			const vector<Matrix33> &orientations2, double cellSize);
	vector<SampleCloudContainer> m_sampleClouds;
	map<string, vector<Matrix33> > m_objectOrientations;

	//The actual number of orientations
	// will be this number cubed
	int m_orientationQuantization;

	//Number of positional samples in total to go for when sampling 
	//a distribution. 
	unsigned long m_sampleNumberTarget;

	//Width of kernels, relative to the distance between them.
	double m_kernelWidthFactor;
};

}
;

#endif

