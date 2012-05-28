#ifndef KDEFUNCTORS_H
#define KDEFUNCTORS_H
#include "GridMapData.hh"
#include "GridDataFunctors.hh"
#include "SpatialGridMap.hh"
#include <vector>
//using namespace cogx::Math;
using namespace std;
namespace SpatialGridMap {
// For KDE updates. Will loop through nearby samples and
// compute the kernel weight at the given bloxel center.
class GDAddKDE {
private:
	double kernelRadius;
	const vector<vector<double> > &zPositions;
	const vector<vector<double> > &values;
	const vector<double> &xySqDistances;
	double total;
	double multiplier;
public:
	static FunctorType type() {
		return Split;
	}
	GDAddKDE(double radius, const vector<vector<double> > &_zPositions,
			const vector<vector<double> > &_values,
			const vector<double> &_xySqDistances, double _multiplier = 1.0) :
		kernelRadius(radius), zPositions(_zPositions), values(_values),
				xySqDistances(_xySqDistances), multiplier(_multiplier) {
		total = 0.0;
	}
	;

	inline double getTotal() {
		return total;
	}
	inline void operator()(GridMapData & data, double size, double xPos,
			double yPos, double zPos) {

		if (data.occupancy == OCCUPIED)
			return;

		// Loop over columns. Find the lowest-placed kernel that has a potential
		// effect on this bloxel. Compute the effect, and do the same until the
		// kernels are above the limit of effect.
		unsigned int nCols = values.size();

		for (unsigned int i = 0; i < nCols; i++) {
			unsigned int nSamplesInCol = values[i].size();

			//Loop until inside kernel range
			unsigned int j = 0;
			for (; j < nSamplesInCol; j++) {
				if (zPos - zPositions[i][j] < kernelRadius) {
					break;
				}
			}

			//Loop until outside kernel range
			for (; j < nSamplesInCol; j++) {
				double zDiff = zPositions[i][j] - zPos;
				if (zDiff > kernelRadius) {
					break;
				}
				double relZDiff = zDiff / kernelRadius;
				double sq = xySqDistances[i] + relZDiff * relZDiff;
				double kernelValue = values[i][j] * (1 - sq) * (1 - sq) * (1 - sq);
				if (kernelValue > 0.0) {
					data.pdf += multiplier * kernelValue;
					total += multiplier * kernelValue * size;
				}
			}
		}
	}
	;
};

// Functor is used to merge a temporary gridmap into a larger one
// The calling function is required to set the column before each call
class GDMergeKDE {
private:
	vector<Bloxel<GridMapData> > * cookieCutter;
	double zOffset;
	double total;
public:
	static FunctorType type() {
		return Split;
	}
	GDMergeKDE() {
		total = 0.0;
	}
	;

	inline double getTotal() {
		return total;
	}
	inline void setColumn(vector<Bloxel<GridMapData> > & col) {
		cookieCutter = &col;
	}
	// ZOffset = bottom position of column to merge with
	inline void setCutterZ(double z) {
		zOffset = z;
	}
	inline void operator()(GridMapData & data, double size, double xPos,
			double yPos, double zPos) {

		unsigned int bloxelCount = cookieCutter->size();

		double floor = zPos - size / 2;
		double celing = zPos + size / 2;
		double currSum = 0;

		//Loop to find relevant cookie cutter bloxels
		unsigned int i = 0;
		for (; i < bloxelCount; i++) {
			if (floor < (*cookieCutter)[i].celing + zOffset) {
				break;
			}
		}

		//Loop until cookie cutter bloxels are irrellevant
		for (; i < bloxelCount; i++) {
			if (floor >= celing) {
				break;
			}

			// Add value and size multiplier to sum
			// This is to weigh all partial bloxels depending on size
			double
					currCeling =
							celing > (*cookieCutter)[i].celing + zOffset ? (*cookieCutter)[i].celing
									+ zOffset
									: celing;
			currSum += (*cookieCutter)[i].data.pdf * (currCeling - floor);

			floor = currCeling;
		}

		//Divide with bloxel size to get correct value
		currSum /= size;
		data.pdf += currSum;
		total += currSum;
	}
	;
};

class KDEVolumeQuery {
private:
	double kernelRadius;
	const vector<vector<double> > &zPositions;
	const vector<vector<double> > &values;
	const vector<double> &xySqDistances;
	double total;
public:
	static FunctorType type() {
		return Split;
	}
	KDEVolumeQuery(double radius, const vector<vector<double> > &_zPositions,
			const vector<vector<double> > &_values,
			const vector<double> &_xySqDistances, double _multiplier = 1.0) :
		kernelRadius(radius), zPositions(_zPositions), values(_values),
				xySqDistances(_xySqDistances) {
		total = 0.0;
	}
	;

	inline double getTotal() {
		return total;
	}
	inline void operator()(GridMapData & data, double size, double xPos,
			double yPos, double zPos) {
		if (data.occupancy == OCCUPIED)
			return;

		// Loop over columns. Find the lowest-placed kernel that has a potential
		// effect on this bloxel. Compute the effect, and do the same until the
		// kernels are above the limit of effect.
		unsigned int nCols = values.size();

		for (unsigned int i = 0; i < nCols; i++) {
			unsigned int nSamplesInCol = values[i].size();

			//Loop until inside kernel range
			unsigned int j = 0;
			for (; j < nSamplesInCol; j++) {
				if (zPos - zPositions[i][j] < kernelRadius) {
					break;
				}
			}

			//Loop until outside kernel range
			for (; j < nSamplesInCol; j++) {
				double zDiff = zPositions[i][j] - zPos;
				if (zDiff > kernelRadius) {
					break;
				}
				double relZDiff = zDiff / kernelRadius;
				double sq = xySqDistances[i] + relZDiff * relZDiff;
				double kernelValue = values[i][j] * (1 - sq) * (1 - sq) * (1 - sq);
				if (kernelValue > 0.0) {
					total += kernelValue * size;
				}
			}
		}
	}
	;
	double getTotal() const {
		return total;
	}
	;
	void reset() {
		total = 0;
	}
	;
};
}
;
#endif //KDEFUNCTORS_H
