/**
 * \file LaserCorrector.h
 * \author Andrzej Pronobis
 */

#ifndef __CATEGORICAL_LASER_CORRECTOR__
#define __CATEGORICAL_LASER_CORRECTOR__

#include <list>
#include <vector>

/**
 *
 */
class CategoricalLaserCorrector
{
	struct Scan
	{
		double x;
		double y;
		double theta;
		std::vector<double> ranges;
		double startAngle;
		double angleStep;
	};


public:

	CategoricalLaserCorrector(double mapSize, double cellSize, unsigned int scanCount,
			unsigned int medianFilterOrder, double maxRange);

	~CategoricalLaserCorrector();

	/** Adds a new raw scan to the history. */
	void addScan(double x, double y, double theta,
			const std::vector<double> &ranges, double startAngle, double angleStep);

	/** Fills in the array with a corrected scan. */
	void getCorrectedScan(std::vector<double> &ranges, double startAngle, double angleStep, unsigned int beamCount);


private:

	void rebuildMap();
	void medianFilter(const std::vector<double> &in, std::vector<double> &out, unsigned int order);
	void getVirtualScan(std::vector<double> &ranges, double startAngle, double angleStep, unsigned int beamCount);


private:

	unsigned int _scanCount;
	double _mapSize;
	double _cellSize;
	unsigned int _mapCells;
	unsigned int _medianFilterOrder;
	double _maxRange;

	unsigned char **_map;
	double _lastTheta;

	std::list<Scan> _scans;

};


#endif
