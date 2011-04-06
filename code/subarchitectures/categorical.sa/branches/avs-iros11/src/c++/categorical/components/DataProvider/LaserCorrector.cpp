/**
 * \file LaserCorrector.cpp
 * \author Andrzej Pronobis
 */

#include "LaserCorrector.h"
#include <stddef.h>
#include <math.h>


using namespace std;

// ------------------------------------------------------
CategoricalLaserCorrector::CategoricalLaserCorrector(
		double mapSize, double cellSize, unsigned int scanCount,
		unsigned int medianFilterOrder, double maxRange) :
		_scanCount(scanCount), _mapSize(mapSize), _cellSize(cellSize),
		_medianFilterOrder(medianFilterOrder), _maxRange(maxRange)
{
	_mapCells = static_cast<int>(_mapSize/_cellSize)*2;
	_map = new unsigned char*[_mapCells];
	for (size_t i=0; i<_mapCells; ++i)
	{
		_map[i] = new unsigned char[_mapCells];
		for (size_t j=0; j<_mapCells; ++j)
			_map[i][j] = 0;
	}
}



// ------------------------------------------------------
CategoricalLaserCorrector::~CategoricalLaserCorrector()
{
	for (size_t i=0; i<_mapCells; ++i)
		delete [] _map[i];
	delete [] _map;
}


// ------------------------------------------------------
void CategoricalLaserCorrector::addScan(double x, double y, double theta,
		const std::vector<double> &ranges, double startAngle, double angleStep)
{
	// If the new scan is different enough
	bool different = false;
	if (_scans.size()>0)
	{
		Scan &lastScan = _scans.back();
		for (size_t i=0; i<ranges.size(); ++i)
		{
			if ((fabs(lastScan.ranges[i]-ranges[i]))>0.1)
			{
				different = true;
				break;
			}
		}
	}
	else
		different = true;

	if (different)
	{
		// Add it to the list
		_scans.push_back(Scan());
		_scans.back().x = x;
		_scans.back().y = y;
		_scans.back().theta = theta;
		_scans.back().ranges = ranges;
		_scans.back().startAngle = startAngle;
		_scans.back().angleStep = angleStep;

		if (_scans.size()>_scanCount)
			_scans.pop_front();

		// Rebuild the map
		rebuildMap();
	}
}


// ------------------------------------------------------
void CategoricalLaserCorrector::getCorrectedScan(std::vector<double> &ranges, double startAngle, double angleStep, unsigned int beamCount)
{
	vector<double> virtScan;
	getVirtualScan(virtScan, startAngle, angleStep, beamCount);
	medianFilter(virtScan, ranges, _medianFilterOrder);
}


// ------------------------------------------------------
void CategoricalLaserCorrector::rebuildMap()
{
	const Scan &lastScan = _scans.back();
	_lastTheta = lastScan.theta;

	// Clean the map
	for (size_t i=0; i<_mapCells; ++i)
	{
		for (size_t j=0; j<_mapCells; ++j)
			_map[i][j] = 0;
	}


	// Go over all scans
	for (list<Scan>::iterator it = _scans.begin(); it!=_scans.end(); ++it)
	{
		const Scan &scan = *it;
		double relX = scan.x - lastScan.x;
		double relY = scan.y - lastScan.y;

	    // Add obstacles
	    double angle = scan.startAngle;
	    for(size_t j=0; j<scan.ranges.size(); ++j)
	    {
	        double r = scan.ranges[j];
	        if (r<_maxRange)
	        {
	            double x = r * cos(angle + scan.theta) + relX;
	            double y = r * sin(angle + scan.theta) + relY;
	            int xCell=static_cast<int>(round((_mapSize+x)/_cellSize));
	            int yCell=static_cast<int>(round((_mapSize+y)/_cellSize));
	            _map[yCell][xCell] = 1;
	        }
	        angle+=scan.angleStep;
	    }
	}
}


// ------------------------------------------------------
void CategoricalLaserCorrector::getVirtualScan(std::vector<double> &ranges, double startAngle, double angleStep, unsigned int beamCount)
{
	// Prepare output
	ranges.resize(beamCount);

	// Generate virtual scan
	double angle = startAngle + _lastTheta;
	for(size_t i=0; i<beamCount; ++i)
	{
	    ranges[i] = _maxRange;
	    for(double r=0; r<=_maxRange; r+=(_cellSize/2.0))
	    {
	        double x = r * cos(angle);
	        double y = r * sin(angle);
	        int xCell = static_cast<int>(round((_mapSize+x)/_cellSize));
	        int yCell = static_cast<int>(round((_mapSize+y)/_cellSize));
	        if (_map[yCell][xCell]>0)
	        {
	    	    ranges[i] = r;
	            break;
	        }
	    }
	    angle+=angleStep;
	}

}


// ------------------------------------------------------
void CategoricalLaserCorrector::medianFilter(const std::vector<double> &in, std::vector<double> &out, unsigned int order)
{
	out = in;
//    virtRanges(1) = virtRanges(2);
//    virtRanges(end) = virtRanges(end-1);
}





//	  if (_convertToSick)
//	  {
//		  const double sickStartAngle = -1.5708;
//		  const double sickAngleStep = 0.017453;
//		  const int sickCount = 181;
//
//		  ranges = new double[sickCount];
//		  rangesCount = sickCount;
//		 // log("%f %f %d", scan.angleStep, scan.startAngle, scan.ranges.size());
//
//		  double angle = scan.startAngle;
//		  for (size_t i=0; i<scan.ranges.size(); ++i)
//		  {
//			  // Convert the hokuyo angle to the nearest sick angle
//			  if ((angle >= sickStartAngle) && (angle <= -sickStartAngle))
//			  {
//				  int j = ((angle - sickStartAngle)/sickAngleStep);
//				  if (j<sickCount)
//				  {
//	//log("%d %d %f", i, j, scan.ranges[i]);
//					  ranges[j] = scan.ranges[i];
//	//		  ranges[j] = scan.ranges[i];
//				  }
//			  }
//			  angle += scan.angleStep;
//		  }
//
//	      growNonMax(ranges, rangesCount);
//
//
//	  }
//	  else

//
//void CategoricalLaserProcessor::growNonMax(double *ranges, int size)
//{
//  const int winSize = 5;
//
//  double *ranges2 = new double[size];
//  for (int i=0; i<size; ++i)
//    ranges2[i] = ranges[i];
//
//
//  int winSize2 = winSize/2;
//
//    for (int i=winSize2; i<size-winSize2; ++i)
//    {
//	  bool isMax=false;
//	  double avg=0.0;
//	  int nonMaxCount=0;
//	  int maxCount=0;
//
//	for (int j=i-winSize2; j<i+winSize2+1; ++j)
//	{
//	  if (ranges2[j]<5.57)
//	  {
//	     nonMaxCount++;
//	     avg+=ranges2[j];
//	  }
//	  else
//	  {
//	      maxCount++;
//	  }
//	}
//
//	avg/=(double)(nonMaxCount);
//
//	if ((maxCount>0) && (nonMaxCount>0))
//	{
//	  for (int j=i-winSize2; j<i+winSize2+1; ++j)
//	  {
//	    if (ranges2[j]>=5.57)
//	    {
//	      ranges[j]=avg;
//	    }
//	  }
//
//
//	}
//    }
//
//
//   delete [] ranges2;
//}

