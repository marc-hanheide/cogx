// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * DataReader class.
 * \file DataReader.h
 * \author Andrzej Pronobis
 * \date 2008-08-18
 */


#ifndef __PLACE_DATA_READER__
#define __PLACE_DATA_READER__

// Place.SA
#include <CategoricalData.hpp>
// Boost
#include <boost/filesystem.hpp>
// Std
#include <string>
#include <list>


namespace categorical
{


/**
 * Reads data from disk.
 */
class DataReader
{
public:

  /** Constructor. */
  DataReader(): _framesNo(0), _useLaser(false), _useVision(false), _useOdometry(false)
  {}

  /** Reads the data config file. Must be invoked before any data is read.*/
  bool readDataConfigFile(std::string filePath);

  /** Reads an image. */
  bool readImage(const CategoricalData::ImagePtr image);

  /** Reads info about an image. Image data are not loaded. */
  bool readImageInfo(std::string &iPath, CategoricalData::ImagePtr image, bool checkFrameNo=true);

  /** Reads a scan. */
  bool readLaserScan(CategoricalData::LaserScanPtr scan, bool checkFrameNo=true);

  /** Reads an odometry. */
  bool readOdometry(CategoricalData::OdometryPtr odom, bool checkFrameNo=true);

  /** Reads target. */
  bool readTarget(CategoricalData::TargetPtr target, bool checkFrameNo=true);

  /** Reads a PGM image. */
  bool readPgmImage(CategoricalData::ImagePtr image, std::string filePath);

  bool isUseVision()
  { return _useVision; }

  bool isUseLaser()
  { return _useLaser; }

  bool isUseOdometry()
  { return _useOdometry; }

  long getFramesNo()
  { return _framesNo; }


private:

  long _framesNo;
  bool _useLaser;
  bool _useVision;
  bool _useOdometry;

  long _lastImageFrameNo;
  long _lastScanFrameNo;
  long _lastOdometryFrameNo;
  long _lastTargetFrameNo;

  std::string _imageDir;
  std::string _scansFile;
  std::string _targetFile;
  std::string _odometryFile;

  std::list<std::string> _imageFileNames;


private: // Pointers to next data samples

  std::list<std::string>::const_iterator _nextImageFileName;
  long _nextScanPos;
  long _nextOdometryPos;
  long _nextTargetPos;
};

}


#endif
