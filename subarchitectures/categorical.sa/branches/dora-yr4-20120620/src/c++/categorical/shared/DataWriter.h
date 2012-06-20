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
 * DataWriter class.
 * \file DataWriter.h
 * \author Andrzej Pronobis
 * \date 2008-08-15
 */


#ifndef __PLACE_DATA_WRITER__
#define __PLACE_DATA_WRITER__

// Place.SA
#include <CategoricalData.hpp>
// Boost
#include <boost/filesystem.hpp>
// Std
#include <string>
#include <map>


namespace categorical
{


/**
 * Writes data to disk.
 */
class DataWriter
{
public:

  enum Format {F_PGM};


public:

  /** Constructor. */
  DataWriter(std::string dirPath, std::string baseName) :
    _dirPath(dirPath), _baseName(baseName)
  {}

  void setFrameNo(long frameNo)
    {_frameNo=frameNo;}

  /** Writes data config file. */
  bool writeDataConfigFile(bool imagesSaved, bool scansSaved, bool odometrySaved, long framesSaved);

  /** Writes image. */
  bool writeImage(const CategoricalData::ImagePtr image, Format format);

  /** Copies an image from the source file. The file name is recreated based on the CategoricalData::Image. */
  bool copyImage(std::string sourceImage, const CategoricalData::ImagePtr image);

  /** Writes scan. */
  bool writeLaserScan(const CategoricalData::LaserScanPtr scan);

  /** Writes odometry. */
  bool writeOdometry(const CategoricalData::OdometryPtr odom);

  /** Writes target. */
  bool writeTarget(int targetNo, std::string targetName);


private:

  boost::filesystem::path getImagesDirPathRel();
  boost::filesystem::path getScansFilePathRel();
  boost::filesystem::path getOdometryFilePathRel();
  boost::filesystem::path getTargetFilePathRel();
  boost::filesystem::path getImagesDirPath();
  boost::filesystem::path getScansFilePath();
  boost::filesystem::path getOdometryFilePath();
  boost::filesystem::path getTargetFilePath();
  boost::filesystem::path getDataConfigFilePath();

  bool writePgmImage(const CategoricalData::ImagePtr image, std::string path);

private:

  std::string _dirPath;
  std::string _baseName;
  long _frameNo;
};

}


#endif
