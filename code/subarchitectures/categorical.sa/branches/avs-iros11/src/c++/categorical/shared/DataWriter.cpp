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
 * \file DataWriter.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-15
 */

#include "DataWriter.h"
#include "ConfigFile.h"
#include <cast/core/CASTTimer.hpp>
#include <fstream>
#include <sstream>

using namespace categorical;
using namespace std;
using namespace boost;
using namespace boost::filesystem;

// ------------------------------------------------------
const string dataConfigFileExt = ".data";
const string dataSubDir = "data";
const string imagesSubDir = "images";
const string odometryFile = "odometry.dat";
const string scansFile = "scans.dat";
const string targetFile = "target.dat";


// ------------------------------------------------------
boost::filesystem::path DataWriter::getImagesDirPathRel()
{
  return path(_baseName+"_"+dataSubDir) / imagesSubDir;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getScansFilePathRel()
{
  return path(_baseName+"_"+dataSubDir) / scansFile;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getOdometryFilePathRel()
{
  return path(_baseName+"_"+dataSubDir) / odometryFile;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getTargetFilePathRel()
{
  return path(_baseName+"_"+dataSubDir) / targetFile;
}


// ------------------------------------------------------
boost::filesystem::path DataWriter::getImagesDirPath()
{
  return path(_dirPath) / (_baseName+"_"+dataSubDir) / imagesSubDir;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getScansFilePath()
{
  return path(_dirPath) / (_baseName+"_"+dataSubDir) / scansFile;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getOdometryFilePath()
{
  return path(_dirPath) / (_baseName+"_"+dataSubDir) / odometryFile;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getTargetFilePath()
{
  return path(_dirPath) / (_baseName+"_"+dataSubDir) / targetFile;
}

// ------------------------------------------------------
boost::filesystem::path DataWriter::getDataConfigFilePath()
{
  return path(_dirPath) / (_baseName+dataConfigFileExt);
}


// ------------------------------------------------------
bool DataWriter::writeDataConfigFile(bool imagesSaved, bool scansSaved, bool odometrySaved, long framesSaved)
{
  ConfigFile dcf;
  dcf.setValue("", "FramesSaved", framesSaved);
  dcf.setValue("Vision", "Available", imagesSaved);
  dcf.setValue("Laser", "Available", scansSaved);
  dcf.setValue("Odometry", "Available", odometrySaved);
  if (imagesSaved)
    dcf.setValue("Vision", "DataDir", getImagesDirPathRel());
  if (scansSaved)
    dcf.setValue("Laser", "DataFile", getScansFilePathRel());
  if (odometrySaved)
    dcf.setValue("Odometry", "DataFile", getOdometryFilePathRel());
  dcf.setValue("Target", "DataFile", getTargetFilePathRel());

  return dcf.write(getDataConfigFilePath().string());
}


// ------------------------------------------------------
bool DataWriter::writeImage(const CategoricalData::ImagePtr image, Format format)
{
  // Create directory
  path p=getImagesDirPath();
  try
  {
    create_directories(p);
  }
  catch(basic_filesystem_error<path>)
  {
    return false;
  }

  // Create filePath
  stringstream sstr;
  sstr.width(6);
  sstr.fill('0');
  sstr<<_frameNo;
  sstr<<"_t";
  sstr.width(17);
  sstr.precision(6);
  sstr<<fixed<<(image->realTimeStamp.s+image->realTimeStamp.us*1e-06);
  p = p / (sstr.str());

  if (format==F_PGM)
  {
    return writePgmImage(image, p.string()+".pgm");
  }
  else
    return false;
}


// ------------------------------------------------------
bool DataWriter::copyImage(std::string sourceImage, const CategoricalData::ImagePtr image)
{
  // Create directory
  path p=getImagesDirPath();
  try
  {
    create_directories(p);
  }
  catch(basic_filesystem_error<path>)
  {
    return false;
  }

  // Create filePath
  stringstream sstr;
  sstr.width(6);
  sstr.fill('0');
  sstr<<_frameNo;
  sstr<<"_t";
  sstr.width(17);
  sstr.precision(6);
  sstr<<fixed<<(image->realTimeStamp.s+image->realTimeStamp.us*1e-06);
  p = p / (sstr.str());

  // Get extension of the source file
  path sourcePath(sourceImage);
  path destPath(p.string()+extension(sourcePath));

  // Copy file
  copy_file(sourcePath, destPath);
  return true;
}


// ------------------------------------------------------
bool DataWriter::writeLaserScan(const CategoricalData::LaserScanPtr scan)
{
  fstream f(getScansFilePath().string().c_str(), ios_base::out|ios_base::app);
  if (!f.is_open())
    return false;

  f.precision(6);
  f<<fixed<<_frameNo<<" "<<(scan->realTimeStamp.s+scan->realTimeStamp.us*1e-06)<<" ";
  f.unsetf(ios_base::fixed);
  f<<scan->scanBuffer.ranges.size()<<" ";
  for (unsigned int i=0; i<722; ++i)
  {
    if (i>=scan->scanBuffer.ranges.size())
      f<<0<<" ";
    else
      f<<scan->scanBuffer.ranges[i]<<" ";
  }
  f<<scan->scanBuffer.startAngle<<" "<<scan->scanBuffer.angleStep<<" ";
  f<<scan->scanBuffer.maxRange<<" "<<scan->scanBuffer.rangeRes<<endl;

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataWriter::writeOdometry(const CategoricalData::OdometryPtr odom)
{
  fstream f(getOdometryFilePath().string().c_str(), ios_base::out|ios_base::app);
  if (!f.is_open())
    return false;

  f.precision(6);
  f<<fixed<<_frameNo<<" "<<(odom->realTimeStamp.s+odom->realTimeStamp.us*1e-06)<<" ";
  f<<"0"<<" ";
  if (odom->odometryBuffer.odompose.size()>0)
    f<<odom->odometryBuffer.odompose[0].x<<" "<<odom->odometryBuffer.odompose[0].y<<" "<<odom->odometryBuffer.odompose[0].theta<<" ";
  else
    f<<0.0<<" "<<0.0<<" "<<0.0<<" ";
  if (odom->odometryBuffer.encoder.size()>0)
    f<<odom->odometryBuffer.encoder[0]<<" ";
  else
    f<<0<<" ";
  if (odom->odometryBuffer.encoder.size()>1)
    f<<odom->odometryBuffer.encoder[1]<<" ";
  else
    f<<0<<" ";
  if (odom->odometryBuffer.speed.size()>0)
    f<<odom->odometryBuffer.speed[0]<<" ";
  else
    f<<0.0<<" ";
  if (odom->odometryBuffer.speed.size()>1)
    f<<odom->odometryBuffer.speed[1]<<endl;
  else
    f<<0.0<<endl;

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataWriter::writeTarget(int targetNo, std::string targetName)
{
  fstream f(getTargetFilePath().string().c_str(), ios_base::out|ios_base::app);
  if (!f.is_open())
    return false;

  f<<_frameNo<<" "<<targetNo<<" "<<targetName<<endl;

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataWriter::writePgmImage(const CategoricalData::ImagePtr image, std::string path)
{
  int width=image->imageBuffer.width;
  int stride=image->imageBuffer.width*3;
  int height=image->imageBuffer.height;

  FILE* file;
  file = fopen(path.c_str(),"wb");
  if (!file)
    return false;

  // Print header
  fprintf(file,"P5\n%u %u\n255\n", width, height);

  // Print rows
//  unsigned char *row = new unsigned char[width];
//  for(int i=0; i<height; ++i)
//  {
//    // Get one row
//    int shift=i*stride;
//    for(int j=0; j<width; ++j)
//    {
//      int pos = shift+j*3;
//      row[j]=static_cast<unsigned char>( ( static_cast<unsigned int>(image->imageBuffer.data[pos])*299   +   // R
//                                           static_cast<unsigned int>(image->imageBuffer.data[pos+1])*587 +   // G
//                                           static_cast<unsigned int>(image->imageBuffer.data[pos+2])*114     // B
//                                         ) / 1000 );
//    }

    unsigned char *row = new unsigned char[width];
    for(int i=0; i<height; ++i)
    {
      // Get one row
      int shift=i*width;
      for(int j=0; j<width; ++j)
      {
        int pos = shift+j;
        row[j]=image->imageBuffer.data[pos];
      }

    // Write row
    fwrite(row, 1, width, file);
    // Yield
    sched_yield();
  }

  delete row;
  fclose(file);

  return true;
}


