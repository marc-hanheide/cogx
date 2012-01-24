// ==================================================================
// Categorical.SA - Categorical Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Categorical.SA.
//
// Categorical.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Categorical.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Categorical.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * DataReader class.
 * \file DataReader.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-18
 */

// Categorical.SA
#include "DataReader.h"
#include "ConfigFile.h"
// CAST
#include <cast/core/CASTTimer.hpp>
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
// Std
#include <fstream>
#include <sstream>
#include <math.h>

using namespace categorical;
using namespace std;
using namespace boost;
using namespace boost::filesystem;


// ------------------------------------------------------
bool DataReader::readDataConfigFile(std::string filePath)
{
  _lastImageFrameNo=-1;
  _lastScanFrameNo=-1;
  _lastOdometryFrameNo=-1;
  _lastTargetFrameNo=-1;

  ConfigFile dcf;
  if (!dcf.read(filePath))
    return false;

  _framesNo=dcf.getIntValue("", "FramesSaved", 0);
  _useVision=dcf.getBoolValue("Vision", "Available", false);
  _useLaser=dcf.getBoolValue("Laser", "Available", false);
  _useOdometry=dcf.getBoolValue("Odometry", "Available", false);

  _imageDir=dcf.getStrValue("Vision", "DataDir", "");
  _scansFile=dcf.getStrValue("Laser", "DataFile", "");
  _targetFile=dcf.getStrValue("Target", "DataFile", "");
  _odometryFile=dcf.getStrValue("Odometry", "DataFile", "");

  // Check if what should be there is there
  if (_framesNo<=0)
    return false;
  if ((_useVision) && (_imageDir.empty()))
    return false;
  if ((_useLaser) && (_scansFile.empty()))
    return false;
  if ((_useOdometry) && (_odometryFile.empty()))
    return false;
  if (_targetFile.empty())
    return false;

  // Convert relative paths to absolute
  path dcfDirPath=path(filePath).remove_leaf();
  if (_imageDir[0]!='/')
    _imageDir=(dcfDirPath / path(_imageDir)).string();
  if (_scansFile[0]!='/')
    _scansFile=(dcfDirPath / path(_scansFile)).string();
  if (_odometryFile[0]!='/')
    _odometryFile=(dcfDirPath / path(_odometryFile)).string();
  if (_targetFile[0]!='/')
    _targetFile=(dcfDirPath / path(_targetFile)).string();

  // Read contents of the image directory
  if (_useVision)
  {
    _imageFileNames.clear();
    for (directory_iterator itr(_imageDir); itr!=directory_iterator(); ++itr)
      if (is_regular(itr->status()))
        _imageFileNames.push_back(itr->path().leaf());
    _imageFileNames.sort();
    _nextImageFileName = _imageFileNames.begin();
  }

  // Setup othr pointers
  _nextOdometryPos=0;
  _nextScanPos=0;
  _nextTargetPos=0;

  return true;
}


// ------------------------------------------------------
bool DataReader::readImage(CategoricalData::ImagePtr image)
{
  if (!_useVision)
    return false;

  // Get the image filename
  std::string imageFileName = *_nextImageFileName;

  // And parse
  vector<string> tokens;
  split(tokens, imageFileName, is_any_of("_t."));
  long frameNo;
  long seconds;
  long useconds;
  try
  {
    frameNo = lexical_cast<long>(tokens[0]);
    seconds = lexical_cast<long>(tokens[2]);
    useconds = lexical_cast<long>(tokens[3]);
  }
  catch (bad_lexical_cast)
  {
    cerr<<"Bad frame number or timestamp!"<<endl;
    return false;
  }
  path imagePath = path(_imageDir) / path(imageFileName);
  string ext = extension(imagePath);
  string imagePathStr = imagePath.string();

  // Check if the frame numbers are correct
  if (_lastImageFrameNo+1!=frameNo)
  {
    cerr<<"Incorrect frame number "<<frameNo<<". Should be "<<_lastImageFrameNo+1<<endl;
    return false;
  }

  // Set the image metadata
  image->status=CategoricalData::DsValid;
  image->realTimeStamp.s=seconds;
  image->realTimeStamp.us=useconds;
  image->frameNo=frameNo;

  // Read the image
  if (ext==".pgm")
  {
    if (!readPgmImage(image, imagePathStr))
      return false;
  }
  else
  {
    cerr<<"Unknown image file extension '"+ext+"'"<<endl;
    return false;
  }

  // Increment the pointers
  _nextImageFileName++;
  _lastImageFrameNo++;
  if (_nextImageFileName == _imageFileNames.end())
  {
    _nextImageFileName = _imageFileNames.begin();
    _lastImageFrameNo=-1;
  }

  return true;
}


// ------------------------------------------------------
bool DataReader::readImageInfo(std::string &iPath, CategoricalData::ImagePtr image, bool checkFrameNo)
{
  if (!_useVision)
    return false;

  // Get the image filename
  std::string imageFileName = *_nextImageFileName;

  // And parse
  vector<string> tokens;
  split(tokens, imageFileName, is_any_of("_t."));
  long frameNo;
  long seconds;
  long useconds;
  try
  {
    frameNo = lexical_cast<long>(tokens[0]);
    seconds = lexical_cast<long>(tokens[2]);
    useconds = lexical_cast<long>(tokens[3]);
  }
  catch (bad_lexical_cast)
  {
    cerr<<"Bad frame number or timestamp!"<<endl;
    return false;
  }
  path imagePath = path(_imageDir) / path(imageFileName);
  string ext = extension(imagePath);
  string imagePathStr = imagePath.string();

  // Check if the frame numbers are correct
  if ((checkFrameNo)&&(_lastImageFrameNo+1!=frameNo))
  {
    cerr<<"Incorrect frame number "<<frameNo<<". Should be "<<_lastImageFrameNo+1<<endl;
    return false;
  }

  // Set the image metadata
  image->status=CategoricalData::DsValid;
  image->realTimeStamp.s=seconds;
  image->realTimeStamp.us=useconds;
  image->frameNo=frameNo;

  iPath=imagePathStr;

  // Increment the pointers
  _nextImageFileName++;
  _lastImageFrameNo++;
  if (_nextImageFileName == _imageFileNames.end())
  {
    _nextImageFileName = _imageFileNames.begin();
    _lastImageFrameNo=-1;
  }

  return true;
}


// ------------------------------------------------------
bool DataReader::readLaserScan(CategoricalData::LaserScanPtr scan, bool checkFrameNo)
{
  if (!_useLaser)
    return false;

  ifstream f(_scansFile.c_str());
  if (!f.is_open())
    return false;

  // Go to next scan
  f.seekg(_nextScanPos);

  // Grab a line
  string line;
  getline(f, line);
  stringstream sstr(line);

  // Read the frameNo
  int frameNo;
  sstr>>frameNo;

  // Check if the frame numbers are correct
  if ((checkFrameNo)&&(_lastScanFrameNo+1!=frameNo))
  {
    cerr<<"Incorrect frame number "<<frameNo<<". Should be "<<_lastScanFrameNo+1<<endl;
    return false;
  }

  // Read timestamp
  string timestamp;
  sstr>>timestamp;
  vector<string> tokens;
  split(tokens, timestamp, is_any_of("."));
  long seconds;
  long useconds;
  try
  {
    seconds = lexical_cast<long>(tokens[0]);
    useconds = lexical_cast<long>(tokens[1]);
  }
  catch (bad_lexical_cast)
  {
    cerr<<"Bad timestamp!"<<endl;
    return false;
  }
  scan->realTimeStamp.s=seconds;
  scan->realTimeStamp.us=useconds;
  scan->scanBuffer.time.s=seconds;
  scan->scanBuffer.time.us=useconds;
  scan->frameNo=frameNo;

  // Read contents of scan
  int numpts;
  sstr>>numpts;
  scan->scanBuffer.ranges.resize(numpts);
  for (int i=0; i<numpts; ++i)
  {
    double r;
    sstr>>r;
    scan->scanBuffer.ranges[i]=r;
  }

  // Read the remaining zeros (the old laserserver stored always 722 values)
  for (int i=numpts; i<722; ++i)
  {
    double r;
    sstr>>r;
  }

  // Read the rest
  double tmp;
  sstr>>tmp;
  scan->scanBuffer.startAngle=tmp;
  sstr>>tmp;
  scan->scanBuffer.angleStep=tmp;
  sstr>>tmp;
  scan->scanBuffer.maxRange=tmp;
  scan->scanBuffer.minRange=0;
  sstr>>tmp;
  scan->scanBuffer.rangeRes=tmp;

  // Fill in missing scan fields
  scan->status = CategoricalData::DsValid;

  // Increment the pointers
  _lastScanFrameNo++;
  _nextScanPos=f.tellg();
  char c;
  f.get(c);
  if (f.eof())
  {
    _nextScanPos = 0;
    _lastScanFrameNo=-1;
  }
  f.unget();

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataReader::readOdometry(CategoricalData::OdometryPtr odom, bool checkFrameNo)
{
  if (!_useOdometry)
    return false;

  ifstream f(_odometryFile.c_str());
  if (!f.is_open())
    return false;

  // Go to next reading
  f.seekg(_nextOdometryPos);

  // Grab a line
  string line;
  getline(f, line);
  stringstream sstr(line);

  // Read the frameNo
  int frameNo;
  sstr>>frameNo;

  // Check if the frame numbers are correct
  if ((checkFrameNo)&&(_lastOdometryFrameNo+1!=frameNo))
  {
    cerr<<"Incorrect frame number "<<frameNo<<". Should be "<<_lastOdometryFrameNo+1<<endl;
    return false;
  }

  // Read timestamp
  string timestamp;
  sstr>>timestamp;
  vector<string> tokens;
  split(tokens, timestamp, is_any_of("."));
  long seconds;
  long useconds;
  try
  {
    seconds = lexical_cast<long>(tokens[0]);
    useconds = lexical_cast<long>(tokens[1]);
  }
  catch (bad_lexical_cast)
  {
    cerr<<"Bad timestamp!"<<endl;
    return false;
  }
  odom->realTimeStamp.s=seconds;
  odom->realTimeStamp.us=useconds;
  odom->odometryBuffer.time.s=seconds;
  odom->odometryBuffer.time.us=useconds;
  odom->frameNo=frameNo;

  // Read the rest
  double tmpD;
  long tmpL;
  sstr>>tmpL;
  //odom.odometryBuffer.m_flags=tmpL;
  sstr>>tmpD;
  odom->odometryBuffer.odompose.resize(1);
  odom->odometryBuffer.odompose[0].x=tmpD;
  sstr>>tmpD;
  odom->odometryBuffer.odompose[0].y=tmpD;
  sstr>>tmpD;
  odom->odometryBuffer.odompose[0].theta=tmpD;
  sstr>>tmpL;
  odom->odometryBuffer.encoder.resize(0);
  // odom.odometryBuffer.m_enc[0]=tmpL;
  sstr>>tmpL;
  // odom.odometryBuffer.m_enc[1]=tmpL;
  sstr>>tmpD;
  odom->odometryBuffer.speed.resize(0);
  // odom.odometryBuffer.m_speed=tmpD;
  sstr>>tmpD;
  // odom.odometryBuffer.m_rotspeed=tmpD;

  // Fill in missing fields
  odom->status = CategoricalData::DsValid;

  // Increment the pointers
  _lastOdometryFrameNo++;
  _nextOdometryPos=f.tellg();
  char c;
  f.get(c);
  if (f.eof())
  {
    _nextOdometryPos = 0;
    _lastOdometryFrameNo=-1;
  }
  f.unget();

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataReader::readTarget(CategoricalData::TargetPtr target, bool checkFrameNo)
{
  ifstream f(_targetFile.c_str());
  if (!f.is_open())
    return false;

  // Go to next reading
  f.seekg(_nextTargetPos);

  // Grab a line
  string line;
  getline(f, line);
  stringstream sstr(line);

  // Read the frameNo
  int frameNo;
  sstr>>frameNo;

  // Check if the frame numbers are correct
  if ((checkFrameNo)&&(_lastTargetFrameNo+1!=frameNo))
  {
    cerr<<"Incorrect frame number "<<frameNo<<". Should be "<<_lastTargetFrameNo+1<<endl;
    return false;
  }

  // Read the rest
  int tmpI;
  sstr>>tmpI;
  target->targetNo=tmpI;
  string tmpS;
  sstr>>tmpS;
  target->targetName=tmpS.c_str();

  // Fill in missing fields
  target->status = CategoricalData::DsValid;
  target->frameNo=frameNo;

  // Increment the pointers
  _lastTargetFrameNo++;
  _nextTargetPos=f.tellg();
  char c;
  f.get(c);
  if (f.eof())
  {
    _nextTargetPos = 0;
    _lastTargetFrameNo=-1;
  }
  f.unget();

  f.close();
  return true;
}


// ------------------------------------------------------
bool DataReader::readPgmImage(CategoricalData::ImagePtr image, std::string filePath)
{
  ifstream f(filePath.c_str(), ios_base::in|ios_base::binary);
  if (!f.is_open())
    return false;

  // Read the header
  char magic[2] = {'0', '0'};
  f.read(magic, 2);
  if ((magic[0]!='P') || (magic[1]!='5'))
  {
    cerr<<"Invalid magic number. Not a PGM file?"<<endl;
    return false;
  }

  // Read width&height
  int width;
  int height;
  int maxval;
  f>>width;
  f>>height;
  f>>maxval;
  if (maxval!=255)
  {
    cerr<<"Maximal value != 255."<<endl;
    return false;
  }
  if (f.eof())
  {
    cerr<<"Unexpected EOF."<<endl;
    return false;
  }

  // Skip white space
  char c;
  f.get(c);
  if (f.eof())
  {
    cerr<<"Unexpected EOF."<<endl;
    return false;
  }

  // Prepare image
  image->imageBuffer.data.resize(width*height);
  image->imageBuffer.width=width;
  image->imageBuffer.height=height;

  // Read the image
  char *row = new char[width];
  for (int i=0; i<height; ++i)
  {
    f.read(row, width);
    if (f.fail())
    {
      cerr<<"Unexpected EOF."<<endl;
      return false;
    }

    int shift=i*width;
    for(int j=0; j<width; ++j)
    {
      int pos = shift+j;
      image->imageBuffer.data[pos]=row[j];
    }

    // Yield
    sched_yield();
  }

  // Close
  f.close();
  return true;
}


