// ==================================================================
// LaserExtractor
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of LaserExtractor.
//
// LaserExtractor is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// LaserExtractor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with LaserExtractor. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================


/**
 * LaserExtractor
 * \file main.cpp
 * \author Andrzej Pronobis
 * \date 2008-09-06
 */

#include "tools/sequentialadaboost/SequentialAdaBoostClassifier.h"
#include "shared/DataReader.h"

#include <string>
#include <iostream>
using namespace std;
using namespace categorical;

// The classifier
SequentialAdaBoostClassifier classifier;


// ------------------------------------------------------
oscar::dyntab_featuresInfo* readLaserFeatureConfigFile(string featureFilePath)
{
  // Open the file
  FILE* f = fopen(featureFilePath.c_str(), "r");
  if ( f == NULL )
    return 0;

  // Create list
  oscar::dyntab_featuresInfo* featureInfoList = new dyntab_featuresInfo(2);

  // Read
  int ret = 0;
  int op = 0;
  while ( !feof(f) )
  {
    FeatureInfo *fi = new FeatureInfo;

    ret = fscanf(f, "%s %d", fi->name, &op);
    if ( ret != 2 )
    {
      delete featureInfoList;
      delete fi;
      return 0;
    }

    // Read parameter
    if ( op == 1 )
    {
      fi->hasParam = true;
      if (fscanf(f, " %lf", &fi->param)!=1)
      {
        delete featureInfoList;
        delete fi;
        return 0;
      }
      cout << "- Using feature " << fi->name << " with param " << fi->param << endl;
    }
    else
      cout << "- Using feature " << fi->name << endl;

    // Add to list
    featureInfoList->add(fi);
  }

  cout << "Total number of features: " << featureInfoList->num() << endl;

  return featureInfoList;
}


// ------------------------------------------------------
int main(int argc, char ** argv)
{
  // Print header
  cout << "-----------------" << endl;
  cout << " Laser Extractor " << endl;
  cout << "-----------------" << endl;
  cout << endl;

  if (argc != 4)
  {
    cout << "Usage: laserextractor <feature_config_file> <data_config_file> <output_file>" << endl;
    return EXIT_FAILURE;
  }

  // Read command-line arguments
  string featureFilePath = argv[1];
  string dataConfigFilePath = argv[2];
  string outputFilePath = argv[3];

  // Read the feature list
  cout << "Reading feature list:" << endl;
  oscar::dyntab_featuresInfo* featureInfoList = readLaserFeatureConfigFile(featureFilePath);
  if (!featureInfoList)
  {
    cerr<<"Couldn't read the feature config file '"<<featureFilePath<<"'"<<endl;
    return EXIT_FAILURE;
  }
  if (featureInfoList->num() == 0)
  {
    cerr<<"No features defined in the config file!"<<endl;
    return EXIT_FAILURE;
  }

  // Open the files
  cout << "Opening files... ";
  DataReader dr;
  if (!dr.readDataConfigFile(dataConfigFilePath))
  {
    cerr<<endl;
    cerr<<"Couldn't read the data config file '"<<dataConfigFilePath<<"'"<<endl;
    return EXIT_FAILURE;
  }
  long framesNo=dr.getFramesNo();
  ofstream outputFile(outputFilePath.c_str());
  cout << "Done!" << endl;

  cout << "Extracting:" << endl;
  int lastProgress=-1;
  oscar::RangeExampleGeneral *rangeExample = new RangeExampleGeneral();
  for (long i=0; i<framesNo; ++i)
  {
    // Progress
    int progress = 100.0*((double)i)/((double)framesNo);
    if (progress!=lastProgress)
    {
      cout<<"["<<progress<<"%]"<<endl;
      lastProgress=progress;
    }

    // Read the laser scan
    CategoricalData::LaserScanPtr scan = new CategoricalData::LaserScan;
    if (!dr.readLaserScan(scan, false))
    {
      cerr<<endl;
      cerr<<"Couldn't read laser scans from file!"<<endl;
      return EXIT_FAILURE;
    }
    rangeExample->clean();
    if (rangeExample->setRanges(scan->scanBuffer.ranges.size(), &scan->scanBuffer.ranges[0])<0)
    {
      cerr<<endl;
      cerr<<"Couldn't set ranges!"<<endl;
      return EXIT_FAILURE;
    }

    // Read target
    CategoricalData::TargetPtr target = new CategoricalData::Target;
    if (!dr.readTarget(target, false))
    {
      cerr<<endl;
      cerr<<"Couldn't read target from file!"<<endl;
      return EXIT_FAILURE;
    }

    // Do they match?
    if (scan->frameNo!=target->frameNo)
    {
      cerr<<endl;
      cerr<<"Scan and target do not match!"<<endl;
      return EXIT_FAILURE;
    }

    // Extract features
    classifier.calcSelectedFeatures(rangeExample, featureInfoList);

    // Save to file
    outputFile<<target->targetNo<<" ";
    for (int j=0; j<featureInfoList->num(); j++)
    {
      FeatureInfo *fi = featureInfoList->getElement(j);
      outputFile<<j+1<<":"<<fi->result<<" ";
    }
    outputFile<<endl;

  }
  cout << "[100%] Done!" << endl;

  cout << "Closing files... ";
  cout << "Done!" << endl;

  // Clean up
  outputFile.close();
  delete rangeExample;
  delete featureInfoList;
}



