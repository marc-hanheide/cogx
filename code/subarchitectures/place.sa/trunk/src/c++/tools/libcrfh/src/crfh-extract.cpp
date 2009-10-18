// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
* \file crfh.cpp
* \author Andrzej Pronobis
*
* CRFH Extractor - main file.
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QFile>

#include "global.h"
#include "CImage.h"
#include "CPerformance.h"
#include "CSystem.h"
#include "CCrfh.h"

using namespace std;


/**
* Possible tuning:
* - size of the gaussian kernel (how many sigmas)
*   Chosen to be 3. Setting to 5 does not give better recognition
*   performance.
* - skipBorderPixels
* - Using _sum-=i.value(); in CCrfh::filter()
*   Decreses the classification performance. We should not do it.
* - When filtering the min value should be:
*   + normalized wrt sum of all values (which is constant right?)
*   + normalized wrt maximum value (which varies from histogram to histogram)
*/


int main(int argc, char *argv[])
{
  aout << "----------------------" << endl;
  aout << " CRFH Extractor "<<VERSION<< endl;
  aout << "----------------------" << endl;
  aout << endl;

  if ((argc != 4) && (argc != 5))
  {
    aout << "Usage: crfh-extract <descriptor> <input_file> <output_file> [<min_value>]" << endl;
    aout << endl;
    aout << "Arguments:" << endl;
    aout << " <descriptor> - String defining the image descriptors used to build the histogram." << endl;
    aout << "                Descriptors must be separated with '+'." << endl;
    aout << "                Currently implemented descriptors:" << endl;
    aout << "                L(<scale>,<hist_bins>)" << endl;
    aout << "                Lx(<scale>,<hist_bins>), Ly(<scale>,<hist_bins>)" << endl;
    aout << "                Lxx(<scale>,<hist_bins>), Lxy(<scale>,<hist_bins>), Lyy(<scale>,<hist_bins>)" << endl;
    aout << "                Example: \"Lxx(8,28)+Lxy(8,28)+Lyy(8,28)+Lxx(2,28)+Lxy(2,28)+Lyy(2,28)\"" << endl;
    aout << " <input_file> - Input image file or a file containing a list of image files. "<< endl;
    aout << "                If the input file starts with @, it is interpreted as a list file." <<endl;
    aout << " <output_file> - Output file for generated histogram(s)." <<endl;
    aout << " <min_value> - Minimum value that should be preserved in the sparse representation." <<endl;
    return EXIT_FAILURE;
  }

  // Read command-line arguments
  QString sysDescr = argv[1];
  QString inputFileName = argv[2];
  QString outputFileName = argv[3];
  bool listFile = (inputFileName[0]=='@');
  double minHistValue = (argc==5)?atof(argv[4]):0.0;

  // Read list of images
  QStringList imageFileList;
  QStringList classLabelList;
  if (listFile)
  {
    inputFileName=inputFileName.right(inputFileName.size()-1);
    aout<<"* Reading list file: "<<inputFileName<<endl<<endl;

    // Open file
    QFile listFile(inputFileName);
    if (!listFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      aout<<"ERROR: Unable to open the list file!"<<endl;
      exit(-1);
    }
    QTextStream listFileStream(&listFile);

    // Read the lines
    while (!listFileStream.atEnd())
    {
      QStringList line = listFileStream.readLine().split(' ');
      imageFileList.append(line[0]);
      if (line.size()>1)
        classLabelList.append(line[1]);
      else
        classLabelList.append("");
    }

    // Close file
    listFile.close();
  }
  else
  {
    imageFileList.append(inputFileName);
    classLabelList.append("");
  }


  // Define a system
  CSystem syst(sysDescr);

  // Open the output file
  QFile outputFile(outputFileName);
  if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    aout<<"ERROR: Unable to open the output file "<<outputFileName<<endl;
    exit(-1);
  }
  QTextStream outputFileStream(&outputFile);


  // Extract features from each file
  aout<<"* Processing files:"<<endl;
  double avgTime=0;
  for(int i=0; i<imageFileList.size(); ++i)
  {
    QString percent=QString("%1").arg((((double)i+1)/((double)imageFileList.size()))*100.0, 5, 'f', 1, '0');
    aout << "("<<percent<<"%) " << imageFileList[i] << endl;

    // Load an image
    CImage image;
    if (image.loadFromFile(imageFileList[i]))
    {
      aout<<"ERROR: Unable to open the file "<<imageFileList[i]<<endl;
      exit(-1);
    }

    // Perform histogram extraction
    CPerformance::tic();
    CCrfh *crfh = syst.computeHistogram(image, 15);
    if (minHistValue>0) crfh->filter(minHistValue);
    crfh->normalize();
    avgTime+=CPerformance::toc(false);

    // Save the histogram to the output file
    if (classLabelList[i]!="")
      outputFileStream<<classLabelList[i]<<" ";
    crfh->serialize(outputFileStream);
    outputFileStream<<endl;

    // Delete the histogram
    delete crfh;
  }

  // Display average time
  avgTime/=imageFileList.size();
  aout<<endl<<"* Finished! Average processing time per image: "<<avgTime<<"s"<<endl<<endl;

  // Close the output file
  outputFile.close();

  exit(0);
}

