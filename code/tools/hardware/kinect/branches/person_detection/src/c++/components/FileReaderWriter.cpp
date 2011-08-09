/**
 * @file FileReaderWriter.h
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Read and write point clouds from/to files.
 */

#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace AR
{
  
bool writeToFile(const char *filename, cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud) 
{
  char *row = (char*) &cloud.rows;
  long rowsize = sizeof(cloud.rows);
  char *col = (char*) &cloud.cols;
  long colsize = sizeof(cloud.cols);
  char *data = (char*) cloud.data;
  long datasize = cloud.dataend - cloud.datastart;
  char *coldata = (char*) colCloud.data;
  long coldatasize = colCloud.dataend - colCloud.datastart;

  if(datasize != coldatasize) 
    cout << "FileReaderWriter: Warning: Invalid data size of clouds";
  
  ofstream file (filename, ios::out|ios::binary|ios::ate);
  if (file.is_open())
  {
    file.seekp(0, ios::beg);
    file.write (row, rowsize);
    file.write (col, colsize);
    file.write (data, datasize);
    file.write (coldata, datasize);
    file.close();
  }
  else
  {
    cout << "FileReaderWriter: Warning: Unable to open file";
    return false;
  }
  return true;
}

bool readFromFile(const char *filename, cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud) 
{
  long rowsize = sizeof(cloud.rows);
  long colsize = sizeof(cloud.cols);
  char *rowbuf = new char[rowsize];
  char *colbuf = new char[colsize];

  ifstream file (filename, ios::in|ios::binary|ios::ate);
  if (file.is_open())
  {
    file.seekg (0, ios::beg);

    // get size of rows/cols
    file.read (rowbuf, rowsize);
    file.read (colbuf, colsize);
    int *rows = (int*) rowbuf;
    int *cols = (int*) colbuf;
    
    // get data
    cloud = cv::Mat_<cv::Point3f>(*rows, *cols);
    char *data = (char*) cloud.data;
    long datasize = cloud.dataend - cloud.datastart;
    file.read(data, datasize);
    
    colCloud = cv::Mat_<cv::Point3f>(*rows, *cols);
    char *coldata = (char*) colCloud.data;
    long coldatasize = colCloud.dataend - colCloud.datastart;
    file.read(coldata, coldatasize);
    
    file.close();
  }
  else
  {
    cout << "FileReaderWriter: Warning: Unable to open file";
    return false;
  }
  return true;
}

}



























