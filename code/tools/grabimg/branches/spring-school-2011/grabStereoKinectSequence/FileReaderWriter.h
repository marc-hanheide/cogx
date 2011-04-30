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

namespace AR
{
  
int writeToFile(char *filename, cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud) 
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
    std::cout << "FileReaderWriter: Warning: Invalid data size of clouds";
  
  std::ofstream file (filename, std::ios::out|std::ios::binary|std::ios::ate);
  if (file.is_open())
  {
    file.seekp(0, std::ios::beg);
    file.write (row, rowsize);
    file.write (col, colsize);
    file.write (data, datasize);
    file.write (coldata, datasize);
    file.close();
  }
  else std::cout << "FileReaderWriter: Warning: Unable to open file";
}

int readFromFile(char *filename, cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud) 
{
  long rowsize = sizeof(cloud.rows);
  long colsize = sizeof(cloud.cols);
  char *rowbuf = new char[rowsize];
  char *colbuf = new char[colsize];

  std::ifstream file (filename, std::ios::in|std::ios::binary|std::ios::ate);
  if (file.is_open())
  {
    file.seekg (0, std::ios::beg);

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
  else std::cout << "FileReaderWriter: Warning: Unable to open file";
  return 0;
}

}



























