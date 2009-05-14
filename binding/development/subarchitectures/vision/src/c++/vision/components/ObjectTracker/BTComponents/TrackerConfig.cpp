/** @file TrackerConfig.cpp
 *  @brief Manages the configuration of tracker.
 *  
 *  @author Somboon Hongeng
 *  @date march 2008
 *  @bug No known bugs.
 */
#include <fstream>
#include <map>
#include <sstream>
#include "TrackerConfig.h"
#include <string>

using namespace std;

TrackerConfig::TrackerConfig() {
}

TrackerConfig::~TrackerConfig() {
}

// need to catch readfile errors, dont have time now
//re-written to set defaults, then only overwrite if
//in config file. If no config file, or it fails to
//load, this is not an issue.
//There are still more readfile errors that could occur!
void TrackerConfig::configure(string filename){
  
    this->MAX_OBJECTS = 3;
    this->MAX_VIEWS = 2;
    this->MAX_CONTOURS = 5;
    this->DOFNUM = 3;
    this->PARTICLENUM = 300;
    this->SAMPLEPTNUM = 36;
    this->SAMPLE_ANGLE_NUM = 36;
    this->DEPTH_THRESHOLD = 7.0;
    this->DOWNSAMPLE = 2;
    this->OutOfRangeCOST = 30.0;
    this->MaxColorMatchingSCORE = 255.0;
    this->DISTTHRESH = 10;
    this->DOFNUM_BOX = 5;
    this->MAX_CONTOUR_NUM = 10;
    this->HAND_DOFNUM = 4;
    this->roi_is_set = false;
  if (filename=="") {
    return;
  };
  
  ifstream ifile(filename.c_str());
  if (ifile.fail()==1){
    cout<<"Config file not found/error reading in. Defaults being used"<<endl;
    return;
    };
  string strKey;
  string strValue;
  map<string,string> config;

  while(!ifile.eof()) {
    ifile >> strKey >> strValue;
    config[strKey] = strValue;
  }
  ifile.close();
  

/*
  char c;
  while((c=ifile.get()) && !ifile.eof()) {
    ifile.putback(c);
    ifile >> strKey;
    
    cout << "key: " << strKey.c_str() << endl; 
    
    ifile.getline(instring, 1024);

    config[strKey] = string(instring);
    
    cout << "value: " << config[strKey].c_str() << endl; 
  }
  ifile.close();
  */
  
  map<string,string>::iterator iter;
  if ((iter=config.find("MAX_OBJECTS")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->MAX_OBJECTS;
  }

  if ((iter=config.find("MAX_VIEWS")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->MAX_VIEWS;
  }
  
  if ((iter=config.find("MAX_CONTOURS")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->MAX_CONTOURS;
  }
    
  if ((iter=config.find("DOFNUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->DOFNUM;
  }
    
  if ((iter=config.find("PARTICLENUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->PARTICLENUM;
  }

  if ((iter=config.find("SAMPLEPTNUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->SAMPLEPTNUM;
  }

  if ((iter=config.find("SAMPLE_ANGLE_NUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->SAMPLE_ANGLE_NUM;
  }
  
  if ((iter=config.find("DEPT_THRESHOLD")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->DEPTH_THRESHOLD;
  }
  
  if ((iter=config.find("DOWNSAMPLE")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->DOWNSAMPLE;
  }

  if ((iter=config.find("OutOfRangeCOST")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->OutOfRangeCOST;
  }

  if ((iter=config.find("MaxColorMatchingSCORE")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->MaxColorMatchingSCORE;
  }

  if ((iter=config.find("DISTTHRESH")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->DISTTHRESH;
  }

  if ((iter=config.find("DOFNUM_BOX")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->DOFNUM_BOX;
  }
    
  if ((iter=config.find("MAX_CONTOUR_NUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->MAX_CONTOUR_NUM;
  }
    
  if ((iter=config.find("HAND_DOFNUM")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->HAND_DOFNUM;
  }

    
  if ((iter=config.find("ROI_CENTER_X")) != config.end()) {
    istringstream ss(iter->second);
    ss >> this->Roi.m_center.x;

    if ((iter=config.find("ROI_CENTER_Y")) != config.end()) {
      istringstream ss(iter->second);
      ss >> this->Roi.m_center.y;
    }
    if ((iter=config.find("ROI_WIDTH")) != config.end()) {
      istringstream ss(iter->second);
      ss >> this->Roi.m_size.x;
    }
    if ((iter=config.find("ROI_HEIGHT")) != config.end()) {
      istringstream ss(iter->second);
      ss >> this->Roi.m_size.y;
    }
    roi_is_set = true;
  }
}
