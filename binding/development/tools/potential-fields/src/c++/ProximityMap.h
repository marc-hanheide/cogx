#ifndef POTENTIAL_FIELDS_PROXIMITY_MAP_H_
#define POTENTIAL_FIELDS_PROXIMITY_MAP_H_

#include "myVector.h"
#include "potentialField.h"
#include "scene.h"

#include <iostream>
#include <map>

using namespace std;


typedef myVector<float> FloatVector;
typedef potentialField<float> FloatField;
typedef scene<float> FloatScene;
typedef map<string, FloatField*> FloatFieldMap;

class ProximityMap {

 private:
  int m_width;
  int m_height;
  
  FloatVector * m_pViewerPosition;
  
  FloatField * m_pSceneField;
  FloatField * m_pDistractorField;

  FloatField * m_pCurrentLandmark;
  string m_currentLandmarkID;
  vector<string> * m_pCurrentDistractorIDs;


  FloatFieldMap * m_pDistractorMap;
  FloatFieldMap * m_pLinearDistanceMap;
  
  void excludeFromDistractors(const string &_id);
  void excludeFromDistractors(const string &_id1, const string &_id2);

  
 public:
  ProximityMap(int _width, int _height);
  ~ProximityMap();
  
  void addObject(const string & _id, const float & _x, 
		 const float & _y, const float & _z);

  void setViewerPosition(float _x, float _y, float _z);

  void makeMap(const string &_landmark);
  
  void makeMap(const string &_landmark, 
	       const vector<string> _distractors, 
	       bool _inhibitLandmark);

  float proximityValue(const string &_target, const string &_landmark);

  FloatField * getSceneField() {
    return m_pSceneField;
  }

  float getFieldValue(const string &_id);

  bool nextSweetSpot(float &_x, float &_y, float &_val);

};

#endif 

