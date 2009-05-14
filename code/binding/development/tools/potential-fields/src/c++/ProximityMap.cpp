#include "ProximityMap.h"

#define INHIBITION_DISTANCE 15
//#define PROXIMITY_THRESHOLD 0.055
#define PROXIMITY_THRESHOLD 0.009

ProximityMap::ProximityMap(int _width, int _height) {

  
  m_width = _width;
  m_height = _height;

  //cout<<"Scene size: "<<m_width<<"x"<<m_height<<endl;
 

  m_pSceneField = new FloatField(m_width,
				 m_height, 
				 new FloatVector(0,0,0));
  
  
  m_pDistractorField = new FloatField(m_width,
				      m_height, 
				      new FloatVector(0,0,0));
  
  m_pDistractorMap = new FloatFieldMap();



  m_pLinearDistanceMap = new FloatFieldMap();

  m_pViewerPosition = NULL;

  m_pCurrentDistractorIDs = new vector<string>();
  m_pCurrentLandmark = NULL;

}


ProximityMap::~ProximityMap() {


  //cout<<"ProximityMap::~ProximityMap()"<<endl;

  for(FloatFieldMap::iterator i = m_pLinearDistanceMap->begin();
      i != m_pLinearDistanceMap->end();
      i++) {
    delete i->second;
    i->second = NULL;
  }
  
  for(FloatFieldMap::iterator i = m_pDistractorMap->begin();
      i != m_pDistractorMap->end();
      i++) {
    delete i->second;
    i->second = NULL;
  }

 m_pLinearDistanceMap->clear();
 m_pDistractorMap->clear();

 delete m_pLinearDistanceMap;
 delete m_pDistractorMap;

 delete m_pSceneField;
 delete m_pDistractorField;
 delete m_pViewerPosition;

  
 m_pCurrentDistractorIDs->clear();
 delete m_pCurrentDistractorIDs;
}


void ProximityMap::setViewerPosition(float _x, float _y, float _z) {

  if(m_pViewerPosition) {
    delete m_pViewerPosition;
  }

  //cout<<"Viewer Position: ("<<_x<<","<<_y<<","<<_z<<")"<<endl;

  m_pViewerPosition = new FloatVector(_x,
				      _y,
				      _z);

}


//TODO --- position checking!!
void ProximityMap::addObject(const string & _id, const float & _x, 
			     const float & _y, const float & _z) {


  //cout<<"Adding object: "<<_id<<"("<<_x<<","<<_y<<","<<_z<<")"<<endl;

//   FloatVector fv(_x,_y,_z);

//   FloatField f(m_width,
// 	       m_height, 
// 	       &fv);
  
  //TODO Check for previous ids

  //  create object as a landmark field
  (*m_pLinearDistanceMap)[_id] = new FloatField(m_width,
						m_height, 
						new FloatVector(_x,
								_y,
								_z));


  if(!(*m_pLinearDistanceMap)[_id]) {
    cerr<<"allocation error!"<<endl;
  }



  (*m_pLinearDistanceMap)[_id]->constructLinearDistancePF();
  (*m_pLinearDistanceMap)[_id]->invert();


// //   for(int k = 200; k < 201; k++) {
// //     for(int l = 0; l < m_height; l++) {
      
// //       cout<<(*m_pLinearDistanceMap)[_id]->getPFValue(k,l)<<endl;
      
// //     }
// //   }
  

  //and as a distractor field
  (*m_pDistractorMap)[_id] = new FloatField(m_width,
					    m_height, 
					    new FloatVector(_x,
							    _y,
							    _z));
  
  (*m_pDistractorMap)[_id]->constructDistractorPF(INHIBITION_DISTANCE);

//   if(!(*m_pDistractorMap)[_id]) {
//     cerr<<"allocation error!"<<endl;
//   }




  

}


void ProximityMap::excludeFromDistractors(const string &_id) {
  
  m_pCurrentDistractorIDs->clear();
  for(FloatFieldMap::iterator i = m_pDistractorMap->begin();
      i != m_pDistractorMap->end();
      i++) {
    if(i->first != _id) {
      m_pCurrentDistractorIDs->push_back(i->first);
    }
  }

}

//handy cheat to save vector overhead
void ProximityMap::excludeFromDistractors(const string &_id1,
					  const string &_id2) {
  
  m_pCurrentDistractorIDs->clear();
  for(FloatFieldMap::iterator i = m_pDistractorMap->begin();
      i != m_pDistractorMap->end();
      i++) {
    if((i->first != _id1) && (i->first != _id2)) {
      m_pCurrentDistractorIDs->push_back(i->first);
    }
  }

}


void ProximityMap::makeMap(const string &_landmark) {
  
  excludeFromDistractors(_landmark);
  makeMap(_landmark,*m_pCurrentDistractorIDs, true);


}

void ProximityMap::makeMap(const string &_landmark,
			   const vector<string> _distractors,
			   bool _inhibitLandmark) {

  FloatFieldMap::iterator i = m_pLinearDistanceMap->find(_landmark);

  if(i == m_pLinearDistanceMap->end()) {
    m_pCurrentLandmark = NULL;
    cerr<<"ProximityMap: Unknown landmark: "<<_landmark<<endl;
    return;
  }

  m_pCurrentLandmark = i->second;

  //fill with zeroes as per JK's code (instead of init and invert)
  m_pDistractorField->fillPF(0);
  //  m_pDistractorField->invert();
  

  //add each distractor field to the overall field
  for(vector<string>::const_iterator j = _distractors.begin();
      j < _distractors.end();
      j++) {

    //get the distractor field
    i = m_pLinearDistanceMap->find(*j);
    if(i != m_pLinearDistanceMap->end()) {
      //merge into overall distractor field
      m_pDistractorField->mergeByMax(i->second->getPotentialField());
      //cout<<"distractor: "<<*j<<endl;
    }
    else {
      cerr<<"ProximityMap: Distractor mismatch: "<<(*j)<<endl;
      return;
    }
  }  

  //construct the overall field from the landmark if over a particular
  //threshold

  //cout<<"landmark: "<<m_pCurrentLandmark->getRow()<<" "<<m_pCurrentLandmark->getColumn()<<endl;
  float landmarkValue, distractorValue;
  for(int k = 0; k < m_width; k++) {
    for(int l = 0; l < m_height; l++) {
      
      landmarkValue = m_pCurrentLandmark->getPFValue(k,l);

      
      distractorValue =  m_pDistractorField->getPFValue(k,l);


      
      //if the difference in distance between a point and the landmark
      //and that point and the nearest distractor is < threshold than
      //the proximity applicability at that point = 0
      
      //cout<<landmarkValue<<" - "<<distractorValue<<endl;
      
      if(( landmarkValue - distractorValue) < PROXIMITY_THRESHOLD) {
	m_pSceneField->setPFValue(k,
				  l,
				  0);
      } 
      else {
	m_pSceneField->setPFValue(k,
				  l,
				  landmarkValue);
      }
    }
  }


  if(_inhibitLandmark) {
    //get landmark distractor field
    i = m_pDistractorMap->find(_landmark);

    if(i == m_pDistractorMap->end()) {
      cerr<<"ProximityMap: Unknown landmark as distractor: "<<_landmark<<endl;
      return;
    }
    
    //add in a distractor field around the landmark to give a gap!
    m_pSceneField->mergeByMultiplication(i->second->getPotentialField());
  }


  FloatField * pLandmarkOffset = new FloatField(m_width,
						m_height, 
						new FloatVector(m_pCurrentLandmark->getOrigin()->getX(),
								m_pCurrentLandmark->getOrigin()->getY(),
								m_pCurrentLandmark->getOrigin()->getZ()));
  
  pLandmarkOffset->constructLinearDistancePF();
  pLandmarkOffset->invert();
  m_pSceneField->mergeByMultiplication(pLandmarkOffset->getPotentialField());


  //normalise resulting field
  m_pSceneField->normaliseByMax();
  
  delete pLandmarkOffset;

}


float ProximityMap::getFieldValue(const string &_landmark) {


  if(m_pCurrentLandmark != NULL) {

    //find the object
    FloatFieldMap::iterator i = m_pLinearDistanceMap->find(_landmark);
    if(i == m_pLinearDistanceMap->end()) {
      m_pCurrentLandmark = NULL;
      cerr<<"ProximityMap: Unknown landmark: "<<_landmark<<endl;
      return 0;
    }

    //return the value of the scene field at the point where the
    //landmark is

    //cout<<"getting pf value at: "<<i->second->getOrigin()->getX()<< " "<<i->second->getOrigin()->getY()<<endl;

    return m_pSceneField->getPFValue(i->second->getOrigin()->getX(),
				     i->second->getOrigin()->getY());

  }
  else {
    cerr<<"ProximityMap: No landmark set for reference"<<endl;
    return 0;
  }

}


float ProximityMap::proximityValue(const string &_target,
				   const string &_landmark) {

//   if(_landmark == "obj1") {
//     for(int k = 0; k < m_width; k++) {
//       for(int l = 0; l < m_height; l++) {
// 	cout<<(*m_pLinearDistanceMap)[_landmark]->getPFValue(k,l)<<endl;
//       }
//     }
//   }

//  cout<<"proximal?: "<<_target<<" "<<_landmark<<endl;

  //remove landmark and target from distractors
  excludeFromDistractors(_target,_landmark);

  makeMap(_landmark,*m_pCurrentDistractorIDs, false);
  
  //cout<<"proxVal in C++: "<<getFieldValue(_target)<<endl;
  return getFieldValue(_target);
}



bool ProximityMap::nextSweetSpot(float &_x, float &_y, float &_val) {

  if(m_pCurrentLandmark != NULL) {

    //get position and value of sweet spot
    FloatVector * pMax = m_pSceneField->getMaxLocation();
    _x = pMax->getX();
    _y = pMax->getY();
    _val = m_pSceneField->getPFValue(_x,_y);
    //cout << "inner val: "<<_val<<endl;

    //and then inhibit sweet spot in map
    FloatField sweetSpotDistractor(m_width,
				   m_height, 
				   new FloatVector(_x, _y, 0));

    sweetSpotDistractor.constructDistractorPF(INHIBITION_DISTANCE);

    m_pSceneField->mergeByMultiplicationNoNormalise(sweetSpotDistractor.getPotentialField());
    
    delete pMax;
    return true;
  }
  else {
    cerr<<"ProximityMap: No landmark set for reference for sweet spot"<<endl;
    return false;
  }


}
