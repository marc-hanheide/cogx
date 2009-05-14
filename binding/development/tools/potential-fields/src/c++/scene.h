#ifndef SCENE_H_
#define SCENE_H_

#include <vector>

#include "myVector.h"
#include "potentialField.h"

using namespace std;

template <class data_t> class scene {
	
  potentialField<data_t>* scene_pf;
  potentialField<data_t>* landmark_pf;
  vector<potentialField<data_t>*> distractor_pf_vec;
	
 public:
  scene(int row, int column, myVector<data_t>* origin);
				
  void computeProjectiveWaypoint(myVector<data_t>* viewerPosition, string direction, int MaxAngle, data_t dist_offset, int distractor_inhibition_dist);		
  void computeProximityWaypoint(data_t threshold, data_t distance_offset);
		
  void applyDistractors(int distractor_inhibition_dist);
		
  /*
   * Accessor Functions
   */
  int getRow() { return scene_pf->getRow(); }
		
  int getColumn() { return scene_pf->getColumn(); }
		
  data_t getPFValue(data_t i, data_t j) {
    if(scene_pf) {
      return scene_pf->getPFValue(i,j); 
    } else {
      cout << "Error: scene.getPFValue() - array == NULL" << endl;
      exit(EXIT_FAILURE);
    }
  }
		
  data_t** getPotentialField() { 
    if(scene_pf) {
      return scene_pf->getPotentialField(); 
    } else {
      cout << "Error: scene.getPotentialField() - array == NULL" << endl;
      exit(EXIT_FAILURE);
    }
  }		
		
  myVector<data_t>* getPFMax() { return scene_pf->getMaxLocation(); }		

  void fillScene(const data_t &val) {scene_pf->fillPF(val);}

  void setLandmark(int r, int c, myVector<data_t>* pos) { 	
    //TODO potential memory leak if used repeatedly
    landmark_pf = new potentialField<data_t>(r, c, pos);
  }

  void addAndApplyDistractor(int r, int c, myVector<data_t>* pos, int distractor_inhibition_dist); 
  void addDistractor(int r, int c, myVector<data_t>* pos);
		
  void removeDistractors() { 	
    distractor_pf_vec.clear();
  }
		
  void removeDistractor(int index) { 	
    distractor_pf_vec.erase(index);
  }

  vector<potentialField<data_t>*> getDistractorsVec() { 	
    return(distractor_pf_vec);
  }
};

/***************************************************************
 * 
 *  Function Definitions
 * 
 **************************************************************/

template <class data_t> 
scene<data_t>::scene(int r, int c, myVector<data_t>* origin) { 	
  scene_pf = new potentialField<data_t>(r, c, origin);
}
	
template <class data_t> 
void scene<data_t>::computeProjectiveWaypoint(myVector<data_t>* viewerPosition, 
					      string direction, 
					      int MaxAngle, 
					      data_t distance_offset, 
					      int distractor_inhibition_dist) { 		      
	
  landmark_pf->constructProjectivePF(viewerPosition, direction, MaxAngle);

  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());


  //		scene_pf->mergeByAddition(landmark_pf->getPotentialField());
  landmark_pf->constructOffsetDistancePF(distance_offset);
  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());	
		

  int vec_size = this->distractor_pf_vec.size();
  vector<potentialField<data_t>*> pf_ptr;
  for(int i = 0; i < vec_size; i++) {

    distractor_pf_vec[i]->constructDistractorPF(distractor_inhibition_dist);
    scene_pf->mergeByMultiplication(distractor_pf_vec[i]->getPotentialField());

  }

  landmark_pf->constructDistractorPF(16);
  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());
	
  landmark_pf->constructOffsetDistancePF(distance_offset);
  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());	
	
  scene_pf->normaliseByMax();

}

template <class data_t> void scene<data_t>::applyDistractors(int distractor_inhibition_dist) {
  int vec_size = this->distractor_pf_vec.size();
  vector<potentialField<data_t>*> pf_ptr;
  for(int i = 0; i < vec_size; i++) {
    distractor_pf_vec[i]->constructDistractorPF(distractor_inhibition_dist);
    scene_pf->mergeByMultiplication(distractor_pf_vec[i]->getPotentialField());
  }
}


template <class data_t> 
void scene<data_t>::addAndApplyDistractor(int r, int c, myVector<data_t>* pos, int distractor_inhibition_dist) {
  addDistractor(r,c,pos);
  distractor_pf_vec.back()->constructDistractorPF(distractor_inhibition_dist);
  scene_pf->mergeByMultiplication(distractor_pf_vec.back()->getPotentialField());
}
	
template <class data_t> 
void scene<data_t>::addDistractor(int r, int c, myVector<data_t>* pos) { 	
  distractor_pf_vec.push_back(new potentialField<data_t>(r, c, pos));
}
	

template <class data_t> 
void scene<data_t>::computeProximityWaypoint(data_t threshold, data_t distance_offset) { 	
	
  int r = scene_pf->getRow();
  int c = scene_pf->getColumn();

  landmark_pf->constructLinearDistancePF();
  landmark_pf->invert();
	
  //pf to hold max values at each point from all the distractors
  potentialField<data_t>* distractors_pf = new potentialField<data_t>(r, c,
								      new myVector<data_t>(0,0,0));
  //pf initialised to 1 so invert
  distractors_pf->invert();

	
  //set each point in distractor_pf field to the max value of any of the
  //distractors at that point
  int vec_size = this->distractor_pf_vec.size();
  for(int i = 0; i < vec_size; i++) {
    distractor_pf_vec[i]->constructLinearDistancePF();
    distractor_pf_vec[i]->invert();
    distractors_pf->mergeByMax(distractor_pf_vec[i]->getPotentialField());
  }
	
  data_t lv, dv;
  for(int i = 0; i < r; i++) {
    for(int j = 0; j < c; j++) {
      lv = landmark_pf->getPFValue(i,j);
      dv =  distractors_pf->getPFValue(i,j);
      //if the difference in distance between a point and the landmark
      //and that point and the nearest distractor is < threshold than
      //the proximity applicability at that point = 0
      if(( lv - dv) < threshold){
	scene_pf->setPFValue(i,j,0);
      } else {
	scene_pf->setPFValue(i,j,lv);				
      }
    }
  }
	
	
  landmark_pf->constructDistractorPF(20);
  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());
	
  landmark_pf->constructOffsetDistancePF(distance_offset);
  scene_pf->mergeByMultiplication(landmark_pf->getPotentialField());	
	
  scene_pf->normaliseByMax();
}


#endif /*SCENE_H_*/
