#ifndef POTENTIALFIELD_H_
#define POTENTIALFIELD_H_


#define Z_ACCESSOR 0
//#define Z_ACCESSOR origin->getZ()

/*
 *  potentialField.h
 *  Created by jk on 30/05/2006.
 *  potentials range from 0 (black) to 1 (white).
 *  A low potential indicates that the location does not fulfil the constraint.
 *  A high potential indicates that the location does fulfil the constraint.
 *  You can use a hill-climbing algorithm to find the best location.
 *
 *  The invert() function will reverse the potentials.
 */

#include <new>
#include <string>

#include "myVector.h"


using namespace std;

template <class data_t> class potentialField {
  int row;
  int column;
  data_t** array;
  myVector<data_t>* origin;
		
 public:		
  data_t** allocateMatrix(int row, int column);

  /*
   * Constructor
   */
  potentialField();
  /*
   * Constructor
   */
  potentialField(int r, int c, data_t x, data_t y, data_t z);
  /*
   * Constructor
   */
  potentialField(int r, int c, myVector<data_t>* vec);
  /*
   * Destructor
   */
  ~potentialField() { 
    
    for (int i = 0; i < row; i++) {
      delete [] array[i];
    }

    delete [] array;

    delete origin; 
  }
				



  /*
   * Data Member Access Functions
   */
  int getRow() { return row; }
		
  int getColumn() { return column; }
		
  data_t** getPotentialField() { 
    if(array) {
      return array;
    } else {
      cout << "Error: potentialField.getPotentialField() - array == NULL" << endl;
      exit(EXIT_FAILURE);
    }
  }
		
  data_t getPFValue(data_t i, data_t j) {
    return array[(int)i][(int)j];
  }
		
  void setPFValue(data_t i, data_t j, data_t value) {
    array[(int)i][(int)j] = value;
  }
		
  void fillPF(const data_t &val);
		
  /*
   * 
   */
  void constructProjectivePF(myVector<data_t>* viewerPosition, string direction, int MaxAngle);
  void constructLinearDistancePF();
  void constructPowerDistancePF(int e);
  void constructOffsetDistancePF(data_t targetDist);
  void constructATanDistancePF();
  void constructDistractorPF(data_t inhibition_distance);
		
  void invert();
  void normaliseByMax();
  void roundPFValues();
  void mergeByMultiplicationNoNormalise(data_t** a);
  void mergeByAddition(data_t** a);
  void mergeByAdditionWeighted(data_t** a, data_t weight);
  void mergeByMultiplication(data_t** a);
  void mergeByMax(data_t** a);
  void outputField();
  data_t getMax();
  data_t getMin();
  myVector<data_t>* getOrigin() { return(origin); } 
  myVector<data_t>* getMinLocation();
  myVector<data_t>* getMaxLocation();
};

/*
 * Constructors
 */
 
template <class data_t> 
potentialField<data_t>::potentialField() { 
  row = 0; 
  column = 0; 
  array = NULL;
  origin = new myVector<data_t>(0, 0, 0);
}

template <class data_t> 		
potentialField<data_t>::potentialField(int r, int c, data_t x, data_t y, data_t z) { 
  row = r; 
  column = c; 
  array = allocateMatrix(r,c);
  if(array==NULL) {
    cout << "Error: potentialField(int, int) - array == NULL" << endl;
    exit(EXIT_FAILURE);
  }
  origin = new myVector<data_t>(x, y, z);
}

template <class data_t> 				
potentialField<data_t>::potentialField(int r, int c, myVector<data_t>* vec) { 
  row = r; 
  column = c; 
  array = allocateMatrix(r,c);
  if(array==NULL) {
    cout << "Error: potentialField(int, int) - array == NULL" << endl;
    exit(EXIT_FAILURE);
  }
  origin = new myVector<data_t>(vec);
}

template <class data_t> 				
void potentialField<data_t>::outputField() {
  if(array) {
    for (int i = 0; i < row; i++) {
      for(int j=0; j < column; j++) {
	cout << "pf[" << i << "][" << j << "] = " << array[i][j] << endl;
      }
    }
  } else {
    printf("potentialField: ERROR - array == NULL\n");
    exit(EXIT_FAILURE);
  }
}

template <class data_t> 				
data_t** potentialField<data_t>::allocateMatrix(int row, int column) {
  data_t** matrixPtr;
  int i,j;

  //Allocate space for pointers to rows of A matrix
  matrixPtr = new data_t*[row];

  if (!matrixPtr){
    cout << "Allocation for rows failed." << endl;
    exit(EXIT_FAILURE);
  } //End if 

  // Allocate space for entries of the columns of the matrix
  for (i = 0; i < row; i++) {
    matrixPtr[i] = new data_t[column];
    if (!matrixPtr[i]){//If failure, release memory allocated before ending program
      for (j = (i - 1); j >= 0; j--){
	delete [] matrixPtr[j];
      }
      delete [] matrixPtr;
      cout << "Allocation for columns of matrix failed.\n";
      exit(EXIT_FAILURE);
    } //End if !A
				
  }//End for i

  for(i=0; i<row; i++) {
    for(j=0; j<column; j++) {
      matrixPtr[i][j] = 1;
    }
  }


  return matrixPtr;
}


template <class data_t> 				
void potentialField<data_t>::invert() {
  for (int j = 0; j < row; j++) {
    for(int k = 0; k < column; k++) {
      array[j][k] = 1 - array[j][k];

/*       if(isnan(array[j][k])) { */
/* 	cout << "Error: potentialField::invert - dist isnan! for x = " << j << " y = " << k << endl; */
/* 	exit(EXIT_FAILURE); */
/*       } */

    }
  }
}


/*
 * This function assumes the the origin data members of this potential field define the location of the landmark
 * the potential field is to be constructed around.
 * The first argument defines the location of the viewer.
 * The second argument is one of: "front", "back", "left", "right".
 * The thrird argument is the maxiumum angle of the spatial template.
 */
template <class data_t> 				
void potentialField<data_t>::constructProjectivePF(myVector<data_t>* viewerPosition, string sDir, int MaxAngle) {	
  //get the viewers position in the local coordinate system - this defines the front vector
	
  //decide which direction is required and rotate the front vector as necessary
  myVector<data_t>* vDir;
  myVector<data_t>* tmp;
  if(sDir == "front") {
    tmp = new myVector<data_t>(viewerPosition);
    tmp->translate(origin);
    vDir = new myVector<data_t>(tmp);
  } else if (sDir == "back") {
    tmp = new myVector<data_t>(origin);
    tmp->translate(viewerPosition);
    vDir = new myVector<data_t>(tmp);
  } else if (sDir == "left") {
    tmp = new myVector<data_t>(viewerPosition);
    tmp->translate(origin); //tmp = front vector
    myVector<data_t>* vVertical = new myVector<data_t>(0.0, 0.0, 1.0);
    vDir = new myVector<data_t>(tmp->crossProduct(vVertical));
  } else if (sDir == "right") {
    tmp = new myVector<data_t>(viewerPosition);
    tmp->translate(origin);
    myVector<data_t>* vVertical = new myVector<data_t>(0.0, 0.0, 1.0);
    vDir = new myVector<data_t>(vVertical->crossProduct(tmp));
  } else {
    cout << "Error: potentialField::projectiveDistance - sDir not recognised" << endl;
    exit(EXIT_FAILURE);
  }
	
  myVector<data_t>* LocalPoint = new myVector<data_t>();
  for(int y = 0; y < row; y++) {
    for(int x = 0; x < column; x++) {		
      LocalPoint->setVec((data_t) x, (data_t) y, Z_ACCESSOR);
					
      LocalPoint->translate(origin);
      data_t LocalAngleDeviation;
      if(!LocalPoint->isOrigin()) {
	LocalAngleDeviation = LocalPoint->angleBetween(vDir);
	if(isnan(LocalAngleDeviation)) {
	  cout << "Error: potentialField::angularDeviation - LocalAngleDeviation isnan! for x = " << x << " y = " << y << endl;
	  exit(EXIT_FAILURE);
	}
      } else {
	LocalAngleDeviation = 0;
      }
			
      if(LocalAngleDeviation >= MaxAngle) {
	array[x][y] = 0;
      } else {	
	array[x][y] = 1 - (LocalAngleDeviation/MaxAngle);
      }
    }
  }
	
}


template <class data_t> 				
void potentialField<data_t>::constructATanDistancePF() {
  myVector<data_t>* LocalPoint = new myVector<data_t>();
  data_t dist, maxDist;
	
  for(int x = 0; x < row; x++) {
    for(int y = 0; y < column; y++) {		
      LocalPoint->setVec((data_t) x, (data_t) y, Z_ACCESSOR);
					
      LocalPoint->translate(origin);
      if(!LocalPoint->isOrigin()) {
	dist = LocalPoint->distanceFromOrigin();					
	if(isnan(dist)) {
	  cout << "Error: potentialField::distance - dist isnan! for x = " << x << " y = " << y << endl;
	  exit(EXIT_FAILURE);
	}
	array[x][y] = dist;
      } else {
	array[x][y] = 0;
      }		
    }
  }
		
  //to use the atan function we need to convert the distance into radians
  //to do this we fist convert distance to the degree range i.e. 0 to 360 
  //we then convert these degree values into rads
  maxDist = this->getMax();
  data_t deg, rad;
  for(int x = 0; x < row; x++) {
    for(int y = 0; y < column; y++) {		
      deg = ((array[x][y]/maxDist)*360);
      rad = ((2*PI)/360) * deg;
      array[x][y]=atan(rad);
    }
  }
  this->normaliseByMax();
}



template <class data_t> 				
void potentialField<data_t>::constructPowerDistancePF(int e) {
  myVector<data_t>* LocalPoint = new myVector<data_t>();
  data_t dist, eDist;
	
  for(int x = 0; x < row; x++) {
    for(int y = 0; y < column; y++) {		
      LocalPoint->setVec((data_t) x, (data_t) y, Z_ACCESSOR);
					
      LocalPoint->translate(origin);
      if(!LocalPoint->isOrigin()) {
	dist = LocalPoint->distanceFromOrigin();					
	if(isnan(dist)) {
	  cout << "Error: potentialField::distance - dist isnan! for x = " << x << " y = " << y << endl;
	  exit(EXIT_FAILURE);
	}
	array[x][y] = pow(dist,e);
      } else {
	array[x][y] = 0;
      }		
    }
  }
  normaliseByMax();
}



template <class data_t> 				
void potentialField<data_t>::constructLinearDistancePF() {

  myVector<data_t>* LocalPoint = new myVector<data_t>(0,0,0);

  double dist = 0;
  
  for(int x = 0; x < row; x++) {

    for(int y = 0; y < column; y++) {		

      LocalPoint->setVec((data_t) x, (data_t) y, Z_ACCESSOR);
					
      LocalPoint->translate(origin);

      if(!LocalPoint->isOrigin()) {

	dist = LocalPoint->distanceFromOrigin();

	if(isnan(dist)) {
	  cout << "Error: potentialField::distance - dist isnan! for x = " << x << " y = " << y << endl;
	  exit(EXIT_FAILURE);
	}

	array[x][y] = dist;

      } 
      else {

	array[x][y] = 0;
      }
    }
  }

  normaliseByMax();

}

template <class data_t> 				
void potentialField<data_t>::constructDistractorPF(data_t inhibition_distance) {
  myVector<data_t>* LocalPoint = new myVector<data_t>();
  double dist;
  for(int x = 0; x < row; x++) {
    for(int y = 0; y < column; y++) {		
      
      LocalPoint->setVec((data_t) x, (data_t) y, Z_ACCESSOR);
					
      LocalPoint->translate(origin);
  
      if(!LocalPoint->isOrigin()) {
	
	dist = LocalPoint->distanceFromOrigin();
	
	if(isnan(dist)) {
	  cout << "Error: potentialField::distance - dist isnan! for x = " << x << " y = " << y << endl;
	  exit(EXIT_FAILURE);
	}
	if(dist > inhibition_distance) {
	  array[x][y] = 1;
	} 
	else {
	  array[x][y] = 0;
	}
      } 
      else {
	array[x][y] = 0;
      }
    }
  }
}



template <class data_t> 				
void potentialField<data_t>::constructOffsetDistancePF(data_t targetDist) {
  int x, y;
  data_t dist;
  this->constructLinearDistancePF();
  this->invert();
  for(x = 0; x < row; x++) {
    for(y = 0; y < column; y++) {
      dist = (array[x][y]);
      array[x][y] = 1 - fabs(targetDist - dist);
    }
  }
  normaliseByMax();
}


template <class data_t> 				
data_t potentialField<data_t>::getMax() {
  double max = 0;
		
  int i, j;
  for(i = 0; i < row; i++) {
    for(j = 0; j < column; j++) {
      if(max < array[i][j]) {
	max = array[i][j];
      }
    }
  }
  return max;
}

template <class data_t> 				
data_t potentialField<data_t>::getMin() {
  double min;
  if(array) {
    min = array[0][0];
		
    int i, j;
    for(i = 0; i < row; i++) {
      for(j = 0; j < column; j++) {
	if(min < array[i][j]) {
	  min = array[i][j];
	}
      }
    }
  }
  return min;
}

template <class data_t> 				
myVector<data_t>* potentialField<data_t>::getMaxLocation() {
  myVector<data_t>* maxloc = new myVector<data_t>();
  maxloc->setVec(0,0,0);
	
  data_t max;
  if(array) {
    max = array[0][0];
		
    int i, j;
    for(i = 0; i < row; i++) {
      for(j = 0; j < column; j++) {
	if(max < array[i][j]) {
	  max = array[i][j];
	  maxloc->setVec(i,j,0);
	  //cout<<"max: "<<max<<endl;
	}
      }
    }
  }
  return maxloc;
}

template <class data_t> 				
myVector<data_t>* potentialField<data_t>::getMinLocation() {
  myVector<data_t>* minloc = new myVector<data_t>();
  minloc->setVec(0,0,0);
	
  data_t min;
  if(array) {
    min = array[0][0];
		
    int i, j;
    for(i = 0; i < row; i++) {
      for(j = 0; j < column; j++) {
	if(min > array[i][j]) {
	  min = array[i][j];
	  minloc->setVec(i,j,0);
	}
      }
    }
  }
  return minloc;
}

template <class data_t> 				
void potentialField<data_t>::normaliseByMax() {
  int i, j;
  
  data_t max = this->getMax();
  
  if(max == 0) {
    return;
  }

  for(i = 0; i < row; i++) {
    for(j = 0; j < column; j++) {
      array[i][j] = array[i][j]/max;
    }
  }
}

template <class data_t> 				
void potentialField<data_t>::roundPFValues() {
  int i, j;
  data_t max = this->getMax();
  for(i = 0; i < row; i++) {
    for(j = 0; j < column; j++) {
      array[i][j] = round(array[i][j]);
    }
  }
}


template <class data_t> 				
void potentialField<data_t>::mergeByAddition(data_t** a) {
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {
      array[r][c] = array[r][c]+a[r][c];
    }
  }
  normaliseByMax();
}

template <class data_t> 				
void potentialField<data_t>::mergeByAdditionWeighted(data_t** a, data_t weight) {
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {
      array[r][c] = array[r][c]+(a[r][c]*weight);
    }
  }
  normaliseByMax();
}

template <class data_t> 				
void potentialField<data_t>::mergeByMultiplication(data_t** a) {
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {


      array[r][c] = array[r][c]*a[r][c];

      if(isnan(array[r][c])) {
	cout << "Error: potentialField::mergeByMultiplication - dist isnan! for x = " << r << " y = " << c << endl;
	exit(EXIT_FAILURE);
      }

      
    }
  }
  normaliseByMax();
}

template <class data_t> 				
void potentialField<data_t>::mergeByMax(data_t** a) {
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {
      if(array[r][c] < a[r][c]) {
	array[r][c] = a[r][c];
	
	if(isnan(array[r][c])) {
	  cout << "Error: potentialField::mergeByMax - dist isnan! for x = " << r << " y = " << c << endl;
	  exit(EXIT_FAILURE);
	}

	
      }	
    }
  }
}


template <class data_t> 				
void potentialField<data_t>::mergeByMultiplicationNoNormalise(data_t** a) {
  
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {
      array[r][c] = array[r][c]*a[r][c];
    }
  }
}


template <class data_t> 				
inline void potentialField<data_t>::fillPF(const data_t &val) {
  for(int r = 0; r < row; r++) {
    for(int c = 0; c < column; c++) {
      array[r][c] = val;
    }
  }
}

#endif

