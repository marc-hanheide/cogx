#ifndef MYVECTOR_H_
#define MYVECTOR_H_

/*
 *  myVector.h
 *  Created by jk on 30/05/2006.
 *
 */

#include <math.h>
#include <stdlib.h>
#include <iostream>

#define PI 3.1415926535897932

using namespace std;

template <class data_t> class myVector {

 private:
 
  data_t x;
  data_t y;
  data_t z;
 
 public:
  
  myVector() { 
    x = 0; 
    y = 0; 
    z = 0; 
  }
  
  myVector(data_t a, data_t b, data_t c) { 
    x=a; 
    y=b; 
    z=c; 
  }
  
  myVector(myVector<data_t>* v) { 
    x = v->getX(); 
    y = v->getY(); 
    z = v->getZ(); 
  }

  myVector(const myVector<data_t> &v) { 
    x = v.getX(); 
    y = v.getY(); 
    z = v.getZ(); 
  }
  


  
  void setVec(data_t a, data_t b, data_t c) { x=a; y=b; z=c; }
  data_t getX() const { return x; }
  data_t getY() const { return y; }
  data_t getZ() const { return z; }
		
  void outputVector() { cout << "x = " << x << " y = " << y << " z = " << z << endl; }

  data_t dotProduct(myVector<data_t> *a);
  myVector<data_t>* crossProduct(myVector<data_t>* a);

  data_t distanceFromOrigin();


  data_t distanceFromArg(myVector<data_t>* a);
  data_t angleBetween(myVector<data_t>* a); 
  void copy(const myVector<data_t> &rv);
  void translate(myVector<data_t>* a);
  data_t toRadians(const data_t degrees);
  data_t toDegrees(data_t radians);
  bool isOrigin();
};

/*
 * Function Definitions
 */

/*
  distance from the vector to the origin
*/
template <class data_t> inline
data_t myVector<data_t>::distanceFromOrigin() {
  return sqrt((x*x) + (y*y) + (z*z));
}

template <class data_t> inline
data_t myVector<data_t>::distanceFromArg(myVector<data_t>* a) {
  data_t xd = x - a->x;
  data_t yd = y - a->y;
  data_t zd = z - a->z;
	
  return sqrt((xd*xd) + (yd*yd) + (zd*zd));	
}

template <class data_t> inline
myVector<data_t>* myVector<data_t>::crossProduct(myVector<data_t>* a) {
  data_t cp1 = (y*a->z) - (z*a->y);
  data_t cp2 = (z*a->x) - (x*a->z);
  data_t cp3 = (x*a->y) - (y*a->x);
  return(new myVector<data_t>(cp1, cp2, cp3));
}

template <class data_t> inline
data_t myVector<data_t>::dotProduct(myVector<data_t>* a) {
  return ((x*a->x)+(y*a->y)+(z*a->z));
}

template <class data_t> inline
data_t myVector<data_t>::toRadians(const data_t degrees) {
  return ((2*PI)/360)*degrees;
}

template <class data_t> inline
data_t myVector<data_t>::toDegrees(data_t radians) {
  return (360/(2*PI))*radians;
}

template <class data_t> inline
bool myVector<data_t>::isOrigin() {
  if((x == 0) && (y == 0)) {
    return true;
  }
  return false;
}

template <class data_t> inline
data_t myVector<data_t>::angleBetween(myVector<data_t>* a) {
	
  data_t degrees;
  data_t rads;
  data_t tmp;
  data_t dprod = this->dotProduct(a);
  data_t dist = this->distanceFromOrigin();
  data_t distA = a->distanceFromOrigin();
		
  tmp = dprod/(dist*distA);
  if (tmp >= 1.0) {
    return 0;
  } 
  if (tmp <= -1.0) {
    return 180;
  }
  rads = acos(tmp);
	
  if(isnan(rads)) {
    cout << "Error: myVector::angleBetween: rads is not a number" << endl;
    cout << "tmp = " << tmp << endl;
    for(data_t x = 1.0; x >= -1; x = x - 0.1) {
      cout << "x = " << x << endl;
      cout << "acos(x) = " << acos(x) << endl;
      cout << "toDegrees(acos(x)) = " << toDegrees(acos(x)) << endl;
    }		
    exit(EXIT_FAILURE);
  }
	
  degrees = toDegrees(rads);
	
  if(isnan(degrees)) {
    cout << "Error: myVector::angleBetween: isnan(degrees)" << endl;
    exit(EXIT_FAILURE);
  }
  return degrees;
}

template <class data_t> inline
void myVector<data_t>::copy(const myVector &rv) {
  x = rv.x;
  y = rv.y;
  z = rv.z;
}

template <class data_t> inline
void myVector<data_t>::translate(myVector<data_t>* a) {
  x = x - a->x;
  y = y - a->y;
  z = z - a->z;
 
}


#endif

