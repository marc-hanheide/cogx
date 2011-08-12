#ifndef KINECT_PERSON_DETECT_ICE
#define KINECT_PERSON_DETECT_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module kinect {
 module slice {

  class KinectPerson {
    int id;
    int distance;
    int size;
  };

  dictionary<int, KinectPerson> PersonsDict;

  interface PersonDetectorInterface {
	PersonsDict getPersons();
  };


 };
};

#endif
