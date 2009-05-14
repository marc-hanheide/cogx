#include "org_cognitivesystems_spatial_pf_ProjectiveMap.h"

#include <iostream>
using namespace std;

#include "ProjectiveMap.h"

static ProjectiveMap *pLeftMap = NULL;
static ProjectiveMap *pRightMap = NULL;
static ProjectiveMap *pFrontMap = NULL;
static ProjectiveMap *pBackMap = NULL;

static const string pfpClassString = "org/cognitivesystems/spatial/pf/PotentialFieldPoint";
static jclass pfpClass = NULL;
static const string pfpConstructorSig = "(FFF)V";
static jmethodID pfpConstructor = NULL;


#define LEFT 0
#define RIGHT 1
#define FRONT 2
#define BACK 3

ProjectiveMap ** selectMap(int _dir) {

  switch(_dir) {
  case LEFT: return &pLeftMap;
  case RIGHT: return &pRightMap;
  case FRONT: return &pFrontMap;
  case BACK: return &pBackMap; 
  default : return NULL;
  }

}







JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_createNativeMap
(JNIEnv * _env, jclass _this, jint _width, jint _height, jint _dir) {
  //cout<<_width<<" "<<_height<<endl;

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {
    delete *ppProjectiveMap;
    *ppProjectiveMap = NULL;
  }

  switch(_dir) {
  case LEFT: (*ppProjectiveMap) = new ProjectiveMap(_width, _height, ProjectiveMap::LEFT_PROJ); break;
  case RIGHT: (*ppProjectiveMap) = new ProjectiveMap(_width, _height, ProjectiveMap::RIGHT_PROJ); break;
  case FRONT: (*ppProjectiveMap) = new ProjectiveMap(_width, _height, ProjectiveMap::FRONT_PROJ); break;
  case BACK: (*ppProjectiveMap) = new ProjectiveMap(_width, _height, ProjectiveMap::BACK_PROJ); break;
  default : cerr<<"ProjectiveMapJNI unknown direction constant: "<<_dir<<endl; break;
  }


}
 


/*
 * Class:     org_cognitivesystems_spatial_pf_ProjectiveMap
 * Method:    addObjectNative
 * Signature: (Ljava/lang/String;FFF)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_addObjectNative
(JNIEnv * _env, jclass _this, jstring _id, jfloat _x, jfloat _y, jfloat _z, jint _dir) {

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {
   
    const char *idChars = _env->GetStringUTFChars(_id, 0);
    string id(idChars);
    
    (*ppProjectiveMap)->addObject(id,_x,_y,_z);
    
    _env->ReleaseStringUTFChars(_id, idChars);

  }
  else {
    cerr<<"ProjectiveMap not constructed for direction: "<<_dir<<endl;
  }

}

/*
 * Class:     org_cognitivesystems_spatial_pf_ProjectiveMap
 * Method:    setViewerPositionNative
 * Signature: (FFF)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_setViewerPositionNative
(JNIEnv * _env, jclass _this, jfloat _x, jfloat _y, jfloat _z, jint _dir) {

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {
    (*ppProjectiveMap)->setViewerPosition(_x,_y,_z);
  }
  else {
    cerr<<"ProjectiveMap not constructed for direction: "<<_dir<<endl;
  }

}



/*
 * Class:     org_cognitivesystems_spatial_pf_ProjectiveMap
 * Method:    projectiveValueNative
 * Signature: (Ljava/lang/String;Ljava/lang/String;)F
 */
JNIEXPORT jfloat JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_projectiveValueNative
(JNIEnv * _env, jclass _this, jstring _target, jstring _landmark, jint _dir) {

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {
    
    const char *targetChars = _env->GetStringUTFChars(_target, 0);
    string target(targetChars);
    const char *landmarkChars = _env->GetStringUTFChars(_landmark, 0);
    string landmark(landmarkChars);
    
    jfloat val = (*ppProjectiveMap)->projectiveValue(target,landmark);
    
    _env->ReleaseStringUTFChars(_target, targetChars);
    _env->ReleaseStringUTFChars(_landmark, landmarkChars);


    //     ProjectiveMap * proxMap = new ProjectiveMap(400,400);
    //     proxMap->setViewerPosition(200, 10, 0);
    //     proxMap->addObject("obj1", 250, 150, 0);
    //     proxMap->addObject("obj2", 150, 150, 0);
    //     proxMap->addObject("obj3", 150, 250, 0);
    //     proxMap->addObject("obj4", 250, 180, 0);

    //     cout<<"proxVal: "<<proxMap->projectiveValue("obj4","obj1")<<endl;
    //     val = proxMap->projectiveValue("obj4","obj1");


    return val;

  }
  else {
    cerr<<"ProjectiveMap not constructed for direction: "<<_dir<<endl;
    return -1;
  }

}

/*
 * Class:     org_cognitivesystems_spatial_pf_ProjectiveMap
 * Method:    projectiveMapNative
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_projectiveMapNative
(JNIEnv * _env, jclass _this, jstring _landmark, jint _dir) {

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {
    
    const char *landmarkChars = _env->GetStringUTFChars(_landmark, 0);
    string landmark(landmarkChars);
    
    (*ppProjectiveMap)->makeMap(landmark);
    
    _env->ReleaseStringUTFChars(_landmark, landmarkChars);

  }
  else {
    cerr<<"ProjectiveMap not constructed for direction: "<<_dir<<endl;
  }

}





/*
 * Class:     org_cognitivesystems_spatial_pf_ProjectiveMap
 * Method:    nextSweetSpotNative
 * Signature: ()Lorg/cognitivesystems/spatial/pf/PotentialFieldPoint;
 */
JNIEXPORT jobject JNICALL Java_org_cognitivesystems_spatial_pf_ProjectiveMap_nextSweetSpotNative
(JNIEnv * _env, jclass _this, jint _dir) {

  ProjectiveMap ** ppProjectiveMap = selectMap(_dir);

  if(*ppProjectiveMap) {

    float x = -999;
    float y = -999;
    float val = -999;

    if((*ppProjectiveMap)->nextSweetSpot(x,y,val)) {

      ///Not sure about thread safety of doing it in init, so do it
      ///each time for now
      
      //find the FrameworkBasics::BALTTime contructor
      pfpClass = _env->FindClass(pfpClassString.c_str());
      if(pfpClass == NULL) {
	cerr<<"Unable to lookup Java class: "<<pfpClassString<<endl;
	exit(1);
      }

      //initialise JNI interface stuff
      pfpConstructor = _env->GetMethodID(pfpClass,
					 "<init>", 
					 pfpConstructorSig.c_str());

      if(pfpConstructor == NULL) {
	cerr<<"Unable to lookup PotentialFieldPoint constructor: "<<pfpConstructorSig<<endl;
	exit(1);
      }



      //   if(pfpClass == NULL) {
      //     cerr<<"time class eerror: "<<pfpClassString<<endl;
      //   }


      //   if(pfpConstructor == NULL) {
      //     cerr<<"time constructor eerror: "<<pfpClassString<<endl;
      //   }

      return _env->NewObject(pfpClass,pfpConstructor,x,y,val);
    }
    else {
      cerr<<"ProjectiveMap sweet spot error"<<endl;
      return NULL;
    }

  }
  else {
    cerr<<"ProjectiveMap not constructed for direction: "<<_dir<<endl;
    return NULL;
  }


}
