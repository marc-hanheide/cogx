#include "org_cognitivesystems_spatial_pf_ProximityMap.h"

#include <iostream>
using namespace std;

#include "ProximityMap.h"

static ProximityMap *pProximityMap = NULL;

static const string pfpClassString = "org/cognitivesystems/spatial/pf/PotentialFieldPoint";
static jclass pfpClass = NULL;
static const string pfpConstructorSig = "(FFF)V";
static jmethodID pfpConstructor = NULL;


JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_createNativeMap
(JNIEnv * _env, jclass _this, jint _width, jint _height) {
  //cout<<_width<<" "<<_height<<endl;

  if(pProximityMap) {
    //cout<<"Deleting existing proximity map"<<endl;
    delete pProximityMap;
  }

  pProximityMap = new ProximityMap(_width,_height);

}
 


/*
 * Class:     org_cognitivesystems_spatial_pf_ProximityMap
 * Method:    addObjectNative
 * Signature: (Ljava/lang/String;FFF)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_addObjectNative
(JNIEnv * _env, jclass _this, jstring _id, jfloat _x, jfloat _y, jfloat _z) {

  if(pProximityMap) {
   
    const char *idChars = _env->GetStringUTFChars(_id, 0);
    string id(idChars);
    
    pProximityMap->addObject(id,_x,_y,_z);
    
    _env->ReleaseStringUTFChars(_id, idChars);

  }
  else {
    cerr<<"ProxmityMap not constructed!"<<endl;
  }

}

/*
 * Class:     org_cognitivesystems_spatial_pf_ProximityMap
 * Method:    setViewerPositionNative
 * Signature: (FFF)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_setViewerPositionNative
(JNIEnv * _env, jclass _this, jfloat _x, jfloat _y, jfloat _z) {
  if(pProximityMap) {
    pProximityMap->setViewerPosition(_x,_y,_z);
  }
  else {
    cerr<<"ProxmityMap not constructed!"<<endl;
  }

}



/*
 * Class:     org_cognitivesystems_spatial_pf_ProximityMap
 * Method:    proximityValueNative
 * Signature: (Ljava/lang/String;Ljava/lang/String;)F
 */
JNIEXPORT jfloat JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_proximityValueNative
(JNIEnv * _env, jclass _this, jstring _target, jstring _landmark) {
  if(pProximityMap) {
    
    const char *targetChars = _env->GetStringUTFChars(_target, 0);
    string target(targetChars);
    const char *landmarkChars = _env->GetStringUTFChars(_landmark, 0);
    string landmark(landmarkChars);
    
    jfloat val = pProximityMap->proximityValue(target,landmark);
    
    _env->ReleaseStringUTFChars(_target, targetChars);
    _env->ReleaseStringUTFChars(_landmark, landmarkChars);


    //     ProximityMap * proxMap = new ProximityMap(400,400);
    //     proxMap->setViewerPosition(200, 10, 0);
    //     proxMap->addObject("obj1", 250, 150, 0);
    //     proxMap->addObject("obj2", 150, 150, 0);
    //     proxMap->addObject("obj3", 150, 250, 0);
    //     proxMap->addObject("obj4", 250, 180, 0);

    //     cout<<"proxVal: "<<proxMap->proximityValue("obj4","obj1")<<endl;
    //     val = proxMap->proximityValue("obj4","obj1");


    return val;

  }
  else {
    cerr<<"ProxmityMap not constructed!"<<endl;
    return -1;
  }

}

/*
 * Class:     org_cognitivesystems_spatial_pf_ProximityMap
 * Method:    proximityMapNative
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_proximityMapNative
(JNIEnv * _env, jclass _this, jstring _landmark) {
  if(pProximityMap) {
    
    const char *landmarkChars = _env->GetStringUTFChars(_landmark, 0);
    string landmark(landmarkChars);
    
    pProximityMap->makeMap(landmark);
    
    _env->ReleaseStringUTFChars(_landmark, landmarkChars);

  }
  else {
    cerr<<"ProxmityMap not constructed!"<<endl;
  }

}





/*
 * Class:     org_cognitivesystems_spatial_pf_ProximityMap
 * Method:    nextSweetSpotNative
 * Signature: ()Lorg/cognitivesystems/spatial/pf/PotentialFieldPoint;
 */
JNIEXPORT jobject JNICALL Java_org_cognitivesystems_spatial_pf_ProximityMap_nextSweetSpotNative
(JNIEnv * _env, jclass _this) {

  if(pProximityMap) {

    float x = -999;
    float y = -999;
    float val = -999;

    if(pProximityMap->nextSweetSpot(x,y,val)) {

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
      cerr<<"ProxmityMap sweet spot error"<<endl;
      return NULL;
    }

  }
  else {
    cerr<<"ProxmityMap not constructed!"<<endl;
    return NULL;
  }


}
