/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 * 
 */


#include "balt_jni_NativeProcessLauncher.h"
#include <balt/core/BALTCore.hpp>
#include <balt/interface/BALTInterface.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <map>
using namespace std;


LocalConnectionManager *pManager = NULL;

//JNI stuff for time communication

const string baltTimeClassString = "balt/corba/autogen/FrameworkBasics/BALTTime";
jclass baltTimeClass = NULL;
const string baltTimeConstructorSig = "(II)V";
jmethodID baltTimeConstructor = NULL;



/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    init
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_init
(JNIEnv * _env, jclass _callingClass, jstring _namingHost, jstring _namingPort) {

  const char *hostChars = _env->GetStringUTFChars(_namingHost, 0);
  string hostname(hostChars);
  
  const char *portChars = _env->GetStringUTFChars(_namingPort, 0);
  string port(portChars);
  
  
  if(!pManager) {
    //cout<<"Creating local manager"<<endl;
    pManager = new LocalConnectionManager(hostname,port);
  }

  //  if(!pTimer) {
    //cout<<"Creating local timer"<<endl;
    //pTimer = new BALTTimer();
  //}




  //find the FrameworkBasics::BALTTime contructor
  baltTimeClass = _env->FindClass(baltTimeClassString.c_str());
  if(baltTimeClass == NULL) {
    cerr<<"Unable to lookup Java class: "<<baltTimeClassString<<endl;
    std::abort();
  }



  //initialise JNI interface stuff
  baltTimeConstructor = _env->GetMethodID(baltTimeClass,
					  "<init>", 
					  baltTimeConstructorSig.c_str());

  if(baltTimeConstructor == NULL) {
    cerr<<"Unable to lookup BALTTime constructor: "<<baltTimeConstructorSig<<endl;
    std::abort();
  }




  //free strings
  _env->ReleaseStringUTFChars(_namingHost, hostChars);
  _env->ReleaseStringUTFChars(_namingPort, portChars);

}


/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    createNativeProcess
 * Signature: (Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;[Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL 
Java_balt_jni_NativeProcessLauncher_createNativeProcess(
							JNIEnv *_env, 
							jclass _callingClass, 
							jstring _className, 
							jstring _procName, 
							jobjectArray _configKeys, 
							jobjectArray _configValues) {
  if(!pManager) {
    cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
    return;
  }

  jsize keyLen = _env->GetArrayLength(_configKeys);
  jsize valLen = _env->GetArrayLength(_configValues);

  if(keyLen != valLen) {
    cerr<<"Unequal numbers of keys and values in configuration!"<<endl;
    return;
  }

  //get C copy of class name
  const char *classChars = _env->GetStringUTFChars(_className, 0);
  string className(classChars);
  
  const char *nameChars = _env->GetStringUTFChars(_procName, 0);
  string procName(nameChars);
  
  
  map<string,string> configMap;
  jstring key;
  jstring value;

  for(int i = 0; i < keyLen; i++) {
    key = (jstring)_env->GetObjectArrayElement(_configKeys, i);
    value = (jstring)_env->GetObjectArrayElement(_configValues, i);

    const char *keyChars = _env->GetStringUTFChars(key, 0);  
    const char *valueChars = _env->GetStringUTFChars(value, 0);

    //cout<<"config map key: "<<keyChars<<endl;
    //cout<<"config map val: "<<valueChars<<endl;

    configMap[keyChars] = valueChars;

    _env->ReleaseStringUTFChars(key, keyChars);
    _env->ReleaseStringUTFChars(value, valueChars);

    ///TODO RELEASE ARRAY ELEMENTS TOO?
        
  }


  
  //cout<<"JNI Interface to launch "<<procName<<" of type "<<className<<endl;
  
  pManager->createProcess(className,procName,configMap);


  //free strings
  _env->ReleaseStringUTFChars(_className, classChars);
  _env->ReleaseStringUTFChars(_procName, nameChars);
  

}


/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    createPullConnection
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_createNativePullConnection
(JNIEnv * _env, jclass _callingClass, jobjectArray _senderNames, jstring _receiverName, jstring _dataType, jstring _id) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }

    vector<string> * p_senderNames = new vector<string>();

    jint length = _env->GetArrayLength(_senderNames);
    for(int i = 0; i < length; i++)  {
      //should do some checking here
      jstring senderName = (jstring) _env->GetObjectArrayElement(_senderNames, i);
      const char* senderChars = _env->GetStringUTFChars(senderName, 0);
      p_senderNames->push_back(string(senderChars));
      _env->ReleaseStringUTFChars(senderName, senderChars);
    }

    const char *receiverChars = _env->GetStringUTFChars(_receiverName, 0);
    string receiverName(receiverChars);
    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);
    const char *idChars = _env->GetStringUTFChars(_id, 0);
    string id(idChars);


    pManager->createPullConnection(*p_senderNames,receiverName,dataType,id);

    _env->ReleaseStringUTFChars(_receiverName, receiverChars);
    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, idChars);

    delete p_senderNames;

}


/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    createPushConnection
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_createNativePushConnection
(JNIEnv * _env, jclass _callingClass, jobjectArray _senderNames, jobjectArray _receiverNames, jstring _dataType, jstring _id) {
    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }



    vector<string> * p_senderNames = new vector<string>();

    jint length = _env->GetArrayLength(_senderNames);
    for(int i = 0; i < length; i++)  {
      //should do some checking here
      jstring senderName = (jstring) _env->GetObjectArrayElement(_senderNames, i);
      const char* senderChars = _env->GetStringUTFChars(senderName, 0);
      p_senderNames->push_back(string(senderChars));
      _env->ReleaseStringUTFChars(senderName, senderChars);
    }

    vector<string> * p_receiverNames = new vector<string>();

    length = _env->GetArrayLength(_receiverNames);
    for(int i = 0; i < length; i++)  {
      //should do some checking here
      jstring receiverName = (jstring) _env->GetObjectArrayElement(_receiverNames, i);
      const char* receiverChars = _env->GetStringUTFChars(receiverName, 0);
      p_receiverNames->push_back(string(receiverChars));
      _env->ReleaseStringUTFChars(receiverName, receiverChars);
    }



    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);

    const char *idChars = _env->GetStringUTFChars(_id, 0);
    string id(idChars);


    pManager->createPushConnection(*p_senderNames,*p_receiverNames,dataType,id);

    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, idChars);

    delete p_senderNames;
    delete p_receiverNames;
}

/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    startProcess
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_startNativeProcess
(JNIEnv * _env, jclass _callingClass, jstring _procName) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);

    pManager->startProcess(procName);

    _env->ReleaseStringUTFChars(_procName, nameChars);
}


/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    runProcess
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_runNativeProcess
(JNIEnv * _env, jclass _callingClass, jstring _procName) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);

    pManager->runProcess(procName);

    _env->ReleaseStringUTFChars(_procName, nameChars);
}



/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    stopProcess
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_stopNativeProcess
(JNIEnv * _env, jclass _callingClass, jstring _procName) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);

    pManager->stopProcess(procName);

    _env->ReleaseStringUTFChars(_procName, nameChars);
}



/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    cleanupNativeInterface
 * Signature: ()V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_cleanupNativeInterface
(JNIEnv *_env, jclass _callingClass) {

  //Ccout<<"Java_balt_jni_NativeProcessLauncher_cleanupNativeInterface"<<endl;

    if(pManager) {
        //this should handle thread shutdown etc.
        delete pManager;
    }

    pManager = NULL;

}




/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    connectRemotePushConnectionSender
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_connectRemotePushConnectionSender
(JNIEnv * _env, jclass _callingClass, jstring _procName, jstring _remoteID, jstring _dataType, jstring _id) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);
    const char *idChars = _env->GetStringUTFChars(_remoteID, 0);
    string remoteID(idChars);
    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);
    const char *connIDChars = _env->GetStringUTFChars(_id, 0);
    string connID(connIDChars);


    pManager->connectRemotePushConnectionSender(procName,remoteID,dataType,connID);

    _env->ReleaseStringUTFChars(_procName, nameChars);
    _env->ReleaseStringUTFChars(_remoteID, idChars);
    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, connIDChars);

}

/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    connectRemotePushConnectionReceiver
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_connectRemotePushConnectionReceiver
(JNIEnv * _env, jclass _callingClass, jstring _procName, jstring _remoteID, jstring _dataType, jstring _id) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);
    const char *idChars = _env->GetStringUTFChars(_remoteID, 0);
    string remoteID(idChars);
    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);
    const char *connIDChars = _env->GetStringUTFChars(_id, 0);
    string id(connIDChars);

    pManager->connectRemotePushConnectionReceiver(procName,remoteID,dataType,id);

    _env->ReleaseStringUTFChars(_procName, nameChars);
    _env->ReleaseStringUTFChars(_remoteID, idChars);
    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, connIDChars);
}

/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    connectRemotePullConnectionSender
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_connectRemotePullConnectionSender
(JNIEnv * _env, jclass _callingClass, jstring _procName, jstring _remoteID, jstring _dataType, jstring _id) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);
    const char *idChars = _env->GetStringUTFChars(_remoteID, 0);
    string remoteID(idChars);
    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);
    const char *connIDChars = _env->GetStringUTFChars(_id, 0);
    string id(connIDChars);

    pManager->connectRemotePullConnectionSender(procName,remoteID,dataType,id);

    _env->ReleaseStringUTFChars(_procName, nameChars);
    _env->ReleaseStringUTFChars(_remoteID, idChars);
    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, connIDChars);

}

/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    connectRemotePullConnectionReceiver
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL
Java_balt_jni_NativeProcessLauncher_connectRemotePullConnectionReceiver
(JNIEnv * _env, jclass _callingClass, jstring _procName, jstring _remoteID, jstring _dataType, jstring _id) {

    if(!pManager) {
        cerr<<"LocalConnectionManager not initialised, call init first!"<<endl;
        return;
    }


    const char *nameChars = _env->GetStringUTFChars(_procName, 0);
    string procName(nameChars);
    const char *idChars = _env->GetStringUTFChars(_remoteID, 0);
    string remoteID(idChars);
    const char *dataTypeChars = _env->GetStringUTFChars(_dataType, 0);
    string dataType(dataTypeChars);
    const char *connIDChars = _env->GetStringUTFChars(_id, 0);
    string id(connIDChars);

    pManager->connectRemotePullConnectionReceiver(procName,remoteID,dataType,id);

    _env->ReleaseStringUTFChars(_procName, nameChars);
    _env->ReleaseStringUTFChars(_remoteID, idChars);
    _env->ReleaseStringUTFChars(_dataType, dataTypeChars);
    _env->ReleaseStringUTFChars(_id, connIDChars);
}


//clock methods

/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    resetNativeClock
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_balt_jni_NativeProcessLauncher_resetNativeClock
(JNIEnv * _env, jclass _callingClass) {

  //  if(!pTimer) {
  //cerr<<"BALTTimer not initialised, call init first!"<<endl;
  //return;
  //}

  //pTimer->reset();
  BALTTimer::reset();
}


/*
 * Class:     balt_jni_NativeProcessLauncher
 * Method:    getNativeBALTTime
 * Signature: ()V
 */
JNIEXPORT jobject JNICALL Java_balt_jni_NativeProcessLauncher_getNativeBALTTime
(JNIEnv * _env, jclass _callingClass) {

  FrameworkBasics::BALTTime t = BALTTimer::getBALTTime();


  //Have to do this every time? Why? Is it a threading issue?

  //   cout<<"t.m_s: "<<t.m_s<<endl;
  //   cout<<"t.m_us: "<<t.m_us<<endl;

  //find the FrameworkBasics::BALTTime contructor
  baltTimeClass = _env->FindClass(baltTimeClassString.c_str());
  if(baltTimeClass == NULL) {
    cerr<<"Unable to lookup Java class: "<<baltTimeClassString<<endl;
    std::abort();
  }



  //initialise JNI interface stuff
  baltTimeConstructor = _env->GetMethodID(baltTimeClass,
					  "<init>", 
					  baltTimeConstructorSig.c_str());

  if(baltTimeConstructor == NULL) {
    cerr<<"Unable to lookup BALTTime constructor: "<<baltTimeConstructorSig<<endl;
    std::abort();
  }



//   if(baltTimeClass == NULL) {
//     cerr<<"time class eerror: "<<baltTimeClassString<<endl;
//   }


//   if(baltTimeConstructor == NULL) {
//     cerr<<"time constructor eerror: "<<baltTimeClassString<<endl;
//   }

  return _env->NewObject(baltTimeClass,baltTimeConstructor,t.m_s,t.m_us);
}


JNIEXPORT jint JNICALL 
Java_balt_jni_NativeProcessLauncher_getProcessID(JNIEnv *, jclass) {
  return getpid();
}

