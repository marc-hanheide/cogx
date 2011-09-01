/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader                                    *
 *   bader@acin.tuwien.ac.at                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "naoqi.h"
#include "albroker.h"
#include "albrokermanager.h"
#include "alloggerproxy.h"
#include "almemoryproxy.h"
#include "almotionproxy.h"
#include "altexttospeechproxy.h"
#include "dcmproxy.h"
#include "alvideodeviceproxy.h"
#include "alvisiondefinitions.h"
#include "almemoryfastaccess.h"
#include "alimage.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace AK {

NaoQi::NaoQi ( AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName ) :  AL::ALModule ( pBroker, pName )
        , mCamName( )
        , mpImage ( NULL ) {
    setModuleDescription ( "This an module, to bridge some funktions for matlab" );
}

NaoQi::~NaoQi() {
}

void NaoQi::dataChanged ( const std::string& pDataName, const AL::ALValue& pValue, const std::string& pMessage ) {
}

std::string NaoQi::version() {
    std::stringstream ss;
    ss << mVersion;
    return ss.str();
}


void NaoQi::init ( void ) {
    try {
        mpLogProxy = getParentBroker()->getLoggerProxy();
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "init()", "Error on init LogProxy! " + e.toString() );
    }
    try {
        mpMotionProxy = getParentBroker()->getMotionProxy();
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "connect()", "Error on init MotionProxy! " + e.toString() );
    }
    try {
        mpMemProxy = getParentBroker()->getMemoryProxy();
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "connect()", "Error on init MemoryProxy! " + e.toString() );
    }
}

void NaoQi::unsubscribeCamera() {
    if ( mpCameraProxy ) {
        if ( !mCamName.empty() ) {
            try {
                mpCameraProxy->unsubscribe ( mCamName );
            } catch ( AL::ALError& e ) {
                throw AL::ALError ( getName(), "close()", "Error on unsubscribe CameraProxy! " + e.toString() );
            }
        }
        try {
            mpCameraProxy->destroyConnection();
            cvReleaseImageHeader ( &mpImage );
            std::cout << "CameraProxy disconnected" << std::endl;
        } catch ( AL::ALError& e ) {
            throw AL::ALError ( getName(), "close()", "Error on delete old CameraProxy! " + e.toString() );
        }
    } else {
        std::cout << "CameraProxy was allrady disconnected" << std::endl;
    }
}

int NaoQi::setWalkTargetVelocity ( float x, float y, float theta, float frequency, bool post ) {
    int taskID = -1;
    if ( post ) {
        mpMotionProxy->post.setWalkTargetVelocity ( x, y, theta, frequency );
    } else {
        mpMotionProxy->setWalkTargetVelocity ( x, y, theta, frequency );
    }
    return taskID;
}

void NaoQi::setStiffnesses ( const std::vector<std::string> &names, const  std::vector<float> &stiffnesses ) {
    if ( names.size() != 1 ) {
        AL::ALValue alnames, alstiffnesses;
        for ( unsigned int i = 0; i < names.size(); i++ ) {
            alnames.arrayPush ( names[i] );
            alstiffnesses.arrayPush ( stiffnesses[i] );
            std::cout << "Name: " << names[i] << ", Stiffnes: "<< stiffnesses[i] << std::endl;
        }
        mpMotionProxy->post.setStiffnesses ( alnames, alstiffnesses );
    } else {
        mpMotionProxy->post.setStiffnesses ( names[0], stiffnesses[0] );
    }
}

void NaoQi::stiffnessInterpolation ( const std::vector<std::string> &names, const std::vector<float> &stiffnesses, const std::vector<float> &times ) {

    if ( names.size() != 1 ) {
        AL::ALValue alnames, alstiffnesses, altimes;
        for ( unsigned int i = 0; i < names.size(); i++ ) {
            alnames.arrayPush ( names[i] );
            alstiffnesses.arrayPush ( stiffnesses[i] );
            altimes.arrayPush ( times[i] );
            std::cout << "Name: " << names[i] << ", Stiffnes: "<< stiffnesses[i] << ", time: "<< times[i] << std::endl;
        }
        mpMotionProxy->post.stiffnessInterpolation ( alnames, alstiffnesses, altimes );
    } else {
        mpMotionProxy->post.stiffnessInterpolation ( names[0], stiffnesses[0], times );
    }
}

void NaoQi::logInfo ( const std::string text ) {
    mpLogProxy->info ( getName(), text );
}

int NaoQi::setAngles ( const std::vector<std::string> &names, const std::vector<float> &angles, float fractionMaxSpeed, bool post ) {
    AL::ALValue alnames, alangles;
    for ( unsigned int i = 0; i < names.size(); i++ ) {
        alnames.arrayPush ( names[i] );
        alangles.arrayPush ( angles[i] );
        // std::cout << "Name: " << names[i] << ", Angle: "<< angles[i] << ", FractionMaxSpeed: "<< fractionMaxSpeed << std::endl;
    }
    int taskID = -1;
    if ( post ) {
        taskID = mpMotionProxy->post.setAngles ( alnames, alangles, fractionMaxSpeed );
    } else {
        mpMotionProxy->setAngles ( alnames, alangles, fractionMaxSpeed );
    }
    return taskID;
}

std::vector<float> NaoQi::getAngles ( const std::vector<std::string> &names, const bool &useSensorValues) {
    AL::ALValue alnames;
    for ( unsigned int i = 0; i < names.size(); i++ ) {
        alnames.arrayPush ( names[i] );
        // std::cout << "Name: " << names[i] << ", useSensorValues: "<< useSensorValues << std::endl;
    }
    int taskID = -1;
    std::vector <float> angles = mpMotionProxy->getAngles ( alnames, useSensorValues );
    return angles;
}

std::vector<float> NaoQi::getPosition ( const std::string &name, const int &space, const bool &useSensorValues ) {
    std::vector <float> pos = mpMotionProxy->getPosition ( name, space, useSensorValues );
    return pos;
}

int NaoQi::subscribeCamera ( const int &resolution, const int &colorSpace, const int &fps, const int &camera, bool bShow ) {
    const std::string kOriginalName = "sample_GVM";
    int width = 0;
    int height = 0;
    int channels = 0;
    const int kImgDepth = 8;
    AL::getSizeFromResolution ( resolution, width, height );
    channels = AL::getNumLayersInColorSpace ( colorSpace );
    if ( width == -1 || height == -1 || channels == -1 ) {
        throw AL::ALError ( getName(), "subscribeCamera()", "Invalid resolution or color space." );
    }
    mpImage = cvCreateImageHeader ( cvSize ( width, height ), 8, channels );

    try {
        mpCameraProxy = boost::shared_ptr<AL::ALVideoDeviceProxy> ( new AL::ALVideoDeviceProxy ( this->getParentBroker() ) );
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "subscribeCamera()", "Error on init ALVideoDevice! " + e.toString() );
    }
    // Call the "subscribe" function with the given parameters.
    mCamName = mpCameraProxy->subscribe ( kOriginalName, resolution, colorSpace, fps );
    mpCameraProxy->setParam ( AL::kCameraSelectID, camera );
    mpLogProxy->info ( getName(), "ALVideoDevice registered as " + mCamName );

    if ( bShow ) {
        cvNamedWindow ( "Image", 1 );
        for ( int i = -1; i < 0; i = cvWaitKey ( 100 ) ) {
            AL::ALValue results;
            try {
                results = mpCameraProxy->getImageRemote ( mCamName );
            } catch ( AL::ALError& e ) {
                throw AL::ALError ( getName(), "subscribeCamera()", "Error on getImageRemotee! " + e.toString() );
            }
            if ( results.getType() != AL::ALValue::TypeArray && results.getSize() != 7 ) {
                throw AL::ALError ( getName(), "saveImageRemote", "Invalid image returned." );
            }
            // Set the buffer we received to our IplImage header.
            mpImage->imageData = ( char* ) ( results[6].GetBinary() );
            cvShowImage ( "Image", mpImage );
            //cvReleaseData(*image);
        }
        cvDestroyWindow ( "Image" );
    }
    return AKOK;
}

int NaoQi::getCameraImageSize ( int *pWidth, int *pHeight, int *pChannels ) {
    if ( ( mpCameraProxy == NULL ) || ( mpImage == NULL ) ) return AKERROR;
    *pWidth = mpImage->width;
    *pHeight = mpImage->height;
    *pChannels = mpImage->nChannels;
    return AKOK;
}

int NaoQi::getCameraImage ( char *pDes, const ImageMemFormat eFormat ) {
    if ( ( mpCameraProxy == NULL ) || ( mpImage == NULL ) ) return AKERROR;
    AL::ALValue results;
    try {
        results = mpCameraProxy->getImageRemote ( mCamName );
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "subscribeCamera()", "Error on getImageRemotee! " + e.toString() );
    }
    char *pSrc = ( char* ) ( results[6].GetBinary() );
    switch ( eFormat ) {
    case IN_ROW:
        memcpy ( pDes, pSrc, mpImage->imageSize );
        break;
    case IN_COLUMS:
        unsigned int o = mpImage->width * mpImage->height;
        for ( unsigned int h = 0; h < mpImage->height; h++ ) {
            for ( unsigned int w = 0; w < mpImage->width; w++ ) {
                unsigned int indexSrc = ( h*mpImage->width+w ) * mpImage->nChannels;
                unsigned int indexDes = ( h+w*mpImage->height );
                for ( unsigned int c = 0; c < mpImage->nChannels; c++ ) {
                    pDes[indexDes + o * c] = pSrc[indexSrc+c];
                }
            }
        }
    }
}

int NaoQi::say ( const std::string &text, bool post ) {
    int taskID = -1;
    if ( !mpSpeechProxy ) {
        try {
            mpSpeechProxy = boost::shared_ptr<AL::ALTextToSpeechProxy> ( new AL::ALTextToSpeechProxy ( this->getParentBroker() ) );
        } catch ( AL::ALError& e ) {
            throw AL::ALError ( getName(), "subscribeCamera()", "Error on init ALVideoDevice! " + e.toString() );
        }
    }
    if ( post ) {
        taskID = mpSpeechProxy->post.say ( text );
    } else {
        mpSpeechProxy->say ( text );
    }
    return taskID;
}

int NaoQi::walkTo ( const float &x, const float &y, const float &theta, bool post ) {
    int taskID = -1;
    if ( post ) {
        taskID = mpMotionProxy->post.walkTo ( x, y, theta );
    } else {
        mpMotionProxy->walkTo ( x, y, theta );
    }
    return taskID;
}
bool NaoQi::walkIsActive ( ) {
    return mpMotionProxy->walkIsActive ();
}

void NaoQi::getSensors ( const std::vector<std::string> &names, std::vector<float> &values ) {
    AL::ALPtr<AL::ALMemoryFastAccess> pMemoryFastAccess = AL::ALPtr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess());
    //AL::ALMemoryFastAccess *pMemoryFastAccess = new AL::ALMemoryFastAccess;
    float x = mpMemProxy->getData( std::string("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"), 0);
    std::cout << x << std::endl;
    try {
        pMemoryFastAccess->ConnectToVariables(getParentBroker(), names, false);
        pMemoryFastAccess->GetValues(values);
    } catch ( AL::ALError& e ) {
        throw AL::ALError ( getName(), "getSensors()", "Error on MemoryFastAccess! " + e.toString() );
    }
}

NaoQi *NaoQi::getInstance ( std::string address, int port ) {
    std::string brokerName = "NaoQi";
    std::string brokerIP = "0.0.0.0";
    int brokerPort = 0 ;
    AL::ALPtr<AL::ALBroker> pBroker = AL::ALBroker::createBroker ( brokerName, brokerIP, brokerPort, address,  port );
    pBroker->setBrokerManagerInstance ( AL::ALBrokerManager::getInstance() );
    AL::ALPtr<AK::NaoQi> p = AL::ALModule::createModule<AK::NaoQi> ( pBroker, "NaoQi" );
    return p.operator->();
}
};
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
