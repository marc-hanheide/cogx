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

#ifndef AK_NAOQI_HPP
#define AK_NAOQI_HPP

#include <iostream>
#include <string>
#include <vector>

#include <almodule.h>
#include <alxplatform.h>

#include <opencv/cv.h>



namespace AL {
class ALBroker;
class ALLoggerProxy;
class ALMotionProxy;
class ALVideoDeviceProxy;
class ALMemoryProxy;
class ALTextToSpeechProxy;
class DCMProxy;
}

namespace AK {
class NaoQi;

class NaoQi : public AL::ALModule  {
    static const double mVersion = 0.1;
public:
    enum ImageMemFormat   {
        IN_ROW = 0,
        IN_COLUMS = 1
    };

    static const int AKERROR = 1;
    static const int AKOK = 0;
    NaoQi(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName);
    virtual ~NaoQi();
    void unsubscribeCamera();
    void logInfo(const std::string text);
    int setWalkTargetVelocity (float x, float y, float theta, float frequency, bool post);
    void setStiffnesses (const std::vector<std::string> &names, const std::vector<float> &stiffnesses);
    void stiffnessInterpolation (const std::vector<std::string> &names, const std::vector<float> &stiffnesses, const std::vector<float> &times);
    int setAngles  (const std::vector<std::string> &names, const std::vector<float> &angles, float fractionMaxSpeed, bool post = false);
    std::vector<float> getAngles (const std::vector<std::string> &names, const bool &useSensorValues);
    std::vector<float> getPosition ( const std::string &name, const int &space, const bool &useSensorValues);
    int subscribeCamera (const int &resolution, const int &colorSpace, const int &fps, const int &camera = 0, bool bShow = false);
    int getCameraImageSize (int *pWidth, int *pHeight, int *pChannels);
    int getCameraImage (char *pDes, const ImageMemFormat eFormat = IN_ROW);
    void getSensors (const std::vector<std::string> &names, std::vector<float> &values);
    int say (const std::string &text, bool post = false);
    int walkTo  (const float &x, const float &y, const float &theta, bool post = false);
    bool walkIsActive ( );

    std::string version();
    void dataChanged ( const std::string& pDataName, const AL::ALValue& pValue, const std::string& pMessage );
    void init ( void );
    bool innerTest() {return true;};
    static NaoQi *getInstance(std::string address = "127.0.0.1", int port = 9559);
private:
    AL::ALPtr<AL::ALLoggerProxy> mpLogProxy;
    AL::ALPtr<AL::ALMotionProxy> mpMotionProxy;
    boost::shared_ptr<AL::ALVideoDeviceProxy> mpCameraProxy;
    AL::ALPtr<AL::ALMemoryProxy> mpMemProxy;
    boost::shared_ptr<AL::ALTextToSpeechProxy> mpSpeechProxy; 
    std::string mCamName;
    IplImage *mpImage;
};

}
#endif
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
