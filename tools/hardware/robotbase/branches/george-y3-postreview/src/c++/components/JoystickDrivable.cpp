//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include <sstream>

#include "JoystickDrivable.hpp"

#include <Hardware/Joystick/Rumblepad2Mapping.hh>
#include <AddressBank/ConfigFileReader.hh>

JoystickDrivable::JoystickDrivable()
{
  m_Joystick = 0;
  m_Jsp = 0;

  m_Joydrive = false;
  m_JoydriveRotOnly = false;
  m_JoydriveTransOnly = false;

  m_DeadswitchId = Cure::Hal::JOY_RP2_BUTTON_5;
}

JoystickDrivable::~JoystickDrivable()
{
  if (m_Joystick) {
    m_Joystick->stop(0);
    if (m_Jsp) delete m_Jsp;
    if (m_Joystick) delete m_Joystick;
  }
  m_Joystick = 0;
}

void
JoystickDrivable::configure(const std::map<std::string,std::string> & config)
{
  std::map<std::string,std::string>::const_iterator it;

  std::string cfgFile = "";
  if ((it = config.find("--cure-config-file")) != config.end()) {
    std::istringstream str(it->second);
    str >> cfgFile;
  }
  if ((it = config.find("-c")) != config.end()) {
    std::istringstream str(it->second);
    str >> cfgFile;
  }
  printf("JoystickDrivable: Using cfgFile=%s\n", cfgFile.c_str());

  std::string joyDev = "/dev/input/js0";
  int id_east_west = Cure::Hal::JOY_RP2_AXIS_LEFT_EW;
  int id_north_south = Cure::Hal::JOY_RP2_AXIS_LEFT_NS;
  int zero_trans_id = Cure::Hal::JOY_RP2_BUTTON_6;
  int zero_rot_id = Cure::Hal::JOY_RP2_BUTTON_8;

  double maxJoyV = 0.5;
  double maxJoyW = 0.5;

  Cure::ConfigFileReader cfg;
  if (cfg.init(cfgFile) != 0) {
    printf("JoystickDrivable: Could not open cure config file, testing default joystick setup\n");
  } else {
    std::string tmpStr, usedCfgFile;
    if (cfg.getString("JOYSTICK_DEVICE", true, tmpStr, usedCfgFile) == 0) {
      joyDev = tmpStr;
    }

    std::list<int> ids;
    if (cfg.getIntList("JOYSTICK_DRIVESAXES_IDS", true, ids, usedCfgFile, 2)
        == 0) {
      std::list<int>::iterator iter = ids.begin();
      id_east_west = *iter;
      iter++;
      id_north_south = *iter;
    }

    int id;
    if (cfg.getInt("JOYSTICK_DEADSWITCH_ID", true, id, usedCfgFile) == 0) {
      m_DeadswitchId = id;
    }

    if (cfg.getInt("JOYSTICK_ZEROTRANS_ID", true, id, usedCfgFile) == 0) {
      zero_trans_id = id;
    }

    if (cfg.getInt("JOYSTICK_ZEROROT_ID", true, id, usedCfgFile) == 0) {
      zero_rot_id = id;
    }

    double v;
    if (cfg.getDouble("JOYSTICK_MAX_V", true, v, usedCfgFile) == 0) {
      maxJoyV = v;
      printf("JoystickDrivable: Read maxJoyV=%.2fm/s from config file %s",
            v, usedCfgFile.c_str());
    }

    double w;
    if (cfg.getDouble("JOYSTICK_MAX_W", true, w, usedCfgFile) == 0) {
      maxJoyW = w;
      printf("JoystickDrivable: Read maxJoyW=%.2frad/s from config file %s",
            w, usedCfgFile.c_str());
    }
  }

  if ((it = config.find("--max-joy-v")) != config.end()) {
    std::istringstream str(it->second);
    str >> maxJoyV;
    printf("JoystickDrivable: Read maxJoyV=%.2fm/s from command line", maxJoyV);
  }

  if ((it = config.find("--max-joy-w")) != config.end()) {
    std::istringstream str(it->second);
    str >> maxJoyW;
    printf("JoystickDrivable: Read maxJoyW=%.2frad/s from command line", 
           maxJoyW);
  }


  m_Joystick = new Cure::Hal::JoystickEventThread(joyDev);
  if (m_Joystick->start()) {
    m_Jsp = new Cure::Hal::JoystickSpeedProducer(*m_Joystick);
    m_Jsp->registerEventHandler(*this);
    m_Joystick->setJoystickTimeout(0.2);
    m_Joystick->registerEventHandler(*this);
    m_Jsp->setMaxSpeeds(maxJoyV, maxJoyW);

    printf("JoystickDrivable: Using joystick device \"%s\"", 
           joyDev.c_str());
      
    m_Jsp->setMotionAxisMapping(id_east_west, id_north_south);
    printf("JoystickDrivable: joystick axis east-west: %d, north-south: %d\n", 
            id_east_west, id_north_south);
      
    m_Jsp->setDeadswitchId(m_DeadswitchId);
    printf("JoystickDrivable: joystick dead id: %d\n", m_DeadswitchId);

    m_Jsp->setZeroTranslationId(zero_trans_id);
    printf("JoystickDrivable: joystick zerotrans id: %d\n", zero_trans_id);

    m_Jsp->setZeroRotationId(zero_rot_id);
    printf("JoystickDrivable: joystick zerorot id: %d\n", zero_rot_id);
    
    printf("JoystickDrivable: USING JOYSTICK\n");

  } else {
    printf("JoystickDrivable: NOT USING JOYSTICK\n");
  }
}

void JoystickDrivable::handleJoystickEvent(Cure::Hal::JoystickInterface &joystick,
                                           int type, int id)
{
  m_Joydrive = joystick.getButton(m_DeadswitchId)->pressed;
  //printf("Got joystick event %d %d (joydrive=%d)\n", type, id);
}

void JoystickDrivable::handleSpeedEvent(double v, double w)
{
  m_JoyV = v;
  m_JoyW = w;
}

