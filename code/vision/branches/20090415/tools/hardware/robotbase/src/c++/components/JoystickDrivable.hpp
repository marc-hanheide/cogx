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

#ifndef JoystickDrivable_hpp
#define JoystickDrivable_hpp

#include <map>
#include <Hardware/Joystick/JoystickSpeedProducer.hh>
#include <Hardware/Joystick/JoystickEventThread.hh>


class JoystickDrivable : public Cure::Hal::JoystickEventHandler,
                         public Cure::Hal::JoystickSpeedEventHandler
{
public:
  JoystickDrivable();
  ~JoystickDrivable();

protected:
  void configure(const std::map<std::string,std::string> & config);

  void handleJoystickEvent(Cure::Hal::JoystickInterface &joystick,
			   int type, int id);
  
  void handleSpeedEvent(double v, double w);

  Cure::Hal::JoystickEventThread *m_Joystick;
  Cure::Hal::JoystickSpeedProducer *m_Jsp;
  
  int m_DeadswitchId; 

  bool m_Joydrive;
  bool m_JoydriveRotOnly;
  bool m_JoydriveTransOnly;
  double m_JoyV, m_JoyW;
};

#endif // JoystickDrivable_hpp
