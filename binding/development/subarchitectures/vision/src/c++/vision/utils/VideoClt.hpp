#ifndef VIDEO_CLT_HPP
#define VIDEO_CLT_HPP

#include "vision/idl/Vision.hh"
#include "vision/utils/Connection.hpp"

class VideoClt : public Connection
{
public:
  enum MsgType
  {
    MSG_GET_IMAGE,
    MSG_RET_IMAGE,
    MSG_MAX_TYPE
  };
  static const std::string msgNames[MSG_MAX_TYPE];

  static std::string classConnectionType() {return "Video";}

  VideoClt() {}
  virtual std::string getConnectionType() {return classConnectionType();}
  virtual std::string getConnectionName() {return getConnectionType()+ ":client";}
  // note: all our communication is synchronous (blocking), so we do the
  // decoding in the same methods as the encoding
  virtual void decode(int msgType) {}
  virtual std::string msgTypeToString(int msgType);

  void getImage(int camNum, Vision::ImageFrame &img) throw(BALTException);
};

#endif


