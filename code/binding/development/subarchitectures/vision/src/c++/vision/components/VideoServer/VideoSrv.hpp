#ifndef VIDEO_SRV_HPP
#define VIDEO_SRV_HPP

#include "vision/utils/Connection.hpp"

class VideoServer;

class VideoSrv : public Connection
{
private:
  VideoServer *m_vid;

  void handleGetImage();

public:
  enum MsgType
  {
    MSG_GET_IMAGE,
    MSG_RET_IMAGE,
    MSG_MAX_TYPE
  };
  static const std::string msgNames[MSG_MAX_TYPE];

  static std::string classConnectionType() {return "Video";}

  VideoSrv(VideoServer *vid) : m_vid(vid) {}
  virtual std::string getConnectionType() {return classConnectionType();}
  virtual std::string getConnectionName() {return getConnectionType()+ ":server";}
  virtual void decode(int msgType);
  virtual std::string msgTypeToString(int msgType);
};

#endif

