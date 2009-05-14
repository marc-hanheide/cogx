#include "vision/components/VideoServer/VideoServer.h"
#include "vision/utils/ImageFrameStream.hpp"
#include "VideoSrv.hpp"

const string VideoSrv::msgNames[VideoSrv::MSG_MAX_TYPE] =
{
  "MSG_GET_IMAGE",
  "MSG_RET_IMAGE"
};

void VideoSrv::decode(int msgType)
{
  if(msgType == VideoSrv::MSG_GET_IMAGE)
    handleGetImage();
  else
    throw BALTException(__HERE__, "unknown message type %d", msgType);
}

void VideoSrv::handleGetImage()
{
  int cam;

  getInStream().newMessage();
  getInStream() >> cam;
  getInStream().finishMessage();

  m_vid->retrieveCurrentFrame(cam);

  getOutStream().newMessage(VideoSrv::MSG_RET_IMAGE);
  getOutStream() << m_vid->getCurrentFrame();
  getOutStream().finishMessage();

  getOutStream().sendBlocking();
}

string VideoSrv::msgTypeToString(int msgType)
{
  return msgTypeToName(msgType, VideoSrv::msgNames,
      VideoSrv::MSG_MAX_TYPE);
}

