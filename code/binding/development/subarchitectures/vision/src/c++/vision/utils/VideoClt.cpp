#include <iostream>
#include "vision/utils/ImageFrameStream.hpp"
#include "VideoClt.hpp"

using namespace std;
using namespace Vision;

const string VideoClt::msgNames[VideoClt::MSG_MAX_TYPE] =
{
  "MSG_GET_IMAGE",
  "MSG_RET_IMAGE"
};

void VideoClt::getImage(int camNum, ImageFrame &img) throw(BALTException)
{
  // request an image
  getOutStream().newMessage(VideoClt::MSG_GET_IMAGE);
  getOutStream() << camNum;
  getOutStream().finishMessage();

  // send, wait and receive answer. note: we block here
  getOutStream().sendBlocking();
  getInStream().recvBlocking();

  // receive image
  int msgType = m_stream_in.decodeNewMessage();
  if(msgType != VideoClt::MSG_RET_IMAGE)
    throw BALTException(__HERE__, "unknown message type %d", msgType);
  getInStream().newMessage();
  getInStream() >> img;
  getInStream().finishMessage();
}

string VideoClt::msgTypeToString(int msgType)
{
  return msgTypeToName(msgType, VideoClt::msgNames,
      VideoClt::MSG_MAX_TYPE);
}

