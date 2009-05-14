/**
 * $Id$
 */

#include "ConnHub.hpp"
#include "Connection.hpp"

using namespace std;


Connection::Connection()
{
  m_sock = 0;
}

Connection::~Connection()
{
  delete m_sock;
}

void Connection::init(Socket *sock) throw(BALTException)
{
  if(m_sock != 0)
    throw BALTException(__HERE__, "input socket is already initialised");
  m_sock = sock;
  if(m_sock == 0)
    throw BALTException(__HERE__, "invalid input socket");
  m_stream_in.setSocket(m_sock);
  m_stream_out.setSocket(m_sock);
}

void Connection::receive()
{
  while(m_stream_in.recvNonBlocking())
    decodeMessage();
}

void Connection::send()
{
  m_stream_out.sendNonBlocking();
}

void Connection::decodeMessage()
{
  int msgType = m_stream_in.decodeNewMessage();
  decode(msgType);
}

/**
 * If not overwritten, just return the given message type number.
 */
string Connection::msgTypeToString(int msgType)
{
  char buf[100];
  snprintf(buf, 100, "%d", msgType);
  return buf;
}

string Connection::msgTypeToName(int msgType, const string names[],
    int maxType)
{
  if(msgType < maxType)
    return names[msgType];
  else
    return "invalid message type";
}

