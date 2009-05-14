/**
 * $Id$
 */

#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <string>
#include "balt/core/BALTException.hpp"
#include "XDRStream.hpp"


/**
 * TODO: check for in_message in all read/write functions.
 */
class ConnInputStream : public XDRInputStream
{
private:
  int curMsgType;

public:
  ConnInputStream() {curMsgType = -1;}
  ConnInputStream(Socket *sock) : XDRInputStream(sock), curMsgType(-1) {}
  int decodeNewMessage()
  {
    if(curMsgType == -1)
    {
      XDRInputStream::newMessage();
      *this >> curMsgType;
    }
    else
      throw BALTException(__HERE__,
          "trying to start a new message while already reading a message");
    return curMsgType;
  }
  void newMessage()
  {
    if(curMsgType == -1)
      throw BALTException(__HERE__, "failed");
  }
  void finishMessage()
  {
    curMsgType = -1;
    XDRInputStream::finishMessage();
  }
};


/**
 * TODO: check for in_message in all read/write functions.
 */
class ConnOutputStream : public XDROutputStream
{
public:
  ConnOutputStream() {}
  ConnOutputStream(Socket *sock) : XDROutputStream(sock) {}
  void newMessage(int type)
  {
    XDROutputStream::newMessage();
    *this << type;
  }
};


class Connection
{
protected:
  /**
   * Socket used by in- and output stream.
   */
  Socket *m_sock;
  /**
   * Input stream.
   * Data at this steram can arrive any time, without interfering with the
   * output stream.
   */
  ConnInputStream m_stream_in;
  /**
   * Output stream.
   * Data from this stream can be sent any time, without interfering with the
   * input stream.
   */
  ConnOutputStream m_stream_out;

  std::string msgTypeToName(int msgType, const std::string names[], int maxType);

public:
  static std::string classConnectionType() {return "NONE";}

  Connection();
  virtual ~Connection();
  virtual std::string getConnectionType() = 0;
  /**
   * Just for debugging purposes: return some nice despcriptive name.
   */
  virtual std::string getConnectionName() {return "unnamed";}
  /**
   * Initialise a connection with opened input and output sockets.
   * @param sock  Socket for in- and output stream.
   */
  void init(Socket *sock) throw(BALTException);
  Socket* getSocket() {return m_sock;}
  ConnInputStream& getInStream() {return m_stream_in;}
  ConnOutputStream& getOutStream() {return m_stream_out;}
  /**
   * Receive whatever data is available from input stream.
   * @return  Returns whether a complete message was received, which can now be
   *          decoded.
   */
  void receive();
  void decodeMessage();
  void send();
  /**
   * Decode a message buffered in the input stream and call whatever callbacks
   * are associated with specific messages.
   */
  virtual void decode(int msgType) = 0;
  /**
   * Returns a string description of the given message type. Useful only for
   * debugging. The string description is never used anywhere inside the
   * system.
   */
  virtual std::string msgTypeToString(int msgType);
};


class ConnectionFactoryBase
{
public:
  virtual std::string getConnectionType() = 0;
  virtual Connection *newConnection() = 0;
  // The virtual desctructor just keeps the compiler from complaining.
  virtual ~ConnectionFactoryBase() {}
};

template <class Owner, class Conn>
class ConnectionFactory : public ConnectionFactoryBase
{
private:
  Owner *m_owner;

public:
  ConnectionFactory(Owner *owner) : m_owner(owner) {}
  virtual std::string getConnectionType()
  {
    return Conn::classConnectionType();
  }
  virtual Connection* newConnection()
  {
    return new Conn(m_owner);
  }
};

#endif

