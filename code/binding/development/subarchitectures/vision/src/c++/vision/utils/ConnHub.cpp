/**
 * $Id$
 */

#include <unistd.h>

//included for INET_ADDRSTRLEN
#include<netinet/in.h>
//for O_CREAT
#include <fcntl.h> 

#include "ConnHub.hpp"

using namespace std;


ConnHub::ConnHub(int listenPort)
{
  m_sock_server = new Socket();
  m_listen_port = m_sock_server->openListenInet(listenPort);
}

ConnHub::~ConnHub()
{
  delete m_sock_server;
  for(list<Connection*>::iterator it = m_connects.begin();
      it != m_connects.end(); ++it)
    delete *it;
  for(map<string,ConnectionFactoryBase*>::iterator it = m_connFactories.begin();
      it != m_connFactories.end(); ++it)
    delete it->second;
}

void ConnHub::addConnectionFactory(ConnectionFactoryBase *fact)
    throw(BALTException)
{
  if(fact == 0)
    throw BALTException(__HERE__, "NULL pointer");
  string connType = fact->getConnectionType();
  if(m_connFactories.find(connType) == m_connFactories.end())
    m_connFactories[connType] = fact;
  else
    throw BALTException(__HERE__, "factory '%s' already set", connType.c_str());
}

void ConnHub::addConnection(Connection *conn) throw(BALTException)
{
  if(conn == 0)
    throw BALTException(__HERE__, "NULL pointer");
  m_connects.push_back(conn);
}

void ConnHub::waitAndProcessMessages(bool wait)
{
  SocketSet set;
  int num_ready;

  // prepare the set of sockets to check
  set.addReadSocket(m_sock_server);
  for(list<Connection*>::iterator i = m_connects.begin();
      i != m_connects.end(); ++i)
  {
    set.addReadSocket((*i)->getSocket());
    if((*i)->getOutStream().hasPending())
      set.addWriteSocket((*i)->getSocket());
  }

  if(wait)
    // wait indefinitely for any socket to become ready
    num_ready = set.select();
  else
    // return immediately if no socket is ready
    num_ready = set.selectTimeout(0, 0);
  if(num_ready > 0)
  {
    // somebody knocked on our server socket, let them in
    if(set.isReadReady(m_sock_server))
      acceptConnection();

    for(list<Connection*>::iterator i = m_connects.begin();
        i != m_connects.end(); ++i)
    {
      // ready to read: receive and if complete message, decode
      if(set.isReadReady((*i)->getSocket()))
        (*i)->receive();
      // ready to write: send buffered messages
      if(set.isWriteReady((*i)->getSocket()))
        (*i)->send();
    }
  }
}

/**
 * TODO: avoid blocking send. should be easy: don't create a separate outstream
 * but use the connections out stream and send non-blocking as ususal
 *   conn->init();
 *   conn->sendConnect();
 * TODO: also handle Unix sockets
 */
void ConnHub::openConnection(Connection *conn, const string &hostname, int port)
    throw(BALTException)
{
  if(conn == 0)
    throw BALTException(__HERE__, "NULL pointer");

  Socket *sock = new Socket();
  sock->openInet(hostname, port);
  ConnOutputStream outstream(sock);
  // the message type does not matter
  outstream.newMessage(0);
  outstream << conn->getConnectionType();
  outstream.finishMessage();
  // note: we deliberately want blocking send here
  outstream.sendBlocking();
  conn->init(sock);
  addConnection(conn);
}

/**
 * TODO: avoid blocking receive? not so trivial. we need an in stream to read
 * the connection type, only then can we create the actual connection and have
 * it handle stuff asynchronously.
 * solution: store fresh socket and its in stream (perhaps a special
 * AcceptConnection() along with the other connections. the only message that
 * special connection understands is the CONNECT message, upon which it creates
 * the proper connection of specified type, hands over its socket and deltes
 * itself.
 */
void ConnHub::acceptConnection() throw(BALTException)
{
  // note: acceptSocket() is blocking
  Socket *sock = m_sock_server->accept();
  ConnInputStream instream(sock);
  // here we want to wait until the whole message is received
  instream.recvBlocking();
  string connType;
  instream.decodeNewMessage(); // ignore returned msgType, we know it anyway
  instream.newMessage();
  instream >> connType;
  instream.finishMessage();
  if(m_connFactories.find(connType) != m_connFactories.end())
  {
    Connection *conn = m_connFactories[connType]->newConnection();
    conn->init(sock);
    addConnection(conn);
  }
  else
  {
    delete sock;
    throw BALTException(__HERE__, "unknown connection type '%s'",
        connType.c_str());
  }
}

string ConnHub::getHostName() throw(BALTException)
{
  char buf[INET_ADDRSTRLEN];
  if(gethostname(buf, INET_ADDRSTRLEN) != 0)
    throw BALTException(__HERE__, "gethostname() failed");
  return string(buf);
}

