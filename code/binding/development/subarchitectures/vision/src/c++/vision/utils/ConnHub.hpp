/**
 * $Id$
 */

#ifndef CONN_HUB_HPP
#define CONN_HUB_HPP

#include <list>
#include <map>
#include <string>
#include "balt/core/BALTException.hpp"
#include "Socket.hpp"
#include "Connection.hpp"

 
class ConnHub
{
private:
  int m_listen_port;
  Socket *m_sock_server;
  std::list<Connection*> m_connects;
  std::map<std::string, ConnectionFactoryBase*> m_connFactories;

  void acceptConnection() throw(BALTException);
  void addConnection(Connection *conn) throw(BALTException);

public:
  ConnHub(int listenPort = 0);
  ~ConnHub();
  void addConnectionFactory(ConnectionFactoryBase *fact) throw(BALTException);
  void openConnection(Connection *conn, const std::string &hostname, int port)
    throw(BALTException);
  void waitAndProcessMessages(bool wait);
  int getListenPort() {return m_listen_port;}
  /**
   * Returns own host name.
   */
  static std::string getHostName() throw(BALTException);
};

#endif

