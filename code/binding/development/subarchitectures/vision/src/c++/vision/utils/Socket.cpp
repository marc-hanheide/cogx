#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>  // HACK

#ifdef __APPLE__

#include <sys/un.h> 

struct sockaddr_un sizecheck;
#ifndef UNIX_PATH_MAX
#define UNIX_PATH_MAX sizeof(sizecheck.sun_path)
#endif

#else

#include <linux/un.h> // basically same as, but slightly nicer than <sys/un.h>

#endif



#include "Socket.hpp"

using namespace std;

const int Socket::MIN_PORT = 49152;
const int Socket::MAX_PORT = 65535;
const int Socket::MAX_LISTEN_QUEUE = 50;

void Socket::openInet(const string &host, int port) throw(BALTException)
{
  in_addr host_addr;
  sockaddr_in sock_addr;

  if(isOpen())
    throw BALTException(__HERE__, "socket is already open");

  fd = socket(PF_INET, SOCK_STREAM, 0);
  if(fd == -1)
    throw BALTException(__HERE__, "socket error: %s", strerror(errno));

  hostNameToAddress(host, &host_addr);

  bzero(&sock_addr, sizeof(sock_addr));
  sock_addr.sin_family = PF_INET;
  sock_addr.sin_addr = host_addr;
  sock_addr.sin_port = htons((in_port_t)port);

  // note: connect is blocking - DEADLOCK DANGER!
  if(connect(fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) != 0)
  {
    close();
    throw BALTException(__HERE__, "connect error to internet socket %s:%d: %s",
        host.c_str(), port, strerror(errno));
  }

  is_local = false;
}

void Socket::openLocal(const string &filename) throw(BALTException)
{
  struct sockaddr_un sock_addr;

  if(isOpen())
    throw BALTException(__HERE__, "socket is already open");

  fd = socket(PF_UNIX, SOCK_STREAM, 0);
  if(fd == -1)
    throw BALTException(__HERE__, "socket error: %s", strerror(errno));

  sock_addr.sun_family = PF_UNIX;
  strncpy(sock_addr.sun_path, filename.c_str(), UNIX_PATH_MAX);

  // note: connect is blocking - DEADLOCK DANGER!
  if(connect(fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) != 0)
  {
    close();
    throw BALTException(__HERE__, "connect error to UNIX socket %s: %s",
        filename.c_str(), strerror(errno));
  }

  is_local = true;
}

int Socket::openListenInet(int port, int max_port) throw(BALTException)
{
  struct sockaddr_in sock_addr;
  bool bound = false;

  if(isOpen())
    throw BALTException(__HERE__, "socket is already open");

  fd = socket(PF_INET, SOCK_STREAM, 0);
  if(fd == -1)
    throw BALTException(__HERE__, "socket error: %s", strerror(errno));

  port = max(port, MIN_PORT);
  max_port = min(max_port, MAX_PORT);

  bzero(&sock_addr, sizeof(sock_addr));
  sock_addr.sin_family = PF_INET;
  sock_addr.sin_addr.s_addr = INADDR_ANY;
  sock_addr.sin_port = htons((in_port_t)port);

  // try ports until we find a free one
  while(!bound && port < max_port)
  {
    if(bind(fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) == 0)
      bound = true;
    else
    {
      port++;
      sock_addr.sin_port = htons((in_port_t)port);
    }
  }
  if(port >= max_port)
    throw BALTException(__HERE__, "no free ports available: %s",
        strerror(errno));

  if(listen(fd, MAX_LISTEN_QUEUE) != 0)
    throw BALTException(__HERE__, "listen error: %s", strerror(errno));

  is_local = false;
  is_listen = true;

  cout << "listening on port: " << port << endl;  // HACK
  return port;
}

void Socket::openListenLocal(const string &filename) throw(BALTException)
{
  struct sockaddr_un sock_addr;

  if(isOpen())
    throw BALTException(__HERE__, "socket is already open");

  fd = socket(PF_UNIX, SOCK_STREAM, 0);
  if(fd == -1)
    throw BALTException(__HERE__, "socket error: %s", strerror(errno));

  bzero(&sock_addr, sizeof(sock_addr));
  sock_addr.sun_family = PF_UNIX;
  strncpy(sock_addr.sun_path, filename.c_str(), UNIX_PATH_MAX);

  // in case a socket file was left over from an aborted program, delete the
  // socket file
  unlink(filename.c_str());

  if(bind(fd, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) != 0)
    throw BALTException(__HERE__, "bind error: %s", strerror(errno));

  if(listen(fd, MAX_LISTEN_QUEUE) != 0)
    throw BALTException(__HERE__, "listen error: %s", strerror(errno));

  is_local = true;
  is_listen = true;

  cout << "listening on file: " << filename << endl;  // HACK
}

void Socket::close()
{
  if(fd != -1)
  {
    ::close(fd);
    fd = -1;
  }
}

int Socket::recvNonBlocking(char *buf, int len) throw(BALTException)
{
  int received = (int)recv(fd, buf, len, MSG_DONTWAIT);
  if(received == 0)
  {
    throw BALTException(__HERE__,
        "recv failed: remote end shut down connection");
  }
  else if(received == -1)
  {
    // if the error just means that no more data is available (i.e. we would
    // normally block), that's ok. we just stop reading.
    if(errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    else
      throw BALTException(__HERE__, "recv failed: %s", strerror(errno));
  }
  return received;
}

int Socket::recvBlocking(char *buf, int len) throw(BALTException)
{
  // note: the number of bytes actually received might always be less than the
  // number we specified (e.g. if the system just can't handle receiving in one
  // chunk)
  int total = 0;
  while(total < len)
  {
    int received = (int)recv(fd, buf + total, len - total, 0);
    if(received == 0)
      throw BALTException(__HERE__,
          "recv failed: remote end shut down connection");
    else if(received == -1)
      throw BALTException(__HERE__, "recv failed: %s", strerror(errno));
    total += received;
  }
  return total;
}

int Socket::sendNonBlocking(char *buf, int len) throw(BALTException)
{
  int sent = (int)send(fd, buf, len, MSG_DONTWAIT);
  if(sent == -1)
  {
    // if the error just means that no more data is available (i.e. we would
    // normally block), that's ok. we just stop reading.
    if(errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    else
      throw BALTException(__HERE__, "send of bytes %d failed: %s", len,
         strerror(errno));
  }
  return sent;
}

int Socket::sendBlocking(char *buf, int len) throw(BALTException)
{
  // note: the number of bytes actually sent might always be less than the
  // number we specified (e.g. if the system just can't handle sending in one
  // chunk)
  int total = 0;
  while(total < len)
  {
    int sent = (int)send(fd, buf + total, len - total, 0);
    if(sent == -1)
      throw BALTException(__HERE__, "send of bytes %d failed: %s", len,
         strerror(errno));
    total += sent;
  }
  return total;
}

Socket* Socket::accept() throw(BALTException)
{
  int new_fd = -1;

  if(!is_listen)
    throw BALTException(__HERE__,
        "accept() can only be called with a listen socket");

  // note: accept() is blocking - DEADLOCK DANGER!
  if(is_local)
  {
    sockaddr_un addr;
    socklen_t addr_len = sizeof(struct sockaddr_un);
    new_fd = ::accept(fd, (struct sockaddr *)&addr, &addr_len);
  }
  else
  {
    sockaddr_in addr;
    socklen_t addr_len = sizeof(struct sockaddr_in);
    new_fd = ::accept(fd, (struct sockaddr *)&addr, &addr_len);
  }
  if(new_fd == -1)
    throw BALTException(__HERE__, "failed accept: %s", strerror(errno));

  // return a new, connected (non-listen) socket with same locality
  return new Socket(new_fd, false, is_local);
}

void Socket::hostNameToAddress(const string &host_name,
    struct in_addr *host_addr) throw(BALTException)
{
  struct hostent *he = gethostbyname(host_name.c_str());
  if(he)
  {
    if(he->h_addrtype == PF_INET)
      memcpy(host_addr, he->h_addr_list[0], he->h_length);
    else
      throw BALTException(__HERE__, "only IPv4 addresses are supported");
  }
  else
    throw BALTException(__HERE__, "failed to get address for '%s'",
        host_name.c_str());
}


SocketSet::SocketSet()
{
  FD_ZERO(&read_set);
  FD_ZERO(&write_set);
  max_fd = -1;
}

void SocketSet::addReadSocket(Socket *sock) throw(BALTException)
{
  if(sock == 0)
    throw BALTException(__HERE__, "socket is NULL");
  if(!sock->isOpen())
    throw BALTException(__HERE__, "socket is not connected");
  FD_SET(sock->getFD(), &read_set);
  max_fd = max(sock->getFD(), max_fd);
}

void SocketSet::addWriteSocket(Socket *sock) throw(BALTException)
{
  if(sock == 0)
    throw BALTException(__HERE__, "socket is NULL");
  if(!sock->isOpen())
    throw BALTException(__HERE__, "socket is not connected");
  FD_SET(sock->getFD(), &write_set);
  max_fd = max(sock->getFD(), max_fd);
}

int SocketSet::select() throw(BALTException)
{
  int max = max_fd + 1;
  int ready = ::select(max, &read_set, &write_set, NULL, NULL);
  if(ready == -1)
    throw BALTException(__HERE__, "failed select: %s", strerror(errno));
  return ready;
}

int SocketSet::selectTimeout(int secs, int usecs) throw(BALTException)
{
  int max = max_fd + 1;
  int ready;
  struct timeval tv;
  tv.tv_sec = secs;
  tv.tv_usec = usecs;
  ready = ::select(max, &read_set, &write_set, NULL, &tv);
  if(ready == -1)
    throw BALTException(__HERE__, "failed select: %s", strerror(errno));
  return ready;
}

bool SocketSet::isReadReady(Socket *sock) throw(BALTException)
{
  if(sock == 0)
    throw BALTException(__HERE__, "socket is NULL");
  if(!sock->isOpen())
    throw BALTException(__HERE__, "socket is not connected");
  return FD_ISSET(sock->getFD(), &read_set);
}

bool SocketSet::isWriteReady(Socket *sock) throw(BALTException)
{
  if(sock == 0)
    throw BALTException(__HERE__, "socket is NULL");
  if(!sock->isOpen())
    throw BALTException(__HERE__, "socket is not connected");
  return FD_ISSET(sock->getFD(), &write_set);
}

