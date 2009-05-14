#ifndef SOCKET_HPP
#define SOCKET_HPP

#include "balt/core/BALTException.hpp"


/**
 * A simple shallow socket class.
 * Not really necessary, but hides some of the uglier bits of C sockets (like
 * those wacky socket address structs) and gives it all a C++y touch.
 * Sockets can be local (= UNIX) or network sockets.
 *
 * Technical side note: We could also have included normal files (as all of the
 * above are represented by file descriptors) but STL fstreams are really more
 * appropriate for that.
 */
class Socket
{
private:
  /*
   * The range of ports we can freely use (Dynamic and/or Private Ports, see
   * /etc/servies or http://www.iana.org/assignments/port-numbers)
   */
  static const int MIN_PORT;
  static const int MAX_PORT;
  /**
   * Maximum length the queue of pending connections may grow to.
   */
  static const int MAX_LISTEN_QUEUE;

  /**
   * The socket file descriptor.
   */
  int fd;
  /**
   * Whether this is a listening (server) socket.
   */
  bool is_listen;
  /**
   * Whether the socket is local or network. Useful for debugging only.
   */
  bool is_local;

  /**
   * Get host address (IPv4) from hostname.
   * Hostname can be any valid IPv4 address:
   *   199.232.41.10
   *   www.gnu.org
   *   192.168.0.1
   *   penguin
   *   penguin.your.net.work
   *   localhost
   */
  void hostNameToAddress(const std::string &host_name, struct in_addr *host_addr)
    throw(BALTException);

  /**
   * Constructor used when a listening socket accepts and creates a new socket.
   */
  Socket(int _fd, bool _listen, bool _local)
  {
    fd = _fd;
    is_listen = _listen;
    is_local = _local;
  }

public:
  /**
   * Initialises the socket as unconnected.
   * Use one of the openX() functions to connect the socket.
   */
  Socket() {fd = -1; is_listen = false; is_local = false;}
  ~Socket() {close();}
  bool isOpen() {return fd != -1;}

  /**
   * Open a network socket.
   * @param host  host name, e.g. penguin.your.net, 192.168.0.14, localhost
   * @param port  port to connect to on the given host
   */
  void openInet(const std::string &host, int port) throw(BALTException);

  /**
   * Open a Unix socket, i.e. a socket file in the file system.
   * @param filename  name of the Unix socket file to connect to
   */
  void openLocal(const std::string &filename) throw(BALTException);

  /**
   * Opens a listen (server) socket.
   * @param port  The proposed port to listen on. If that port
   *              is free it will be used. Otherwise another free port will be
   *              found. 
   * @param max_port  Maximum port number to try.
   * @return The actual free port used.
   */
  int openListenInet(int port = MIN_PORT, int max_port = MAX_PORT)
    throw(BALTException);

  /**
   * Opens a local (Unix) socket.
   * @param filename  Name of the socket file, e.g. /tmp/sock-1234
   */
  void openListenLocal(const std::string &filename) throw(BALTException);

  /**
   * Closes the socket.
   */
  void close();

  /**
   * Try to receive at most the given number of bytes. Any number between 0 and
   * len bytes may be received. The function returns when no more data is
   * available.
   * @param buf  the buffer to fill, must be at least len bytes long
   * @param len  number of bytes we'd like to receive
   * @return  number of actually received bytes, 0 if none were available.
   *          Note that this is different from the C socket recv() behaviour,
   *          where a return value of 0 means end of file (remote end closed
   *          connection). We throw an esception for that case.
   */
  int recvNonBlocking(char *buf, int len) throw(BALTException);

  /**
   * Receive exactly the given number of bytes and wait until all have been
   * received.
   * @param buf  the buffer to fill, must be at least len bytes long
   * @param len  number of bytes we'd like to receive
   * @return  number of actually received bytes, which is always the same as len
   */
  int recvBlocking(char *buf, int len) throw(BALTException);

  /**
   * Try to send at most the given number og bytes. Any number between 0 and
   * len may be sent. The function returns when no more data can be sent.
   * @param buf  buffer to read from, must be at least len bytes long
   * @param len  number of bytes we'd like to send
   * @return  number of actually sent bytes.
   */
  int sendNonBlocking(char *buf, int len) throw(BALTException);

  /**
   * Send exactly the given number og bytes and wait until all have been sent.
   * @param buf  buffer to read from, must be at least len bytes long
   * @param len  number of bytes we'd like to send
   * @return  number of actually sent bytes, which is always the same as len.
   */
  int sendBlocking(char *buf, int len) throw(BALTException);

  /**
   * Creates a socket by accepting a pending connection request on a listen
   * (server) socket.
   * @return  The created socket.
   */
  Socket* accept() throw(BALTException);

  /**
   * Returns the actual socket file descriptor. This is needed for select()'ing
   * sockets. Use SocketSet for doing that. (Or do Your own select() if You have
   * other monitored filed descriptors lying around.)
   */
  int getFD() {return fd;}
};


/**
 * A socket set class used for monitoring sockets for read/write readiness.
 */
class SocketSet
{
private:
  fd_set read_set;
  fd_set write_set;
  int max_fd;

public:
  SocketSet();

  /**
   * Monitor a socket for read readiness.
   */
  void addReadSocket(Socket *sock) throw(BALTException);

  /**
   * Monitor a socket for write readiness./
   */
  void addWriteSocket(Socket *sock) throw(BALTException);

  /**
   * Monitor a socket for read and write readiness./
   */
  void addReadWriteSocket(Socket *sock)
  {
    addReadSocket(sock);
    addWriteSocket(sock);
  }

  /**
   * Wait indefinitely for any socket file descriptor to change.
   * @return The number of ready read and write sockets. Use isReadReady() and
   *         isWriteReady() to find out whether a particular socket is ready.
   */
  int select() throw(BALTException);

  /**
   * Wait for the specified time in seconds and microseconds for any socket file
   * descriptor to change.
   * Specifying zero waiting time means that the function returns immediately if
   * no socket is ready.
   * @return The number of ready read and write sockets. Use isReadReady() and
   *         isWriteReady() to find out whether a particular socket is ready.
   *         If timeout is 0, return immediately if no socket is ready.
   */
  int selectTimeout(int secs, int usecs) throw(BALTException);

  /**
   * Returns whether the given socket is ready for read after a call to select.
   */
  bool isReadReady(Socket *sock) throw(BALTException);

  /**
   * Returns whether the given socket is ready for write after a call to select.
   */
  bool isWriteReady(Socket *sock) throw(BALTException);
};

#endif
