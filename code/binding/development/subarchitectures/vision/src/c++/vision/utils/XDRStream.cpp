/**
 * $Id: XDRStream.cc,v 1.2 2006/11/21 11:42:11 mxz Exp mxz $
 */

#include "XDRStream.hpp"

using namespace std;


const int XDRStream::Buffer::DEFAULT_INIT_SIZE = 1024;

XDRStream::Buffer::Buffer(int init_cap)
{
  buf = 0;
  size = 0;
  cap = 0;
  ensureCapacity(init_cap);
}

XDRStream::Buffer::~Buffer()
{
  delete[] buf;
}

bool XDRStream::Buffer::ensureCapacity(int need_size)
{
  if(need_size > cap)
  {
    int newcap = max(need_size, cap > 0 ? 2*cap : DEFAULT_INIT_SIZE);
    char *newbuf = new char[newcap];
    if(cap > 0)
    {
      if(size > 0)
        memcpy(newbuf, buf, size);
      delete[] buf;
    }
    cap = newcap;
    buf = newbuf;
    return true;
  }
  else
    return false;
}

bool XDRStream::Buffer::setSize(int s)
{
  bool enlarged = ensureCapacity(s);
  size = s;
  return enlarged;
}


XDRStream::XDRStream(xdr_op op, Socket *s)
{
  mode = op;
  sock_side = 0;
  user_side = 1;
  // note: buffer capacity must be > 0, otherwise initialisation of XDR stream
  // with the buffer would fail
  buf[sock_side].ensureCapacity(Buffer::DEFAULT_INIT_SIZE);
  buf[user_side].ensureCapacity(Buffer::DEFAULT_INIT_SIZE);
  initXDR(sock_side);
  initXDR(user_side);
  sock = 0;
  if(s != 0)
    setSocket(s);
}

XDRStream::~XDRStream()
{
  exitXDR(sock_side);
  exitXDR(user_side);
}

void XDRStream::setSocket(Socket *s) throw(BALTException)
{
  if(s == 0)
    throw BALTException(__HERE__, "trying to connect stream to NULL socket");
  if(sock != 0)
    throw BALTException(__HERE__, "stream is already connected");
  sock = s;
}

void XDRStream::initXDR(int which)
{
  // Actually this should be bufsize, rather than bufcap, because only
  // [0, bufsize) contains valid data.
  // But then we would have to call initXDR() for each message, which would
  // cause unnecessary overhead.
  xdrmem_create(&xdrs[which], buf[which].data(),
      (u_int)buf[which].getCapacity(), mode);
}

void XDRStream::exitXDR(int which)
{
  xdr_destroy(&xdrs[which]);
}

int XDRStream::getXDRPos(int which)
{
  return (int)xdr_getpos(&xdrs[which]);
}

void XDRStream::setXDRPos(int pos, int which) throw(BALTException)
{
  if(!xdr_setpos(&xdrs[which], u_int(pos)))
    throw BALTException(__HERE__, "xdr set pos failed");
}

void XDRStream::incBufSize(int inc, int which)
{
  if(buf[which].getSize() + inc <= buf[which].getCapacity())
    buf[which].incSize(inc);
  // if the buffer has to be enlared (and thus memory reallocated) we have to
  // re-initialise the XDR stream.
  else
  {
    int pos = getXDRPos(which);
    exitXDR(which);
    buf[which].incSize(inc);
    initXDR(which);
    setXDRPos(pos, which);
  }
}


XDRInputStream::XDRInputStream(Socket *sock)
  : XDRStream(XDR_DECODE, sock)
{
  cur_msg_end = -1;
  num_messages[sock_side] = num_messages[user_side] = 0;
  status = RECV_HEADER;
  pending_recv = sizeof(Header);
  incBufSize(pending_recv, sock_side);
}

void XDRInputStream::swapBuffers()
{
  swap(sock_side, user_side);

  // reset size and position for new socket side buffer
  buf[sock_side].setSize(0);
  setXDRPos(0, sock_side);
  // and we are always expecting a header in a fresh socket side buffer
  incBufSize(sizeof(Header), sock_side);

  // reset XDR position for new user side buffer
  setXDRPos(0, user_side);
}

void XDRInputStream::readHeader(Header &header, int which) throw(BALTException)
{
  // Note that we read the header from the _input_ buffer. Normally we only read
  // from the output buffer, but in case of a message about to be received into
  // the input buffer, we have to read its size from that buffer.
  if(!xdr_int(&xdrs[which], &header.size))
    throw BALTException(__HERE__, "failed XDR read");
}

void XDRInputStream::newMessage() throw(BALTException)
{
  if(isDecoding())
    throw BALTException(__HERE__,
        "trying to start a new message while already reading a message");
  if(num_messages[user_side] <= 0)
    throw BALTException(__HERE__, "there is no message to read");

  // note: we read the header again, because
  // a) it is in the buffer and we need to get past it anyway
  // b) knowing the message size allows us to check whether we read past the
  //    current message while decoding
  Header header;
  readHeader(header, user_side);
  cur_msg_end = getXDRPos(user_side) + header.size;
}

void XDRInputStream::finishMessage() throw(BALTException)
{
  if(!isDecoding())
    throw BALTException(__HERE__, "there is no message to finish");

  cur_msg_end = -1;
  num_messages[user_side]--;
}

/**
 * We keep reading until either no more data is available or we have read a
 * complete message. 
 * The latter is almost always true for rather small messages and reasonably
 * fast network connections.
 * Note that this constitutes no busy waiting, as we drop out as soon as no
 * more data is available. The while-loop is necessary as we need at least
 * two steps: read fixed size message header (containing the a priori unknown
 * size of the following message) and the variable size message itself.
 */
bool XDRInputStream::recvNonBlocking() throw(BALTException)
{
  bool full_msg_rececived = false;
  bool try_reading_more = true;
  // keep reading until either no more data is available or we have read a
  // complete message.
  while(try_reading_more)
  {
    // the buffer is always exactly big enough to contain pending bytes
    int received = sock->recvNonBlocking(
        buf[sock_side].data() + buf[sock_side].getSize() - pending_recv,
        pending_recv);
    if(received > 0)
    {
      pending_recv -= received;
      // if we have completed reading some part (header or data)
      if(pending_recv == 0)
      {
        if(status == RECV_HEADER)
        {
          Header header;
          setXDRPos(buf[sock_side].getSize() - sizeof(Header), sock_side);
          readHeader(header, sock_side);
          pending_recv = header.size;
          incBufSize(pending_recv, sock_side);
          status = RECV_DATA;
          // and keep reading, because the messsage data is probably also ready
        }
        else // status == RECV_DATA
        {
          try_reading_more = false;
          full_msg_rececived = true;
          num_messages[sock_side]++;
          // and prepare for next header
          pending_recv = sizeof(Header);
          incBufSize(pending_recv, sock_side);
          status = RECV_HEADER;
        }
      }
    }
    else
      try_reading_more = false;
  }
  if(full_msg_rececived)
    // if the output buffer is empty, we can switch buffers now
    if(num_messages[user_side] == 0)
      swapBuffers();
  return full_msg_rececived;
}

/**
 * TODO: this could use blockig read from the socket direcly.
 *       receive everything: receive swap receive
 *       receive from all sockets? or just from this one?
 */
void XDRInputStream::recvBlocking() throw(BALTException)
{
  bool complete = false;
  SocketSet set;

  set.addReadSocket(sock);
  while(!complete)
  {
    // wait indefinitely for the socket to be ready and then receive whatever is
    // available
    if(set.select() > 0)
      complete = recvNonBlocking();
  }
}

bool XDRInputStream::readBool() throw(BALTException)
{
  bool_t bt;
  if(!checkRead(multOf4(sizeof(bool_t)), user_side))
    throw BALTException(__HERE__, "buffer underflow");
  if(!xdr_bool(&xdrs[user_side], &bt))
    throw BALTException(__HERE__, "failed XDR read");
  return (bool)bt;
}

int XDRInputStream::readInt() throw(BALTException)
{
  int i;
  if(!checkRead(multOf4(sizeof(int)), user_side))
    throw BALTException(__HERE__, "buffer underflow");
  if(!xdr_int(&xdrs[user_side], &i))
    throw BALTException(__HERE__, "failed XDR read");
  return i;
}

double XDRInputStream::readDouble() throw(BALTException)
{
  double d;
  if(!checkRead(multOf4(sizeof(double)), user_side))
    throw BALTException(__HERE__, "buffer underflow");
  if(!xdr_double(&xdrs[user_side], &d))
    throw BALTException(__HERE__, "failed XDR read");
  return d;
}

/*
 * TODO: buffering the string is rather inefficient. maybe use own XDR string
 * I/O
 */
string XDRInputStream::readString() throw(BALTException)
{
  // buffer, will be allocated by xdr_wrapstring
  char *tmp = 0;
  // TODO: we can't checkRead() here, as we don't know the strings size.
  // xdr_wrapstring will allocate necessary memory if pointer is 0
  if(!xdr_wrapstring(&xdrs[user_side], &tmp))
    throw BALTException(__HERE__, "failed XDR read");
  string s(tmp);
  delete[] tmp;
  return s;
}

void XDRInputStream::readBinary(char *data, int len) throw(BALTException)
{
  if(!checkRead(multOf4(len), user_side))
    throw BALTException(__HERE__, "buffer underflow");
  if(!xdr_opaque(&xdrs[user_side], data, (u_int)len))
    throw BALTException(__HERE__, "failed XDR read");
}


XDROutputStream::XDROutputStream(Socket *sock)
  : XDRStream(XDR_ENCODE, sock)
{
  cur_msg_start = -1;
  pending_send = 0;
}

void XDROutputStream::swapBuffers()
{
  swap(sock_side, user_side);

  // reset size and positon of user side buffer
  buf[user_side].setSize(0);
  setXDRPos(0, user_side);

  // set pending to what is now in the socket side buffers size
  pending_send = buf[sock_side].getSize();
}

void XDROutputStream::writeHeader(Header &header, int which)
  throw(BALTException)
{
  // we use xdr_int() instead of our own method writeInt() because the latter
  // automatically increases buffer size, while here we want to write somewhere
  // into an empty space in the middle of the buffer
  if(!xdr_int(&xdrs[which], &header.size))
    throw BALTException(__HERE__, "failed XDR write");
}

void XDROutputStream::newMessage() throw(BALTException)
{
  if(isEncoding())
    throw BALTException(__HERE__,
        "trying to start a new message while already constructing a message");

  cur_msg_start = getXDRPos(user_side);
  incBufSize(sizeof(Header), user_side);
  setXDRPos(cur_msg_start + sizeof(Header), user_side);
}

void XDROutputStream::finishMessage() throw(BALTException)
{
  if(!isEncoding())
    throw BALTException(__HERE__, "there is no message to finish");

  Header header;
  int msg_end = getXDRPos(user_side);

  // size of message not including the size of the header
  header.size = msg_end - cur_msg_start - sizeof(Header);
  // note that the header is written to the start of the current message
  setXDRPos(cur_msg_start, user_side);
  writeHeader(header, user_side);
  setXDRPos(msg_end, user_side);
  cur_msg_start = -1;

  // if output buffer has run dry, let's swap
  if(pending_send == 0)
    swapBuffers();
}

void XDROutputStream::send(bool blocking)
{
  if(pending_send > 0)
  {
    int sent;
    if(blocking)
    {
      sent = sock->sendBlocking(
        buf[sock_side].data() + buf[sock_side].getSize() - pending_send,
        pending_send);
    }
    else
    {
      sent = sock->sendNonBlocking(
        buf[sock_side].data() + buf[sock_side].getSize() - pending_send,
        pending_send);
    }
    pending_send -= sent;
    // If we have nothing more to send and the in buffer has data, let's swap.
    // (Just to be sure we also check whether the user buffer is currently
    // encoding a message, though strictly speaking that should not happen in
    // our single thread world.)
    // Note that this is important: We might have stuck a lot of messages in the
    // user buffer while the socket was not write ready, but now we are done,
    // i.e.  no more new/finishMessages(). So we can not rely solely on
    // finishMessage() to do the swaps.
    if(pending_send == 0 && buf[user_side].getSize() > 0 && !isEncoding())
      swapBuffers();
  }
}

void XDROutputStream::sendNonBlocking()
{
  send(false);
}

/**
 * TODO  receive everything: receive swap receive
 *       receive from all sockets? or just from this one?
 */
void XDROutputStream::sendBlocking()
{
  send(true);
}

void XDROutputStream::writeBool(bool b) throw(BALTException)
{
  bool_t bt = (bool_t)b;
  incBufSize(multOf4(sizeof(bool_t)), user_side);
  if(!xdr_bool(&xdrs[user_side], &bt))
    throw BALTException(__HERE__, "failed XDR write");
}

void XDROutputStream::writeInt(int i) throw(BALTException)
{
  incBufSize(multOf4(sizeof(int)), user_side);
  if(!xdr_int(&xdrs[user_side], &i))
    throw BALTException(__HERE__, "failed XDR write");
}

void XDROutputStream::writeDouble(double d) throw(BALTException)
{
  incBufSize(multOf4(sizeof(double)), user_side);
  if(!xdr_double(&xdrs[user_side], &d))
    throw BALTException(__HERE__, "failed XDR write");
}

/*
 * TODO: buffering the string is rather inefficient. maybe use own XDR string
 * I/O
 */
void XDROutputStream::writeString(const string &s) throw(BALTException)
{
  // We have to make a copy of the string because xdr_string takes a
  // non-const pointer and string::c_str() returns a const pointer.
  char *tmp = new char[s.length() + 1];
  // copy, including the terminating 0-byte
  memcpy(tmp, s.c_str(), s.length() + 1);
  // an XDR string consists of its length as unsigned int plus characters padded
  // to a multiple of 4
  incBufSize(sizeof(unsigned int) + multOf4(s.length()), user_side);
  if(!xdr_wrapstring(&xdrs[user_side], &tmp))
    throw BALTException(__HERE__, "failed XDR write");
  delete[] tmp;
}

void XDROutputStream::writeBinary(char *data, int len) throw(BALTException)
{
  incBufSize(multOf4(len), user_side);
  if(!xdr_opaque(&xdrs[user_side], data, (u_int)len))
    throw BALTException(__HERE__, "failed XDR write");
}


XDROutputStream& operator<<(XDROutputStream &stream, bool b)
  throw(BALTException)
{
  stream.writeBool(b);
  return stream;
}

XDRInputStream& operator>>(XDRInputStream &stream, bool &b) throw(BALTException)
{
  b = stream.readBool();
  return stream;
}

XDROutputStream& operator<<(XDROutputStream &stream, int i) throw(BALTException)
{
  stream.writeInt(i);
  return stream;
}

XDRInputStream& operator>>(XDRInputStream &stream, int &i) throw(BALTException)
{
  i = stream.readInt();
  return stream;
}

XDROutputStream& operator<<(XDROutputStream &stream, double d)
  throw(BALTException)
{
  stream.writeDouble(d);
  return stream;
}

XDRInputStream& operator>>(XDRInputStream &stream, double &d)
  throw(BALTException)
{
  d = stream.readDouble();
  return stream;
}

XDROutputStream& operator<<(XDROutputStream &stream, const string &s)
  throw(BALTException)
{
  stream.writeString(s);
  return stream;
}

XDRInputStream& operator>>(XDRInputStream &stream, string &s)
  throw(BALTException)
{
  s = stream.readString();
  return stream;
}

