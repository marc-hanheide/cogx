/**
 * $Id: XDRStream.hh,v 1.2 2006/11/21 11:42:11 mxz Exp mxz $
 *
 */

#ifndef XDR_STREAM_HPP
#define XDR_STREAM_HPP

#include <string>
#include <vector>
#include <rpc/types.h>
#include <rpc/xdr.h>
#include "balt/core/BALTException.hpp"
#include "vision/utils/Socket.hpp"


/**
 * A buffered stream for handling data transfer in XDR format via sockets.
 * Data is sent and received in messages consisting of several data items,
 * rather than in many small packges item per item.
 */
class XDRStream
{
protected:
  /**
   * Message header.
   * The first item of every message.
   */
  class Header
  {
  public:
    /**
     * Size of the message, excluding the size of the leading header itself.
     */
    int size;
    // TODO: possibly add: checksum, priority?
  };

  /**
   * Invariant: 0 <= size <= capacity
   */
  class Buffer
  {
  public:
    static const int DEFAULT_INIT_SIZE;

  private:
    /**
     * The buffer itself.
     */
    char *buf;
    /**
     * Buffer capacity.
     */
    int cap;
    /**
     * Current buffer size, <= capacity.
     */
    int size;

  public:
    Buffer(int init_cap = 0);
    ~Buffer();
    /**
     * Make sure the indicated capacity is available and allocate more buffer if
     * necessary. Used size remains unchanged.
     * @return  Returns whether the buffer had to be expanded and thus memory
     *          reallocated in order to fit ensure the requested capacity.
     */
    bool ensureCapacity(int need_size);
    int getCapacity() {return cap;}
    /**
     * Set buffer size, allocating more buffer if necessary.
     * @return  Returns whether the buffer had to be expanded and thus memory
     *          reallocated in order to fit the requested size.
     */
    bool setSize(int s);
    /**
     * Increase buffer size, allocating more buffer if necessary.
     * @return  Returns whether the buffer had to be expanded and thus memory
     *          reallocated in order to fit the requested size.
     */
    bool incSize(int inc) {return setSize(getSize() + inc);}
    int getSize() {return size;}
    /**
     * Returns the buffers actual data.
     */
    char* data() {return buf;}
  };

  /**
   * The socket associated with the stream.
   * Note that the socket is opened and closed externally.
   */
  Socket *sock;
  /**
   * XDR operation can be XDR_ENCODE (output stream) or XDR_DECODE (input
   * stream).
   */
  xdr_op mode;
  /**
   * The XDR object associated with the stream.
   */
  XDR xdrs[2];
  Buffer buf[2];
  int sock_side;
  int user_side;

  /**
   * Returns the given number rounded up to a multiple of 4.
   */
  static int multOf4(int i) {return (i%4 != 0 ? i + 4 - i%4 : i);}
  /**
   * Let XDR know where its buffer is and whether it should encode or decode.
   */
  void initXDR(int which);
  /**
   * Free XDR internal data structure (but not the buffer!).
   */
  void exitXDR(int which);
  /**
   * Get position of XDR stream in buffer.
   */
  int getXDRPos(int which);
  /**
   * Set position of XDR stream in buffer.
   */
  void setXDRPos(int pos, int which) throw(BALTException);
  /**
   * Increase buffer size.
   * Re-initialises XDR stream if necessary (i.e. if buffer memory was
   * reallocated).
   */
  void incBufSize(int inc, int which);

  /**
   * Construct unconnected stream.
   * Only accessible for derived classes. A user never needs to create an
   * XDRStream directly.
   * @param op  Mode of operation: XDR_ENCODE or XDR_DECODE.
   * @param sock  A connected socket or 0, if we want to create the stream
   *              unconnected.
   */
  XDRStream(xdr_op op, Socket *s = 0);

public:
  /**
   * The destructor leaves the sockets as they are and only deletes buffers.
   */
  ~XDRStream();

  /**
   * Make stream connected by setting socket.
   * A stream may only be connected once and the socket must not be NULL (a NULL
   * socket is only allowd in the constructor).
   */
  void setSocket(Socket *s) throw(BALTException);
};

/**
 * An XDR stream for input.
 * Note that Flush() needs to be called before each new message.
 */
class XDRInputStream : public XDRStream
{
protected:
  /**
   * Receiving state: we can either be expecting a message header or the message
   * data itself.
   */
  enum Status {RECV_HEADER, RECV_DATA};
  /**
   * Stream status while asynchronously receiving a message.
   */
  Status status;
  /**
   * End of currently decoded message.
   * Set to -1 if no message is currently being decoded.
   */
  int cur_msg_end;
  /**
   * Number of complete messages in the respective buffer.
   */
  int num_messages[2];
  /**
   * Number of bytes we are currently waiting for while asynchronously receiving
   * a message.
   */
  int pending_recv;

  /**
   * Returns hether we are currently decoding a message, i.e. between
   * newMessage() and finishMessage().
   */
  bool isDecoding() {return cur_msg_end != -1;}
  /**
   * Check whether a read of size bytes would be ok.
   * @return  false if read would cause buffer underflow, true if ok.
   * TODO: if there are several messages in the buffer we could still read
   * beyond the end of the current message right into the next buffered message.
   */
  bool checkRead(int size, int which)
  {return getXDRPos(which) + size <= cur_msg_end;}
  /**
   * Read the message header containing the message size (excluding the header
   * size itslef).
   */
  void readHeader(Header &header, int which) throw(BALTException);
  /**
   * Swap socket and user side buffers and reset sizes and XDR positions.
   */
  void swapBuffers();

public:
  XDRInputStream(Socket *sock = 0);
  /**
   * Prepare the stream for next reading.
   */
  void newMessage() throw(BALTException);
  void finishMessage() throw(BALTException);
  /**
   * Receive what data is available, possibly continuing a previous message
   * which was only partially received.
   * Does not block. Messages can be read piece-wise.
   * @return True if a complete message was received. Now the message can be
   *         deserialsed. False if nothing or only a partial message was
   *         received.
   */
  bool recvNonBlocking() throw(BALTException);
  /**
   * Receive a complete message, blocking while waiting,
   * Call Recv() repeatedly if necessary, until a complete message is received.
   * Block while waiting for outstanding data.
   */
  void recvBlocking() throw(BALTException);
  bool hasPending() {return pending_recv;}
  bool readBool() throw(BALTException);
  int readInt() throw(BALTException);
  double readDouble() throw(BALTException);
  std::string readString() throw(BALTException);
  void readBinary(char *data, int len) throw(BALTException);
};

/**
 * An XDR stream for output.
 * Note that Flush() needs to be called to actually send the buffered message.
 */
class XDROutputStream : public XDRStream
{
protected:
  /**
   * Start of the message being currently encoded.
   * Set to -1 if no message is currently being encoded.
   */
  int cur_msg_start;
  /**
   * Number of bytes that wait being sent asynchronously.
   */
  int pending_send;

  /**
   * Returns whether we are currently encoding a message, i.e. between
   * newMessage() and finishMessage().
   */
  bool isEncoding() {return cur_msg_start != -1;}
  /**
   * Write the message header containing the message size (excluding the header
   * size itself).
   * The header is written to the start of the buffer (so space for it must be
   * reserved in advance).
   */
  void writeHeader(Header &header, int whcih) throw(BALTException);
  void send(bool blocking);
  /**
   * Swap socket and user side buffers and reset sizes and XDR positions.
   */
  void swapBuffers();

public:
  XDROutputStream(Socket *sock = 0);
  /**
   * Write current buffer and prepare for fresh writing.
   * Note: Blocks until all data is written.
   */
  void newMessage() throw(BALTException);
  void finishMessage() throw(BALTException);
  void sendNonBlocking();
  void sendBlocking();
  bool hasPending() {return pending_send;}
  void writeBool(bool b) throw(BALTException);
  void writeInt(int i) throw(BALTException);
  void writeDouble(double d) throw(BALTException);
  void writeString(const std::string &s) throw(BALTException);
  void writeBinary(char *data, int len) throw(BALTException);
};

// in/out operators for basic types
XDROutputStream& operator<<(XDROutputStream &stream, bool b)
  throw(BALTException);
XDRInputStream& operator>>(XDRInputStream &stream, bool &b)
  throw(BALTException);
XDROutputStream& operator<<(XDROutputStream &stream, int i)
  throw(BALTException);
XDRInputStream& operator>>(XDRInputStream &stream, int &i) throw(BALTException);
XDROutputStream& operator<<(XDROutputStream &stream, double d)
  throw(BALTException);
XDRInputStream& operator>>(XDRInputStream &stream, double &d)
  throw(BALTException);
XDROutputStream& operator<<(XDROutputStream &stream, const std::string &s)
  throw(BALTException);
XDRInputStream& operator>>(XDRInputStream &stream, std::string &s)
  throw(BALTException);

// in/out for vectors of stuff
// note: operators for stuff must obviously be defined
template<class T>
XDROutputStream& operator<<(XDROutputStream &stream, std::vector<T> &vec)
{
  int n = vec.size();
  stream << n;
  for(int i = 0; i < n; i++)
    stream << vec[i];
  return stream;
}

template<class T>
XDRInputStream& operator>>(XDRInputStream &stream, std::vector<T> &vec)
{
  int n;
  stream >> n;
  vec.resize(n);
  for(int i = 0; i < n; i++)
    stream >> vec[i];
  return stream;
}

#endif

