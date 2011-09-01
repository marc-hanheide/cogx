/*
 * UDPHdl.h
 *
 *  Created on: Feb 23, 2011
 *      Author: max
 */

#ifndef UDPHDL_H_
#define UDPHDL_H_

#include <iostream>
#include <deque>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/concept_check.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>



namespace V4R {
using boost::asio::ip::udp;
///Simple class to handle UDP messages
template<typename T>
class UDPHdl {
    typedef boost::shared_ptr<boost::thread> ThreadPtr;
    typedef boost::shared_ptr<udp::socket> SocketPtr;
    typedef boost::shared_ptr<udp::resolver::query> QueryPtr;
    typedef boost::shared_ptr<udp::resolver> ResolverPtr;
public:
    enum State {
        NA = 0,
        RECEIVER = 1,
        TRANSMITTER = 2,
    };
    /**
     * Constructor
     * @post initReceiver or initTransmitter
     **/
    UDPHdl() {
        use_callback_ = false;
        state_ = NA;
    }
    ///Destructor
    ~UDPHdl() {
        if (thread_receiverPtr_.get() != NULL) thread_receiverPtr_->join();
    }
    /**
     * Initializes the class as receiver
     * @param port
     * @param queySize
     */
    int initReceiver(short port, int queySize = 1) {
        if (socketPtr_.get()) {
            std::cerr << "UDPHdl::initReceiver: class already initialized!\n";
            return 1;
        }
        try {
            socketPtr_ = SocketPtr(new udp::socket(ioService_, udp::endpoint(udp::v4(), port)));
            max_queue = queySize;
            bindFnc();
            state_ = RECEIVER;
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
            return 1;
        }
        return 0;
    }


    /**
     * Initializes the class as transmitter (sender)
     * @param host
     * @param port
     */
    int initTransmitter(std::string &host, short port) {
        if (socketPtr_.get()) {
            std::cerr << "UDPHdl::initReceiver: class already initialized!\n";
            return 1;
        }
        try {
            socketPtr_ = SocketPtr(new udp::socket(ioService_, udp::endpoint(udp::v4(), 0)));
            resolverPtr = ResolverPtr(new udp::resolver(ioService_));
            queryPtr = QueryPtr(new udp::resolver::query(udp::v4(), host, boost::lexical_cast<std::string>(port)));
            state_ = TRANSMITTER;
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
            return 1;
        }
        return 0;
    }


    /**
     * Sends a object
     * @param data
     */
    void send(const T &data) {
        udp::resolver::iterator iterator_;
        iterator_ = resolverPtr->resolve(*queryPtr);
        socketPtr_->send_to(boost::asio::buffer(&data, sizeof(data)), *iterator_);
    }

    /**
     * starts with the receiving
     */
    void run() {
        ioService_.run();
    }

    /**
     * starts a thread to receive
     */
    void runThread() {
        if (thread_receiverPtr_.get() == NULL) {
            thread_receiverPtr_ = ThreadPtr(new boost::thread(boost::bind(&V4R::UDPHdl<T>::run, this)));
        } else {
            std::cerr << "UDPHdl::runThread, thread exists already!\n";
        }
    }

    /**
     * stops the thread to receive
     */
    void stopThread() {
        if (thread_receiverPtr_.get() != NULL) {
            ioService_.stop();
            thread_receiverPtr_->join();
            thread_receiverPtr_.reset();
        } else {
            std::cerr << "UDPHdl::stopThread, no thread to stop!\n";
        }
    }

    /**
     * returns zero if the com runs with the argument configuration
     * @param state
     * @param port
     * @param host
     * @return 0 on match, 1 if thread is not running, 2 on state missmatch
     * 3 on port missmatch, 4 on hostname missmatch, 5 state does not exist
     */
    int checkState(State state, short port, std::string host = "") {
        if (!socketPtr_.get()) return 1;
        if (state_ != state) return 2;
        short local_port;
        switch (state_) {
        case  TRANSMITTER:
            if (boost::lexical_cast<std::string>(port).compare(queryPtr->service_name()) != 0) return 3;
            if (host.compare(queryPtr->host_name()) != 0) return 4;
            break;
        case  RECEIVER:
            socketPtr_->local_endpoint().port();
            if (port != local_port) return 3;
            break;
        case NA:
            break;
        default:
            return 5;
        }
        return 0;
    }

    /**
     * returns ture if a thread is running
     */
    bool isRunning() {
        if (socketPtr_.get()) {
            return 1;
        } else {
            return 0;
        }
    }

    /**
     * Number of currently queued msg
     **/
    unsigned int nrOfQueuedMsg() {
        return msg_queue_.size();
    }

    /**
     * deques a element threadsave
     * @param msg copy of the dequed object
     * @param front on false it will deque the oldest message
     */
    unsigned int deque(T &msg, bool front = true) {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        unsigned int size = msg_queue_.size();
        if (size > 0) {
            if (front) {
                msg = msg_queue_.front();
                msg_queue_.pop_front();
            } else {
                msg = msg_queue_.back();
                msg_queue_.pop_back();
            }
        }
        return size;
    }

    /**
     * Returns the configuration send or receiver
     * @see NA, RECEIVER, TRANSMITTER
     */
    State getState() {
        return state_;
    }

    /**
     * Sets a callbackfunction which is called on incommig messages
     * @param f 
     * @code udp_.setCallback(boost::bind(&CDS::Crane::Receiver::receiveCallback, this, _1));
     */
    void setCallback(boost::function<void (T &)> f) {
	use_callback_ = true;
        callback_ = f;
    }

private:
    inline void bindFnc() {
        try {
            socketPtr_->async_receive_from(boost::asio::buffer(receiveBuffer_, max_length), sender_endpoint_, boost::bind(
                                               &UDPHdl::handle_receive_from, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd) {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        if ((!error && bytes_recvd > 0) && (bytes_recvd == sizeof(T))) {
            T &msg = *((T*) receiveBuffer_);
            while (msg_queue_.size() > max_queue) {
                msg_queue_.pop_back();
            }
            msg_queue_.push_front(msg);
            if(use_callback_) callback_(msg);
        } else {
            std::cerr << "received object does not match size : " << bytes_recvd << " bytes received, I expected " << sizeof(T) << "!\n";
        }
        bindFnc();
    }

private:
    State state_;
    boost::asio::io_service ioService_;
    SocketPtr socketPtr_;
    QueryPtr queryPtr;
    ResolverPtr resolverPtr;
    udp::endpoint sender_endpoint_;
    void *pReceiveFncPtr_;
    enum {
        max_length = sizeof(T) * 10
    };
    char receiveBuffer_[max_length];
    ThreadPtr thread_receiverPtr_;
    std::deque<T> msg_queue_;
    unsigned int max_queue;
    boost::interprocess::interprocess_mutex mutex_; /// used to prevent concurrent access to the data

    bool use_callback_;
    boost::function<void (T &)> callback_;

};
};

#endif /* UDPHDL_H_ */
