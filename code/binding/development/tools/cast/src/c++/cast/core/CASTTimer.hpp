#ifndef CASTTimer_hpp
#define CASTTimer_hpp

#include <sys/time.h>

namespace cast {
  
  /**
   * Class for measuring how many things happen per second
   **/
  class CASTRateMeter {
    
  public:
    CASTRateMeter();

    void increment();

    double getRate() const { 
      return m_rate; 
    }

    bool rateChange() const {
      return m_sigChange;
    }
    
  private:
    int m_count;
    double m_rate;
    double m_lastRate;
    double m_changeThres;
    timeval m_startTime;
    bool m_sigChange;
  }; // class RateMeter

  /**
   * A timer class 
   */
  class CASTTimer {
  public:
    CASTTimer(bool autoStart = false);
    
    /**
     * Call this method to start the timer. Has no effect if the timer
     * is already running
     *
     * @return 0 if started, 1 if already started
     */
    int start();

    /**
     * @return true if the timer is running
     */
    bool isRunning() { return m_running; }

    /**
     * Call this method to restart the timer. Has same effect as start
     * if timer not running, if running it will restart it.
     *
     */
    void restart();

    /**
     * Stop the timer and return the time bewteen start/restart and
     * the first stop after that. This means that if stop is called
     * more than once it returns the same value after that.
     *
     * @return returns the time between timer statr and the first time
     * stop was called after that.
     */
    double stop();

    /**
     * @return the time since the timer was started. Will return the
     * stop time if the timer is not running.
     */
    double split();

  protected:
    bool m_running;
    timeval m_startTime;
    double m_stopTime;
  };

};

#endif // CASTTimer_hpp
