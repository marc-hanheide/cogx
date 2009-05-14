/** @file Report.h
 *  @brief Configurator of report messages.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _REPORT_H_
#define _REPORT_H_

class Report {
 protected:
    bool m_bShow; // for displaying best value of hypothesis
    bool m_bDebug; // level-0 debug for displaying all values of hypothesis
    bool m_bDebug5;  // level 5 debug for displaying function calls
    bool m_bGraphic; // graphic display
    bool m_bLogEvent; // log messages to event.log

    bool show() {return m_bShow;}
    bool debug() {return m_bDebug;}
    bool debug5() {return m_bDebug5;}
    bool graphic() {return m_bGraphic;}
    bool logevent() {return m_bLogEvent;}

 public:
    Report();
    virtual ~Report();

    void show(bool onoff) {m_bShow=onoff;}
    void debug(bool onoff) {m_bDebug=onoff;}
    void debug5(bool onoff) {m_bDebug5=onoff;}
    void graphic(bool onoff) {m_bGraphic=onoff;}
    void logevent(bool onoff) {m_bLogEvent=onoff;}
};

#endif
