/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef CAST_CAST_COMPONENT_H_
#define CAST_CAST_COMPONENT_H_

#include "InspectableComponent.hpp"
#include "CASTUtils.hpp"

#include <balt/core/BALTCore.hpp>
#include <cast/cdl/CAST.hh>
          
#include <cstdarg>

/**
 * A basic component in the CAST framework. This class provides some
 * basic functionality useful for debugging, and also overrides some of
 * the abstract methodsd on FrameworkProcess so that all processes in
 * C++ and Java look as similar as possible.
 * 
 * 
 * @author nah
 */


namespace cast {

  class CASTComponent : 
    public InspectableComponent {

  public:

    /**
     * Construct a new component with the given unique ID.
     * 
     * @param _id
     *            The id used to identify this component.
     */
    CASTComponent(const std::string &_id);

    /**
     * Empty virtual destructor;
     */
    virtual ~CASTComponent();


    virtual void stop();

    /**
     * Overrides the configure method from FrameworkProcess to use
     * _config to set the subarchitecture ID.
     * 
     * @param _config
     *            The ID of the subarchitecture which contains this
     *            component.
     */
    virtual void configure(std::map<std::string,std::string> & _config);

    /**
     * Called on component start, after configure, before
     * runComponent. At this stage all connections are complete.
     */
    virtual void start();

  protected:


    /**
     * Print out the input in a formatted way.
     * 
     * @param _s
     *            Std::String to print.
     */
    virtual void println(const std::string & _s) const;


    /**
     * printf-like method
     * @param format   (in) printf-style format std::string
     */
    virtual void println(const char *format, ...) const;

  
    /**
     * Override the C++-specific run method from FrameworkProcess. This
     * method calls runComponent() then handles the return value
     * correctly.
     */
    virtual void* run_undetached(void *arg);

    /**
     * Method called in separate thread to run processing component.
     * This method is called for each component when it is started by
     * the framework process server.
     */
    virtual void runComponent() = 0;



    /**
     * Log a stl std::string. Only does anything if the variable m_bLogOutput
     * is true.
     *
     * @param _s The stl std::string to log.
     */  
    virtual void log(const std::string & _s) const;

    /**
     * printf-like method
     * @param format   (in) printf-style format std::string
     */
    virtual void log(const char *format, ...) const;


    /**
     * Use an stl std::string as debug output. Only does anything if the
     * variable m_bDebugOuput is true.
     *
     * @param _s The stl std::string to log.
     */  
    virtual void debug(const std::string & _s) const;

    /**
     * printf-like method
     * @param format   (in) printf-style format std::string
     */
    virtual void debug(const char *format, ...) const;

    /**
     * Acquire the semaphore for access to this component.
     */

    void lockProcess();

    /**
     * Release the semaphore for access to this component.
     */
    void unlockProcess();


    /**
     * Wait until the unlockProcess is successfully called.
     */
    void waitForUnlock();

    /**
     * Put the processes thread to sleep for a number of milliseconds.
     * 
     * @param _millis
     *            Number of milliseconds to sleep for.
     */
    void sleepProcess(unsigned long _millis);


    omni_mutex m_unlockNotificationMutex;
    omni_condition * m_pUnlockNotificationCondition;


    ///Controls log output
    bool m_bLogOutput;
    ///Controls debug output
    bool m_bDebugOutput;


  private:
    std::ostream & 
    startColourise(std::ostream &_stream) const;
    
    std::ostream & 
    endColourise(std::ostream &_stream) const;

    std::string m_startColourEscape;
    static const std::string END_COLOUR_ESCAPE;

    bool m_startCalled;
    bool m_configureCalled;


  };


} //namspace cast

#endif
