
/*
 * BALT - The Boxes and Lines Toolkit for component communication.
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
package balt.core.processes;

import java.util.Properties;

/**
 * Abstract class for a process in Java. The process uses the Runnable
 * interface to support multi-threading. Users must implement the stop,
 * start and run methods in order to provide some functionality.
 */

public abstract class FrameworkProcess implements Runnable {

    /**
     * The ID for the process. Currently hidden to it remains unchanged
     * after construction.
     */
    private String m_fpid;

    public enum ProcessStatus {
        STOP, RUN
    };

    protected ProcessStatus m_status;

    /**
     * Protected default constructor. Unary String constructor must be
     * used to specify a process ID.
     */
    private FrameworkProcess() {
    // do nothing, but hide
    }

    /**
     * Main constructor for a FrameworkProcess. Used to specify the ID
     * of the process which is then used to uniquely indentify the
     * process.
     * 
     * @param _id
     *            The ID for the process. Must be unique.
     */
    public FrameworkProcess(String _id) {
        m_fpid = _id;
        m_status = ProcessStatus.STOP;
    }

    /**
     * Get the unique ID of this process.
     * 
     * @return The ID of this process.
     */
    public final String getProcessIdentifier() {
        return m_fpid;
    }

    /**
     * Tell the process that processing is about to start. This will be
     * called before the run() method.
     */
    public void start() {
        m_status = ProcessStatus.RUN;
    }

    /**
     * Tell the process that processing is about to end. This will be
     * called before the enclosing Thread object is terminated with a
     * join or exit.
     */
    public void stop() {
        m_status = ProcessStatus.STOP;
    }

    /**
     * Called when the processing thread actually starts running. Should
     * be the starting point of the main processing.
     */
    public abstract void run();
    
    /**
     * Give the process some kind of configuration information. 
     * @param _config The configuration information. Could be XML or a filename etc.
     */
    public abstract void configure(Properties _config);
}
