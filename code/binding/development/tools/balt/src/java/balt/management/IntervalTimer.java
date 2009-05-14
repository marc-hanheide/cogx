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

/**
 * 
 */
package balt.management;

import balt.corba.autogen.FrameworkBasics.BALTTime;
import balt.jni.NativeProcessLauncher;

/**
 * Object used to time executions of things.
 * 
 * @author nah
 */
public class IntervalTimer {

    private BALTTime m_startTime;

    private int m_count;

    private double m_totalSeconds = 0;

    public synchronized void startInterval() {
        m_startTime = NativeProcessLauncher.getBALTTime();
    }

    public synchronized void stopInterval() {
        if (m_startTime != null) {
            m_totalSeconds += NativeProcessLauncher.toSeconds(NativeProcessLauncher.timeDiff(m_startTime,NativeProcessLauncher.getBALTTime()));
            m_count++;
            m_startTime = null;
        }
        else {
            throw new RuntimeException("interval not started");
        }
    }

    /**
     * @return the cycles
     */
    public int getCount() {
        return m_count;
    }

    public double averageIntervalDuration() {
        return m_totalSeconds/m_count;
    }

}
