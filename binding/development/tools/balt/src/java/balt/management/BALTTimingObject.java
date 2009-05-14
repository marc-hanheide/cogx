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
public class BALTTimingObject {

    private BALTTime m_startTime;

    private int m_count;

    public void startTiming() {
        m_startTime = NativeProcessLauncher.getBALTTime();
    }

    
    /**
     * @return the cycles
     */
    public int getCount() {
        return m_count;
    }
    
    public void incrementCount() {
        m_count++;
    }

    public double countsPerSecond() {
        BALTTime current = NativeProcessLauncher.getBALTTime();
        BALTTime diff = NativeProcessLauncher.timeDiff(m_startTime,
            current);
        return m_count / NativeProcessLauncher.toSeconds(diff);
    }

}
