/*
 * Comedian example code to demonstrate CAST functionality.
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
package comedyarch.subarchitectures.audience;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.cdl.WorkingMemoryEntry;
import cast.core.data.CASTWorkingMemoryEntry;

/**
 * Specialise sub-arch
 * 
 * @author nah
 */
public class AudienceWorkingMemory extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public AudienceWorkingMemory(String _id) {
        super(_id);
//        setOntology(ComedyOntologyFactory.getOntology());

        // determines whether this wm should broadcast to oher
        // sub-architectures
        setSendXarchChangeNotifications(true);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureWorkingMemory#receivePushData(java.lang.String,
     *      cast.core.data.CASTWorkingMemoryEntry)
     */
    @Override
    public void receivePushData(String _src,
                                CASTWorkingMemoryEntry _data) {
//        boolean talk = (_src.equals("audience.member"));
         boolean talk = false;

        if (talk) {
            println("local wme received");
            println("having a sleep before doing anything");
            sleepProcess(5000);
            println("woken up");
        }

        super.receivePushData(_src, _data);

        if (talk) {
            println("done stuff");
        }

    }

    @Override
    public void receivePushData(String _src,
                                WorkingMemoryEntry _data) {
//        boolean talk = (_src.equals("audience.member"));
         boolean talk = false;

        if (talk) {
            println("local wme received");
            println("having a sleep before doing anything");
            sleepProcess(10000);
            println("woken up");
        }

        super.receivePushData(_src, _data);

        if (talk) {
            println("done stuff");
        }

    }
    /*
     * (non-Javadoc)
     * 
     * @see framework.core.processes.FrameworkProcess#run()
     */
    @Override
    public void runComponent() {
    // handy for debugging!
    // while (m_status == ProcessStatus.RUN) {
    //
    // try {
    // m_semaphore.acquire();
    // //just print out memory contents
    // println(m_workingMemory);
    //
    // m_semaphore.release();
    // Thread.sleep(10000);
    // }
    // catch (InterruptedException e) {
    // e.printStackTrace();
    // }
    // }

    }
}
