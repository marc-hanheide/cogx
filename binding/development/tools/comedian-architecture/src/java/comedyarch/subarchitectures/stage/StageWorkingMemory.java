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
package comedyarch.subarchitectures.stage;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;

/**
 * Specialise sub-arch
 * 
 * @author nah
 */
public class StageWorkingMemory extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public StageWorkingMemory(String _id) {
        super(_id);
//        setOntology(ComedyOntologyFactory.getOntology());

        // determines whether this wm should broadcast to oher
        // sub-architectures
        setSendXarchChangeNotifications(true);


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
