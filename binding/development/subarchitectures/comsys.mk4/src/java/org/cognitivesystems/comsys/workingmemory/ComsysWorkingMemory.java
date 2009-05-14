// =================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
// =================================================================

package org.cognitivesystems.comsys.workingmemory;

// =================================================================
// IMPORTS
// =================================================================


import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.core.data.CASTWorkingMemoryItem;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
 * The class <b>ComsysWorkingMemory</b> implements the working memory
 * that is shared between processing components in the communication
 * subsystem. The general data structure for storing content in the
 * working memory is the SitDialInt structure; see ComsysEssentials.idl
 * for its definition, and the definitions of the data structures
 * underlying it.
 * 
 * @version 060924 (started 060924)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @author Nick A. Hawes (n.a.hawes@cs.bham.ac.uk)
 */

/**
 * Specialise sub-arch
 * 
 * @author nah
 */

public class ComsysWorkingMemory extends SubarchitectureWorkingMemory {

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    /**
     * @param _id
     */

    public ComsysWorkingMemory(String _id) {
        super(_id);
        setSendXarchChangeNotifications(true);
    }

    // =================================================================
    // OPERATION METHODS
    // =================================================================

    @Override
    protected boolean addToWorkingMemory(String _id,
            CASTWorkingMemoryItem _data) {
        boolean result = super.addToWorkingMemory(_id, _data);
        log("WM size: [" + m_workingMemory.size() + "]");
        return result;
    } // end addToWorkingMemory

    @Override
    protected CASTWorkingMemoryItem deleteFromWorkingMemory(String _id, 
							    String _component) {
        CASTWorkingMemoryItem result = m_workingMemory.remove(_id);
        log("WM size: [" + m_workingMemory.size() + "]");
        return result;
    } // end deleteFromWorkingMemory

    @Override
	protected boolean overwriteWorkingMemory(String _id, 
						 CASTWorkingMemoryItem _data,
						 String _component) {
        boolean result = super.m_workingMemory.overwrite(_id, _data);
        log("WM size: [" + m_workingMemory.size() + "]");
        return result;
    } // end overwriteWorkingMemory

    // =================================================================
    // RUN METHODS
    // =================================================================

    /**
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
    	    	

    } // end runComponent

} // end class
