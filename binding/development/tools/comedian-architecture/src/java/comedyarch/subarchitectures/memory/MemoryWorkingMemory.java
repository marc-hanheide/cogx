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
package comedyarch.subarchitectures.memory;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.core.data.CASTWorkingMemoryItem;

/**
 * @author nah
 */
public class MemoryWorkingMemory extends SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public MemoryWorkingMemory(String _id) {
        super(_id);
    }

    @Override
    protected boolean addToWorkingMemory(String _id,
            CASTWorkingMemoryItem _data) {
        // println("adding to working memory: " + _ontEntry);
        return super.addToWorkingMemory(_id, _data);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureWorkingMemory#overwriteWorkingMemory(java.lang.String,
     *      java.lang.String, org.omg.CORBA.Any)
     */
    @Override
    protected boolean overwriteWorkingMemory(String _id,
            CASTWorkingMemoryItem _data, String _component) {
        boolean result = super.overwriteWorkingMemory(_id, _data, _component);
        // println("overwriting working memory: " + _ontEntry + "... "
        // + result);
        return result;
    }

}
