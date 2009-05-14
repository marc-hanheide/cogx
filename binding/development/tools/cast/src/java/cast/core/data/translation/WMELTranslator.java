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

/**
 * 
 */
package cast.core.data.translation;

import org.omg.CORBA.*;

import balt.corba.data.translation.FrameworkDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryEntryListHelper;

/**
 * @author nah
 */
public class WMELTranslator implements
        FrameworkDataTranslator<WorkingMemoryEntry[]> {

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.data.translation.FrameworkDataTranslator#getTCKind()
     */
    public TCKind getTCKind() {
        return WorkingMemoryEntryListHelper.type().kind();
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.data.translation.FrameworkDataTranslator#getTransClass()
     */
    public Class<WorkingMemoryEntry[]> getTransClass() {
        return WorkingMemoryEntry[].class;
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.data.translation.FrameworkDataTranslator#translate(java.lang.Object)
     */
    public Any translate(WorkingMemoryEntry[] _data)
            throws FrameworkDataTranslatorException {
        Any a = ORB.init().create_any();
        WorkingMemoryEntryListHelper.insert(a, _data);
        return a;
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.data.translation.FrameworkDataTranslator#translate(org.omg.CORBA.Any)
     */
    public WorkingMemoryEntry[] translate(Any _data)
            throws FrameworkDataTranslatorException {
        return WorkingMemoryEntryListHelper.extract(_data);
    }

}
