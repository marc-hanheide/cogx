//=================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package interconnectivity.data;

//=================================================================
// IMPORTS
//=================================================================


//-----------------------------------------------------------------
// INTERCONNECTIVITY IMPORTS
//-----------------------------------------------------------------

import java.util.HashMap;
import java.util.Iterator;

import interconnectivity.io.SyncException;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/** 

The class <b>SyncConstraint</b> implements a data structure for a 
information about a synchronization constraint, given in the 
synchronization specification. A constraint holds for an identifiable
process (by process name), and specifies a list of data constraints on 
which the process is to be synchronized. For each data constraint, 
the specification gives whether it is a "waiting" constraint (wait 
until data becomes available in working memory) or a "time-out" 
constraint (wait for only a limited period). 

@version 061018 (started 061018) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 

*/ 

public class SyncConstraint { 

    //=================================================================
    // GLOBAL VARIABLES
    //=================================================================

	// Process name for which synchronization constraint holds
	String processName;
	// HashMap with data constraints
	HashMap<Integer,SyncDataConstraint> dataConstraints;  
	
    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================
	
	public SyncConstraint () { 
		init();
	} // end constructor
	
	public SyncConstraint (String pid) { 
		init();
		processName = pid; 	
	} // end constructor
	
	private void init () { 
		processName = "";
		dataConstraints = new HashMap ();
	} // end init

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** Adds a data constraint 
	
		@throws SyncException if there is already a data constraint with the given id. 
	*/ 
	public void addDataConstraint (SyncDataConstraint dc) 
		throws SyncException 
	{ 
		Integer dcId = new Integer(dc.getId());
		if (!dataConstraints.containsKey(dcId)) { 
			dataConstraints.put(dcId,dc);
		} else { 
			throw new SyncException("Duplicate data constraint identifier ["+dcId.toString()+"] in constraint for process ["+processName+"]"); 
		} // end if..else check for unique data constraint id's
	} // end addDataConstraint

	/** Returns the process name */
	public String getProcessName () { return processName; } 

	/** Returns an iterator over the data constraints (value: SyncDataConstraint)*/ 
	public Iterator getDataConstraints () {
		return dataConstraints.values().iterator();
	} // end getDataConstraints

	/** Sets the process name */
	public void setProcessName (String pid) { processName = pid; }
	
} // end class
