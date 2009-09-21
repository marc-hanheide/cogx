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
// CAST IMPORTS
//-----------------------------------------------------------------

import java.util.*;

import interconnectivity.io.SyncException;
import interconnectivity.io.SyncNoSuchEdgeException;

import cast.core.CASTData;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>SyncState</b> implements a data structure for a state
in a state-based specification of synchronization constraints. A state
specifies one or more processes that are runnable at that point in time, 
a collection of incoming transition edges, a collection of outgoing 
transition edges, and a collection of working memory data-items that 
can be generated at that state. The generated data-items are the union 
over the output specified for the processes. 

@version 061018 (started 061018) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/

public class SyncState { 

    //=================================================================
    // GLOBAL VARIABLES
    //=================================================================

	// Identifier of the state
	private int id; 
	// Identifiers of the processes runable at this state
	private Vector<String> processIds;
	// Generated data types (key), with the id of the generating process (value)
	private HashMap<String,String> generatedDataTypes;
	// Incoming edges
	private Vector<SyncEdge> inEdges; 
	// Outgoing edges
	private Vector<SyncEdge> outEdges;
	// Whether the state has a timed-out transition 
	private boolean hasTimeOut; 
	// Which edge is timing out
	private SyncEdge timingOutEdge; 
	// Time-out time in milliseconds
	private int timeOutValue; 
	
	// Explicitly indicated start and end state
	private boolean isIndicatedStartState = false;
	private boolean isIndicatedEndState = false;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public SyncState (int i) { 
		init();
		id = i;
	} // 
	
	private void init () { 
		id = -1;
		processIds = new Vector();
		generatedDataTypes = new HashMap();
		inEdges = new Vector();
		outEdges = new Vector();
		hasTimeOut = false; 
		timingOutEdge = null; 
		timeOutValue = -1;
	} // end init
	
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** The method <i>addProcess</i> adds a process (by id) and a vector
		of data-items it generates for working memory. 
		
		@param  pid The string identifier of the process
		@param  items A vector with types of data items the process generates
		@throws SyncException if a process with the same id has already been added, 
						or if a data type has already been added for another process. 
	*/ 

	public void addProcess (String pid, Vector items) throws SyncException { 
		if (!processIds.contains(pid)) { 
			processIds.addElement(pid);
			for (Iterator itemsIter = items.iterator(); itemsIter.hasNext(); ) { 
				String nextItem = (String) itemsIter.next();
				if (!generatedDataTypes.containsKey(nextItem)) { 
					generatedDataTypes.put(nextItem,pid); 
				} else { 
					String opid = (String) generatedDataTypes.get(nextItem);
					throw new SyncException("Data type ["+nextItem+"] is already generated by process ["+opid+"]");
				} // end if..else.. check for unique data types
			} // end for
						
		} else { 
			throw new SyncException("Process with id ["+pid+"] already known to state "+id);
		} // end if..else check for new process id
	} // end addProcess

	
	/** The method <i>addInEdge</i> adds an incoming edge to the state 
		
		@param	edge The edge to be added 	
	*/

	public void addInEdge (SyncEdge edge) { 
		inEdges.addElement(edge);
	} // end addInEdge


	/** The method <i>addOutEdge</i> tries to add an outgoing edge to the state. The method
		throws an exception if an outgoing edge is being added for a data type that is not generated by 
		a process at this state. If the state has one or more timed-out edges, the time-out is set to the
		edge with the lowest time-out value. 
		
		@param	edge The edge to be added 	
		@throws SyncException if no process generates the required data type
	*/

	public void addOutEdge (SyncEdge edge) 
		throws SyncException 
	{ 
		String dataType = edge.getDataConstraint().getDataType();
		if (generatedDataTypes.containsKey(dataType)) { 
			outEdges.addElement(edge);
			if (edge.getTransitionMode() == SyncDataConstraint.TRANSIT_TIMEOUT) { 
				if (!hasTimeOut) {
					hasTimeOut = true;
					timingOutEdge = edge;
					timeOutValue = edge.getTimeoutValue();
				} else {
					if (edge.getTimeoutValue() < timeOutValue) { 
						timingOutEdge = edge;
						timeOutValue = edge.getTimeoutValue();
					} // end if.. check for briefer time-out
				} // end if..else check for available time-out information
			} // end if.. check for time-out connection
		} else {
			throw new SyncException("The data type ["+dataType+"] on edge ["+edge.getId()+"] is not generated by any process on state ["+id+"]");
		} // end if..else 
	} // end addInEdge

	/** Returns a boolean indicating whether the given data type is generated by 
		the process(es) at the state. 
	*/ 
	public boolean generatesDataType (String dataType) {
		return (generatedDataTypes.containsKey(dataType)); 
	} // end generatesDataType

	/** Returns the Id of the state */
	public int getId () { return id; }

	/** Returns the process id's */
	
	public Vector getProcessIds () { return processIds; }

	/** The method <i>getSatisfyingEdges</i> returns a Vector with SyncEdge objects along which a transit can be made 
	    in the current state, given the provided data type. If there is no such edge, an exception is thrown (rather 
		than that we provide a null value). 
		
		@throws SyncNoSuchEdgeException if there is no satisfying outgoing edge. 
	*/ 
	public Vector getSatisfyingEdges (CASTData dataItem) 
		throws SyncNoSuchEdgeException
	{ 
		Vector result = new Vector();
		Iterator edgeIter = outEdges.iterator();
		boolean found = false;
		while (edgeIter.hasNext() && !found) { 
			SyncEdge edge = (SyncEdge) edgeIter.next();
			if (edge.isSatisfied(dataItem)) { 
				result.addElement(edge);
			} // end if.. check whether satisfying edge
		} // end for
		if (result.size() != 0) { 
			return result ;
		} else { 
			throw new SyncNoSuchEdgeException("No outgoing edge for data type ["+dataItem.getType()+"] in state ["+id+"]");
		} // end if..else check for return result
	} //end getSatisfyingEdge





	/** Returns the (quickest) timing-out edge */ 
	public SyncEdge getTimingOutEdge () { return timingOutEdge; }
	
	
	/** Returns a boolean indicating whether the state has incoming edges */ 
	public boolean hasInEdges () { 
		return (inEdges.size() != 0); 
	} // end hasInEdges
	
	/** Returns a boolean indicating whether the state has outgoing edges */ 
	public boolean hasOutEdges () { 
		return (outEdges.size() != 0); 
	} // end hasOutEdges	
	
	/** Returns a boolean whether the state has a timing-out edge */ 
	public boolean hasTimeOut () { return hasTimeOut; }

	/** Returns a boolean whether the state supports an information 
		process with the given identifier */
	
	public boolean supports (String ipid) { return processIds.contains(ipid); } 

	/** Returns a formatted representation of the state */
	public String toString () { 
		String result = "State ["+id+"]\n"; 
		result = result+"Processes: ["+processIds.toString()+"]\n\n"; 
		result = result+"In-Edges:\n"; 
		for (Iterator inIter = inEdges.iterator(); inIter.hasNext(); ) { 
			result = result + ( ((SyncEdge) inIter.next()).toString() );
		} // end for
		result = "\n\n"+result+"Out-Edges:\n"; 
		for (Iterator outIter = outEdges.iterator(); outIter.hasNext(); ) { 
			result = result + ( ((SyncEdge) outIter.next()).toString() );
		} // end for		
		if (!hasTimeOut) { 
			result = result+"\n"+"No time-out constraints";
		} else { 
			result = result+"\n"+"Time-out on edge ["+timingOutEdge.getId()+"] at "+timeOutValue+"ms.";
		} // end if..else
		return result;
	} // end toString
	
	public boolean isIndicatedEndState() {return isIndicatedEndState; }
	
	public void setAsIndicatedEndState() {isIndicatedEndState = true ; }
	
	public boolean isIndicatedStartState() {return isIndicatedStartState; }
	
	public void setAsIndicatedStartState() {isIndicatedStartState = true ; }

	
} // end class