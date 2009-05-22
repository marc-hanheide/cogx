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

import cast.core.CASTData;


//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>SyncEdge</b> implements a data structure for a state
transition a state-based specification of synchronization constraints. 
A transition specifies a data-constraint, and a timing constraint. The 
transition is triggered when data of a given data-type is added to 
working memory, possibly with further constraints of attribute-values. 
The timing constraint specifies whether the connection can be transited 
after a certain time (in milliseconds) should no data be provided, or 
that a transition is only possible once data is added to working memory.  

@version 061018 (started 061018) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/

public class SyncEdge { 

	//=================================================================
    // GLOBAL VARIABLES
    //=================================================================


	
	int timeoutValue; 
	
	// Identifier of the edge
	private int id;
	// Mode of the transition (time-out, wait)
	private int transitionMode;
	// Data type
	private SyncDataConstraint dataConstraint;
	
	// State from which the edge connects
	private int fromState;
	// State to which the edge connects
	private int toState; 
	
	//=================================================================
    //  CONSTRUCTOR METHODS
    //=================================================================

	public SyncEdge (int i) { 
		init(); 
		id = i; 
	} // end constructor
	
	public SyncEdge (int i, SyncDataConstraint dc) { 
		init();
		id = i;
		dataConstraint = dc;
	} // end constructor

	public SyncEdge (int i, SyncDataConstraint dc, int tm) { 
		init();
		id = i;
		dataConstraint = dc;
		transitionMode = tm; 
	} // end constructor

	
	private void init () { 
		id = -1;
		transitionMode = SyncDataConstraint.TRANSIT_WAIT;
		dataConstraint = new SyncDataConstraint();
		fromState = -1;
		toState = -1;
		timeoutValue = -1;
	} // end init

	//=================================================================
    //  ACCESSOR METHODS
    //=================================================================

	/** The method <i>getDataConstraint</i> returns the data constraint
		on the edge */

	public SyncDataConstraint getDataConstraint () { 
		return dataConstraint;
	} // end getDataConstraint

	/** Returns the identifier of the state from which the edge connects */
	public int getFromState () { return fromState; } 
	
	/** Returns the identifier of the edge */
	public int getId () { return id; }

	/** Returns the time-out value (in milli-seconds) */ 
	public int getTimeoutValue () { return timeoutValue; }

	/** Returns the identifier of the state to which the edge connects */ 
	public int getToState () { return toState; }

	/** Returns the transition mode of the edge */
	public int getTransitionMode () { return transitionMode; }

	/** The method <i>isSatisfied</i> returns a boolean indicating 
		whether the data constraint on the edge is satisfied, on the
		given data item. */

	public boolean isSatisfied (CASTData dataItem) { 
		return dataConstraint.isSatisfied(dataItem);
	} // end isSatisfied

	/** Sets the data constraint */
	public void setDataConstraint (SyncDataConstraint dc) { dataConstraint = dc; }

	/** Sets the id of the state from which the edge connects */
	public void setFromState (int fs) { fromState = fs; }

	/** Sets the id of the state to which the edge connects */
	public void setToState (int ts) { toState = ts; }

	/** Sets the transition mode */
	public void setTransitionMode (int m) { transitionMode = m; } 

	/** Sets the time-out value (in milli-seconds) */
	public void setTimeoutValue (int tov) { timeoutValue = tov; }
	
	/** Returns a formatted representation of the edge information */ 
	public String toString () { 
		String result = "Edge ["+id+"]\n";
		result = result+" From state: ["+fromState+"] to state: ["+toState+"]\n";
		result = result+" "+dataConstraint.toString()+"\n";  
		return result;
	} // end toString




} // end class