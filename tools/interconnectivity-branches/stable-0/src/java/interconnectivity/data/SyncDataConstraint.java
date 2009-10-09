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

import java.util.HashMap;

import interconnectivity.io.SyncException;

import cast.core.CASTData;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>SyncDataConstraint</b> implements a data structure for a 
data constraint, which is part of the label of a transition in the 
synchronization automaton. A data constraint specifies a data type 
(the type of data item added to working memory), and possibly one 
or more attribute-value pairs. For each specified attribute in a data
item, the given value needs to match, for the data constraint to be
satisfied.  

@version 061019 (started 061018) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/

public class SyncDataConstraint { 

	//=================================================================
    // GLOBAL VARIABLES
    //=================================================================

	// Constants
	public final static int TRANSIT_TIMEOUT = 0;
	public final static int TRANSIT_WAIT    = 1;

	// Identifier
	private int id;
	// The ontological type of the data item 
	private String dataType; 
	// Hashmap with attributes and values to be matched, if any
	private HashMap<String,String> attrValues; 
	// The transition mode
	private int transitionMode;
	// The time-out value, if any (default is -1)
	private int timeOutValue;

	//=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================
	
	public SyncDataConstraint () { 
		init(); 
	} // end constructor
	
	
	public SyncDataConstraint (int i) { 
		init();
		id = i;
	} // end constructor
	
	private void init () { 
		id = -1;
		dataType = "";
		attrValues = new HashMap();
		transitionMode = TRANSIT_WAIT;
		timeOutValue = -1;
	} // end init
	
	
	//=================================================================
    // ACCESSOR METHODS
    //=================================================================
	
	/** Adds an attribute-value constraint to the data constraint.  
		
		@throws SyncException if the attribute has already been specified
	*/

	public void addAttributeValue(String attr, String value) 
		throws SyncException 
	{ 
		if (!attrValues.containsKey(attr)) { 
			attrValues.put(attr,value);
		} else { 
			String v = (String) attrValues.get(attr); 
			throw new SyncException("Attribute ["+attr+"] has already been specified with value ["+v+"] in data constraint "+id);
		} // end if..else check for attribute existence
	} // end addAttributeValue

	/** Returns the ontological data type of the constraint. */

	public String getDataType () { return dataType; }

	/** Returns the identifier of the constraint */ 
	public int getId () { return id; }
	
	/** Returns the time-out value */
	public int getTimeoutValue () { return timeOutValue; }

	/** Returns the transition mode */
	public int getTransitionMode () { return transitionMode; } 


	/** The method <i>isSatisfied</i> returns a boolean indicating 
		whether the data constraint is satisfied on the given data 
		item. 
		
		<p><b>Note:</b> currently, only data types are checked!</p>
		
		@param  dataItem the data item to apply the constraint to
		@return boolean indicating constraint satisfaction on item
	*/

	public boolean isSatisfied(CASTData dataItem) { 
		boolean result = false; 
		String type = dataItem.getType();
		
		if (type.equals(dataType)) { 
			result = true; 
			// here, more checking needs to be done!!
			// System.out.println("Match constraint on ["+dataType+"] with given ["+type+"]");
		} // end if.. check for right data type
		return result;
	} // end isSatisfied
	
	/** Sets the identifier of the data constraint */
	public void setId (int i) { id = i; }

	/** Sets the data type of the constraint */ 
	public void setDataType (String type) { dataType = type; }

	/** Sets the transition mode. See the constants defined in this file. */
	public void setTransitionMode (int tm) { transitionMode = tm; }
	
	/** Sets the time-out value for the data constraint */ 
	public void setTimeoutValue (int tov) { timeOutValue = tov; }

	/** Formatted output of the constraint */
	public String toString () { 
		String result = "DC["+id+"]";
		if (transitionMode == TRANSIT_WAIT) { 
			result = result+" mode:[WAIT]";
		} else { 
			result = result+" mode:[TIMEOUT] timeOutValue:["+timeOutValue+"]";	
		} // end if..else check for mode
		result = result+" data-type:["+dataType+"]"; 
		return result; 
	} // end toString 

} // end SyncEdge

