// =================================================================
// Copyright (C) 2006-2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.cast;

//=================================================================
// IMPORTS

// Java
import cast.cdl.WorkingMemoryAddress;
import java.util.Iterator;
import java.util.Vector;

// CAST
import cast.core.CASTData;

/** 
 * The class <b>ProcessingData</b> provides a wrapper object around
 * a Vector of CASTTypedDataWithID objects, to be operated on when 
 * carrying out an accepted task. Each ProcessingData object can be
 * provided a unique identifier, so that it can be easily referenced.
 * 
 * @version 100608
 * @since   061027
 * @author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 
@Deprecated
public class ProcessingData {

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	// The identifier of the data wrapper
	private String id; 
	
	// A vector with the data 
	private Vector<CASTData> data; 
	
	// A vector with the data types
	private Vector<String> types;

	private WorkingMemoryAddress wma = null;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public ProcessingData () {
		init ();
	} // end constructor


	public ProcessingData (String i) { 
		init();
		id = i;
	} // end constructor

	private void init () { 
		id = "";
		data = new Vector<CASTData>();
		types = new Vector<String>();
	} // 

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Adds a data item to the data */
	public void add (CASTData d) { 
		data.addElement(d);
		types.addElement(d.getType());
	} // end add

	/** Returns an Iterator<CASTTypedDataWithID> over the data */ 
	public Iterator<CASTData> getData () { 
		return data.iterator();
	} // end getData

	/** Returns the data item on the data list, with the given type; or null if no such item is present. */

	public CASTData getByType (String type) { 
		CASTData result = null;
		Iterator<CASTData> dIter = data.iterator();
		while (dIter.hasNext()) { 
			CASTData item = dIter.next();
			if (item.getType().equals(type)) { 
				result = item;
				break; 
			} // end if check for item type
		} // end while
		return result;
	} // end getByType

	/** Returns the ID of the object */ 
	public String getID() { return id; }

	/** Returns the vector with types of data objects stored */
	public Vector getTypes () { return types; }

	public WorkingMemoryAddress getWorkingMemoryAddress () { return wma; }

	/** Sets the ID of the object */
	public void setId (String i) { id = i; }

	public void setWorkingMemoryAddress (WorkingMemoryAddress wma) {
		this.wma = wma;
	}

}
	
