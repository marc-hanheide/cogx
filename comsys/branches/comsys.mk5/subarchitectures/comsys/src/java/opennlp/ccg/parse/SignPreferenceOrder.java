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

package opennlp.ccg.parse;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// JAVA
//-----------------------------------------------------------------
import java.util.Iterator;
import java.util.Vector;

import opennlp.ccg.synsem.Sign;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================


/**
	The class <b>SignPreferenceOrder</b> stores an ordered set of Signs. 
	
	@author Geert-Jan M. Kruijff (gj@dfki.de)
	@version 061106 (started 061106)
*/ 

public class SignPreferenceOrder { 

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	/** Vector with Sign objects */ 
	protected Vector order; 
	
	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public SignPreferenceOrder () { 
		init();
	} // end constr

	protected void init () { 
		order = new Vector<SignPreference>();
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Adds the given Sign to end of the ordering */ 
	public void add (Sign sign) { order.addElement(new SignPreference(sign,1.0d)); }
	
	/** Adds the given Sign with the given preference score (double). The 
		Sign is added to the ordering after the first element with a preference
		score that is not great or equal to the given preference score. 
	*/
	  
	public void add (Sign sign, double score) { 
		Iterator<SignPreference> orderIter = order.iterator();
		int pos = 0;
		while (orderIter.hasNext()) { 
			SignPreference compare = orderIter.next();
			if (compare.pref >= score) { 
				pos++;
			} else { 
				break;
			}
		} // end while over elements
		order.insertElementAt(new SignPreference(sign,score),pos); 
	} // end add
	
	/** Returns an Iterator<Sign> over the preference ordering */
	public Iterator<Sign> getOrder() { 
		Vector result = new Vector();
		Iterator<SignPreference> orderIter = order.iterator();
		while (orderIter.hasNext()) { 
			SignPreference pref = orderIter.next();
			result.addElement(pref.sign); 
		} // end while
		return result.iterator();
	} // end getOrder

	
	/** Returns an Iterator<SignPreference> over the preference ordering */
	
	public Iterator<SignPreference> getPreferences() { 
		return order.iterator();

	} // end getPreferences

	/** Resets the preference ordering (creates empty collection) */ 
	public void reset() { order = new Vector<SignPreference>(); }

	/** returns the size of the vector with signs */
	public int size() { return order.size(); }

} // end class
