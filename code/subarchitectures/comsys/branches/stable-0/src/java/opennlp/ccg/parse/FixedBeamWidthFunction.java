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
import opennlp.ccg.synsem.SignHash;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================


/**
	The class <b>FixedBeamWidthFunction</b> implements a function 
	for using a fixed beam width (<i>n</i>-best). 

	@author Geert-Jan M. Kruijff (gj@dfki.de)
	@version 061106 (started 061106)
*/ 

public class FixedBeamWidthFunction 
	implements BeamWidthFunction
{ 

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	/** The size of the beam width */ 
	protected int beamWidthSize; 

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public FixedBeamWidthFunction () { 
		init(); 
	} // end constr

	public FixedBeamWidthFunction (int size) { 
		init();
		beamWidthSize = size; 
	} // end constr

	private void init() { 
		beamWidthSize = 10;
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Sets the size of the beam width (in number of analyses to be kept). */ 

	public void setBeamWidth (int s) { beamWidthSize = s; }

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================


	/** Returns a restricted SignHash, by applying beam width constraints
		to the given SignHash with the specified preference ordering. The
		function does not restrict the hash if its size is already smaller
		than the beamwidth. 
	 */ 

	public SignHash restrict (SignHash hash, SignPreferenceOrder ordering) {  
		Vector restriction = new Vector();
		// first check the size of the hash, to see whether to do anything at all
		if (ordering.size() > beamWidthSize) { 
			int i=0; 
			Iterator signsIter = ordering.getOrder();
			while (signsIter.hasNext() && i < beamWidthSize) { 
				Sign sign = (Sign) signsIter.next();
				restriction.addElement(sign);
				i++;
			} // end while
			return new SignHash(restriction);
		} else { 
			Iterator signsIter = ordering.getOrder();
			while (signsIter.hasNext()) { 
				Sign sign = (Sign) signsIter.next();
				restriction.addElement(sign);
			} // end while 
			return new SignHash(restriction);
		} // end if..else check on hash size
	} // end restrict
	
	
	
} // end class
