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

import java.util.Iterator;
import java.util.Vector;

//-----------------------------------------------------------------
// OPENCCG
//-----------------------------------------------------------------
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================


/**
	The class <b>UnrestrictedBeamWidthFunction</b> implements a 
	beam width function that imposes no restrictions. 

	@author Geert-Jan M. Kruijff (gj@dfki.de)
	@version 061106 (started 061106)
*/ 

public class UnrestrictedBeamWidthFunction 
	implements BeamWidthFunction
{ 

	/** Returns the provided SignHash, applying no constraints
		to the given SignHash.  
	 */ 

	public SignHash restrict (SignHash hash, SignPreferenceOrder ordering) {
		Vector restriction = new Vector();
		Iterator signsIter = ordering.getOrder();
		while (signsIter.hasNext()) { 
			Sign sign = (Sign) signsIter.next();
			restriction.addElement(sign);
		} // end while 
		return new SignHash(restriction);
	} 
	
} // end class
