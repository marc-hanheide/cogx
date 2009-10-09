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
// OPENCCG
//-----------------------------------------------------------------
import opennlp.ccg.synsem.SignHash;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================


/**
	The interface <b>BeamWidthFunction</b> specifies the method(s) 
	required for classes that implement beam width functions. 
	A beam width function is normally used together with a 
	<b>SignHashScorer</b> class to prune entire sign hashes. 

	@author Geert-Jan M. Kruijff (gj@dfki.de)
	@version 061106 (started 061106)
*/ 

public interface BeamWidthFunction { 

	/** Returns a restricted SignHash, by applying beam width constraints
		to the given SignHash with the specified preference ordering.  
	 */ 

	public SignHash restrict (SignHash hash, SignPreferenceOrder ordering); 

	
	
} // end interface
