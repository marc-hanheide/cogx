// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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
// =================================================================

package opennlp.ccg.parse;

// =================================================================
// IMPORTS
// =================================================================



// =================================================================
// DOCUMENTATION
// =================================================================

/**
	The interface <b>IncrFrontierFilter</b> specifies what methods
	a qualitative filter should implement, for determining whether 
	an intermediate set of analyses provided by an incremental parser
	provides an eligible frontier, qualitatively speaking. 
	
	@version 070814 
	@since   070814
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public interface IncrFrontierFilter {

		/** 
		The method <i>eligibleFrontierReached</i> checks for a provided
		parse results object whether it contains one or more items that would
		pass the specified filter -- making the current results an eligible frontier 
		in an incremental analysis. 
		
		@param pr The (intermediate) parse results to be checked
		@return boolean Indicating whether (part of) the parse results pass the filter
		*/ 

		public boolean eligibleFrontierReached (ParseResults pr);


		public void setLogging (boolean l); 
		
		public void log (String msg); 


} // end interface
