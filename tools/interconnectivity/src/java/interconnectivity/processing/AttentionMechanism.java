//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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

package interconnectivity.processing;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// CAST
//-----------------------------------------------------------------
import cast.core.CASTData;

//=================================================================
// DOCUMENTATION
//=================================================================

/**
	The interface <b>AttentionMechanism</b> defines what methods 
	an attention mechanism should provide -- basically, a scoring
	over, and possibly pruning of, a CASTData structure which is
	then returned. The interface leaves open whether the attention 
	mechanism is a single mechanism, or a pipeline of such mechanisms. 
	
	@version 070814
	@since	 070814
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/


public interface AttentionMechanism {
	
	/**
		The method <i>score</i> scores, and possibly prunes, the 
		provided data structure. The result is returned. 
		
		@param  source	The source data structure to be scored, possibly pruned
		@param  scoring	The scoring to be used in pruning 
		@return Object  The resulting pruned data
		@since	070814
	*/ 
	
	public Object score (Object source, CASTData scoring) throws Exception; 
	
	
	/**
		The method <i>getSourceDataType</i> returns the source type 
		the mechanism operates on. 
	
		@return String The type of data operated on
		@since  070814
	*/ 

	public String getDataType (); 


} // end interface
