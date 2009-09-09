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

package interconnectivity.processing;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------
import cast.core.CASTData;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.Iterator;
import java.util.List;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
 * The interface <b>ContextActiveProcess</b> defines methods that 
 * any process should provide, which is assumed to be "active" in 
 * that its processing is guided by context information. 
 *
 * @version 070814 (Started: 070810)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
*/

public interface ContextActiveProcess { 

	/**
	*   The method <i>getContextDataTypes</i> returns an iterator over a list of
	*   CASTData types that contain context information which 
	*   can guide this process. 
	* 
	*   @return Iterator An iterator of CASTData types
	*/
	public Iterator getContextDataTypes();
	
	/** 
	*   The method <i>updateContextData</i> gives a feed into
	*   the process to provide update context data. 
	* 
	*   @param updatedInfo A CASTData object with (updated) context info. 
	* 
	*/ 
	
	public void updateContextData(CASTData updatedInfo);

	/**
		The method <i>registerAttentionMechanism</i> registers an attention mechanism 
		with the process, used to score (and possibly prune) input data before processing. 

		@param am The attention mechanism
	*/

	public void registerAttentionMechanism (AttentionMechanism am); 



} // end interface