//=================================================================
// Copyright (C) 2005 Geert-Jan M. Kruijff (gj@acm.org)
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

package comsys.processing.uttplan;

//=================================================================
// IMPORTS
//=================================================================

import comsys.datastructs.lf.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPGException</b> provides an exception
 *  wrapper class for the Utterance Planning Grammar processing. 
 * 
 *  @version 050208 (started 050208)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGException  extends Exception 
    {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	String msg = "";

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public UPGException (String m) {
	    msg = m;
	} 

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	public String getMessage () { 
	    return msg;
	} 


} // end class definition 
