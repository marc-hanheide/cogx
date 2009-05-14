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

package org.cognitivesystems.comsys.util.uttplan;

//=================================================================
// IMPORTS
//=================================================================

// ORG.COGNITIVESYSTEMS packages

import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;


//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The interface <b>UPReqHandler </b> implements an interface for
    classes that implement handling of the requests posed by the
    inquiries of an utterance planner.
 * 
 *  @version 050608 (started 050405)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public interface UPReqHandler {
    
      
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     * The method <i>solveRequest</i> passes an inquiry-id (plus
       arguments) on to the plug-in module that implements the code
       for this inquiry. The method returns the answer of the inquiry,
       given the arguments, as a <tt>String</tt>. 
     * 
     */

    public String solveRequest (String req, String[] args); 

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

    //=================================================================
    // I/O METHODS
    //=================================================================

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================


    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
