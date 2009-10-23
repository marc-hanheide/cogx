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

import java.util.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPResult</b> is a data structure that is passed back
 *  from a <b>UPSystem</b> to the planner, containing a logical form
 *  and a locus which may have been updated as a result of running
 *  the system.
 * 
 *  @version 050211 (started 050211)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPResult {
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    UPLocus locus;
    LogicalForm lf; 

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     * The basic constructor initializes the internal variables. 
     */

    public UPResult () {
	locus = new UPLocus ();
	lf = null; 
    } // end constructor

    /**
     * The binary constructor takes a locus and a logical form, and
     * initializes the internal variables with these.
     */

    public UPResult (UPLocus l, LogicalForm logf) { 
	locus = l;
	lf = logf;
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>getLF</i> returns the logical form. 
     */

    public LogicalForm getLF () { return lf; }

    /**
     *  The method <i>getLocus</i> returns the locus. 
     */

    public UPLocus getLocus () { return locus; }

    /**
     * The method <i>setLF</i> sets the logical form to the provided object. 
     */

    public void setLF (LogicalForm logf) { lf = logf; }

    /**
     * The method <i>setLocus</i> sets the locus to the provided object.
     */

    public void setLocus (UPLocus l) { locus = l; }

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    public String toString () { 
	return "["+locus.toString()+"]" + lf.toString();
    } // end toString

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
