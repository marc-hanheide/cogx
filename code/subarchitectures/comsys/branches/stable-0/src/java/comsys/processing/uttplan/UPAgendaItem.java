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
 *  The class <b>UPAgendaItem</b> implements the datastructure for
 *  items that can appear on the agenda used in the utterance
 *  planner. An agenda item contains the name of a locus nominal, and
 *  the identifier of an applicable system.
 * 
 *  @version 050210 (started 050210)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPAgendaItem {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private String nomvar; 
    private String systemid; 

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables
     */

    public UPAgendaItem () {
	nomvar   = "";
	systemid = "";
    } // end constructor

    /**
     *  The binary constructor sets the nominal and the system id to
     *  the provided values.
     */

    public UPAgendaItem (String n, String sid) { 
	nomvar = n;
	systemid = sid; 
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>getNomvar</i> returns the identifier of the
     *  nominal stored on the agenda item.
     */

    public String getNomvar () { return nomvar; }

    /**
     *  The method <i>getSystemId</i> returns the identifier of the
     *  applicable system stored on the agenda item.
     */

    public String getSystemId () { return systemid; }

    /**
     *  The method <i>setNomvar</i> sets the nominal stored on the
     *  agenda item.
     */

    public void setNomvar (String n) { nomvar = n; }

    /**
     *  The method <i>setSystemId</i> sets the id of the applicable
     *  system stored on the agenda item.
     */ 

    public void setSystemId (String id) { systemid = id; } 

    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	return "AGENDA_ITEM: for locus ["+nomvar+"] apply ["+systemid+"]"; 
    } // end toString

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
