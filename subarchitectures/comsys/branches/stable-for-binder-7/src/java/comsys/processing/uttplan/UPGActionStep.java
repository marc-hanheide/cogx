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
 * 
 *  @version 050209 (started 050209)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGActionStep {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private String id = "";
    private HashMap attrs = new HashMap ();

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UPGActionStep () {
	id = "";
	attrs = new HashMap ();
    } // end constructor

    /**
     *  The unary constructor sets the id of the step to the given
     *  value, and initializes remaining internal variables.
     */

    public UPGActionStep (String i) { 
	id = i;
	attrs = new HashMap();
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> returns a copy of this object. 
     */

    public UPGActionStep copy () { 
	UPGActionStep cp = new UPGActionStep(id);
	for (Iterator aIter = attrs.keySet().iterator(); aIter.hasNext(); ) { 
	    String attr = (String) aIter.next();
	    String val  = (String) attrs.get(attr);
	    cp.setAttribute(attr,val);
	} // end for over attributes
	return cp;
    } // end copy

    /**
     *  The method <i>getAttributeValue</i> returns the value for the
     *  given attribute. If there is no such attribute, the "unknown"
     *  string is returned.
     */

    public String getAttributeValue (String a) { 
	String result = (attrs.containsKey(a))?(String)attrs.get(a):"unknown";
	return result;
    } // end getAttributeValue

    /**
     *  The method <i>getId</i> returns the id of the step. 
     */

    public String getId () { return id; }  

    /**
     *  The method <i>setAttribute</i> sets the given attribute to the
     *  given value.
     */

    public void setAttribute (String a, String v) { attrs.put(a,v); }

    /**
     *  The method <i>setId</i> sets the id of the step to the given
     *  value.
     */

    public void setId (String i) { id = i; }  

    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	String result = id+"(";
	for (Iterator aIter = attrs.keySet().iterator(); aIter.hasNext(); ) { 
	    String at = (String) aIter.next();
	    String v  = (String) attrs.get(at);
	    result = result+at+":"+v;
	    result += (aIter.hasNext())?", ":")";
	} // end for over attributes
	return result;
    } // end toString

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
