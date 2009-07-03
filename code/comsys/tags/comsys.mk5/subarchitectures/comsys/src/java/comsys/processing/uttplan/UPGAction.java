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

public class UPGAction {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private String choice = "";
    private Vector steps = new Vector ();

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables
     */

    public UPGAction () {
	choice = "";
	steps = new Vector ();
    } // end constructor


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> returns a copy of this object.
     */

    public UPGAction copy () { 
	UPGAction cp = new UPGAction (); 
	cp.setChoice(choice);
	for (Iterator sIter = steps.iterator(); sIter.hasNext(); ) { 
	    UPGActionStep s = (UPGActionStep) sIter.next();
	    cp.addStep(s.copy());
	} // end for over steps
	// System.out.println("Original:\n"+this.toString());
	// System.out.println("Copying returns:\n"+cp.toString());
	return cp;
    } // end copy

     /**
     *  The method <i>addStep</i> adds an action step to the end of
     *  the list of steps taken under this action.
     */

    public void addStep (UPGActionStep s) { 
	steps.addElement(s); 
	// System.out.println("[UPGAction] for choice "+choice+" adding step "+s.toString());
    }

    /**
     *  The method <i>getChoice</i> returns the choice for this
     *  action, i.e. the return result of the chooser required to fire
     *  this action.
     */

    public String getChoice () { return choice; }


    /**
     *  The method <i>getSteps</i> returns the Vector of action steps
     *  (of class <tt>UPGActionStep</tt>) that constitute this action.
     */

    public Vector getSteps () { return steps; }

    /**
     *  The method <i>setChoice</i> sets the choice for this action,
     *  i.e. the return result of the chooser required to fire this
     *  action.
     */

    public void setChoice (String c) { 
	choice = c; 
    }



    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	String result = "";
	result = result+choice+":";
	for (Iterator sIter = steps.iterator(); sIter.hasNext(); ) { 
	    UPGActionStep st = (UPGActionStep) sIter.next();
	    result = result+" "+st.toString();
	} // end for over steps
	return result;
    } 

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
