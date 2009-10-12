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
 *  @version 050208
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGSystem {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================
    
    private String id = "";
    private String region = "";
    private String metafunction = "";
    private String chooserId = "";
    private Vector conditions = new Vector();
    private HashMap actions = new HashMap ();
	
	private boolean logging = false;
	
    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables;
     */

    public UPGSystem () { 
	id = "";
	region = "<none>";
	metafunction = "<none>";
	chooserId = "";
	conditions = new Vector();
	actions = new HashMap ();
    } // end constructor


    /** 
     * The ternary constructor initializes the system with the given
     * values for its id, region, and metafunction.
     */

    public UPGSystem (String i, String r, String m) { 
	id = i;
	region = r;
	metafunction = m;
	conditions = new Vector();
	actions = new HashMap();
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> creates a copy of the current system
     *  object.
     */

    public UPGSystem copy () { 
	UPGSystem cp = new UPGSystem (id,region,metafunction); 
	cp.setChooserId(chooserId);
	for (Iterator cIter = conditions.iterator(); cIter.hasNext(); ) {
	    UPGCondition c = (UPGCondition) cIter.next();
	    cp.addCondition(c);
	} // end for over conditions
	for (Iterator aIter = actions.keySet().iterator(); aIter.hasNext(); ) { 
	    String choice = (String) aIter.next();
	    UPGAction act = (UPGAction) actions.get(choice);
	    cp.addAction(act);
	} // end for over actions
	return cp;
    } // end copy

    //----------------------------------------------------------------
    // ADD METHODS (alphabetical order)
    //----------------------------------------------------------------    

    /**
     *  The method <i>addAction</i> adds an action to the HashMap with
     *  all the actions of the system, keyed by the choice result
     *  (i.e. the result of the chooser that triggers this action).
     */

    public void addAction (UPGAction act) { 
	String ans = act.getChoice(); 
	actions.put(ans.toLowerCase(),act);
	// System.out.println(actions.size()+"--adding action for choice "+ans+": "+act.toString());
    } // end addAction

    /**
     *  The method <i>addCondition</i> adds a condition to the list of
     *  conditions for the system. A condition consists of one or more
     *  features that all need to be satisfied by the locus, and thus
     *  models an AND-list. The conditions for the system is an
     *  OR-list, i.e. the system can be fired as soon as at least one
     *  condition is satisfied.
     */

    public void addCondition (UPGCondition c) { conditions.addElement(c); } 

    //----------------------------------------------------------------
    // GET METHODS (alphabetical order)
    //----------------------------------------------------------------    

    /**
     *  The method <i>getAction</i> returns the action in the system
     *  associated with the given answer. If no action is associated
     *  with the answer, a null object is returned. (No exception is
     *  thrown!)
     */

    public UPGAction getAction (String ans) { 
	// System.out.println("Trying to get action for answer ["+ans+"]");
	UPGAction result = (actions.containsKey(ans.toLowerCase()))?(UPGAction)actions.get(ans.toLowerCase()):null;
	return result;
    } // end getAction


    /**
     *  The method <i>getChooserId</i> returns the id of the chooser associated with
     *  the system.
     */

    public String getChooserId () { return chooserId; }


    /**
     *  The method <i>getId</i> returns the id of the system.
     */

    public String getId () { return id; }

    /**
     *  The method <i>getMetafunction</i> returns the metafunction of
     *  the system.
     */

    public String getMetafunction () { return metafunction; }


    /**
     *  The method <i>getRegion</i> returns the region of the system.
     */

    public String getRegion () { return region; }

	
	public Iterator getActions() { return actions.keySet().iterator(); } 
	
	
    //----------------------------------------------------------------
    // IS METHODS (alphabetical order)
    //----------------------------------------------------------------    

    /**
     *  The method <i>isApplicable</i> checks whether any of the
     *  conditions of the system are satisfied, given the current
     *  locus (given as parameter to the method). If there is at least
     *  one condition satisfied, the method returns true; otherwise,
     *  false.
     */

    public boolean isApplicable (UPLocus l) { 
	boolean result = false;
	int satconds = 0;
	for (Iterator cIter = conditions.iterator(); cIter.hasNext(); ) { 
	    UPGCondition cond = (UPGCondition) cIter.next();
	    if (cond.isSatisfied(l)) {satconds++; } 
	} // end for over conditions
	result = (satconds > 0); 
	log("System ["+id+"] applicable: ["+result+"]");
	return result;
    } // end isApplicable


    //----------------------------------------------------------------
    // SET METHODS (alphabetical order)
    //----------------------------------------------------------------    

    /**
     *  The method <i>setChooserId</i> sets the identifier of the
     *  chooser associated with this system.
     */

    public void setChooserId (String i) { chooserId = i; }

    /**
     *  The method <i>setId</i> sets the identifier of the system.
     */

    public void setId (String i) { id = i; }


    /**
     *  The method <i>setMetafunction</i> sets the metafunction of the system.
     */

    public void setMetafunction (String m) { metafunction = m; }

    /**
     *  The method <i>setRegion</i> sets the region of the system.
     */

    public void setRegion (String r) { region = r; }


    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    public String toString () { 
		String result = "SYSTEM:";
		result = result+" "+id+", "+region+", "+metafunction+"\n";
		result = result+"Chooser:\n";
		result = result+"\t"+chooserId+"\n";
		result = result+"Conditions:\n";
		for (Iterator cIter=conditions.iterator(); cIter.hasNext(); ) { 
			UPGCondition cond = (UPGCondition) cIter.next();
			result = result+"\t"+cond.toString()+"\n";
		} // end for over conditions
		result = result+"Actions: ("+actions.size()+")\n";
		for (Iterator aIter=actions.keySet().iterator(); aIter.hasNext(); ) { 
			String choice = (String) aIter.next();
			UPGAction act = (UPGAction) actions.get(choice);
			result = result+"\t"+act.toString()+"\n";
		} // end for over actions
		return result;
    } // end toString


    //=================================================================
    // MAIN METHOD
    //=================================================================

	
	
	public void log (String m) { 
		if (logging) { System.out.println(m); }
		
		
	} 

} // end class definition 
