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
 *  The class <b>UPGCondition</b> models a firing condition for a
 *  system. This is represented as a list of features that the current
 *  locus needs to satisfy.
 * 
 *  @version 050209 (started 050209)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGCondition {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private HashMap features = new HashMap();
    private HashMap featvals = new HashMap();

	private boolean logging = false;
	
    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UPGCondition () {
	features = new HashMap();
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>addFeature</i> adds a feature to the
     *  condition. A feature is of the form
     *  <tt>type_of_feature:value</tt> whereby
     *  <tt>type_of_feature</tt> can be <tt>type</tt> (ontological
     *  sort of the nominal), <tt>prop</tt> (proposition at the
     *  nominal), <tt>feat</tt> (feature at the nominal), or
     *  <tt>rel</tt> (relation at the nominal). If the method does not
     *  detect a colon, then automatically <tt>type</tt> is assumed.
     */

    public void addFeature (String f) { 
		int start = 0;
		String type = "type";
		if (f.indexOf("@") > -1) { start = f.indexOf("@")+1;} 
		f = f.substring(start,f.length());
		if (f.indexOf(":") != -1) { 
			type = f.substring(0,f.indexOf(":"));
			f = f.substring(f.indexOf(":")+1,f.length());
			if (f.indexOf(":") > 0) { 
				String v = f.substring(f.indexOf(":")+1,f.length());
				f = f.substring(0,f.indexOf(":")); 
				featvals.put(f,v);
				// log(">>>> Condition: adding feature "+f+" of type "+type+" with value "+v);
			} else {
				// log(">>>> Condition: adding feature "+f+" of type "+type);
			}
		} // end if check for explicit type mention
		// log("Condition: adding feature "+f+" of type "+type);
		features.put(type+"+"+f,f);
		log("Conditions set of features: "+featvals.keySet());
		
    } // end addFeature


    /**
     *  The method <i>isSatisfied</i> checks whether the
     *  condition is satisfied, given the current locus.
     */

    public boolean isSatisfied (UPLocus l) { 
		boolean result = false;
		LFNominal locus = l.getNominal ();
		String nomvar = locus.nomVar;
			Iterator fIter = features.keySet().iterator();
		while (fIter.hasNext()) { // was &&!result -- but this precludes cycling over all 
			// cycle over all features in the input cond.
			String typeFeat = (String) fIter.next();
			String feat = typeFeat.substring(0,typeFeat.indexOf("+"));
			String val  = (String) features.get(typeFeat);
			log("CHECKING ON ["+nomvar+"] FOR CONDITION ON FEATURE ["+feat+"] WITH VALUE ["+val+"]");


			if (feat.equals("feat")) {							// CHECKING for feat
				log("CHECKING FOR CONDITION ON FEATURE ["+feat+"] WITH VALUE ["+val+"]");
				String lv = "";
				for (int i=0; i < locus.feats.length ; i++) {   // checking whether locus has the feature
					log("Checking ["+val+"] against retrieved locus value ["+locus.feats[i].feat+"]");
					if (locus.feats[i].feat.equals(val)) {   // here is a problem, something doesn't match
						result = true;
						lv = (String) locus.feats[i].value;
					}
					// log("CHECKING FOR CONDITION ON FEATURE WITH VALUE "+val);
				}
				log("Current result flag in-feat: ["+result+"]");
				
				if (result) {                                 //locus has the feature, checking whether value matches
					if (featvals.containsKey(val)) {
						String v = (String) featvals.get(val);
						// log("Check for equality on conditioned ["+v+"] against ["+lv+"]");
							if (!lv.startsWith(v)) { 
								result = false;  break;            // too bad, value doesn't match
								// log("[UPL] !! Substring inequality on conditioned "+v+" against "+lv);
							} 
							else { 
								result = true;               // ok, value matches
							} 
					}
				}  // end if.. check for result
				else {
					result = false;  break;                   // too bad, locus does not have the feature
				}
			} // end check for feature

			
			if (feat.equals("prop")) {						// CHECKING for prop
				log("Checking condition ["+val+"] against proposition ["+locus.prop.prop+"]");
				if (locus.prop.prop.equals(val)) { 
					result = true;							// proposition matches
				} else { 
					result = false; break;					// proposition doesn't match (or isn't present?)
				}
			} // end check for proposition
			
			
			if (feat.equals("rel")) {						// CHECKING for rel 
				// log("The condition is a relation");
				for (int i=0; i < locus.rels.length ; i++) {
					// log("Checking ["+locus.rels[i].mode+"] against ["+val+"]");
					if (locus.rels[i].mode.equals(val)) {
						// log("Check succeeded");
						result = true;						// ok, the rel is present
					}
				}
				if (!result) {
					break;									// too bad, the rel is not present
				}
			} // end check for relation
			
			
			if (feat.equals("type")) {						// CHECKING for type
				// log("Checking condition ["+val+"] against type ["+locus.sort+"]");
				if (locus.sort.equals(val)) { result = true; } // ok, type matches
				else {break;}								   // too bad, type doesn't match
			} // end check for type
			
			
			if (feat.equals("ctxt")) { 
				if (l.holdsCtxtCond(val)) { result = true; }	// ok, context matches (whatever that means?)
				else {break;}								    // too bad, context doesn't match
			} // end check for context condition
			
			if (!result) { break; }							// no condition matches 
			
		} // end for over condition feature 
		//log("Locus: "+locus.toString());
		//log("Cond : "+this.toString());
		//log("Result: "+result);
		return result;
    } //end conditionSatisfied

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    public String toString () { 
	String result = "";
	for (Iterator fIter = features.keySet().iterator(); fIter.hasNext(); ) { 
	    String f = (String) fIter.next();
	    String v = (String) features.get(f); 
	    result = result+"("+f+":"+v+") ";
	} // end for
 	return result;
    } // end toString

    //=================================================================
    // MAIN METHOD
    //=================================================================

	public void log (String m) { 
		if (logging) { System.out.println(m); }
			
		
	} 	
	

} // end class definition 
