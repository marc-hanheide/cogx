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
 * The class <b>UPGInquiry</b> represents the information about an
 * inquiry used in a chooser:
 * <ul>

 * <li> <tt>id</tt>, the identifier of the inquiry </li>

 * <li> <tt>answerset</tt>, the possible answers that this inquiry can
 * return, on the basis of the code that implements its query.</li> 

 * <li> <tt>type</tt>, indicating the data type of the answers

 * <li> <tt>codeid</tt>, the identifier of the code that implements
 * the inquiry. There are several standard functions: 
 * 
 *      <ul> 

 *      <li> <i>q-rel-eq</i>, which represents an inquiry of the
 *      equality between a given term (the value of the "val"
 *      attribute) and an aspect of the current nominal. This aspect
 *      is given by the attribute "focus", and can have the following
 *      values: <tt>type</tt> (type of the nominal), <tt>prop</tt>
 *      (proposition holding at the nominal), <tt>feat</tt> (feature
 *      of the nominal; checked is against the value for that
 *      feature).</li>

 *      <li> <i>q-rel-num</i> represents an inquiry into whether a
 *      given numerical relation (the value of "rel"; possible values
 *      are "&lt;" and "&gt;") holds between a given term (the value
 *      of the "val" attribute) and an feature of the current
 *      nominal (name given as value of the "feat" attribute).</li>

 *      <li> <i>q-ex-feat</i> inquires after the presence of a given
 *      feature of the current nominal (name given as value of the
 *      "feat" attribute).</li>

 *      <li> <i>q-ex-fv</i> inquires after the presence of a given
 *      value (given by the "val" attribute) for a given feature of
 *      the current nominal (name given as value of the "feat"
 *      attribute).</li>

 *      <li> <i>q-ex-prop</i> inquires after the presence of a given
 *      proposition (given by the "val" attribute) for the current
 *      nominal.</li>
 * 
 *      <li> <i>q-ex-rel</i> inquires after the presence of a given
 *      relation (given by the "val" attribute) for the current 
 *      nominal.</li>

 *      <li> <i>q-mod-salience</i> inquires after the salience of the
 *      locus (i.e. the current nominal) in the given modality (given
 *      by the "mod" attribute). This inquiry returns an integer
 *      value.</li>

 *      <li> <i>q-mod-maxevid-str</i> inquires after the maximal
 *      evidencing modality for the locus (i.e. the current
 *      nominal). This is the "strongest" modality in which the
 *      nominal can be anchored (in Clark's sense), using the
 *      following ordering from strongest to weakest: vision -
 *      discourse - belief. This inquiry returns a String value;
 *      there is also a boolean variant, see below. </li>

 *      <li> <i>q-mod-maxevid-bool</i> checks whether the given modality
	(value of the "mod" attribute) is the maximal evidencing
	modality for the locus (i.e. the current nominal). This is
	the "strongest" modality in which the nominal can be anchored
	(in Clark's sense), using the following ordering from
	strongest to weakest: vision &gt; discourse &gt; belief. This
	inquiry returns a boolean value. </li>

 *      </ul> </li>

 * <li> <tt>nomfocus</tt>, which indicates the nominal on which this
 * inquiry focuses, by giving the <i>dependency relation</i> from the
 * current locus to that nominal. By default, the nomfocus is not
 * given explicitly, and the inquiry focuses on the current locus. If
 * the nomfocus is given, then the inquiry focuses on the nominal
 * reachable over the given relation. </li>
 * 
 * </ul>
 * 
 *  @version 050210 (started 050209)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGInquiry {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    String id; 
    String answerset;
    String type;
    String nomfocus;   // the nominal on which the inquiry focuses

    String codeid; 
    HashMap parameters; 

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     * The basic constructor initializes the internal variables. 
     */

    public UPGInquiry () {
	id = "";
	answerset = "";
	type = "";
	codeid = "";
	parameters = new HashMap (); 
    } // end constructor

    /**
     * The ternary constructor sets the id, type and answerset of the
     * inquiry to the given values, and initializes the remaining
     * internal variables.
     */

    public UPGInquiry (String i, String t, String as) { 
	id = i;
	type = t;
	answerset = as;
	codeid = "";
	parameters = new HashMap (); 
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> returns a copy of this object. 
     */

    public UPGInquiry copy () { 
	UPGInquiry cp = new UPGInquiry(id,type,answerset);
	cp.setCodeId(codeid);
	for (Iterator pIter = parameters.keySet().iterator(); pIter.hasNext(); ) { 
	    String p = (String) pIter.next();
	    String v = (String) parameters.get(p);
	    cp.setParameterValue(p,v);
	} // end for over parameters
	// System.out.println("Copy: "+cp.toString());
	return cp;
    } // end copy

    /**
     * The method <i>getAnswerset</i> returns the answerset of the inquiry.
     */

    public String getAnswerset () { return answerset; }


    /**
     * The method <i>getCodeId</i> returns the code id of the inquiry.
     */

    public String getCodeId () { return codeid; }

    /**
     * The method <i>getParameterValue</i> returns the value for the
     * given parameter. If the parameter is not known, "unknown" is
     * returned.
     */

    public String getParameterValue (String f) { 
	String result = (parameters.containsKey(f))?(String)parameters.get(f):"unknown";
	return result;
    } 

    /**
     * The method <i>getId</i> returns the id of the inquiry.
     */

    public String getId () { return id; }

    /**
     * The method <i>getType</i> returns the type of the inquiry.
     */

    public String getType () { return type; }


    /**
     * The method <i>setAnswerset</i> sets the answerset of the
     * inquiry to the given value.
     */

    public void setAnswerset (String as) { answerset = as; } 


    /**
     * The method <i>setParameterValue</i> sets a parameter and a value
     * used as parameters for the inquiry code. 
     */

    public void setParameterValue (String f, String v) { parameters.put(f,v); }

    /**
     * The method <i>setCodeId</i> sets the code id of the inquiry to the given
     * value.
     */

    public void setCodeId (String i) { 
	codeid = i; 
	// System.out.println("[UPGInquiry] Set code-id to "+codeid);
    } 

    /**
     * The method <i>setId</i> sets the id of the inquiry to the given
     * value.
     */

    public void setId (String i) { id = i; } 


    /**
     * The method <i>setType</i> sets the type of the inquiry to the
     * given value.
     */

    public void setType (String t) { type = t; } 



    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	String result = "INQUIRY: "+id+", "+type+", using "+codeid+" for answer in {"+answerset+"}\n";
	return result;
    } 


    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
