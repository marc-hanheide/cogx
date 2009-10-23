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
import comsys.lf.utils.*;

import java.util.*;



//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPLocus</b> implements a simple datastructure for the
 *  locus used in the utterance planner. The datastructure stores the
 *  nominal (an <tt>LFNominal</tt> object) that is the current locus,
 *  as well as HashMaps with (a) local variable names for nominals in
 *  the current logical form, and (b) variable names with a
 *  (lf-)global scope. These variables are set through an
 *  <tt>identify-nomvar</tt> step. This table is reset when the locus
 *  changes, or the planner moves to another system.
 * 
 *  @version 080623 (started 050210)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPLocus {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private LogicalForm lf; 
    private LFNominal nomvar; 
    private HashMap   varids;        // local variables
    private HashMap   globalvarids;  // global variables
    private Vector    addednoms;

    private UPReqHandler reqhandler; // external request handler

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables
     */

    public UPLocus () {
	nomvar = new LFNominal (); 
	varids = new HashMap ();
	globalvarids = new HashMap ();
	addednoms = new Vector();
	reqhandler = null;
    } // end constructor

    /** 
     *  The unary constructor sets the nominal of the locus to the
     *  given value, and initializes the remaining internal variables
     */

    public UPLocus (LFNominal nom) {
	nomvar = nom; 
	varids = new HashMap ();
	addednoms = new Vector();
	globalvarids = new HashMap ();
	reqhandler = null;
    } // end constructor

    /** 
     *  The binary constructor sets the nominal of the locus to the
     *  given value, and the request handler to the second given
     *  value, then initializes the remaining internal variables
     */

    public UPLocus (LFNominal nom, UPReqHandler r) {
	nomvar = nom; 
	varids = new HashMap ();
	addednoms = new Vector();
	globalvarids = new HashMap ();
	reqhandler = r;
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>addInsertedNominal</i> adds a nominal to the list of
     *  nominals that have been inserted into the LF in the current
     *  system.
     */

    public void addInsertedNominal (LFNominal nom) { 
	addednoms.addElement(nom);
    } 

    /**
     *  The method <i>addVariableId</i> adds a local variable and the
     *  name of the nominal in the logical form it identifies.
     */

    public void addVariableId(String v, String nv) { varids.put(v,nv); }

    /**
     *  The method <i>addVariableId</i> adds a variable and the name
     *  of the nominal in the logical form it identifies, with the
     *  given scope ("local" or "global"; defaults to local). 
     */

    public void addVariableId(String v, String nv, String scope) { 
	if (scope.equals("global")) { 
	    globalvarids.put(v,nv); 
	} else {
	    varids.put(v,nv); 
	} // end if..else check scope of variable
    } 

    /**
     *  The method <i>getGlobalVariables</i> returns the HashMap with
     *  the global variables. The key is the variable name, the value
     *  the nominal variable.
     */

    public HashMap getGlobalVariables () { 
	return globalvarids; 
    } 
    
    /**
     *  The method <i>getInsertedNominals</i> returns an Iterator over
     *  nominals (<tt>LFNominal</tt> objects) that have been inserted
     *  into the logical form in the current system.
     */

    public Iterator getInsertedNominals () { return addednoms.iterator(); } 

    /**
     *  The method <i>getLF</i> returns a pointer to the global
     *  logical form being planned.
     */

    public LogicalForm getLF() { return lf; }

    /**
     *  The method <i>getNominal</i> returns the nominal that
     *  constitutes the current locus.
     */

    public LFNominal getNominal () { return nomvar; } 

    /**
     *  The method <i>getNomvar</i> returns the identifier of the
     *  nominal that constitutes the current locus.
     */

    public String getNomvar () { return nomvar.nomVar; }

    /**
     *  The method <i>getVariableId</i> returns the name of the
     *  nominal in the logical form that the given variable
     *  identifies. If the variable is unknown, "unknown" is
     *  returned. We first check for global variables, then for local
     *  variables. <p>
     *
     *  <h4>Warning</h4> If we have the same name in both the local
     *  and the global namespace, then the <i>local</i> variable is
     *  taken. 
     */

    public String getVariableId (String v) { 
	String result = "unknown"; 
	if (globalvarids.containsKey(v)) { result=(String)globalvarids.get(v); }
	else if (varids.containsKey(v)) { result=(String)varids.get(v); }
	return result;
    } // end getVariableId 

    /**
     *  The method <i>hasInsertedNominals</i> returns a boolean
     *  indicating whether the locus stores any nominals that have
     *  been inserted in the current system.
     */

    public boolean hasInsertedNominals () { return (!addednoms.isEmpty()); } 

    /**
     *  The method <i>holdsCtxtCond</i> implements a link to a request
     *  handler, which is (presumably) able to resolve whether the
     *  given condition holds in the context. The method returns
     *  <tt>false</tt> by default if no request handler has been
     *  registered.
     */ 

    public boolean holdsCtxtCond (String val) { 
	String[] args = new String[1];
	args[0] = val;
	if (reqhandler != null) { 
	    String result = reqhandler.solveRequest("resolveCtxtCond",args); 
	    if (result.equals("true")) { 
		return true; 
	    } else { 
		return false;
	    } // end if..else check what is returned
	} else {
	    return false;
	} // end if..else
    } // end holdsCtxtCond

    /**
     *  The method <i>resetInsertedNominals</i> resets the vector with
     *  references to nominals that have been added in the current
     *  system.
     */

    public void resetInsertedNominals () { addednoms = new Vector(); }

    /**
     *  The method <i>resetVariableIds</i> resets the hashmap with variable ids. 
     */

    public void resetVariableIds () { varids = new HashMap(); }

    /**
     *  The method <i>setGlobalVariables</i> sets the HashMap in the
     *  locus containing information about global variables to the
     *  provided HashMap. 
     */

    public void setGlobalVariables (HashMap gvi) { globalvarids = gvi; }

    /**
     *  The method <i>setLF</i> adds a pointer to the LF being
     *  planned, to enable access to the global structure. 
     */ 

    public void setLF (LogicalForm logf) { lf = logf; } 

    /**
     *  The method <i>setNomvar</i> sets the nominal of the locus.
     */ 

    public void setNomvar (LFNominal n) { nomvar = n; }

    /**
     *  The method <i>setReqHandler</i> sets the request handler for
     *  the locus.
     */ 

    public void setReqHandler (UPReqHandler r) { reqhandler = r; } 

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    public String toString () { 
	String result = "Locus: "+LFUtils.lfNominalToString(nomvar)+"\n\t";
	for (Iterator vIter = globalvarids.keySet().iterator(); vIter.hasNext(); ) { 
	    String var = (String) vIter.next(); 
	    String nv  = (String) globalvarids.get(var);
	    result = result + "("+var+"="+nv+")";
	    if (vIter.hasNext()) { result = result+", "; } 
	} 
	result = result + "\n";
	return result;
    } // end toString


} // end class definition 
