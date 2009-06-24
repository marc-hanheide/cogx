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
 * 
 *  @version 050208
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGChooser {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    String id = "";
    String region = "<none>";
    String metafunction = "";
    UPGDecTree dectree = new UPGDecTree (); 
    
    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UPGChooser () {
	id = "";
	region = "<none>";
	metafunction = "";
	dectree = new UPGDecTree (); 
    } // end constructor

    /** 
     *  The ternary constructor sets the id, region and metafunction to the given values, and initializes the remaining internal variables. 
     */

    public UPGChooser (String i, String r, String m) {
	id = i;
	region = r;
	metafunction = m;
	dectree = new UPGDecTree (); 
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================
    
    /**
     *  The method <i>copy</i> returns a copy of this object.
     */

    public UPGChooser copy () { 
	UPGChooser cp = new UPGChooser(id,region,metafunction);
	UPGDecTree dt = dectree.copy();
	cp.setDecisionTree(dt);
	return cp;
    } // end copy

    /**
     *  The method <i>getDecTree</i> returns the decision tree of the chooser. 
     */

    public UPGDecTree getDecTree () { return dectree; }

    /**
     *  The method <i>getId</i> returns the id of the chooser.
     */

    public String getId () { return id; }

    /**
     *  The method <i>getMetafunction</i> returns the metafunction of
     *  the chooser.
     */

    public String getMetafunction () { return metafunction; }


    /**
     *  The method <i>getRegion</i> returns the region of the chooser.
     */

    public String getRegion () { return region; }

    /**
     * The method <i>setDecisionTree</i> sets the decision tree for
     * this chooser to the given tree <tt>t</tt>.
     */

    public void setDecisionTree (UPGDecTree t) { 
	dectree = t;
    } // end setDecisionTree

    //=================================================================
    // I/O METHODS
    //=================================================================
    
    public String toString() { 
	String result = "CHOOSER:";
	result = result+" "+id+", "+region+", "+metafunction+"\n";
	result = result+"Decision tree:\n";
	result = result+dectree.toString();
	return result;
    } // end toString();

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
