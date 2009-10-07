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
 *  The class <b>UPGDecTree</b> is a data structure for storing the
 *  decision tree of a chooser. The main component of the decision
 *  tree is the root <b>ChoiceNode</b> object. 
 * 
 *  @version 050211 (started 050209)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGDecTree {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private UPGChoiceNode root;


    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UPGDecTree () {
		root = new UPGChoiceNode ();
    } // end constructor


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> returns a copy of this object.
     */

    public UPGDecTree copy () { 
		UPGDecTree cp = new UPGDecTree (); 
		cp.setRoot(root.copy());
		return cp;
    } // end copy

    /**
     * The method <i>getRoot</i> gets the root of the decision tree.
     */

    public UPGChoiceNode getRoot () { return root; }

	/**
	 The method <i>getAnswers</i> recursively descends down the tree and gathers all the answers, 
	 returning them as a Vector. 
	*/ 

	public Vector getAnswerSet (UPGChoiceNode node) { 
		Vector answerSet = new Vector();
		if (node.isLeaf()) { 
			answerSet.add(node.getAnswer()); 
		} else { 
			for (Iterator<UPGChoiceNode> childrenIter = node.getChildren(); childrenIter.hasNext(); ) { 
				UPGChoiceNode child = childrenIter.next();
				answerSet.addAll(this.getAnswerSet(child));
			} // end for
		} // 
		return answerSet; 
	} // and getAnswerSet 

	/**
     *  The method <i>setRoot</i> sets the root of the decision tree
     *  to the given node.
     */
	
    public void setRoot (UPGChoiceNode n) { 
	root = n;
    } // end setRoot

    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	return root.toString();
    } 

    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
