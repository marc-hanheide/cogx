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
 *  @version 
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGChoiceNode {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================
    
    Vector answers;
    String result = "";
    HashMap children; 
    UPGInquiry inquiry; 


    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     * The basic constructor initializes the internal variables
     */

    public UPGChoiceNode () {
		answers =  new Vector();
		result = "";
		children = new HashMap (); 
		inquiry = new UPGInquiry ();
    } // end constructor

    /** 
     * The unary constructor sets the answer on which the choicenode
     * gets fired, and initializes the remaining internal variables. 
     */

    public UPGChoiceNode (String a) {
		answers = parseAnswer(a);
		result = "";
		children = new HashMap (); 
		inquiry = new UPGInquiry ();
    } // end constructor


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>copy</i> returns a copy of this choice node
     */

    public UPGChoiceNode copy () { 
		UPGChoiceNode cp = new UPGChoiceNode ();
		cp.setAnswer(answers);
		cp.setInquiry(inquiry.copy());
		cp.setResult(result);
		for (Iterator cIter = children.keySet().iterator(); cIter.hasNext(); ) { 
			String key = (String) cIter.next();
			UPGChoiceNode n = (UPGChoiceNode) children.get(key); 
			cp.addChild(n.copy());
		} // end for over children
		return cp;
    } // end copy

    /** 
     * The method <i>addChild</i> adds a child node under the current
     * choice node. Children are stored in a HashMap, keyed by answer. 
     */

    public void addChild (UPGChoiceNode n) { 
		Vector a = n.getAnswer();
		for (Iterator<String> answersIter = a.iterator(); answersIter.hasNext(); ) { 
			String answer = answersIter.next();
			children.put(answer,n);
		} // end for
    } // end addChild 

    /**
     *  The method <i>getAnswer</i> returns the answer that triggers
     *  this choicenode.
     */

    public Vector getAnswer () { return answers; }

    /**
     *  The method <i>getChild</i> returns the child choice node that
     *  is triggered by the given answer. If there is no such child
     *  node (given the provided answer), an UPGException is thrown.
     */

    public UPGChoiceNode getChild (String ans) throws UPGException { 
		if (children.containsKey(ans.toLowerCase())) { 
			return (UPGChoiceNode) children.get(ans.toLowerCase());
		} else { 
			throw new UPGException ("In choicenode with inquiry "+inquiry.getCodeId()+" no child for answer "+ans+"/"+children.keySet().toString());
		} // end if..else check whether child for answer
    } // end getChild

	
	/**
	 The method <i>getChildren</i> returns an iterator over the values of the HashMap with children. 
	 The iterator runs of UPGChoiceNode objects. 
	*/ 
	
	public Iterator<UPGChoiceNode> getChildren () { 
		return children.values().iterator();
	} // 
	
    /**
     *  The method <i>getInquiry</i> returns the inquiry of this choicenode. 
     */

    public UPGInquiry getInquiry () { return inquiry; } 

    /**
     *  The method <i>getResult</i> returns the result that opting for
     *  this choicenode delivers.
     */

    public String getResult () { return result; }

    /**
     *  The method <i>isLeaf</i> returns a boolean indicating whether
     *  this choicenode is a leaf of the decision tree. 
     */

    public boolean isLeaf () { return (children.size() == 0); }

    /**
     *  The method <i>isRoot</i> returns a boolean indicating whether
     *  the answer triggering this choicenode equals <tt>*TOP*</tt> or
     *  not.
     */

    public boolean isRoot () { return answers.contains("*TOP*"); }

	
	public Vector parseAnswer (String a) { 
		Vector result = new Vector();
		StringTokenizer st = new StringTokenizer(a);
		while (st.hasMoreTokens()) { 
			String answer = st.nextToken();
			result.addElement(answer);
		} 	
		return result;
	} // end parse answer
	
	
    /**
     *  The method <i>setAnswer</i> sets the answer that triggers this
     *  choicenode.
     */

    public void setAnswer (String a) { answers = parseAnswer(a); }

	public void setAnswer (Vector av) { answers = av; }
	
	
    /**
     *  The method <i>setInquiry</i> sets the inquiry of the choice
     *  node to the provided object.
     */

    public void setInquiry (UPGInquiry inq) { inquiry = inq; }

    /**
     *  The method <i>setResult</i> sets the result of this choice node. 
     */

    public void setResult (String r) { result = r; }

    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString() { 
	String res = "Choicenode for trigger answer "+answers+":\n";
	if (this.isLeaf()) { 
	    res = res+"\tReturn result: "+result+"\n"; 
	} else { 
	    res = res+"\nInquiry: "+inquiry.toString();
	    for (Iterator chIter = children.values().iterator(); chIter.hasNext(); ) { 
		UPGChoiceNode child = (UPGChoiceNode) chIter.next();
		res = res+child.toString();
	    } // end for over children
	    res=res+"\n";
	} // end if..else
	return res; 
    } // end toString



    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
