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

import java.io.*;
import java.util.*;

// SAX parser

import org.xml.sax.*;
import org.xml.sax.helpers.*;
import javax.xml.parsers.*;



//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPGChoosersReader</b> constructs the choosers for the
 *  systemic network, as specified in the choosers XML file for the
 *  grammar.
 * 
 *  @version 050609 (started 050209)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGChoosersReader 
    extends DefaultHandler
{
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    String filename = "";
    HashMap choosers = new HashMap ();
    Stack   stack = new Stack ();


    UPGChooser currentChooser; 
    UPGDecTree currentDecTree;
    UPGChoiceNode currentNode; 
    UPGInquiry currentInq; 

    boolean logging = false; 

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     * The basic constructor initializes the internal variables
     */

    public UPGChoosersReader () {
	filename = "./choosers.xml";
	choosers = new HashMap ();
	stack    = new Stack ();
	currentChooser = new UPGChooser ();
	currentDecTree = new UPGDecTree ();
	currentNode = new UPGChoiceNode ();
	currentInq = new UPGInquiry();
    } // end constructor

    
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /** 
     *  The method <i>getChoosers</i> returns the HashMap with the
     *  choosers that have been read from the file. 
     */

    public HashMap getChoosers () { return choosers; }

    //=================================================================
    // I/O METHODS
    //=================================================================

    public void log (String m) { if (logging) { System.out.println("[UPGChoosersRdr] "+m); } }

    //=================================================================
    // XML I/O METHODS
    //=================================================================

    public void read (String fn) {
        try {
	    log("reading ["+fn+"]");
	    filename = fn;
	    File file = new File (fn); 
            DefaultHandler handler = this;
            SAXParserFactory factory = SAXParserFactory.newInstance();
            SAXParser saxParser = factory.newSAXParser();
            saxParser.parse(file,handler);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        } // end try..catch
    } // end read    

    //=================================================================
    // GENERAL XML PROCESSING METHODS
    //=================================================================

    public void endElement(String namespaceURI,
			     String sName, // simple name
			     String qName // qualified name
			   )
    {
	String eName = sName; 
	if ("".equals(eName)) {eName = qName;} // in case not namespace aware
	if (eName.equals("chooser")) { processEndChooser (eName); } 
	if (eName.equals("dectree")) { processEndDecTree (eName); } 
	if (eName.equals("choicenode")) { processEndChoiceNode (eName); } 
	if (eName.equals("inquiry")) { processEndInquiry (eName); } 
	if (eName.equals("result")) { processEndResult (eName); } 
	if (eName.equals("q-deprel-type")) { processEndFModMaxevid (eName); } 
	if (eName.equals("q-rel-eq")) { processEndQRelEq (eName); } 
	if (eName.equals("q-rel-num")) { processEndQRelNum (eName); } 
	if (eName.equals("q-ex-feat")) { processEndQExFeat (eName); } 
	if (eName.equals("q-ex-fv")) { processEndQExFV (eName); } 
	if (eName.equals("q-ex-prop")) { processEndQExProp (eName); } 
	if (eName.equals("q-ex-rel")) { processEndQExRel (eName); } 
	if (eName.equals("f-relation")) { processEndFRelation (eName); } 
	if (eName.equals("f-featval")) { processEndFFeatVal (eName); } 
	if (eName.equals("f-mod-salience")) { processEndFModSalience (eName); } 
		if (eName.equals("f-mod-maxevid")) { processEndFModMaxevid (eName); } 
		if (eName.equals("f-rel-type")) { processEndFModMaxevid (eName); } 
	if (eName.equals("q-mod-maxevid-bool")) { processEndQModMaxevidBool (eName); } 
    } // end endElement

    public void startElement(String namespaceURI,
			     String sName, // simple name
			     String qName, // qualified name
			     Attributes attrs) 
    {
	String eName = sName; 
	if ("".equals(eName)) {eName = qName;} // in case not namespace aware
	if (eName.equals("chooser")) { processStartChooser (eName,attrs); }
	if (eName.equals("dectree")) { processStartDecTree (eName,attrs); }
	if (eName.equals("choicenode")) { processStartChoiceNode (eName,attrs); }
	if (eName.equals("inquiry")) { processStartInquiry (eName,attrs); }
	if (eName.equals("result")) { processStartResult (eName,attrs); }
	if (eName.equals("q-deprel-type")) { processStartQDepRelType (eName,attrs); }			
		
	if (eName.equals("q-rel-eq")) { processStartQRelEq (eName,attrs); }
	if (eName.equals("q-rel-num")) { processStartQRelNum (eName,attrs); }
	if (eName.equals("q-ex-feat")) { processStartQExFeat (eName,attrs); }
	if (eName.equals("q-ex-fv")) { processStartQExFV (eName,attrs); }
	if (eName.equals("q-ex-prop")) { processStartQExProp (eName,attrs); }	
	if (eName.equals("q-ex-rel")) { processStartQExRel (eName,attrs); }	
	if (eName.equals("f-relation")) { processStartFRelation (eName,attrs); }	
	if (eName.equals("f-featval")) { processStartFFeatVal(eName,attrs); }	
	if (eName.equals("f-mod-salience")) { processStartFModSalience (eName,attrs); }	
		if (eName.equals("f-mod-maxevid")) { processStartFModMaxevid (eName,attrs); }	
		if (eName.equals("f-rel-type")) { processStartFRelType (eName,attrs); }			
	if (eName.equals("q-mod-maxevid-bool")) { processStartQModMaxevidBool (eName,attrs); }	
    } // end startElement

    
    //=================================================================
    // ELEMENT-SPECIFIC XML PROCESSING METHODS
    //=================================================================

    //------------------------------
    // PROCESS START ELEMENT METHODS
    //------------------------------

    /**
     *  The method <i>processStartChooser</i> creates a new UPGChooser
     *  object, and stores the attributes "id", "region" and
     *  "metafunction" with it. The internal variable <tt>currentChooser</tt>
     *  stores this chooser for global acces.
     */

    public void processStartChooser (String eName, Attributes attrs) {
	String id     = "";
	String region = "";
	String metafunction  = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("id")) { id = attrs.getValue(i); }
	    if (aName.equals("region")) { region = attrs.getValue(i); }
	    if (aName.equals("metafunction")) { metafunction = attrs.getValue(i); }
	} // end for over attributes
	if (region.equals("")) { region="<none>"; } 
	if (metafunction.equals("")) { metafunction="<none>"; } 
	currentChooser = new UPGChooser (id,region,metafunction); 
	log("new chooser "+id+","+region+","+metafunction);
    } // end processStartComponent

    /**
     *  The method <i>processDecTree</i> creates a new UPGDecisionTree
     *  object. The internal variable <tt>currentDecTree</tt>
     *  stores this decision tree for global acces.
     */

    public void processStartDecTree (String eName, Attributes attrs) {
	currentDecTree = new UPGDecTree (); 
    } // end processStartComponent

    /**
     *  The method <i>processStartChoiceNode</i> creates a new
     *  UPGChoiceNode object, and stores the attribute "answer" with
     *  it. If the value of the answer equals <tt>*TOP*</tt> we make
     *  this choice node the root of the current decision tree.  The
     *  internal variable <tt>currentNode</tt> stores this chooser for
     *  global acces. 
     */

    public void processStartChoiceNode (String eName, Attributes attrs) {
	String answer  = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("answer")) { answer = attrs.getValue(i); }
	} // end for over attributes
	UPGChoiceNode node = new UPGChoiceNode (); 
	node.setAnswer(answer);
	if (node.isRoot()) { 
	    currentNode = node;
	    stack.push(node);
	} else { 
	    stack.push(currentNode.copy());
	    currentNode = node;
	} // end if..else root or child
    } // end processStartComponent

    /**
     *  The method <i>processStartInquiry</i> creates a new inquiry
     *  object and sets the <tt>currentInq</tt> variable to it.
     */

    public void processStartInquiry (String eName, Attributes attrs) {
	String answerset  = "";
	String id = "";
	String type = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("id")) { id = attrs.getValue(i); }
	    if (aName.equals("type")) { type = attrs.getValue(i); }
	    if (aName.equals("answerset")) { answerset = attrs.getValue(i); }
	} // end for over attributes
	UPGInquiry inq = new UPGInquiry (id,type,answerset); 
	currentInq = inq;
	log("New inquiry "+id+", "+type+", "+answerset);
    } // end processStartComponent

    /**
     *  The method <i>processStartResult</i> reads the value of the
     *  "val" attribute and sets the result of the current choice node
     *  to this value. 
     */

    public void processStartResult (String eName, Attributes attrs) {
	String val = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("val")) { val = attrs.getValue(i); }
	} // end for over attributes
	currentNode.setResult(val);
    } /// end processStartComponent

    /**
     *  The method <i>processStartQRelEq</i> sets the code id of the
     *  current inquiry to "q-rel-eq", reads the values of the "focus"
     *  and "val" attributes, and adds the values for these attributes
     *  to the inquiry's parameters.
     */

    public void processStartQRelEq (String eName, Attributes attrs) {
	String focus = "";
	String val = "";
	String nomvar = "locus";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("val")) { val = attrs.getValue(i); }
	    if (aName.equals("focus")) { focus = attrs.getValue(i); }
	    if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
	} // end for over attributes
	currentInq.setCodeId("q-rel-eq");
	currentInq.setParameterValue("focus",focus);	
	currentInq.setParameterValue("val",val);
	currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent


    /**
     *  The method <i>processStartQRelNum</i> sets the code id of the
     *  current inquiry to "q-rel-num", reads the values of the "rel",
     *  "feat" and "val" attributes, and adds the values for these
     *  attributes to the inquiry's parameters.
     */

    public void processStartQRelNum (String eName, Attributes attrs) {
	String rel = "";
	String feat = "";
	String val = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("val")) { val = attrs.getValue(i); }
	    if (aName.equals("feat")) { feat = attrs.getValue(i); }
	    if (aName.equals("rel")) { rel = attrs.getValue(i); }
	} // end for over attributes
	currentInq.setCodeId("q-rel-num");
	currentInq.setParameterValue("rel",rel);
	currentInq.setParameterValue("feat",feat);
	currentInq.setParameterValue("val",val);
    } /// end processStartComponent

    /**
     *  The method <i>processStartQExFeat</i> sets the code id of the
     *  current inquiry to "q-ex-feat", reads the values of the "feat"
     *  attribute, and adds the value for this attribute to the
     *  inquiry's parameters.
     */

    public void processStartQExFeat (String eName, Attributes attrs) {
	String feat = "";
	String nomvar = "locus";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("feat")) { feat = attrs.getValue(i); }
 		 if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }	    
	} // end for over attributes
	currentInq.setCodeId("q-ex-feat");
	currentInq.setParameterValue("feat",feat);
   currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent

    /**
     *  The method <i>processStartQExFV</i> sets the code id of the
     *  current inquiry to "q-ex-fv", reads the values of the "feat"
     *  and "val" attributes, and adds the values for these attributes
     *  to the inquiry's parameters. It stores the value of the
     *  (optional) attribute "nomvar" in a nomvar field; this field is
     *  initialized with "locus".
     */

    public void processStartQExFV (String eName, Attributes attrs) {
	String feat = "";
	String val = "";
	String nomvar = "locus";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("feat")) { feat = attrs.getValue(i); }
	    if (aName.equals("val")) { val = attrs.getValue(i); }
	    if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }	    
	} // end for over attributes
	currentInq.setCodeId("q-ex-fv");
	currentInq.setParameterValue("feat",feat);
	currentInq.setParameterValue("val",val);
	currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent

    /**
     *  The method <i>processStartQExProp</i> sets the code id of the
     *  current inquiry to "q-ex-prop", reads the value of the "val"
     *  attribute, and adds the value for this attribute to the
     *  inquiry's parameters.
     */

    public void processStartQExProp (String eName, Attributes attrs) {
		String val = "";
		String nomvar = "locus";
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			if (aName.equals("val")) { val = attrs.getValue(i); }
			if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }		
		} // end for over attributes
		currentInq.setCodeId("q-ex-prop");
		currentInq.setParameterValue("val",val);
		currentInq.setParameterValue("nomvar",nomvar);		
    } /// end processStartComponent

    /**
     *  The method <i>processStartQExRel</i> sets the code id of the
     *  current inquiry to "q-ex-rel", reads the value of the "val"
     *  attribute, and adds the value for this attribute to the
     *  inquiry's parameters.
     */

    public void processStartQExRel (String eName, Attributes attrs) {
	String val = "";
	String nomvar = "locus";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("val")) { val = attrs.getValue(i); }
	    if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
	} // end for over attributes
	currentInq.setCodeId("q-ex-rel");
	currentInq.setParameterValue("val",val);
	currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent


    /**
     *  The method <i>processStartFModSalience</i> sets the code id of the
     *  current inquiry to "f-mod-salience", reads the value of the "mod"
     *  attribute, and adds the value for this attribute to the
     *  inquiry's parameters.
     */

    public void processStartFModSalience (String eName, Attributes attrs) {
	String mod = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("mod")) { mod = attrs.getValue(i); }
	} // end for over attributes
	currentInq.setCodeId("f-mod-salience");
	currentInq.setParameterValue("mod",mod);
    } /// end processStartComponent

    public void processStartFRelType (String eName, Attributes attrs) {
		String nomvar = "";
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
		} // end for over attributes
		currentInq.setCodeId("f-rel-type");
		currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent	
	
	public void processStartQDepRelType (String eName, Attributes attrs) {
		String nomvar = "";
		String mode = "";
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
			if (aName.equals("mode")) { mode = attrs.getValue(i); }			
		} // end for over attributes
		currentInq.setCodeId("q-deprel-type");
		currentInq.setParameterValue("nomvar",nomvar);
		currentInq.setParameterValue("mode",mode);		
    } /// end processStartComponent	
	
	
	
    /**
     *  The method <i>processStartFModMaxevid</i> sets the code id of the
     *  current inquiry to "f-mod-maxevid". 
     */

    public void processStartFModMaxevid (String eName, Attributes attrs) {
	currentInq.setCodeId("f-mod-maxevid");
    } /// end processStartComponent

    /**
     *  The method <i>processStartFRelation</i> sets the code id of the
     *  current inquiry to "f-relation". 
     */

    public void processStartFRelation (String eName, Attributes attrs) {
	currentInq.setCodeId("f-relation");
    } /// end processStartComponent


    /**
     *  The method <i>processStartFFeatVal</i> sets the code id of the
     *  current inquiry to "f-featval". 
     */

    public void processStartFFeatVal (String eName, Attributes attrs) {
	String feat = "";
	String nomvar = "locus";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("feat")) { feat = attrs.getValue(i); }
	    if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }	    
	} // end for over attributes
	currentInq.setCodeId("f-featval");
	currentInq.setParameterValue("feat",feat);
	currentInq.setParameterValue("nomvar",nomvar);
    } /// end processStartComponent

    /**
     *  The method <i>processStartQModMaxevidBool</i> sets the code id of the
     *  current inquiry to "q-mod-maxevid-bool", reads the value of the "mod"
     *  attribute, and adds the value for this attribute to the
     *  inquiry's parameters.
     */

    public void processStartQModMaxevidBool (String eName, Attributes attrs) {
	String mod = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("mod")) { mod = attrs.getValue(i); }
	} // end for over attributes
	currentInq.setCodeId("q-mod-maxevid-bool");
	currentInq.setParameterValue("mod",mod);
    } /// end processStartComponent


    //------------------------------
    // PROCESS END ELEMENT METHODS
    //------------------------------
    
    /**
     * The method <i>processEndChooser</i> stores a copy of the current
     * chooser stored in <tt>currentChooser</tt> in the <tt>choosers</tt>
     * HashMap, and resets the variable <tt>currentChooser</tt>.
     */

    public void processEndChooser (String eName) { 
	UPGChooser copy = currentChooser.copy();
	String id = copy.getId();
	if (!choosers.containsKey(id)) { 
	    choosers.put(id,copy); 
	} else { 
	    System.err.println("Duplicate id \""+id+"\" for chooser in file <"+filename+">");
	} // end if..else
	currentChooser = new UPGChooser () ; 
    } // end processEndComponent

    /**
     *  The method <i>processEndDecTree</i> stores a copy of the
     *  current decision tree with the current chooser, and then
     *  resets the variable <tt>currentDecTree</tt>.
     */

     public void processEndDecTree (String eName) { 
	UPGDecTree tree = currentDecTree.copy();
	currentChooser.setDecisionTree(tree);
	currentDecTree = new UPGDecTree();
    } // end processEndComponent

    /**
     *  The method <i>processEndChoiceNode</i> checks whether the
     *  current node is the root of the decision tree, or not. If we
     *  have a child node, then we add it as child to the node we pop
     *  from the stack, and make the popped node the new current node. 
     */

    public void processEndChoiceNode (String eName) { 
	UPGChoiceNode save = currentNode.copy();
	if (save.isRoot()) { 
	    currentDecTree.setRoot(currentNode);
	    log("setting choicenode "+currentNode.getAnswer()+" as root");
	    currentNode = (UPGChoiceNode) stack.pop();
	} else { 
	    currentNode = (UPGChoiceNode) stack.pop();
	    currentNode.addChild(save);
	    log("adding choicenode "+save.getAnswer()+" as child of "+currentNode.getAnswer());
	} // end if..else check for root or child. 
    } // end processEndComponent


    /**
     *  The method <i>processEndInquiry</i> adds a copy of the current
     *  inquiry to the current choice node, and resets the variable
     *  <tt>currentInq</tt>.
     */

    public void processEndInquiry (String eName) { 
	UPGInquiry cp = currentInq.copy();
	currentNode.setInquiry(cp); 
	currentInq = new UPGInquiry ();
    } // end processEndComponent

    /**
     *  The method <i>processEndResult</i> has no functionality
     */

    public void processEndResult (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndQRelEq</i> has no functionality
     */

    public void processEndQRelEq (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndQRelNum</i> has no functionality
     */

    public void processEndQRelNum (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndQExFeat</i> has no functionality
     */

    public void processEndQExFeat (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndQExFV</i> has no functionality
     */

    public void processEndQExFV (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndQExProp</i> has no functionality
     */

    public void processEndQExProp (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndQExRel</i> has no functionality
     */

    public void processEndQExRel (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndFModSalience</i> has no functionality
     */

    public void processEndFModSalience (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndFRelation</i> has no functionality
     */

    public void processEndFRelation (String eName) { 
    } // end processEndComponent

    public void processEndFFeatVal (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndFModMaxevid</i> has no functionality
     */

    public void processEndFModMaxevid (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndQModMaxevidBool</i> has no functionality
     */

    public void processEndQModMaxevidBool (String eName) { 
    } // end processEndComponent

    


    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
