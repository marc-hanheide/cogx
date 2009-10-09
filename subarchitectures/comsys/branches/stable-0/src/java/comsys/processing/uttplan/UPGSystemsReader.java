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
 *  The class <b>UPGSystemsReader</b> constructs the systems for the
 *  systemic network, as specified in the systems XML file for the
 *  grammar.
 * 
 *  @version 050719
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPGSystemsReader 
    extends DefaultHandler
{
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    String  filename; 
    HashMap systems;

    UPGSystem currentSystem; 
    UPGAction currentAction;

    boolean logging = false;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UPGSystemsReader () {
	filename = "./systems.xml";
	systems = new HashMap ();
	currentAction = new UPGAction ();
	currentSystem = new UPGSystem ();
    } // end constructor

    /** 
     *  The unary constructor sets the name of the file from which the
     *  systems should be read, and initializes the internal
     *  variables.
     */

    public UPGSystemsReader (String fn) {
	filename = fn;
	systems = new HashMap ();
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     * The method <i>getSystems</i> returns a HashMap object with all
     * the systems, keyed by the system ids.
     */

    public HashMap getSystems () { return systems; } 

    //=================================================================
    // I/O METHODS
    //=================================================================

    public void log (String m) { if (logging) { System.out.println("[UPGSystemsRdr] "+m); } }
    
    //=================================================================
    // XML I/O METHODS
    //=================================================================

    public void read (String fn) {
        try {
	    log("Systems reader: reading ["+fn+"]");
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
	if (eName.equals("system")) { processEndSystem (eName); }	    
	if (eName.equals("chooser")) { processEndChooser (eName); } 
	if (eName.equals("conditions")) { processEndConditions (eName); } 
	if (eName.equals("condition")) { processEndCondition (eName); } 
	if (eName.equals("actions")) { processEndActions (eName); } 
	if (eName.equals("action")) { processEndAction (eName); } 
	if (eName.equals("add-lf")) { processEndAddLF (eName); } 
	if (eName.equals("add-feature")) { processEndAssignFeature (eName); } 
	if (eName.equals("add-proposition")) { processEndAddProposition (eName); } 
	if (eName.equals("add-relation")) { processEndAddRelation (eName); } 
	if (eName.equals("adjoin-lf")) { processEndAdjoinLF (eName); } 
	if (eName.equals("assign-type")) { processEndAssignType (eName); } 
	
	if (eName.equals("copy-lf")) { processEndAssignType (eName); } 
	if (eName.equals("copy-feature")) { processEndAssignType (eName); } 	
	
	
	if (eName.equals("generate-rfx")) { processEndGenerateRfx (eName); } 
	if (eName.equals("identify-nomvar")) { processEndIdentifyNomvar (eName); } 
	if (eName.equals("move-locus")) { processEndMoveLocus (eName); } 
    } // end endElement

    public void startElement(String namespaceURI,
			     String sName, // simple name
			     String qName, // qualified name
			     Attributes attrs) 
    {
	String eName = sName; 
	// System.out.println("\n"+currentAction.toString()+"\n\n");
	if ("".equals(eName)) {eName = qName;} // in case not namespace aware
	if (eName.equals("system")) { processStartSystem (eName,attrs); }
	if (eName.equals("chooser")) { processStartChooser (eName,attrs); }
	if (eName.equals("conditions")) { processStartConditions (eName,attrs); }
	if (eName.equals("condition")) { processStartCondition (eName,attrs); }
	if (eName.equals("actions")) { processStartActions (eName,attrs); }
	if (eName.equals("action")) { processStartAction (eName,attrs); }
	if (eName.equals("add-lf")) { processStartAddLF (eName,attrs); }
	if (eName.equals("add-feature")) { processStartAssignFeature (eName,attrs); }
	if (eName.equals("add-proposition")) { processStartAddProposition (eName,attrs); }
		if (eName.equals("add-relation")) { processStartAddRelation (eName,attrs); }
	if (eName.equals("adjoin-lf")) { processStartAdjoinLF (eName,attrs); }
	if (eName.equals("assign-type")) { processStartAssignType (eName,attrs); }
	
	if (eName.equals("copy-lf")) { processStartCopyLF (eName,attrs); }	
	if (eName.equals("copy-feature")) { processStartCopyFeature (eName,attrs); }		
	
	
	if (eName.equals("generate-rfx")) { processStartGenerateRfx (eName,attrs); }
	if (eName.equals("identify-nomvar")) { processStartIdentifyNomvar (eName,attrs); }
	if (eName.equals("move-locus")) { processStartMoveLocus (eName,attrs); }
		
	if (eName.equals("replace-relation")) { processStartReplaceRelation (eName,attrs); }		
		
		
    } // end startElement


    //=================================================================
    // ELEMENT-SPECIFIC XML PROCESSING METHODS
    //=================================================================

    //------------------------------
    // PROCESS START ELEMENT METHODS
    //------------------------------

    /**
     *  The method <i>processStartSystem</i> creates a new UPGSystem
     *  object, and stores the attributes "id", "region" and
     *  "metafunction" with it. The internal variable
     *  <tt>currentSystem</tt> stores this system for global acces.
     */

    public void processStartSystem (String eName, Attributes attrs) {
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
	log("adding system ["+id+","+region+","+metafunction+"]");
	currentSystem = new UPGSystem (id,region,metafunction); 
    } // end processStartComponent

    /**
     *  The method <i>processStartChooser</i> takes the value of the
     *  attribute "id" and stores that as the chooser id with the
     *  current system.
     */

    public void processStartChooser (String eName, Attributes attrs) {
	String id     = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("id")) { id = attrs.getValue(i); }
	} // end for over attributes
	log("adding chooser ["+id+"]");
	currentSystem.setChooserId(id);
    } // end processStartComponent

    /**
     *  The method <i>processStartConditions</i> has no functionality. 
     */

    public void processStartConditions (String eName, Attributes attrs) {
    } // end processStartComponent

    /**
     *  The method <i>processStartCondition</i> takes the value of the
     *  attribute "features", creates a new UPGCondition object, and
     *  stores each of the features as condition in the object. The
     *  UPGCondition object is then added to the conditions for the
     *  current system.
     */

    public void processStartCondition (String eName, Attributes attrs) {
	String features = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("features")) { features = attrs.getValue(i); }
	} // end for over attributes
	StringTokenizer st = new StringTokenizer(features); 
	UPGCondition cond = new UPGCondition();
	while (st.hasMoreTokens()) { 
	    String feat = st.nextToken();
	    cond.addFeature(feat);
	} // end while over features
	log("adding conditions ["+features+"]");
	currentSystem.addCondition(cond);
    } // end processStartComponent

    /**
     *  The method <i>processStartActions</i> has no functionality. 
     */

    public void processStartActions (String eName, Attributes attrs) {
    } // end processStartComponent

    /**
     *  The method <i>processStartAction</i> creates a new UPGAction
     *  object, and sets the currentAction to be this object. It then
     *  takes the value of the attribute "choice" and stores that as
     *  the return value of the chooser required to fire this action.
     */

    public void processStartAction (String eName, Attributes attrs) {
	String choice = "";
	currentAction = new UPGAction ();
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("choice")) { choice = attrs.getValue(i); }
	} // end for over attributes
	log("adding action for choice ["+choice+"]");
	currentAction.setChoice(choice);
    } // end processStartComponent



    /**
     *  The method <i>processStartAddLF</i> creates a new
     *  UPGActionStep object for an "add-lf" step, and adds it to
     *  the current action.
     */

    public void processStartAddLF (String eName, Attributes attrs) {
	String src = "unknown";
	String dest = "unknown";
	String lf = "unknown";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("dest")) { dest = attrs.getValue(i); }
	    if (aName.equals("lf")) { lf = attrs.getValue(i); }
	    if (aName.equals("src")) { src = attrs.getValue(i); }
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("add-lf");
	step.setAttribute("dest",dest);
	step.setAttribute("lf",lf);
	step.setAttribute("src",src);
	currentAction.addStep(step);
	log("adding action step add-lf for ["+lf+"] with destination "+dest+" and source ["+src+"]");
    } // end processStartComponent

    /**
     *  The method <i>processStartAssignType</i> creates a new
     *  UPGActionStep object for an "assign-type" step, and adds it to
     *  the current action.
     */

    public void processStartAssignType (String eName, Attributes attrs) {
	String type = "";
	String dest = "unknown";	
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("type")) { type = attrs.getValue(i); }
	    if (aName.equals("dest")) { dest = attrs.getValue(i); }
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("assign-type");
	step.setAttribute("type",type);
	step.setAttribute("dest",dest);
	currentAction.addStep(step);
	log("adding action step assign-type for type ["+type+"] at destination "+dest);
    } // end processStartComponent

    /**
     *  The method <i>processStartAddProposition</i> creates a new
     *  UPGActionStep object for an "add-proposition" step, and adds
     *  it to the current action.
     */

    public void processStartAddProposition (String eName, Attributes attrs) {
	String props = "";
	String dest = "unknown";	
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("propositions")) { props = attrs.getValue(i); }
	    if (aName.equals("dest")) { dest = attrs.getValue(i); }		
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("add-proposition");
	step.setAttribute("propositions",props);
	step.setAttribute("dest",dest);
	currentAction.addStep(step);
	log("adding action step add-proposition at destination "+dest+" for propositions ["+props+"]");
    } // end processStartComponent

    /**
     *  The method <i>processStartAssignFeature</i> creates a new
     *  UPGActionStep object for an "add-feature" step, and adds
     *  it to the current action.
     */

    public void processStartAssignFeature (String eName, Attributes attrs) {
	String feat = "";
	String dest = "unknown";
	String val = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("feature")) { feat = attrs.getValue(i); }
	    if (aName.equals("value")) { val = attrs.getValue(i); }
	    if (aName.equals("dest")) { dest = attrs.getValue(i); }		
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("add-feature");
	step.setAttribute("feature",feat);
	step.setAttribute("value",val);
	step.setAttribute("dest",dest);
	currentAction.addStep(step);
	log("adding action step add-feature for feature, value ["+feat+"/"+val+"]");
    } // end processStartComponent

    /**
     *  The method <i>processStartAddRelation</i> creates a new
     *  UPGActionStep object for an "add-relation" step, and adds
     *  it to the current action.
     */

    public void processStartAddRelation (String eName, Attributes attrs) {
	String mode = "";
	String nomvar = "";
	String type = "*VAR*";
	String orientation = "src";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("mode")) { mode = attrs.getValue(i); }
	    if (aName.equals("src"))  { nomvar = attrs.getValue(i); orientation="src"; }
	    if (aName.equals("dest")) { nomvar = attrs.getValue(i); orientation="dest"; }	    
	    if (aName.equals("type")) { type = attrs.getValue(i); }
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("add-relation");
	step.setAttribute("mode",mode);
	step.setAttribute("nomvar",nomvar);
	step.setAttribute("orientation",orientation);
	step.setAttribute("type",type);
	currentAction.addStep(step);
	log("adding action step add-relation for relation/nomvar/orientation ["+mode+"/"+type+":"+nomvar+":"+orientation+"]");
    } // end processStartComponent

    /**
     *  The method <i>processStartAdjoinLF</i> creates a new
     *  UPGActionStep object for an "adjoin-lf" step, and adds
     *  it to the current action.
     */

    public void processStartAdjoinLF (String eName, Attributes attrs) {
	String foot = "unknown";
	String lf   = "unknown";
	String mode  = "unknown";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("foot")) { foot = attrs.getValue(i); }
	    if (aName.equals("lf"))   { lf   = attrs.getValue(i); }
	    if (aName.equals("mode")) { mode = attrs.getValue(i); }
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("adjoin-lf");
	step.setAttribute("foot",foot);
	step.setAttribute("lf",lf);
	step.setAttribute("mode",mode);
	currentAction.addStep(step);
	log("adding action step adjoin-lf for lf/foot  ["+lf+"/"+foot+"] under relation ["+mode+"]");
    } // end processStartComponent


    /**
     *  The method <i>processStartCopyFeature</i> creates a new
     *  UPGActionStep object for an "copy-feature" step, and adds it to
     *  the current action. 
     */

    public void processStartCopyFeature (String eName, Attributes attrs) {
		String srcNom = "unknown";
		String destNom = "unknown";
		String srcFeat = "unknown";
		String destFeat = "unknown";
		String mode = "unknown";
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			if (aName.equals("srcnom")) { srcNom = attrs.getValue(i); }
			if (aName.equals("destnom")) { destNom = attrs.getValue(i); }
			if (aName.equals("srcfeat")) { srcFeat = attrs.getValue(i); }
			if (aName.equals("destfeat")) { destFeat = attrs.getValue(i); }		
			if (aName.equals("mode")) { mode = attrs.getValue(i); }					
		} // end for over attributes
		UPGActionStep step = new UPGActionStep("copy-feature");
		step.setAttribute("destnom",destNom);
		step.setAttribute("srcnom",srcNom);
		step.setAttribute("srcfeat",srcFeat);
		step.setAttribute("destfeat",destFeat);	
		step.setAttribute("mode",mode);			
		currentAction.addStep(step);
		log("adding action step copy-feature for value of feature ["+srcFeat+"] under nominal ["+srcNom+"] to destination ["+destFeat+"] under nominal ["+destNom+"] in mode ["+mode+"]");
    } // end processStartComponent


    /**
     *  The method <i>processStartCopyLF</i> creates a new
     *  UPGActionStep object for an "copy-lf" step, and adds it to
     *  the current action.
     */

    public void processStartCopyLF (String eName, Attributes attrs) {
	String src = "unknown";
	String dest = "unknown";
	String mode = "unknown";
	String excl = "unknown";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("mode")) { mode = attrs.getValue(i); }
	    if (aName.equals("dest")) { dest = attrs.getValue(i); }
	    if (aName.equals("src")) { src = attrs.getValue(i); }
	    if (aName.equals("excludes")) { excl = attrs.getValue(i); }		
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("copy-lf");
	step.setAttribute("dest",dest);
	step.setAttribute("mode",mode);
	step.setAttribute("src",src);
	step.setAttribute("excludes",excl);	
	currentAction.addStep(step);
	log("adding action step copy-lf in mode ["+mode+"] with destination "+dest+" and source ["+src+"] using as exclude-set ["+excl+"]");
    } // end processStartComponent





    /**
     *  The method <i>processStartGenerateRfx</i> creates a new
     *  UPGActionStep object for an "generate-rfx" step, and adds
     *  it to the current action.
     */

    public void processStartGenerateRfx (String eName, Attributes attrs) {
		String mode = "";
		String nomvar = "";
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			// if (aName.equals("mode")) { mode = attrs.getValue(i); }
			if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
		} // end for over attributes
		UPGActionStep step = new UPGActionStep("generate-rfx");
		step.setAttribute("nomvar",nomvar);
		currentAction.addStep(step);
		log("adding action step generate-rfx");
    } // end processStartComponent




    /**
     *  The method <i>processStartIdentifyNomvar</i> creates a new
     *  UPGActionStep object for an "identify-nomvar" step, and adds
     *  it to the current action.
     */

    public void processStartIdentifyNomvar (String eName, Attributes attrs) {
	String mode = "";
	String nomvar = "";
	String scope = "local";
	String orientation = "src";
	String root = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("mode")) { mode = attrs.getValue(i); }
	    if (aName.equals("src")) { nomvar = attrs.getValue(i); orientation = "src";}
	    if (aName.equals("dest")) { nomvar = attrs.getValue(i); orientation = "dest";}
	    if (aName.equals("scope")) { scope = attrs.getValue(i); }
	    if (aName.equals("root")) { root = attrs.getValue(i); }		
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("identify-nomvar");
		step.setAttribute("root",root);		
		step.setAttribute("mode",mode);
	step.setAttribute("nomvar",nomvar);
	step.setAttribute("orientation",orientation);
	step.setAttribute("scope",scope);
		
	currentAction.addStep(step);
	log("adding action step identify-nomvar for relation/orientation/nomvar/scope/root ["+mode+"/"+orientation+"/"+nomvar+"/"+scope+"/"+root+"]");
    } // end processStartComponent


    /**
     *  The method <i>processStartMoveLocus</i> creates a new
     *  UPGActionStep object for an "move-locus" step, and adds
     *  it to the current action.
     */

    public void processStartMoveLocus (String eName, Attributes attrs) {
	String nomvar = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("nomvar")) { nomvar = attrs.getValue(i); }
	} // end for over attributes
	UPGActionStep step = new UPGActionStep("move-locus");
	step.setAttribute("nomvar",nomvar);
	currentAction.addStep(step);
	log("adding action step move-locus for nomvar ["+nomvar+"]");
    } // end processStartComponent



    /**
     *  The method <i>processStartReplaceRelation</i> creates a new
     *  UPGActionStep object for an "replace-relation" step, and adds
     *  it to the current action.
     */
	
    public void processStartReplaceRelation (String eName, Attributes attrs) {
		String mode = "";
		String root = "";
		String newmode = "";
		String lf = "";
		String foot = "";		
		for (int i = 0; i < attrs.getLength(); i++) {
			String aName = attrs.getQName(i);
			if (aName.equals("mode")) { mode = attrs.getValue(i); }
			if (aName.equals("root")) { root = attrs.getValue(i);  }
			if (aName.equals("newmode")) { newmode = attrs.getValue(i); }	    
			if (aName.equals("lf")) { lf = attrs.getValue(i); }
			if (aName.equals("foot")) { foot = attrs.getValue(i); }			
		} // end for over attributes
		UPGActionStep step = new UPGActionStep("replace-relation");
		step.setAttribute("mode",mode);
		step.setAttribute("root",root);
		step.setAttribute("newmode",newmode);
		step.setAttribute("lf",lf);
		step.setAttribute("foot",foot);		
		currentAction.addStep(step);
		log("adding action step replace-relation for relation/root ["+mode+"/"+root+"]");
    } // end processStartComponent
    //------------------------------
	
	
    // PROCESS END ELEMENT METHODS
    //------------------------------
    
    /**
     * The method <i>processEndSystem</i> stores a copy of the current
     * system stored in <tt>currentSystem</tt> in the <tt>systems</tt>
     * HashMap, and resets the variable <tt>currentSystem</tt>.
     */

    public void processEndSystem (String eName) { 
	UPGSystem copy = currentSystem.copy();
	String id = copy.getId();
	if (!systems.containsKey(id)) { 
	    systems.put(id,copy); 
	} else { 
	   System.err.println("Duplicate id \""+id+"\" for systems in file <"+filename+">");
	} // end if..else
	currentSystem = new UPGSystem () ; 
    } // end processEndComponent
    
    /**
     *  The method <i>processEndChooser</i> has no functionality
     */

    public void processEndChooser (String eName) { 

    } // end processEndComponent


    /**
     *  The method <i>processEndConditions</i> has no functionality
     */

    public void processEndConditions (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndCondition</i> has no functionality
     */

    public void processEndCondition (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndActions</i> has no functionality
     */

    public void processEndActions (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndAction</i> stores a copy of the
     *  current action (<tt>currentAction</tt>) in the current system,
     *  and resets the <tt>currentAction</tt> variable.
     */

    public void processEndAction (String eName) { 
	UPGAction cp = currentAction.copy();
	currentSystem.addAction(cp);
	currentAction = new UPGAction();
    } // end processEndComponent


    /**
     *  The method <i>processEndAddLF</i> has no functionality
     */

    public void processEndAddLF (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndAssignType</i> has no functionality
     */

    public void processEndAssignType (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndAddProposition</i> has no functionality
     */

    public void processEndAddProposition (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndAssignFeature</i> has no functionality
     */

    public void processEndAssignFeature (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndAddRelation</i> has no functionality
     */

    public void processEndAddRelation (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndAdjoinLF</i> has no functionality
     */

    public void processEndAdjoinLF (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndCopyLF</i> has no functionality
     */

    public void processEndCopyLF (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndGenerateRfx</i> has no functionality
     */

    public void processEndGenerateRfx (String eName) { 
    } // end processEndComponent


    /**
     *  The method <i>processEndIdentifyNomvar</i> has no functionality
     */

    public void processEndIdentifyNomvar (String eName) { 
    } // end processEndComponent

    /**
     *  The method <i>processEndMoveLocus</i> has no functionality
     */

    public void processEndMoveLocus (String eName) { 
    } // end processEndComponent



    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
