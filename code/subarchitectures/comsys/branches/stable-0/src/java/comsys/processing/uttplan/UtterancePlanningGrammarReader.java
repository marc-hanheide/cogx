//=================================================================
// Copyright (C) 2005--2009 Geert-Jan M. Kruijff (gj@acm.org)
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

import java.io.*;
import java.util.*;

// SAX parser

import org.xml.sax.*;
import org.xml.sax.helpers.*;
import javax.xml.parsers.*;

// ORG.COGNITIVESYSTEMS packages

import comsys.datastructs.lf.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UtterancePlanningGrammarReader</b> implements an
 *  XML-based parser for reading in grammars that are used in
 *  utterance planning, and which are stored as XML-files. 
 * 
 *  @version 050207 (started 050207)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UtterancePlanningGrammarReader 
    extends DefaultHandler
{
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    String filename = "";
    String grammarname = "";
    
    Vector systemsfiles; 
    Vector choosersfiles;

    boolean logging = false; 

    String absPath = "./";

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /**
     *  The basic constructor initializes the internal variables. 
     */

    public UtterancePlanningGrammarReader () { 
	filename = "";
	systemsfiles = new Vector();
	choosersfiles = new Vector(); 
    } // end constructor

    /** 
     * The unary constructor initializes the internal variables, and
     * takes the full filename (pointing to a "grammar.xml") from
     * which the grammar specification should be read.
     */

    public UtterancePlanningGrammarReader (String fn) {
	filename = fn;
	absPath = new File(fn).getAbsolutePath();
	absPath = absPath.substring(0,absPath.indexOf("grammar.xml"));
	systemsfiles = new Vector();
	choosersfiles = new Vector(); 
    } // end constructor

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>getChoosersFiles</i> returns a vector with filenames. 
     */

    public Vector getChoosersFiles () { return choosersfiles; }

    /**
     *  The method <i>getName</i> returns the name of the grammar.
     */

    public String getName () { return grammarname; } 

    /**
     *  The method <i>getSystemsFiles</i> returns a vector with filenames. 
     */

    public Vector getSystemsFiles () { return systemsfiles; }     

    //=================================================================
    // I/O METHODS
    //=================================================================

    public void log (String m) { if (logging) { System.out.println(m); } }

    //=================================================================
    // XML I/O METHODS
    //=================================================================

    public void read (String fn) {
        try {
			// System.out.println("Grammar reader: reading ["+fn+"]");
			if (fn.indexOf("grammar.xml") == -1) { 
				fn = fn + "/grammar.xml"; }
			File file = new File (fn); 
			absPath = file.getAbsolutePath();
			absPath = absPath.substring(0,absPath.indexOf("grammar.xml"));
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
	if (eName.equals("grammar")) { processEndGrammar (eName); }	    
	if (eName.equals("choosers")) { processEndChoosers (eName); }	    
	if (eName.equals("systems")) { processEndSystems (eName); }	    
    } // end endElement

    public void startElement(String namespaceURI,
			     String sName, // simple name
			     String qName, // qualified name
			     Attributes attrs) 
    {
	String eName = sName; 
	if ("".equals(eName)) {eName = qName;} // in case not namespace aware
	if (eName.equals("grammar")) { processStartGrammar (eName,attrs); }
	if (eName.equals("systems")) { processStartSystems (eName,attrs); }
	if (eName.equals("choosers")) { processStartChoosers (eName,attrs); }
    } // end startElement


    //=================================================================
    // ELEMENT-SPECIFIC XML PROCESSING METHODS
    //=================================================================

    //------------------------------
    // PROCESS START ELEMENT METHODS
    //------------------------------

    /**
     *  The method <i>processStartGrammar</i> gets the name of the
     *  grammar, and initializes the vectors for storing the filenames
     *  for the systems files and the choosers files.
     */

    public void processStartGrammar (String eName, Attributes attrs) {
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("name")) { grammarname = attrs.getValue(i); }
	} // end for over attributes
       	systemsfiles = new Vector ();
	choosersfiles = new Vector ();
    } // end processStartComponent

    /**
     *  The method <i>processStartSystems</i> gets the name of a file
     *  with system definitions, and adds that name to the vector with
     *  filenames for the systems files.
     */

    public void processStartSystems (String eName, Attributes attrs) {
	String filename = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("file")) { filename = attrs.getValue(i); }
	} // end for over attributes
	filename = absPath+filename;
	systemsfiles.addElement(filename);
	log("[UPGReader] add systems file "+filename);
    } // end processStartComponent

    /**
     *  The method <i>processStartChoosers</i> gets the name of a file
     *  with chooser definitions, and adds that name to the vector with
     *  filenames for the choosers files.
     */

    public void processStartChoosers (String eName, Attributes attrs) {
	String filename = "";
	for (int i = 0; i < attrs.getLength(); i++) {
	    String aName = attrs.getQName(i);
	    if (aName.equals("file")) { filename = attrs.getValue(i); }
	} // end for over attributes
	filename = absPath+filename;
	choosersfiles.addElement(filename);
	log("[UPGReader] add choosers file "+filename);
    } // end processStartComponent

    //------------------------------
    // PROCESS END ELEMENT METHODS
    //------------------------------
    
    public void processEndGrammar (String eName) { 
	// do nothing
    } 

    public void processEndSystems (String eName) { 
	// do nothing
    } 

    public void processEndChoosers (String eName) { 
	// do nothing
    } 









    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
