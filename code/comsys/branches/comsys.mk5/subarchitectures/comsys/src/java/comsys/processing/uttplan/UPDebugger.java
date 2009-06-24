//=================================================================
// Copyright (C) 2005-2009 Geert-Jan M. Kruijff (gj@acm.org)
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
import opennlp.ccg.lexicon.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.parse.*;
import opennlp.ccg.util.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.unify.*;
import opennlp.ccg.ngrams.*;
import opennlp.ccg.test.*;

import org.jdom.*;

import java.io.*;
import java.net.*;
import java.util.*;

// this is to handle user interaction                                                                                        
import jline.*;

import comsys.lf.utils.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPDebugger</b> implements a debugger around a 
 *  sentence planner. The debugger defines a text-based interface
 *  to the user, prompting the user (when necessary) to provide 
 *  an answer to an inquiry that is being posed by the planner. 
 * 
 *  @version 050721 (started 050405)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPDebugger 
    implements UPReqHandler
{
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    boolean logging = true; 

    ConsoleReader reader; 
    
    public static final String HISTORY = "Command Line History";

    private final String version = "0.25";

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables
     */

    public UPDebugger () {

    } // end constructor



    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     * 
     */

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================


	/** This method creates a new diamond-element representing a feature of
	 *  the given LFNominal. It appends this new Element to the given one and
	 *  returns the combined Element. It is a help-method for creating an XML-doument
	 *  that represents a given LogicalForm
	 * @return Element 
	 * @param LogicalForm complete, LFNominal to extract feature value from, String containing the feature,
	 * Element to which will be appended
	 */
	private Element editFeatures(LogicalForm input,LFNominal nom,String feature,Element el) {

		// create root diamond node
		Element diamond = new Element("diamond");
		diamond.setAttribute("mode",feature);
		// create its proposition node
		
		Element prop = new Element("prop");
		
		comsys.datastructs.lf.Feature[] feats = nom.feats;
		
		for (int i=0; i < feats.length ; i++) {
			if (feats[i].value.equals(feature)) {
				prop.setAttribute("name",feats[i].value);		
			}
		}
		// add all in a tree-like manner
		diamond.addContent(prop);
		el.addContent(diamond);
		return el;

	} // end editFeatures
	
	
	/** This method creates a new diamond-element representing the given relation of
	 *  the given LFNominal. If the relation itself has relations, recursively call
	 *  this method to append the relations to the current element. The this new Element is
	 *  appended to the originally given one and that combined Element is returned.
	 *  It is a help-method for creating an XML-doument that represents a given LogicalForm
	 * @return Element 
	 * @param LogicalForm complete, LFRelation, Element to which will be appended
	 */	
    private Element editRelations(LogicalForm input, LFRelation rel, Element el) {
	
	//--------------------------------------------
	//GJ: let's have a look at what's going on ...
	//-------------------------------------------- 
	
	// create root diamond node
	Element diamond = new Element("diamond");
	diamond.setAttribute("mode",rel.mode);
	
	// create its nom node containing dependent and type
	    
	Element nom = new Element("nom");
	LFNominal helper = LFUtils.lfGetNominal(input, rel.dep) ;
	String s = helper.nomVar +":"+ helper.sort;
	nom.setAttribute("name",s);
	// create its proposition node
	Element prop = new Element("prop");
	prop.setAttribute("name",helper.prop.prop);
	// add them
	diamond.addContent(nom);
	diamond.addContent(prop);
	// if the diamond itself has any features add those recursively
	if (helper.feats.length > 0) {
	    Iterator<comsys.datastructs.lf.Feature> iter = 
	    	(Arrays.asList(helper.feats)).iterator();
	    while (iter.hasNext()) {
		diamond = editFeatures(input,helper,(String)iter.next().feat,diamond);
	    }
	}
	// same for relations
	if (helper.rels.length > 0) {
	    Iterator<LFRelation> iter = (Arrays.asList(helper.rels)).iterator();
	    while (iter.hasNext()) {
		LFRelation hrel = (LFRelation) iter.next();

		diamond = editRelations(input,hrel,diamond);
	    }
	}
	el.addContent(diamond);
       	return el;
    } // end editRelations
    

    public String realizeLF (Realizer realizer, LogicalForm logform) { 
		
	String output = "";
	LF lf = transformLF(realizer,logform);
	
	realizer.realize(lf);
	opennlp.ccg.realize.Chart chart = realizer.getChart();
	// chart.printBestEdge();

	List bestEdges = chart.bestEdges();

	for (Iterator beIter = bestEdges.iterator(); beIter.hasNext(); ) {
	    Edge edge = (Edge) beIter.next(); 
	    Sign sign = edge.getSign();
	    output = output+sign.toString()+"\n";
	} // end for over best edges
	System.out.println();
	msg("Realization(s):");
	System.out.println();	
	msg(output); 
	return output;
    } // end realizeLF




    public LogicalForm solvePlanGRE (LFNominal nom) { 
	LogicalForm lf = new LogicalForm ();
	
	try { 
	    msg(" Please provide a logical form as GRE for nominal "+nom.nomVar);
	    String input = reader.readLine ("answer> ");
	    input = input.trim();
	    lf = LFUtils.convertFromString(input);

	} catch (IOException e) { 
	    error(e.getMessage());
	}	
	
	return lf;
    } // end solvePlanGRE

    public String solveRequest (String req, String[] args) {
	String answer = "";
	
	try { 
	    if (req.startsWith("f-")) { 
		msg(" Please provide an answer for the inquiry "+req);
	    } else { 
		if (req.indexOf("-bool") > -1) { 
		    msg(" Please answer 'true' or 'false' for the inquiry "+req);
		} else {
		    msg(" Please provide an answer for the inquiry "+req+" for locus nominal "+args[0]);
		    if (args.length > 1) { 
			msg(" from the answerset "+args[1]);
		    } // end if.. check whether answerset has been provided
		}
	    }
	    String input = reader.readLine ("answer> ");
	    answer = input.trim();

	} catch (IOException e) { 
	    error(e.getMessage());
	}
	return answer;
    } 

   /** The method <b>transformLF</b> takes as input a logical form to
     * be transformed into a CCG-LF, and returns that (possibly empty) 
	 * LF-object.
	 * First an XML-document is created with the input LogicalForm and then the
	 * openccg-Realize.java extracts an LF-object from that xml
	 * @return LF (the transformed)
	 * @param LogicalForm
	 */ 
   
    public LF transformLF (Realizer realizer, LogicalForm input) {
    	
	Document doc;
	LF lf = null;
	 try { 
		// LogicalForm to xml-File
      		// -------------------------------------------
		// GJ: Make it a proper XML document, with a
		// document type and an indication of the root
		// element.
		// -------------------------------------------

		DocType doctype = new DocType("xml");
		Element root1 = new Element("xml");
		doc = new Document(root1,doctype);
		
		// basic Element "lf"

		Element lf1 = new Element("lf");

		//----------------------------------------------
		//GJ: First, deal with the root of the LF
		//----------------------------------------------

		LFNominal inputNom = (LFNominal)input.root;

		// basic Element "satop" with nom = "id:type"
		Element sat = new Element("satop");
		String s = inputNom.nomVar;
		s = s + ":" + inputNom.sort;
		sat.setAttribute("nom",s);

		// basic proposition, added to "satop"
		Element prop = new Element("prop");
		prop.setAttribute("name",inputNom.prop.prop);
		sat.addContent(prop);

		//----------------------------------------------
		//GJ: Now the root has a nominal, type, and 
		//proposition. 
		//----------------------------------------------
		
		// this test shows whether a satop-element has been created, 
		// it is only successful if it contains a NomVar and a Proposition

		//----------------------------------------------
		// if there are features create and add elements in 'editFeatures()'
		// GJ: still dealing with the root at this point.
		//---------------------------------------------- 

		if (inputNom.feats.length > 0) { 
			Iterator<comsys.datastructs.lf.Feature> iter = 
	    	(Arrays.asList(inputNom.feats)).iterator();    
			while (iter.hasNext()) {
				sat = editFeatures(input,inputNom,(String)iter.next().feat,sat);
			}
		}

		//----------------------------------------------
		//GJ: okay, so now we start some recursion down
		//the relations of the nominal ...  
		//----------------------------------------------


		// if there are Relations create and add elements in 'editRelations()' 
		if (inputNom.rels.length > 0) {
			Iterator<LFRelation> it = Arrays.asList(inputNom.rels).iterator();
			while (it.hasNext()) {
				sat = editRelations(input,(LFRelation)it.next(),sat);
			} // end while over relations
		} // end if.. check for relations


		//----------------------------------------------
		//GJ: now we supposedly have a complete structure 
		// ----------------------------------------------
		
		lf1.addContent(sat);
	    doc.getRootElement().addContent(lf1);			
		Element targetElt = new Element("target");
		targetElt.setText("*** dummy target***");
		root1.addContent(targetElt);
		

		//------------- openccg - Realize -----------------
		// call ccg.Realizer with XML-document which extracts the contained LF and returns it
		lf = realizer.getLfFromDoc(doc);
		
	 } catch (Exception e) {
	    log("error in LF: "+e.getMessage()); 
	 } 
	// return resulting LF
	return lf;

    } // end transformLF




    //=================================================================
    // I/O METHODS
    //=================================================================

    /**
     *  The method <i>startInterface</i> starts the text-based user
     *  interface for the debugger. The interface knows the following
     *  commands:
     *  <ul>
     *  <li> <tt>:l(oad) &lt;grammarfile&gt; loads a new planning grammar </li>
     *  <li> <tt>:p(lan) &lt;comm.goal&gt; plans a logical form given the communicative goal </li>
     *  <li> <tt>:q(uit)</tt>  quits the debugger
    */

    public void startInterface (String[] args) { 
	String usage = "Utterance Planner Debugger v."+version+"\n"
	                        +"Command line options:\n"
	                        +"-help, -usage: this message\n"
	                        +"-grammar <file>: load the planning grammar <file>\n";

	// Default the grammar file to grammar.xml in the current working directory
	String curDir = System.getProperty("user.dir");
	String grammarfile = curDir+"/grammar.xml";

	// Set the ccg grammar file and the realizer to null
	String ccgfile = null; 
	Realizer realizer = null;

	// Check whether debugger is started with cry for help
        if (args.length > 0 && (args[0].equals("-help")||args[0].equals("-usage"))) {
            System.out.println("Usage: " + usage);
            System.exit(0);
        } // end if

	// Check whether there are any arguments 
       	if (args.length > 0) { 
	    for (int i=0; i < args.length; i++) { 
		if (args[i].equals("-grammar")) { grammarfile=args[++i]; }
		if (args[i].equals("-ccg")) { ccgfile=args[++i]; }
	    } // end for
	} // end if

	// Print header 
	System.out.println("Utterance Planner Debugger v."+version);
	System.out.println();
       	System.out.println("Loading the planning grammar from ["+grammarfile+"]");

	// Initialize the planner with the grammarfile 

	UtterancePlanner planner = new UtterancePlanner(grammarfile,this);

	// Initialize the realizer with the CCG grammar, if there is one specified
	if (ccgfile != null) { 
	    try { 
		URL grammarURL = new File(ccgfile).toURL();
		System.out.println("Loading a CCG grammar from URL: " + grammarURL);
		Grammar grammar = new Grammar(grammarURL);
		realizer = new Realizer(grammar);	
	    } catch (IOException e) {
		error(e.getMessage());
	    }
	} // end if check whether there is already a CCG grammar

	// Now start up the interface
	try { 
	    reader = new ConsoleReader (); 
	    LogicalForm lastLF = null;

	    // store commands for 'tab' argument completion                                                                      
	    List completors = new LinkedList();
	    completors.add(new SimpleCompletor(new String[]
		{ ":ccg", ":help", ":load", ":plan", ":quit", ":traceoff", ":tracelvl", "traceon"}));
	    reader.addCompletor(new ArgumentCompletor(completors));

	    // initialize history
	    jline.History history = new jline.History();
	    String historyKey = HISTORY;

	    // prepare a string buffer to write the (persistent) command input history into                              
	    StringWriter swriter = new StringWriter();
	    PrintWriter pwriter = new PrintWriter(swriter);
	    history.setOutput(pwriter);
	    reader.setHistory(history);
 
	    // print out help when starting
	    System.out.println();
	    System.out.println("Type ':plan <comm.goal>' to plan a logical form given the communicative goal.");
	    System.out.println("Type ':h' for help on display options and ':q' to quit.");
	    System.out.println("You can use the tab key for command completion, ");
	    System.out.println();

	    planner.setLogging(false);

	    // next, loop over the commandline until the user wants to quit
	    while (true) {
		String input = reader.readLine ("debug> ");
		if (input == null)      // control-D or the like                                                     
		    break;
		input = input.trim();
		if (input.equals(":q") || input.equals(":quit")) {
		    break; // end of while loop                                                                                  
		} else if (input.equals(":h")||input.equals(":help")) {
		    showHelp();
		} else if (input.startsWith(":ccg")) {
		    int slash = input.indexOf("/"); 
		    if (slash > -1) { 
			String filename = input.substring(slash,input.length());
			// load grammar                                                                                                      
			URL grammarURL = new File(grammarfile).toURL();
			System.out.println("Loading grammar from URL: " + grammarURL);
			Grammar grammar = new Grammar(grammarURL);
			realizer = new Realizer(grammar);
		    } else {
			error("':ccg' requires a filename including full path specification.");			
		    } // end if..else check for path
		} else if (input.startsWith(":load")||input.startsWith(":l ")) {
		    int slash = input.indexOf("/"); 
		    if (slash > -1) { 
			String filename = input.substring(slash,input.length());
			planner.load(filename);
		    } else {
			error("':load' requires a filename including full path specification.");			
		    } // end if..else check for path
		} else if (input.startsWith(":plan")||input.startsWith(":p ")) { 
		    int atop = input.indexOf("@");
		    if (atop > -1) { 
			String lfstr = input.substring(atop,input.length());
			LogicalForm lf = LFUtils.convertFromString(lfstr);
			LogicalForm planlf = planner.plan(lf);
			lastLF = planlf;
			System.out.println();
			System.out.println("Resulting logical form:");
			msg("");
			System.out.println(planlf.toString());
			msg("");
		    } else {
			error("':plan' requires a communicative goal, specified as HLDS logical form.");
		    } // end if..else check for lf 
		} else if (input.equals(":realize")||input.equals(":r")) { 
		    if (realizer != null) { 
			if (lastLF != null) { 
			    realizeLF(realizer,lastLF); 
			} else { 
			    error("Cannot realize a logical form: no logical form has been planned yet.");
			} // end if..else check for logical form
		    } else {
			error("The realizer has not yet been initialized with a grammar -- use :ccg to load a CCG grammar.");
		    } // end if..else check for realizer
		} else if (input.equals(":traceoff")) {  
		    planner.setLogging(false);
		    msg("Tracing turned off");
		} else if (input.equals(":traceon")) { 
		    planner.setLogging(true);
		    msg("Tracing turned on");
		} else if (input.startsWith(":tracelvl ")) { 
		    planner.setLogging(true);
		    int lastColon = input.lastIndexOf(":"); 
		    String lvlstr = input.substring(lastColon+1,input.length());
		    int lvl = (new Integer(lvlstr)).intValue();
		    msg("Tracing level set to "+lvl);                    
		    planner.setLoggingLevel(lvl);
		} else { 
		    if (input.length() > 0) {
			String possnum = input.substring(1,input.length());
			int lvl = -1;
			try { 
			    Integer lvlInt = new Integer(possnum);
			    lvl = lvlInt.intValue();
			} catch (NumberFormatException e) { 
			    // do nothing
			} // end try..catch
			if (lvl > -1) { 
			    msg("Tracing level set to "+lvl);                    
			    planner.setLoggingLevel(lvl);
			}
		    } // end if.. check whether there is input at all
		} 
	    } // end while

	} catch (IOException e) { 
	    System.out.println(e.getMessage());
	} // end try.catch
	

    } // end 

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    private void error (String msg) { 
	System.out.println("ERROR! "+msg);
    } // end error

    private void msg (String msg) { 
	System.out.println("       "+msg);
    } // end error



    private void showHelp () { 
	System.out.println();
	System.out.println("Commands for the utterance planner debugger");
	System.out.println();
	System.out.println(":h(elp) \t\t shows this message");
	System.out.println(":q(uit) \t\t quits the debugger");
	System.out.println();
	System.out.println(":ccg <file>         \t loads the given CCG grammar, and initializes the realizer with it");
	System.out.println(":l(oad) <file>      \t loads the given planning grammar");
	System.out.println(":p(lan) <comm.goal> \t plans a logical form for the communicative goal (also a logical form)");
	System.out.println(":r(ealize)          \t realizes the last planned logical form (if any), provided the realizer has been initialized");
	System.out.println(":traceoff           \t turns off tracing of steps in the planner");
	System.out.println(":traceon            \t turns on tracing of steps in the planner");
	System.out.println(":trace  :<lvl>      \t shows tracing of steps of level <lvl> and higher in the planner");
	System.out.println(":<lvl>              \t shows tracing of steps of level <lvl> and higher in the planner");	
	System.out.println();
    }

    private void log (String m) {
        if (logging) { System.out.println("[] "+m); }
    }


    //=================================================================
    // MAIN METHOD
    //=================================================================

    /**
     *  The debugger can be started up without any command-line args, 
     *  in which case the debugger looks for a <tt>grammar.xml</tt>
     *  file in the current directory. Available command-line
     *  arguments are:
     *  <ul>
     *  <li> <tt>-usage</tt> or <tt>-help</tt>: print help message</li>
     *  <li> <tt>-grammar &lt;file&gt;</tt>: load the planning grammar <tt>file</tt>
     *  </ul>
     */

    public static void main (String[] args) { 

	UPDebugger debugger = new UPDebugger ();
	debugger.startInterface(args);

    } // end main

} // end class definition 
