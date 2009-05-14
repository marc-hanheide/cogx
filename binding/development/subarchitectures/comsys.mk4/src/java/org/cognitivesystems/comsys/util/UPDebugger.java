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

package org.cognitivesystems.comsys.util;

//=================================================================
// IMPORTS
//=================================================================

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
import org.jdom.output.XMLOutputter;

import java.io.*;
import java.net.*;
import java.util.*;

// this is to handle user interaction                                                                                        
import jline.*;

// ORG.COGNITIVESYSTEMS packages

import org.cognitivesystems.repr.lf.utils.*;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.comsys.util.uttplan.UtterancePlanner;
import org.cognitivesystems.comsys.util.uttplan.UPGChooser;
import org.cognitivesystems.comsys.util.uttplan.UPGSystem;
import org.cognitivesystems.comsys.util.uttplan.UPReqHandler;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPDebugger</b> implements a debugger around a 
 *  sentence planner. The debugger defines a text-based interface
 *  to the user, prompting the user (when necessary) to provide 
 *  an answer to an inquiry that is being posed by the planner. 
 * 
 * @started  050405
 *  @version 080708 
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPDebugger 
    implements org.cognitivesystems.comsys.util.uttplan.UPReqHandler
{
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    boolean logging = false; 

    ConsoleReader reader; 
    
    public static final String HISTORY = "Command Line History";

	// 0.25		050405	basic functionality
	// 0.4		080623	porting; 
	// 0.45		080630	added realize <subgraph>, improved CCG loading
	// 0.47		080630	realization tested, finished porting
	// 0.48		080708	added chooser output in "showgrammar"
	// 0.50		080716	added model reduction / filtering, feature copy 
	// 0.51		080822	utterance planning with random choosers/inquiries, add-lf maintains proper destination variable value 
	// 0.52		080822	utterance planning with identify-nomvar starting at any nominal in the LF
	// 0.53		080829	translation to LF produces only nominal references for Subject, Scope relations
	// 0.54		080829	utterance planning also executes actions after move-locus
	// 0.55		080829	utterance planning with q-deprel-type and f-rel-type inquiries
	// 0.56		080904	utterance planning with add-proposition also including dest with variable
	// 0.57		080904	statistical models for calling the realizer; use the -ngrams command line option to load a file with targets
	// 0.58		080904	generation of RFX properly adds nominals from GRE LF
	// 0.59		080904	utterance planning with q-ex-prop at any nomvar
	// 0.60		080904	adjoining fully works
	// 0.61		080906	reload command 
	// 0.62		080980	reload also includes reloading the model redux
	
	private final String whatsNew = " 0.62 -- reloading includes reloading the model redux";
	
    private final String version = "0.62";

	private String ngramsCorpus; 

	private Vector modelRedux; 

	private NgramScorer ngramScorer; 
	
	private String reduxfile;
	
    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables
     */

    public UPDebugger () {
		init();
    } // end constructor


	
	private void init () { 
		modelRedux = new Vector();
		ngramsCorpus = null;
		ngramScorer = null;
	} // end init


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     * 
     */

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

    public String realizeLF (Realizer realizer, LogicalForm logform) { 
			
		String output = "";
		LogicalForm reduxLF = applyModelReduction(logform);
		
		//LF lf = transformLF(realizer,reduxLF);
		LF lf = LFUtils.convertToLF(reduxLF);
		opennlp.ccg.realize.Chart chart = null;
		if (ngramScorer == null) { 
			realizer.realize(lf);
			chart = realizer.getChart();
		} else {
			realizer.realize(lf,ngramScorer);
			chart = realizer.getChart();			
		} 
		// chart.printBestEdge();

		List bestEdges = chart.bestEdges();

		for (Iterator beIter = bestEdges.iterator(); beIter.hasNext(); ) {
			opennlp.ccg.realize.Edge edge = 
				(opennlp.ccg.realize.Edge) beIter.next(); 
			// Sign sign = edge.getSign();
			output = output+edge.toString()+"\n";
		} // end for over best edges
		System.out.println();
		msg("Realization(s), best-first:");
		System.out.println();	
		msg(output); 
		return output;
    } // end realizeLF




    public LogicalForm solvePlanGRE (LFNominal nom) { 
		LogicalForm lf = LFUtils.newLogicalForm(); 
		
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
	
    //=================================================================
    // I/O METHODS
    //=================================================================


	public LogicalForm applyModelReduction (LogicalForm lf) { 
		// log("Root nominal: \n"+LFUtils.lfNominalToString(lf.root));
		// Create a new result
		LogicalForm result = LFUtils.newLogicalForm();
		result.noms = null;
		// Cycle over the nominals in the given lf
		for (Iterator<LFNominal> nomsIter = LFUtils.lfGetNominals(lf); nomsIter.hasNext(); ) { 
			// Get the next nominal
			LFNominal nom = nomsIter.next();
			// log("Reducing nominal: \n"+LFUtils.lfNominalToString(nom));

			LFNominal newNom = LFUtils.lfNominalClone(nom);
			// Reset the features in the cloned nominal
			newNom.feats = new org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature[0];
			// Cycle over the features in the nominal
			for (Iterator<org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
				org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature feat = featsIter.next();
				if (modelRedux.contains(feat.feat)) { 
					// do sweet bugger all, this is one to delete
					// log("Removing feature from nominal: ["+feat.feat+"]");
				} else { 
					// log("Adding feature from nominal: ["+feat.feat+"]");
					newNom.feats = LFUtils.lfNominalAddFeature(newNom,feat);
				} // end if.. check whether to delete
			} // end for over features
			// Finally, add the updated nominal
			// log("Reduced nominal: \n"+LFUtils.lfNominalToString(newNom));
			result.noms = LFUtils.lfAddNominal(result,newNom);
			// log("Updated reduced LF: "+LFUtils.lfToString(result));
		} // end for over nominals
		// log("End for over nominals for applying model reduction");
		result.noms = LFUtils.lfRemoveNominal(result.noms,"");
		LFNominal resultRoot = LFUtils.lfGetNominal(result,lf.root.nomVar);
		result.root = resultRoot;
		log("Final reduced LF: "+LFUtils.lfToString(result));
		System.out.print("\n Final reduced LF: \n"+LFUtils.lfToString(result));
		return result;
	} // end applyModelReduction

	private void loadModelRedux (String filename) { 
        Scanner s = null;
        try {
            s = new Scanner(new BufferedReader(new FileReader(filename)));
			System.out.print("Redux features: [");
            while (s.hasNext()) { 
				String redux = s.next();
				if (s.hasNext()) {
					System.out.print(redux+", ");
				} else { 
					System.out.println(redux+"]");					
				} 
				modelRedux.add(redux); } 
		} catch (IOException e) { 
			System.out.println("Error while loading model reduction file:\n"+e.getMessage());
        } finally {
            if (s != null) { s.close(); } 
        } // end try..finally
    } // and loadModelReduction

	private String[] loadCorpus (String filename) { 
		System.out.println("Loading an ngram corpus file from ["+filename+"]");
        Scanner s = null;
		Vector<String> targets = new Vector<String>();
        try {
            s = new Scanner(new BufferedReader(new FileReader(filename)));
            while (s.hasNext()) { 
				String redux = s.next();
				// System.out.println("Corpus: ["+redux+"]");
				targets.add(redux); } 
		} catch (IOException e) { 
			System.out.println("Error while loading a corpus file for constructing ngrams:\n"+e.getMessage());
        } finally {
            if (s != null) { s.close(); } 
        } // end try..finally
		String[] result = new String[targets.size()];
		int i = 0;
		for (String target : targets) { 
			result[i] = target;
			i++;
		} // end for
		return result;
	} // end loadCorpus

	/** 
	Shows the loaded grammar network
	*/ 

	public void showGrammar (UtterancePlanner up) { 
		System.out.println("Loaded systems:\n");
		for (Iterator<UPGSystem> sysIter = up.getSystems(); sysIter.hasNext(); ) { 
			UPGSystem sys = sysIter.next();
			System.out.println(sys.toString());
			msg("\n");
		} // 
		for (Iterator<UPGChooser> chsIter = up.getChoosers(); chsIter.hasNext(); ) { 
			UPGChooser chooser = chsIter.next();
			System.out.println(chooser.toString()); 
			msg("\n");
		} // end for over choosers
	} // 


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
	String usage = "---------------------------------------\n"+
					"Utterance Planner Debugger v."+version+"\n"
	                        +"Command line options:\n"
	                        +"-help, -usage: this message\n"
	                        +"-grammar <file>: load the planning grammar <file>\n"+
					"---------------------------------------\n";

	// Default the grammar file to grammar.xml in the current working directory
	String curDir = System.getProperty("user.dir");
	String grammarfile = curDir+"/grammar.xml";

	// Set the ccg grammar file and the realizer to null
	String ccgfile = null; 
	Realizer realizer = null;
	
	// Set the model reduction file
	 reduxfile = null;

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
			if (args[i].equals("-redux")) { reduxfile=args[++i]; }
			if (args[i].equals("-ngrams")) { ngramsCorpus=args[++i]; }
	    } // end for
	} // end if

	// Print header 
		System.out.println("====================================================================================");
		System.out.println("\nUtterance Planner Debugger v."+version+"\n");
		System.out.println("------------------------------------------------------------------------------------");		
		System.out.println("What's new:\n "+whatsNew);
		System.out.println("------------------------------------------------------------------------------------");		
       	System.out.println("Loading the planning grammar from ["+grammarfile+"]");

	// Initialize the planner with the grammarfile 

	UtterancePlanner planner = new UtterancePlanner(grammarfile,this);

	// Initialize the realizer with the CCG grammar, if there is one specified
	if (ccgfile != null) { 
	    try { 
			if (ccgfile.indexOf("grammar.xml") == -1) { ccgfile = ccgfile+"/grammar.xml"; }
			URL grammarURL = new File(ccgfile).toURL();
			System.out.println("Loading a CCG grammar from URL: " + grammarURL);
			Grammar grammar = new Grammar(grammarURL);
			realizer = new Realizer(grammar);	
			if (ngramsCorpus != null) { 
				String[] targets = loadCorpus(ngramsCorpus);
				ngramScorer = new NgramPrecisionModel(targets);
			} 
		} catch (IOException e) {
			error(e.getMessage());
	    }
	} // end if check whether there is already a CCG grammar


	if (reduxfile != null) { 
		System.out.println("Loading a model redux file from: ["+reduxfile+"]");
		loadModelRedux(reduxfile);
	} // end if.. 
	System.out.println("====================================================================================");

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
				grammarfile = filename;
				planner.load(grammarfile);
		    } else {
				error("':load' requires a filename including full path specification.");			
		    } // end if..else check for path
		} else if (input.startsWith(":rl")) {
			System.out.println("Reloading grammar from ["+grammarfile+"]");
			planner = new UtterancePlanner(grammarfile,this);
			System.out.println("Reloading model redux from ["+reduxfile+"]");
			loadModelRedux(reduxfile);
		} else if (input.startsWith(":plan")||input.startsWith(":p ")) { 
		    int atop = input.indexOf("@");
		    if (atop > -1) { 
			String lfstr = input.substring(atop,input.length());
			LogicalForm lf = LFUtils.convertFromString(lfstr);
			
			System.out.println("Planning with the following LF: \n"+LFUtils.lfToString(lf));
			
			LogicalForm planlf = planner.plan(lf);
			lastLF = planlf;
			System.out.println();
			System.out.println("Resulting logical form:");
			msg("");
			System.out.println(LFUtils.lfToString(planlf));
					
		//	printInfo(planlf);
			
		//	planlf = GenerationUtils.removeDuplicateNominalsAndFeatures(planlf);
			
		//	System.out.println("AFTER REMOVAL: " + LFUtils.lfToString(planlf));
			
		//	printInfo(planlf);
			
			msg("");
		    } else {
			error("':plan' requires a communicative goal, specified as HLDS logical form.");
		    } // end if..else check for lf 
		} else if (input.startsWith(":realize")||input.startsWith(":r")) { 
		    if (realizer != null) { 
			if (lastLF != null) { 
				if (input.indexOf("@") == -1) { 
					realizeLF(realizer,lastLF); 
				} else { 
					String rootNomVar = input.substring(input.indexOf("@")+1,input.length());
					msg("\nRealizing subgraph logical form with root ["+rootNomVar+"]:");
					LFNominal root = LFUtils.lfGetNominal(lastLF,rootNomVar);
					if (root != null) { 
						LogicalForm subLF = LFUtils.lfConstructSubtree(root,lastLF);
						if (subLF != null) { 
							msg(LFUtils.lfToString(subLF)); 
							realizeLF(realizer,subLF);
						} // end check for non-null subtree
					} // end check for non-null root
				}
				
			} else { 
			    error("Cannot realize a logical form: no logical form has been planned yet.");
			} // end if..else check for logical form
		    } else {
			error("The realizer has not yet been initialized with a grammar -- use :ccg to load a CCG grammar.");
		    } // end if..else check for realizer
		} else if (input.equals(":showgrammar") || input.equals(":sg")) {  
		    showGrammar(planner); 
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

    private static void printInfo(LogicalForm planlf) {
		System.out.println("number of nominals: " + planlf.noms.length);
		System.out.println("root of lf: " + planlf.root.nomVar);
		for (int i=0; i < planlf.noms.length; i++) {
			LFNominal nom = planlf.noms[i];
			System.out.println("nominal: " + nom.nomVar);
			System.out.println("prop: " + nom.prop.prop);
			System.out.println("sort: " + nom.sort);
			System.out.println("number of features: " + nom.feats.length);
			for (int j=0; j < nom.feats.length; j++) {
				org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature feat = nom.feats[j];
				System.out.println("feature: " + feat.feat + "=" + feat.value);
			}
			for (int j=0; j < nom.rels.length; j++) {
				LFRelation rel = nom.rels[j];
				System.out.println("relation: " + rel.head + "->" + rel.dep + " (label=" + rel.mode + ")");
			}
			System.out.println("----");
		}

    }
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
		System.out.println(":rl(oad)        \t loads the planning grammar specified on the command line");		
	System.out.println(":p(lan) <comm.goal> \t plans a logical form for the communicative goal (also a logical form)");
	System.out.println(":r(ealize)          \t realizes the last planned logical form (if any), provided the realizer has been initialized");
	System.out.println(":r(ealize) @<nomvar>\t realizes the logical form subgraph with root [nomvar], provided there is a last planned logical form and the realizer has been initialized");
	System.out.println(":sg					\t show the systems in the grammar network"); 
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
