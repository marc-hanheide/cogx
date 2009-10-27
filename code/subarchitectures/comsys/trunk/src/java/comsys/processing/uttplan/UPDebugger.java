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
	
	private Vector modelRedux;
	private NgramScorer ngramScorer = null;

	private Grammar grammar; 
	
	
	private final static String CANNED_TEXT = "CannedText";
	private final static String CONTENT_BODY = "Content";
	
	
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
    

/** applyModelReduction removes parts of the LF unknown to the realization grammar
  * copied over from cc_ContentPlanning.java
 */
	
	private LogicalForm applyModelReduction (LogicalForm lf) { 
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
			newNom.feats = new comsys.datastructs.lf.Feature[0];
			// Cycle over the features in the nominal
			for (Iterator<comsys.datastructs.lf.Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
				comsys.datastructs.lf.Feature feat = featsIter.next();
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
		// log("Final reduced LF: "+LFUtils.lfToString(result));
		return result;
	} // end applyModelReduction
	
	
	
    public String realizeLF (Realizer realizer, LogicalForm logicalForm) {
		String output = "";
		if (LFUtils.lfNominalHasFeature(logicalForm.root,CANNED_TEXT)) { 
			String cannedTextKey = LFUtils.lfNominalGetFeature(logicalForm.root,CANNED_TEXT);
			log("IKK: key: "+cannedTextKey+"\n");
			output = realizeLfCannedText(cannedTextKey);			
		} else { 
			LogicalForm planLF = logicalForm;
			String contentRoot = logicalForm.root.nomVar;
			if (LFUtils.lfNominalHasRelation(logicalForm.root, CONTENT_BODY)) { 
				LFRelation contentR = LFUtils.lfNominalGetRelation(logicalForm.root, CONTENT_BODY);
				contentRoot = contentR.dep;
				LFNominal contentRootNom = LFUtils.lfGetNominal(logicalForm, contentRoot);
				planLF = LFUtils.lfConstructSubtree(contentRootNom,logicalForm);	
			} // end if.. check for embedded content body
			output = realizeLfWithGrammar(realizer,planLF,contentRoot);
		} // end if..else check for canned text or content body
		log("Output realization: " + output);
		// Assign a default in case realization failed
		if ( output == "" || output == " ") {
			output = "I am sorry I am lost for words on this one"; 
		} 
		return output;
	} // end realizeLF
		
		/** Produces a string (hopefully sentence) for a given LF.
			Extracts the root nominal of the LF under contentRoot in logicalForm, and the corresponding tree, 
		    then calls the OpenCCG realizer on this
		 
		 @param realizer the OpenCCG realizer
		 @param logicalForm the LF to realize from
		 @param contentRoot the root of the subtree to realize
		 @return the realized string 
		 */
	
	public String realizeLfWithGrammar (Realizer realizer, LogicalForm planLF, String contentRoot) {
			
	log("Planning a realization for the following logical form: \n"+LFUtils.lfToString(planLF)+"\n");
			
	// Translate the planLF logical form into XML for OpenCCG
	LF lf = LFUtils.convertToLF(planLF);
	// Dump the LF in the XML format to a file
		String curDir = System.getProperty("user.dir");
/*
		String dumpLFfile =  curDir+"/subarchitectures/comsys/grammars/contentPlanning/dumpedLF.xml";
		try { 
			grammar.saveToXml(lf, "", dumpLFfile);
		} catch (Exception ie) { 
			ie.printStackTrace();
		} 
		log("Wrote LF to \"" + dumpLFfile + "\"");
*/	
		log("Calling realizer for the following logical form: \n"+lf.toString()+"\n");
		
	// Realize the XML-based logical form
	if (ngramScorer == null) { 
		realizer.realize(lf); log("Called realizer without ngrams");
	} else {
		realizer.realize(lf,ngramScorer); log("Called realizer with ngrams");
	}

		opennlp.ccg.realize.Chart chart = realizer.getChart();
		log("Chart has "+chart.numEdgesInChart()+" representative edges.");

		// list best edges according to OpenCCG
		List<opennlp.ccg.realize.Edge> bestEdges = chart.bestEdges();	
		String output = "";
		for (Iterator beIter = bestEdges.iterator(); beIter.hasNext(); ) {
			//  log("entered iterator");
			opennlp.ccg.realize.Edge edge = (opennlp.ccg.realize.Edge) beIter.next(); 
			// Sign sign = edge.getSign();
			// output = output+sign.toString()+"\n";
			output = output+edge.toString()+"\n";
			// msg(output);
			// log("Iteration number: "+bestcounter++); 
		} // end for over best edges
		log("Best edges according to OpenCCG:\n"+output+"\n");
		
		/* // needed when debuging realization: 	
		 log("These are all the edges: ");
		 chart.printEdges();
		 log("Number of best edges handed over from chart: "+chart.bestEdges().size()+"\n");
		 */

		// retrieve the one edge with best scoring (even if incomplete)
		opennlp.ccg.realize.Edge bestEdge = chart.bestEdge; 
		log("Best edge: ");
		// chart.printBestEdge();
		if (bestEdge != null) {
			log(bestEdge.toString());	// log("Grammar output realization: "+ bestEdge.getSign().getOrthography().toString());
			output =bestEdge.getSign().getOrthography().toString();
		} else {
			output = "";
		}

		String dumpLFfile =  curDir+"/subarchitectures/comsys/grammars/contentPlanning/dumpedLF.xml";
		try { 
			grammar.saveToXml(lf, output, dumpLFfile);
		} catch (Exception ie) { 
			ie.printStackTrace();
		} 
		log("Wrote LF to \"" + dumpLFfile + "\"");
		
		return output.replace('_',' ');  // replace underscores because of thank_you
    } // end realizeLfWithGrammar

	
	/**  Produces a canned text string by replacing every occurrence of "_" with a space. 

	 @param cannedTextKey the string that specifies the canned text
	 @return the canned text where "_" is replaced with " "
	 
	 */
	
	public String realizeLfCannedText (String cannedTextKey) {
		return cannedTextKey.replace('_',' ');
	}
	
	
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
	 *
	 * had problems --> removed
	 * 
	 * In new comsys, this code has gone into LFUtils.convertToLF so we use that now
	 */ 
   
 
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
	                        +"-grammar <file>: load the planning grammar <file>\n"
							+"-redux <file>: load <fiile> listing the LF features not recognized by the realizer \n"
							+"-ccg <file>: load the realizer grammar <file>\n"
							+"-ngrams <file>: load the ngrams corpus <file>\n";
		
	// Default the contnt planning grammar file to grammar.xml in the current working directory
	String curDir = System.getProperty("user.dir");
	String grammarfile = curDir+"/subarchitectures/comsys/grammars/contentPlanning/grammar.xml";

	// Set default model redux
	String reduxfile = curDir+"/subarchitectures/comsys/grammars/contentPlanning/modelredux.rdx";

	// Default the realizer grammar to moloko6
	String ccgfile = curDir+"/subarchitectures/comsys/grammars/openccg/moloko.v6/grammar.xml"; 
	Realizer realizer = null;

	// Default ngrams corpus
		String ngramsfile = null;
		
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
		if (args[i].equals("-ngrams")) { ngramsfile=args[++i]; }
	    } // end for
	} // end if

	// Print header 
	System.out.println("Utterance Planner Debugger v."+version);
	System.out.println();
       	System.out.println("Loading the planning grammar from ["+grammarfile+"]");

	// Initialize the planner with the grammarfile 

	UtterancePlanner planner = new UtterancePlanner(grammarfile,this);

	// Handle model reduction
	modelRedux = new Vector();
	if (reduxfile != null) { 
		log("Loading a model redux file from: ["+reduxfile+"]");
		loadModelRedux(reduxfile);
	} // end if.. 	
		

	// Initialize the realizer with the CCG grammar, if there is one specified
	if (ccgfile != null) { 
	    try { 
		URL grammarURL = new File(ccgfile).toURL();
		System.out.println("Loading a CCG grammar from ["+grammarURL+"]");
		grammar = new Grammar(grammarURL);
		realizer = new Realizer(grammar);	
		ngramScorer = null;
	    } catch (IOException e) {
		error(e.getMessage());
	    }
	} // end if check whether there is already a CCG grammar
	
		if (ngramsfile != null) {
			// log("Loading ngram corpus from: ["+ngramsfile+"]");
			String[] targets = loadCorpus(ngramsfile);
			ngramScorer = new NgramPrecisionModel(targets);
		}
		
	// Now start up the interface
	try { 
	    reader = new ConsoleReader (); 
	    LogicalForm lastLF = null;

	    // store commands for 'tab' argument completion                                                                      
	    List completors = new LinkedList();
	    completors.add(new SimpleCompletor(new String[]
		{ ":ccg", ":help", ":load", ":plan", ":quit", ":traceoff", ":tracelvl", ":traceon", ":test"}));
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

	    planner.setLogging(true);

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
			grammar = new Grammar(grammarURL);
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
				// Apply model reduction
				lastLF = applyModelReduction(planlf);
				// old: lastLF = planlf;
				//System.out.println();
				log("Resulting logical form: \n"+LFUtils.lfToString(planlf)+"\n");
				log("Resulting logical form after reduction:\n"+LFUtils.lfToString(lastLF)+"\n");
		    } else {
				error("':plan' requires a communicative goal, specified as HLDS logical form.");
		    } // end if..else check for lf 
		} else if (input.startsWith(":test")) { 
			String lfstr = "@d1:dvp(c-goal ^ <SpeechAct>assertion ^ <Relation>answer ^ <Content>(b1:physical ^ ball)";
			log("Testing with LF: "+lfstr); 
			LogicalForm lf = LFUtils.convertFromString(lfstr);
			LogicalForm planlf = planner.plan(lf);
			// Apply model reduction
			lastLF = applyModelReduction(planlf);
			// old: lastLF = planlf;
			log("Resulting logical form:\n"+LFUtils.lfToString(planlf)+"\n");
			log("Resulting logical form after reduction:\n"+LFUtils.lfToString(lastLF)+"\n");
		if (realizer != null) { 
				if (lastLF != null) { 
					realizeLF(realizer,lastLF); 
				} else { 
					error("Cannot realize a logical form: no logical form has been planned yet.");
				} // end if..else check for logical form
		    } else {
				error("The realizer has not yet been initialized with a grammar -- use :ccg to load a CCG grammar.");
		    } // end if..else check for realizer
			// end if..else check for test
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
		} else if (input.startsWith(":whocan")) { 
			String argsString = input.substring(6, input.length());
			StringTokenizer st = new StringTokenizer(argsString);
			Vector argsV = new Vector();
			while (st.hasMoreTokens()) { 
				String token = st.nextToken();
				argsV.add(token);
			} // end while
			HashMap results = checkWhoCan(argsV,planner);
			for (Iterator keysIter = results.keySet().iterator(); keysIter.hasNext(); ) { 
				String systemId = (String)keysIter.next();
				String action = (String)results.get(systemId);
				System.out.println("System ["+systemId+"] can under action with choice ["+action+"]");
			} // end for 
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
			log("Done. \n \n \n");
	    } // end while

	} catch (IOException e) { 
	    System.out.println(e.getMessage());
	} // end try.catch
	

    } // end 

	
	/**
	  *loadModelredux loads a list of features CCG does not understand from a file;
	  * copied from cc_ContentPlanningloading  
	  */ 

	private void loadModelRedux (String filename) { 
		Scanner s = null;
		try {
			s = new Scanner(new BufferedReader(new FileReader(filename)));
			while (s.hasNext()) { 
				String redux = s.next();
				log("Redux: ["+redux+"]");
			modelRedux.add(redux); } 
		} catch (IOException e) { 
			log("Error while loading model reduction file:\n"+e.getMessage());
		} finally {
			if (s != null) { s.close(); } 
		} // end try..finally
	} // and loadModelReduction		

	
	/**
	  * loadCorpus loads an ngram corpus for CCG from a file; 
	  * copied from cc_realizer 
	  */
	
	private String[] loadCorpus (String filename) { 
		log("Loading ngram corpus from file ["+filename+"]");
		Scanner s = null;
		Vector<String> targets = new Vector<String>();
		try {
			s = new Scanner(new BufferedReader(new FileReader(filename)));
			while (s.hasNext()) { 
				String redux = s.next();
			targets.add(redux); } 
		} catch (IOException e) { 
			System.out.println("Error while loading model reduction file:\n"+e.getMessage());
		} finally {
			if (s != null) { s.close(); } 
		} // end try..finally
		String[] result = new String[targets.size()];
		int i = 0;
		for (String target : targets) { 
			result[i] = target;
			i++;
		} // end for
		log("The corpus contains ["+i+"] items");
		return result;
	} // end loadCorpus		
	
		
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

	
	/** checks which systems can perform a certain action, possibly with a certain argument. 
	does not compile  */
	
	private HashMap checkWhoCan (Vector args, UtterancePlanner planner) { 
		HashMap results = new HashMap();
		if (args.size() > 0) { 
			String operand = (String)args.elementAt(0); 
			String argument = null; 
			if (args.size() > 1) { 
				argument = (String)args.elementAt(1);
			} // end if.. check for argument
			for (Iterator systemsIter = planner.getSystems(); systemsIter.hasNext(); ) { 
				UPGSystem system = (UPGSystem) systemsIter.next(); 
				for (Iterator actionsIter = system.getActions(); actionsIter.hasNext(); ) { 
					UPGAction action = (UPGAction) actionsIter.next();
					Vector steps = action.getSteps();
					for (Iterator stepsIter = steps.iterator(); stepsIter.hasNext(); ) { 
						UPGActionStep step = (UPGActionStep) stepsIter.next();
						if (step.getId().equals(operand)) {
							if (argument != null) { 
								if (operand.equals("assign-type")) { 
									String type = step.getAttributeValue("type"); 
									if (type.equals(argument)) { 
										results.put(system.getId(), action.getChoice()); 
									} // end if .. check for type value
								} else if (operand.equals("add-feature")) { 
									String feature = step.getAttributeValue("feature"); 
									if (feature.equals(argument)) { 
										results.put(system.getId(), action.getChoice()); 										
									} // end if .. check for feature value											   
								} else if (operand.equals("add-proposition")) { 
									String propositions = step.getAttributeValue("propositions"); 
									if (propositions.startsWith(argument)) { 
										results.put(system.getId(), action.getChoice()); 										
									} // end if .. check for propositions value										
								} else if (operand.equals("add-relation")) { 
									String relation = step.getAttributeValue("mode"); 
									if (relation.equals(argument)) { 
										results.put(system.getId(), action.getChoice()); 										
									} // end if .. check for mode value	
								} else if (operand.equals("replace-relation")) { 									
									String mode = step.getAttributeValue("mode"); 
									if (mode.equals(argument)) { 
										results.put(system.getId(), action.getChoice()); 										
									} // end if .. check for feature value	
								} else { 
									
								} // end if.. check for specific type 
							} // end check whether argument
						} // end check whether operand equals step
					} // end for over steps
				} // end for over actions
			} // end for over systems
		} else { 
			System.out.println(":whocan requires at least one argument");
		} // end if..else
		return results;
	} // end method
	
	
	
	
	
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
