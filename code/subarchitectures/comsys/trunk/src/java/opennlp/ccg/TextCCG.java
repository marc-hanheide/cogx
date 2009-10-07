///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 Jason Baldridge, Gann Bierner, 
//                      University of Edinburgh (Michael White), 
//                      Alexandros Triantafyllidis and David Reitter
// Copyright (C) 2006 Ben Wing
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

package opennlp.ccg;

import opennlp.ccg.lexicon.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.parse.*;
import opennlp.ccg.util.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.ngrams.*;
import opennlp.ccg.test.*;
import opennlp.ccg.realize.Edge; // only realization edges referenced (for preferences)

import org.jdom.*;

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.prefs.*;

/**
 * A text interface for testing grammars.
 *
 * @author  Jason Baldridge
 * @author  Gann Bierner
 * @author  Michael White
 * @author  Alexandros Triantafyllidis
 * @author  David Reitter
 * @version $Revision: 1.54 $, $Date: 2007/12/20 05:51:10 $
 */
public class TextCCG {
    
    /** Preference key for showing all results. */
    public static final String SHOW_ALL_RESULTS = "Show All Results";
    
    /** Preference key for showing derivations. */
    public static final String SHOW_DERIVATIONS = "Show Derivations";
    
    /** Preference key for showing features. */
    public static final String SHOW_FEATURES = "Show Features";
    
    /** Preference key for showing semantics. */
    public static final String SHOW_SEMANTICS = "Show Semantics";
    
    /** Preference key for showing features. */
    public static final String FEATURES_TO_SHOW = "Features to Show";
    
    /** Preference key for showing realizer timing. */
    public static final String SHOW_TIMING = "Show Timing";
    
    /** Preference key for showing incomplete edges during realization. */
    public static final String SHOW_INCOMPLETE_EDGES = "Show Incomplete Edges";
    
    /** Preference key for visualizing a derivation. */
    public static final String VISUALIZE = "Visualize";

    /** Preference key for command line history. */
    public static final String HISTORY = "Command Line History";	
	
    
	public static String ASRCORRECTION_RULES = 
		"Apply ASR lexical correction rules";
	public static String DISCLEVELCOMPOSITION_RULES = 
		"Apply discourse level composition rules";
	public static String DISFLCORRECTION_RULES = "Apply disfluency correction rules";
	
	
    public static String SCORING = "Apply chart scoring";
    public static String TIMING = "Show timing information";
	public static String INCREMENTAL = "Use incremental parsing";
	
    /** Main method for tccg. */
    public static void main(String[] args) throws IOException, LexException { 

        String usage = "java opennlp.ccg.TextCCG " + 
                       "(<grammarfile>) | (-exportprefs <prefsfile>) | (-importprefs <prefsfile>)";

        if (args.length > 0 && args[0].equals("-h")) {
            System.out.println("Usage: " + usage);
            System.exit(0);
        }

        // args        
        String grammarfile = "grammar.xml";
        String prefsfile = null;
        boolean exportPrefs = false;
        boolean importPrefs = false;
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-exportprefs")) {
                exportPrefs = true; prefsfile = args[++i]; continue; 
            }
            if (args[i].equals("-importprefs")) {
                importPrefs = true; prefsfile = args[++i]; continue; 
            }
            grammarfile = args[i];
        }

        // prefs
        Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
        try {
            if (exportPrefs) {
                System.out.println("Exporting preferences to prefsfile: " + prefsfile);
                prefs.exportNode(new FileOutputStream(prefsfile));
                System.exit(0);
            }
            if (importPrefs) {
                System.out.println("Importing preferences from prefsfile: " + prefsfile);
                Preferences.importPreferences(new FileInputStream(prefsfile));
                System.exit(0);
            }
        } catch (Exception exc) {
            throw (IOException) new IOException().initCause(exc);
        }

        // load grammar
        URL grammarURL = new File(grammarfile).toURL();
        System.out.println("Loading grammar from URL: " + grammarURL);
        Grammar grammar = new Grammar(grammarURL);
		
		if (grammar.getName() != null)
			System.out.println("Grammar '" + grammar.getName() + "' loaded.");
        System.out.println();

        // create parser and realizer
        IncrCKYParser parser = new IncrCKYParser(grammar);
        Realizer realizer = new Realizer(grammar);
        IncrCKYParser incrparser = new IncrCKYParser(grammar);

        // stuff to remember during loop
        Sign[] lastResults = null;
        LF[] lastLFs = null;
        String lastSentence = "";
        int lastReading = 0;
        
		// prepare to accept input from user 
        String[] completions = 
            { ":sh", ":v", ":reset", ":feats", ":nofeats", ":foff", ":sem", ":nosem", ":all", ":notall", 
              ":derivs", ":noderivs", ":doff", ":vison", ":visoff", 
              ":r", ":sel", ":2xml", ":2tb", ":2apml", ":tl", ":notl",
              ":nbtl", ":nonbtl", ":pv", ":nopv", ":upon", ":upoff", ":t", ":toff", ":inc", ":noinc",
              ":ion", ":ioff", ":con", ":coff", ":flon", ":floff", ":ccon", ":ccoff", ":pon", ":poff",
              ":q", ":h", ":scoring", ":noscoring", ":asrcorrection", ":noasrcorrection", 
              ":disclevelcomp", ":nodisclevelcomp", ":showtiming", ":noshowtiming", "incr", "noincr", 
              ":disflcorrection", ":nodisflcorrection" };
        
		LineReader lineReader = LineReader.createLineReader(completions);
		
		// initialize history, per grammar, from prefs
		String historyKey = HISTORY + "_" + grammar.getName();
		String histStr = prefs.get(historyKey, "");
        lineReader.setCommandHistory(histStr);

        // welcome msg
        System.out.println("Enter strings to parse.");
        System.out.println("Type ':r' to realize selected reading of previous parse.");
        System.out.println("Type ':h' for help on display options and ':q' to quit.");
        System.out.println("You can use the tab key for command completion, ");
        System.out.println("Ctrl-P (prev) and Ctrl-N (next) to access the command history, ");
        System.out.println("and emacs-style control keys to edit the line.");
        System.out.println();
        
        while (true) {
        
			String input = lineReader.readLine("tccg> ");
			if (input == null) break; // control-D or the like
            input = input.trim();			
			if (input.equals(":show settings") || input.equals(":sh")) {
                showSettings(prefs);
            } else if (input.equals(":v")) {
                prefs.putBoolean(SHOW_ALL_RESULTS, true);
                prefs.putBoolean(SHOW_DERIVATIONS, true);
                prefs.putBoolean(SHOW_FEATURES, true);
                prefs.putBoolean(SHOW_SEMANTICS, true);
                prefs.put(FEATURES_TO_SHOW, "");
            } else if (input.equals(":q")) {
                break; // end of while loop
            } else if (input.equals(":h")) {
                showHelp();
            } else if (input.equals(":reset")) {
                prefs.putBoolean(SHOW_ALL_RESULTS, false);
                prefs.putBoolean(SHOW_DERIVATIONS, false);
                prefs.putBoolean(SHOW_TIMING, false);
                prefs.putBoolean(SHOW_INCOMPLETE_EDGES, false);
                prefs.putBoolean(Edge.SHOW_COMPLETENESS, false);
                prefs.putBoolean(Edge.SHOW_BITSET, false);
                prefs.putBoolean(SHOW_FEATURES, false);
                prefs.putBoolean(SHOW_SEMANTICS, false);
                prefs.put(FEATURES_TO_SHOW, "");
                prefs.putBoolean(VISUALIZE, false);     
                prefs.put("VISFNAME", "");
                prefs.putBoolean(EdgeFactory.USE_INDEXING, true);
                prefs.putBoolean(EdgeFactory.USE_CHUNKS, true);
                prefs.putBoolean(EdgeFactory.USE_FEATURE_LICENSING, true);
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_COMBOS, true);
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_PACKING, false);
                prefs.putInt(opennlp.ccg.realize.Chart.TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
                prefs.putDouble(opennlp.ccg.realize.Chart.NEW_BEST_TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
                prefs.putInt(opennlp.ccg.realize.Chart.PRUNING_VALUE, opennlp.ccg.realize.Chart.NO_PRUNING);
                prefs.putBoolean(opennlp.ccg.realize.Chart.DO_UNPACKING, true);
            } else if (input.equals(":show feats") || input.equals(":feats") || input.equals(":f")) {
                prefs.putBoolean(SHOW_FEATURES, true);
                prefs.put(FEATURES_TO_SHOW, "");
            } else if (input.startsWith(":show feats ") || input.startsWith(":feats ") || input.startsWith(":f ")) {
                prefs.putBoolean(SHOW_FEATURES, true);
                String s = input.substring(input.indexOf(' ') + 1);
                if (s.startsWith("feats ")) { s = s.substring(6); }
                prefs.put(FEATURES_TO_SHOW, s);
            } else if (input.equals(":nofeats") || input.equals(":foff")) {
                prefs.putBoolean(SHOW_FEATURES, false);
                prefs.put(FEATURES_TO_SHOW, "");
            } else if (input.equals(":show semantics") || input.equals(":sem") || input.equals(":s")) {
                prefs.putBoolean(SHOW_SEMANTICS, true);
            } else if (input.equals(":nosem") || input.equals(":soff")) {
                prefs.putBoolean(SHOW_SEMANTICS, false);
            } else if (input.equals(":show all") || input.equals(":all") || input.equals(":a")) {
                prefs.putBoolean(SHOW_ALL_RESULTS, true);
            } else if (input.equals(":notall") || input.equals(":aoff")) {
                prefs.putBoolean(SHOW_ALL_RESULTS, false);
            } else if (input.equals(":show derivs") || input.equals(":derivs") || input.equals(":d")) {
                prefs.putBoolean(SHOW_DERIVATIONS, true);
            } else if (input.equals(":noderivs") || input.equals(":doff")) {
                prefs.putBoolean(SHOW_DERIVATIONS, false);
                
            } 
            
            else if (input.equals(":scoring")) {
            	prefs.putBoolean(SCORING, true);
            }
            else if (input.equals(":noscoring")) {
            	prefs.putBoolean(SCORING, false);
            } 
            
            else if (input.equals(":asrcorrection")) {
            	prefs.putBoolean(ASRCORRECTION_RULES, true);
            } else if (input.equals(":noasrcorrection")) {
            	prefs.putBoolean(ASRCORRECTION_RULES, false);
            } 
			
            else if (input.equals(":disclevelcomp")) {
            	prefs.putBoolean(DISCLEVELCOMPOSITION_RULES, true);
            } else if (input.equals(":nodisclevelcomp")) {
            	prefs.putBoolean(DISCLEVELCOMPOSITION_RULES, false);
            } 
			
            else if (input.equals(":incr")) {
            	prefs.putBoolean(INCREMENTAL, true);
            } else if (input.equals(":noincr")) {
            	prefs.putBoolean(INCREMENTAL, false);
            } 
			
            else if (input.equals(":showtiming")) {
            	prefs.putBoolean(TIMING, true);
            } else if (input.equals(":noshowtiming")) {
            	prefs.putBoolean(TIMING, false);
            } 
			
            else if (input.equals(":disflcorrection")) {
            	prefs.putBoolean(DISFLCORRECTION_RULES, true);
            } else if (input.equals(":nodisflcorrection")) {
            	prefs.putBoolean(DISFLCORRECTION_RULES, false);
            } 
			
            else if (input.startsWith(":time limit") || input.startsWith(":tl")) {
                String[] tokens = input.split("\\s+");
                String last = tokens[tokens.length-1];
                try {
                    int limit = Integer.parseInt(last);
                    prefs.putInt(opennlp.ccg.realize.Chart.TIME_LIMIT, limit);
                } catch (NumberFormatException exc) {
                    System.out.println("Expecting a time limit in ms, rather than: " + last);
                }
            } else if (input.startsWith(":no time limit") || input.startsWith(":notl")) {
                prefs.putInt(opennlp.ccg.realize.Chart.TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
            } else if (input.startsWith(":nbtl")) {
                String[] tokens = input.split("\\s+");
                String last = tokens[tokens.length-1];
                try {
                    double limit = Double.parseDouble(last);
                    prefs.putDouble(opennlp.ccg.realize.Chart.NEW_BEST_TIME_LIMIT, limit);
                } catch (NumberFormatException exc) {
                    System.out.println("Expecting a time limit in ms, rather than: " + last);
                }
            } else if (input.startsWith(":nonbtl")) {
                prefs.putDouble(opennlp.ccg.realize.Chart.NEW_BEST_TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
            } else if (input.startsWith(":pruning value") || input.startsWith(":pv")) {
                String[] tokens = input.split("\\s+");
                String last = tokens[tokens.length-1];
                try {
                    int val = Integer.parseInt(last);
                    prefs.putInt(opennlp.ccg.realize.Chart.PRUNING_VALUE, val);
                } catch (NumberFormatException exc) {
                    System.out.println("Expecting an integer pruning value, rather than: " + last);
                }
            } else if (input.startsWith(":no pruning value") || input.startsWith(":nopv")) {
                prefs.putInt(opennlp.ccg.realize.Chart.PRUNING_VALUE, opennlp.ccg.realize.Chart.NO_PRUNING);
            } else if (input.equals(":upon")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.DO_UNPACKING, true);
            } else if (input.equals(":upoff")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.DO_UNPACKING, false);
            } else if (input.startsWith(":select reading") || input.startsWith(":sel")) {
                String[] tokens = input.split("\\s+");
                String last = tokens[tokens.length-1];
                try {
                    int reading = Integer.parseInt(last);
                    if (reading > lastResults.length) {
                        System.out.println("Only " + lastResults.length + " parses found.");
                    } else if (lastLFs[reading-1] == null) {
                        System.out.println("LF not available, use :all command and reparse.");
                    } else {
                        lastReading = reading-1;
                    }
                } catch (NumberFormatException exc) {
                    System.out.println("Expecting a reading number, rather than: " + last);
                }
            } else if (input.equals(":timing on") || input.equals(":ton") || input.equals(":t")) {
                prefs.putBoolean(SHOW_TIMING, true);
            } else if (input.equals(":timing off") || input.equals(":toff")) {
                prefs.putBoolean(SHOW_TIMING, false);
            } else if (input.equals(":show incomplete") || input.equals(":inc")) {
                prefs.putBoolean(SHOW_INCOMPLETE_EDGES, true);
                prefs.putBoolean(Edge.SHOW_COMPLETENESS, true);
            } else if (input.equals(":noinc")) {
                prefs.putBoolean(SHOW_INCOMPLETE_EDGES, false);
                prefs.putBoolean(Edge.SHOW_COMPLETENESS, false);
            } else if (input.equals(":indexing on") || input.equals(":ion")) {
                prefs.putBoolean(EdgeFactory.USE_INDEXING, true);
            } else if (input.equals(":indexing off") || input.equals(":ioff")) {
                prefs.putBoolean(EdgeFactory.USE_INDEXING, false);
            } else if (input.equals(":chunks on") || input.equals(":con")) {
                prefs.putBoolean(EdgeFactory.USE_CHUNKS, true);
            } else if (input.equals(":chunks off") || input.equals(":coff")) {
                prefs.putBoolean(EdgeFactory.USE_CHUNKS, false);
            } else if (input.equals(":feature licensing on") || input.equals(":flon")) {
                prefs.putBoolean(EdgeFactory.USE_FEATURE_LICENSING, true);
            } else if (input.equals(":feature licensing off") || input.equals(":floff")) {
                prefs.putBoolean(EdgeFactory.USE_FEATURE_LICENSING, false);
            } else if (input.equals(":combos on") || input.equals(":ccon")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_COMBOS, true);
            } else if (input.equals(":combos off") || input.equals(":ccoff")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_COMBOS, false);
            } else if (input.equals(":pon")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_PACKING, true);
            } else if (input.equals(":poff")) {
                prefs.putBoolean(opennlp.ccg.realize.Chart.USE_PACKING, false);
            } else if (input.startsWith(":realize") || input.startsWith(":r")) {
                LF lf;
                NgramScorer ngramScorer;
                // nb: need to upgrade, consolidate :r FN option with Realize.java ...
                int space = input.indexOf(" ");
                if (space != -1) { // check for filename
                    String filename = readFilename(input.substring(space));
                    if (filename == null) {
                        System.out.println("Expecting a filename to read from.");
                        continue;
                    }
                    try {
                        Document doc = grammar.loadFromXml(filename);
                        lf = Realizer.getLfFromDoc(doc);
                        // nb: just handling explicit targets for now ...
                        List targetElts = doc.getRootElement().getChildren("target");
                        String[] targets = new String[targetElts.size()];
                        for (int i=0; i < targetElts.size(); i++) {
                            Element ex = (Element) targetElts.get(i);
                            String target = ex.getText();
                            targets[i] = target;
                        }
                        ngramScorer = new NgramPrecisionModel(targets);
                    }
                    catch (IOException exc) {
                        System.out.println("Unable to read: " + filename);
                        System.out.println(exc.toString());
                        continue;
                    }
                }
                else { // otherwise use last reading of last LF
                    if (lastLFs == null || lastLFs[lastReading] == null) {
                        System.out.println("Nothing to realize!");
                        continue;
                    }
                    lf = grammar.transformLF(lastLFs[lastReading]);
                    String[] targets = new String[1];
                    targets[0] = lastSentence;
                    ngramScorer = new NgramPrecisionModel(targets);
                }
                realizer.realize(lf, ngramScorer);
                opennlp.ccg.realize.Chart chart = realizer.getChart();
                boolean showIncompleteEdges = prefs.getBoolean(SHOW_INCOMPLETE_EDGES, false);
                boolean showTiming = prefs.getBoolean(SHOW_TIMING, false);
                if (showIncompleteEdges) chart.printEdges();
                else chart.printEdges(true, true);
                if (showTiming) { chart.printTiming(); }
            } else if (input.startsWith(":2xml")) {
                if (lastLFs == null || lastLFs[lastReading] == null) {
                    System.out.println("Nothing to save!");
                    continue;
                }
                String filename = readFilename(input.substring(5));
                if (filename == null) {
                    System.out.println("Expecting a filename to save to.");
                    continue;
                }
                grammar.saveToXml(lastLFs[lastReading], lastSentence, filename);
                System.out.println("Wrote LF to \"" + filename + "\"");
            } else if (input.startsWith(":2tb")) {
                if (lastLFs == null || lastLFs[lastReading] == null) {
                    System.out.println("Nothing to save!");
                    continue;
                }
                String filename = readFilename(input.substring(4));
                if (filename == null) { filename = "testbed.xml"; }
                RegressionInfo.addToTestbed(grammar, lastSentence, lastResults.length, lastLFs[lastReading], filename);
                System.out.println("Added test item to \"" + filename + "\"");
            } else if (input.startsWith(":2apml")) {
                if (lastSentence.length() == 0) {
                    System.out.println("Nothing to save!");
                    continue;
                }
                String filename = readFilename(input.substring(6));
                if (filename == null) {
                    System.out.println("Expecting a filename to save to.");
                    continue;
                }
                grammar.saveToApml(lastResults[lastReading], filename);
                System.out.println("Wrote \"" + lastSentence + "\" to \"" + filename + "\" as APML");
            } else if (input.startsWith(":vison")) {
                prefs.putBoolean(VISUALIZE, true);
                if ((input.startsWith(":vison ")) && (input.length( )>= 8)) {   
                    String fname = input.substring(7);
                    if (fname.lastIndexOf('.')!=-1) {  
                        System.out.println("Filename should not contain a suffix. Suffixes .tex and .dvi are assumed.");
                        prefs.put("VISFNAME", "");
                    } 
                    else
                        prefs.put("VISFNAME", fname);
                } 
                else
                    prefs.put("VISFNAME", "");
            } else if (input.equals(":visoff")) {
                prefs.putBoolean(VISUALIZE, false);     
                prefs.put("VISFNAME", "");
            } else { 
                try {
                    if (input.length() == 0) {
                        if (lastSentence.length() > 0) { 
                            input = lastSentence; 
                        } else {
                            System.out.println("Nothing to parse!");
                            continue;
                        }
                    }
                    
                    AbstractRule.asrcorrectionRules = 
                    	prefs.getBoolean(ASRCORRECTION_RULES, false);
                    AbstractRule.disclevelcompositionRules = 
                    	prefs.getBoolean(DISCLEVELCOMPOSITION_RULES, false);
                    AbstractRule.disfluencycorrectionRules = 
                    	prefs.getBoolean(DISFLCORRECTION_RULES, false);
                    
                    // the parsing operation itself
                    List<Sign> parses;
                    if (!prefs.getBoolean(INCREMENTAL, false)) {
                    	parser.parse(input);
                    	parses =  parser.getResult();
                    }
                    else {
                    	incrparser.incrParse(input);
                    	parses = incrparser.getResult();
                    }
              
                    Sign[] results = new Sign[parses.size()];
                    parses.toArray(results);
                    int resLength = results.length;
                    switch (resLength) {
                        case 0: break;
                        case 1: 
                            System.out.println(resLength + " parse found.\n"); 
                            break;
                        default: System.out.println(resLength + " parses found.\n"); 
                    }
                    
                    lastResults = results;
                    lastLFs = new LF[resLength];
                    if (input.length() > 0) { lastSentence = input; }
                    lastReading = 0;
    
                    boolean showall = prefs.getBoolean(SHOW_ALL_RESULTS, false);
                    boolean showderivs = prefs.getBoolean(SHOW_DERIVATIONS, false);
                    boolean showsem = prefs.getBoolean(SHOW_SEMANTICS, true);
                    boolean visualize = prefs.getBoolean(VISUALIZE, false); 
                    boolean showfeats = prefs.getBoolean(SHOW_FEATURES, false);
                    String feats_to_show = prefs.get(FEATURES_TO_SHOW, "");
                    Visualizer vis = null;
                    String baseFileName = null;
				    grammar.prefs.showSem = showsem;
				    grammar.prefs.showFeats = showfeats;
				    grammar.prefs.featsToShow = feats_to_show;
                    if (visualize) { 
                        vis = new Visualizer(); 
                        if (prefs.get("VISFNAME", "").equals(""))
                            baseFileName = vis.getTempFileName();
                        else
                            baseFileName = prefs.get("VISFNAME", "");
                        vis.writeHeader(baseFileName+".tex");  
                    }
                    int numToShow = (showall) ? resLength : 1;
                    for (int i=0; i < numToShow; i++) {
                        Category cat = results[i].getCategory();
                        LF convertedLF = null;
                        if (cat.getLF() != null) {
                            cat = cat.copy();
                            Nominal index = cat.getIndexNominal(); 
                            convertedLF = HyloHelper.compactAndConvertNominals(cat.getLF(), index);
                            lastLFs[i] = convertedLF; 
                            cat.setLF(null);
                        }
                        String parseNum = (resLength == 1) ? "Parse: " : ("Parse "+(i+1)+": "); 
                        System.out.print(parseNum + cat.toString());
                        if (showsem && convertedLF != null) {
                            System.out.println(" : ");
                            System.out.println("  " + convertedLF.prettyPrint("  "));
                        }
                        else System.out.println();
                        if (showderivs) {
                            System.out.println("------------------------------");
                            System.out.println(results[i].getDerivationHistory());
                        }
                        if (visualize)
                            vis.saveTeXFile(results[i], baseFileName + ".tex" );
                    }
                    if (visualize) {
                        vis.writeFooter(baseFileName + ".tex"); 
                        vis.show(baseFileName); 
                        if (prefs.get("VISFNAME","").equals("")) // If temporary file,
                            vis.cleanFiles(baseFileName);       // clean it
                        else {
                            vis.cleanAuxFiles(baseFileName);
                            System.out.println("Saved to files " + baseFileName + ".tex and " + baseFileName + ".dvi");
                        }
                        vis = null; 
                    }
                } catch(ParseException pe) {
                    System.out.println(pe);
                }
            }
        }
		
		// store command input history in preferences
		prefs.put(historyKey, lineReader.getCommandHistory());
		
        // done
		System.out.println("Exiting tccg.");
		System.exit(0);
    }


    // reads the next token in the string as a filename
    private static String readFilename(String s) throws IOException {
        StreamTokenizer st = new StreamTokenizer(new StringReader(s));
        st.wordChars('/','/'); st.wordChars('\\','\\'); st.wordChars(':',':'); 
        st.nextToken();
        return st.sval;
    }
    
    
    /** Shows help for the command-line tool. */
    public static void showHelp() {
        System.out.println();
        System.out.println("Commands for tccg (otherwise input is parsed):");
        System.out.println();
        System.out.println("  :sh\t\t\tshow current preference settings");
        System.out.println("  :v\t\t\tverbose output");
        System.out.println("  :reset\t\treset options to defaults");
        System.out.println("  :feats (L)\t\tshow features (or just show features in list L)");
        System.out.println("  :nofeats\t\tdon't show features");
        System.out.println("  :sem\t\t\tshow semantics");
        System.out.println("  :nosem\t\tdon't show semantics");
        System.out.println("  :all\t\t\tshow all parse results");
        System.out.println("  :notall\t\tdon't show all parse results");
        System.out.println("  :derivs\t\tshow derivations");
        System.out.println("  :noderivs\t\tdon't show derivations");
        System.out.println("  :vison (FN)\t\tturn visualization on (saving to file with name FN)");
        System.out.println("  :visoff\t\tturn visualization off");
        System.out.println();
        System.out.println("  :r (FN)\t\trealize selected reading (or from XML file with name FN)");
        System.out.println("  :sel N\t\tselect reading N for realization or saving");
        System.out.println("  :2xml FN\t\tsave last input and LF to XML file with name FN");
        System.out.println("  :2tb (FN)\t\tadd last input and LF as a test item (to file with name FN)");
        System.out.println("  :2apml FN\t\tsave last input to APML file with name FN");
        System.out.println();
        System.out.println("  :tl N\t\t\tset realization time limit to N ms");
        System.out.println("  :notl\t\t\tset realization time limit to none");
        System.out.println("  :nbtl N\t\tset realization new best time limit to N ms | N < 1 of first");
        System.out.println("  :nonbtl\t\tset realization new best time limit to none");
        System.out.println("  :pv N\t\t\tset realization pruning value to N");
        System.out.println("  :nopv\t\t\tset realization pruning value to none");
        System.out.println("  :upon\t\t\tturn unpacking on");
        System.out.println("  :upoff\t\tturn unpacking off");
        System.out.println("  :t\t\t\tturn realization timing on");
        System.out.println("  :toff\t\t\tturn realization timing off");
        System.out.println("  :inc\t\t\tshow incomplete realization edges");
        System.out.println("  :noinc\t\tdon't show incomplete realization edges");
        System.out.println();
        System.out.println("  :ion\t\t\tturn index filtering on");
        System.out.println("  :ioff\t\t\tturn index filtering off");
        System.out.println("  :con\t\t\tturn LF chunks on");
        System.out.println("  :coff\t\t\tturn LF chunks off");
        System.out.println("  :flon\t\t\tturn feature licensing on");
        System.out.println("  :floff\t\tturn feature licensing off");
        System.out.println("  :ccon\t\t\tturn collected combos on");
        System.out.println("  :ccoff\t\tturn collected combos off");
        System.out.println("  :pon\t\t\tturn packing on");
        System.out.println("  :poff\t\t\tturn packing off");
        
        System.out.println();
        System.out.println("  :q\t\t\tquit tccg");
        System.out.println("  :h\t\t\tshow this message");
        System.out.println();
    }

    /** Shows current settings. */
    public static void showSettings(Preferences prefs) {
        System.out.println();
        System.out.println("Current preference settings:"); 
        System.out.println();
        boolean showfeats = prefs.getBoolean(SHOW_FEATURES, false);
        boolean showsem = prefs.getBoolean(SHOW_SEMANTICS, true);
        String feats = prefs.get(FEATURES_TO_SHOW, "");
        System.out.println("  show feats:\t\t" + showfeats); 
        System.out.println("  show semantics:\t" + showsem);
        if (showfeats) {
            System.out.println("  feats to show:\t" + ((feats.length() > 0) ? feats : "all")); 
        }
        boolean showall = prefs.getBoolean(SHOW_ALL_RESULTS, false);
        boolean showderivs = prefs.getBoolean(SHOW_DERIVATIONS, false);
        System.out.println("  show all:\t\t" + showall); 
        System.out.println("  show derivs:\t\t" + showderivs);
        boolean visualize = prefs.getBoolean(VISUALIZE, false); 
        String visfname = prefs.get("VISFNAME", "");
        System.out.println("  visualize:\t\t" + visualize);
        if (visfname.length() > 0) {
            System.out.println("  vis file name:\t" + visfname);
        }
        System.out.println();
        int tl = prefs.getInt(opennlp.ccg.realize.Chart.TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
        System.out.println("  time limit:\t\t" + ((tl == opennlp.ccg.realize.Chart.NO_TIME_LIMIT) ? "none" : "" + tl + " ms"));
        double nbtl = prefs.getDouble(opennlp.ccg.realize.Chart.NEW_BEST_TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
        String nbtlStr = (nbtl >= 1) ? (((int)nbtl) + " ms") : (nbtl + " of first");
        System.out.println("  new best time limit:\t" + ((nbtl == opennlp.ccg.realize.Chart.NO_TIME_LIMIT) ? "none" : nbtlStr));
        int pv = prefs.getInt(opennlp.ccg.realize.Chart.PRUNING_VALUE, opennlp.ccg.realize.Chart.NO_PRUNING);
        System.out.println("  pruning value:\t" + ((pv == opennlp.ccg.realize.Chart.NO_PRUNING) ? "none" : "" + pv));
        boolean unpacking = prefs.getBoolean(opennlp.ccg.realize.Chart.DO_UNPACKING, true);
        System.out.println("  unpacking:\t\t" + ((unpacking) ? "on" : "off")); 
        boolean showtiming = prefs.getBoolean(SHOW_TIMING, false);
        System.out.println("  timing:\t\t" + ((showtiming) ? "on" : "off")); 
        boolean showinc = prefs.getBoolean(SHOW_INCOMPLETE_EDGES, false);
        System.out.println("  show incomplete:\t" + ((showinc) ? "on" : "off")); 
        System.out.println();
        boolean indexing = prefs.getBoolean(EdgeFactory.USE_INDEXING, true);
        boolean chunks = prefs.getBoolean(EdgeFactory.USE_CHUNKS, true);
        boolean licensing = prefs.getBoolean(EdgeFactory.USE_FEATURE_LICENSING, true);
        boolean combos = prefs.getBoolean(opennlp.ccg.realize.Chart.USE_COMBOS, true);
        boolean packing = prefs.getBoolean(opennlp.ccg.realize.Chart.USE_PACKING, false);
        System.out.println("  index filtering:\t" + ((indexing) ? "on" : "off"));
        System.out.println("  chunks:\t\t" + ((chunks) ? "on" : "off"));
        System.out.println("  licensing:\t\t" + ((licensing) ? "on" : "off"));
        System.out.println("  combos:\t\t" + ((combos) ? "on" : "off"));
        System.out.println("  packing:\t\t" + ((packing) ? "on" : "off"));
        System.out.println();
    }
}
