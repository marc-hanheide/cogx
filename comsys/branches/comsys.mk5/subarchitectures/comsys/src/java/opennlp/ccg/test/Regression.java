///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-5 Jason Baldridge and University of Edinburgh (Michael White)
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
package opennlp.ccg.test;

import opennlp.ccg.*;
import opennlp.ccg.lexicon.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.parse.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.ngrams.*;

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.prefs.*;
import java.text.NumberFormat;

import org.jdom.*;

/**
 * Automates the testing of CCG grammars.
 *
 * @author  Jason Baldridge
 * @author  Michael White
 * @version $Revision: 1.81 $, $Date: 2006/02/08 20:21:49 $
 */
public class Regression {

    /** Flag for whether to do parsing. */
    public boolean doParsing = true;
    
    /** Flag for whether to do realization. */
    public boolean doRealization = true;

    /** Flag for whether to just do even items. */
    public boolean evenOnly = false;

    /** Flag for whether to just do odd items. */
    public boolean oddOnly = false;
    
    /** Director for writing APML files (if any). */
    public String apmldir = null;
    
    /** Flag for whether to show realization stats. */
    public boolean showStats = true;
    
    /** File to dump stats to (if any). */
    public String statsfile = null;
    
    /** The grammar to use for testing. */
    public Grammar grammar = null;
    
    /** The parser to use for testing. */
    public Parser parser = null;
    
    /** The realizer to use for testing. */
    public Realizer realizer = null;
    
    /** The scorer to use for testing (or null, for default). */
    public SignScorer scorer = null;
    
    /** The n-gram order to use with the default scorer (or 0, for default). */
    public int ngramOrder = 0; 

    //
    // the various totals
    //

    public int rCount = 0;
    public int rDoneCount = 0;
    public int rBadCount = 0;
    public int rExactCount = 0;
    public double totalScore = 0.0;
    public double totalReciprocalRank = 0.0;
    public int totalNominals = 0; 
    public int totalTokens = 0;
    public int minTokens = 0;
    public int maxTokens = 0;
    public int totalRuleApps = 0;
    public int totalEdges = 0;
    public int totalUnprunedEdges = 0;
    public int totalPrunedRemoved = 0;
    public int totalPrunedNeverAdded = 0;
    public int totalNewBest = 0;
    public int totalLex = 0;
    public int totalFirst = 0;
    public int totalBest = 0;
    public int totalPacked = 0;
    public int totalStoppedOrDone = 0;
    public int maxLex = 0;
    public int maxFirst = 0;
    public int maxBest = 0;
    public int maxNewBest = 0;
    public int maxPacked = 0;
    public int maxStoppedOrDone = 0;
    public String maxLexStr = null;
    public String maxFirstStr = null;
    public String maxBestStr = null;
    public String maxNewBestStr = null;
    public String maxPackedStr = null;
    public String maxStoppedOrDoneStr = null;
    public List<Double> bestEstimatedScores = null;
    public List<Double> bestActualScores = null;
    public List<Integer> itemRanks = null;
    public TimingMap lexMap = null;
    public TimingMap firstMap = null;
    public TimingMap bestMap = null; 
    public TimingMap allMap = null; 
    
    /** Constructor. */
    public Regression() {
        // init
        resetTotals();
    }
        
    /** Resets the various totals. */
    public void resetTotals() {
        rCount = 0;
        rDoneCount = 0;
        rBadCount = 0;
        rExactCount = 0;
        totalScore = 0.0;
        totalReciprocalRank = 0.0;
        totalNominals = 0; 
        totalTokens = 0;
        minTokens = 0;
        maxTokens = 0;
        totalRuleApps = 0;
        totalEdges = 0;
        totalUnprunedEdges = 0;
        totalPrunedRemoved = 0;
        totalPrunedNeverAdded = 0;
        totalNewBest = 0;
        totalLex = 0;
        totalFirst = 0;
        totalBest = 0;
        totalPacked = 0;
        totalStoppedOrDone = 0;
        maxLex = 0;
        maxFirst = 0;
        maxBest = 0;
        maxNewBest = 0;
        maxPacked = 0;
        maxStoppedOrDone = 0;
        maxLexStr = null;
        maxFirstStr = null;
        maxBestStr = null;
        maxNewBestStr = null;
        maxPackedStr = null;
        maxStoppedOrDoneStr = null;
        if (doRealization) {
            bestActualScores = new ArrayList<Double>();
            bestEstimatedScores = new ArrayList<Double>();
            itemRanks = new ArrayList<Integer>();
            lexMap = new TimingMap("lex");
            firstMap = new TimingMap("first");
            bestMap = new TimingMap("best");
            allMap = new TimingMap("all");
        }
    }
    
    /** Runs the test on the items in the given file. */
    public void runTest(URL regressionURL) throws IOException {

        // load testfile
        RegressionInfo rinfo = new RegressionInfo(grammar, regressionURL.openStream());

        // do each test
        int numItems = rinfo.numberOfItems();
        System.out.println("Parse\tRealize\tString");
        System.out.println("-----\t-------\t------");
        
        for (int i=0; i < numItems; i++) {

            // check even/odd only
            if (i % 2 == 1 && evenOnly) continue;
            if (i % 2 == 0 && oddOnly) continue;
            
            RegressionInfo.TestItem testItem = rinfo.getItem(i);
            
            System.gc(); 
            Sign[] results = null;
            boolean parsed = false;
            if (doParsing) {
                try {
                    parser.parse(testItem.sentence);
                    List<Sign> parses = parser.getResult();
                    results = new Sign[parses.size()];
                    parses.toArray(results);
                    parsed = true;
                } catch (ParseException e) {
                    results = new Sign[0];
                    parsed = false;
                }
            }
            String starForBadSentence = "";
            if (testItem.numOfParses == 0) {
                starForBadSentence = "*";
            }
            
            String parseResult;
            if (!doParsing) {
                parseResult = "-";
            } else if (testItem.numOfParses == results.length) {
                parseResult = "ok";
            } else if (testItem.numOfParses > 0 && results.length > 0) {
                // show num parses, if not the number expected
                parseResult = "(" + results.length + ")";
            } else if (testItem.knownFailure) {
                parseResult = "(known)";
            } else {
                parseResult = "FAILED";
            }
                
            if (!doRealization || (doParsing && !parsed) || testItem.numOfParses == 0) {
                showOutcome(parseResult, "-", starForBadSentence, testItem.sentence);
                continue;
            }
            
            LF inputLF = null;
            // use given LF
            if (testItem.lfElt != null) {
                Element lfElt = testItem.lfElt;
                Document doc = new Document();
                lfElt.detach();
                doc.setRootElement(lfElt);
                inputLF = grammar.loadLF(doc);
            }
            // or LF from first parse otherwise
            else {
                if (results == null) {
                    String suggestion = (!doParsing) ? "Try leaving off -noparsing option." : "";
                    throw new RuntimeException("No LF to realize! " + suggestion);
                }
                Category cat = results[0].getCategory();
                Nominal index = cat.getIndexNominal();
                LF convertedLF = HyloHelper.compactAndConvertNominals(cat.getLF(), index);
                inputLF = grammar.transformLF(convertedLF);
            }
            
            String[] targets = new String[1];
            targets[0] = testItem.sentence;
            NgramPrecisionModel defaultNgramScorer = new NgramPrecisionModel(targets);
            SignScorer scorerToUse = scorer;
            if (scorerToUse == null) {
                if (ngramOrder > 0) scorerToUse = new NgramPrecisionModel(targets, ngramOrder);
                else scorerToUse = defaultNgramScorer;
            }
            
            System.gc(); 
            realizer.realize(inputLF, scorerToUse);
            opennlp.ccg.realize.Chart chart = realizer.getChart();
            String bestRealization = chart.bestEdge.getSign().getOrthography();
            String realizeResult = "ok";
            if (!chart.bestEdge.complete()) {
                realizeResult = "FAILED";
                rBadCount++;
                showOutcome(parseResult, realizeResult, starForBadSentence, testItem.sentence);
                continue;
            }
            
            // if apmldir non-null, output APML as apmldir/ex(i+1).apml
            if (apmldir != null) {
                String apmlfn = apmldir + "/ex" + (i+1) + ".apml";
                grammar.saveToApml(chart.bestEdge.getSign(), apmlfn);
            }
            
            // compute stats, show outcome
            // nb: use default n-gram precision score for reporting
            rCount++;
            double score = defaultNgramScorer.score(chart.bestEdge.getSign(), false); 
            totalScore += score; 
            int itemRank = 1;
            Tokenizer tokenizer = grammar.lexicon.tokenizer;
            String itemOrth = tokenizer.getOrthography(tokenizer.tokenize(testItem.sentence));
            if (!bestRealization.equals(itemOrth)) {
                itemRank = 0;
                List bestEdges = chart.bestEdges();
                for (int j = 0; j < bestEdges.size(); j++) {
                    Edge edge = (Edge) bestEdges.get(j);
                    String str = edge.getSign().getOrthography();
                    if (str.equals(itemOrth)) {
                        itemRank = j+1; break;
                    }
                }
                if (itemRank > 0) totalReciprocalRank += (1.0 / itemRank);
                realizeResult = nf.format(score);
                if (itemRank > 0 && itemRank < 10) realizeResult += " ";
                if (itemRank > 0 && itemRank < 100) realizeResult += "#" + itemRank;
                showOutcome(parseResult, realizeResult, starForBadSentence, testItem.sentence, bestRealization);
            }
            else {
                rExactCount++;
                totalReciprocalRank += 1.0;
                showOutcome(parseResult, realizeResult, starForBadSentence, testItem.sentence);
            }
            
            totalNominals += chart.numNominals;
            int tokens = testItem.sentence.split("\\s+").length;
            totalTokens += tokens;
            if (tokens < minTokens || minTokens == 0) minTokens = tokens;
            if (tokens > maxTokens) maxTokens = tokens;
            totalRuleApps += chart.edgeFactory.ruleApps();
            totalEdges += chart.numEdgesInChart();
            totalUnprunedEdges += chart.numUnprunedEdges();
            totalPrunedRemoved += chart.numPrunedRemoved;
            totalPrunedNeverAdded += chart.numPrunedNeverAdded;
            totalNewBest += chart.newBest;
            
            bestActualScores.add(new Double(score));
            bestEstimatedScores.add(new Double(chart.bestEdge.score));
            itemRanks.add(new Integer(itemRank));
            
            totalLex += chart.timeTilLex;
            if (chart.timeTilLex > maxLex) {
                maxLex = chart.timeTilLex;
                maxLexStr = testItem.sentence;
            }
            lexMap.add(chart.numNominals, chart.timeTilLex);
            
            totalFirst += chart.timeTilFirst;
            if (chart.timeTilFirst > maxFirst) {
                maxFirst = chart.timeTilFirst;
                maxFirstStr = testItem.sentence;
            }
            firstMap.add(chart.numNominals, chart.timeTilFirst);
            
            totalBest += chart.timeTilBest;
            if (chart.timeTilBest > maxBest) {
                maxBest = chart.timeTilBest;
                maxBestStr = testItem.sentence;
            }
            bestMap.add(chart.numNominals, chart.timeTilBest);
            
            if (chart.newBest > 0 && (chart.timeTilBest - chart.timeTilFirst) >= maxNewBest) {
                maxNewBest = chart.timeTilBest - chart.timeTilFirst;
                maxNewBestStr = testItem.sentence;
            }
            
            totalPacked += chart.timeTilPacked;
            if (chart.timeTilPacked > maxPacked) {
                maxPacked = chart.timeTilPacked;
                maxPackedStr = testItem.sentence;
            }
            
            if (chart.done) {
                rDoneCount++;
                totalStoppedOrDone += chart.timeTilDone;
                if (chart.timeTilDone > maxStoppedOrDone) {
                    maxStoppedOrDone = chart.timeTilDone;
                    maxStoppedOrDoneStr = testItem.sentence;
                }
                allMap.add(chart.numNominals, chart.timeTilDone);
            }
            else {
                totalStoppedOrDone += chart.timeTilStopped;
                if (chart.timeTilStopped > maxStoppedOrDone) {
                    maxStoppedOrDone = chart.timeTilStopped;
                    maxStoppedOrDoneStr = testItem.sentence;
                }
                allMap.add(chart.numNominals, chart.timeTilStopped);
            }
        }

        if (doRealization && showStats) { showStats(); }
    }

    /** Shows the various totals. */
    public void showStats() {
        System.out.println();
        String rInexact = "" + (rCount - rExactCount);
        System.out.println("Strings realized (exact, inexact): " + rCount + " (" + rExactCount + ", " + rInexact + ")");
        System.out.println("Strings unable to realize: " + rBadCount);
        if (rCount == 0) {
            System.out.println("Skipping remaining stats.");
            return;
        }
        System.out.println("Strings where realization finished: " + rDoneCount);
        String avgScore = nf.format(totalScore / rCount);
        System.out.println("Avg score: " + avgScore);
        String meanReciprocalRank = nf.format(totalReciprocalRank / rCount);
        System.out.println("Mean reciprocal rank: " + meanReciprocalRank);
        String residualMRR = (rCount == rExactCount) 
            ? "n/a" 
            : nf.format((totalReciprocalRank - rExactCount) / (rCount - rExactCount));
        System.out.println("Residual mean reciprocal rank: " + residualMRR);
        String avgNodes = nf.format(totalNominals * 1.0 / rCount); 
        String avgTokens = nf.format(totalTokens * 1.0 / rCount); 
        System.out.println("Avg num nodes, words: " + avgNodes + ", " + avgTokens);
        System.out.println("Num words (min-max): " + minTokens + "-" + maxTokens);
        String avgRuleApps = nf.format(totalRuleApps * 1.0 / rCount);
        System.out.println("Avg num rule apps: " + avgRuleApps);
        String avgEdges = nf.format(totalEdges * 1.0 / rCount);
        String avgUnprunedEdges = nf.format(totalUnprunedEdges * 1.0 / rCount);
        System.out.println("Avg num edges in chart: " + avgEdges);
        System.out.println("Avg num unpruned edges: " + avgUnprunedEdges);
        String avgRemoved = nf.format(totalPrunedRemoved * 1.0 / rCount);
        String avgNeverAdded = nf.format(totalPrunedNeverAdded * 1.0 / rCount);
        System.out.println("Avg num pruned edges removed, never added: "  + avgRemoved + ", " + avgNeverAdded);
        String avgNewBest = nf.format(totalNewBest * 1.0 / rCount);
        System.out.println("Total, avg num new best realizations: " + totalNewBest + ", " + avgNewBest);
        String avgLex = nf.format(lexMap.mean());
        String stdLex = nf.format(lexMap.sigma());
        System.out.println("Avg (std) time 'til lex lookup finished: " + avgLex + " (" + stdLex + ")");
        System.out.println("Max time 'til lex lookup finished: " + maxLex + " (" + maxLexStr + ")");
        String avgFirst = nf.format(firstMap.mean());
        String stdFirst = nf.format(firstMap.sigma());
        System.out.println("Avg (std) time 'til first realization: " + avgFirst + " (" + stdFirst + ")");
        System.out.println("Max time 'til first realization: " + maxFirst + " (" + maxFirstStr + ")");
        String avgBest = nf.format(bestMap.mean());
        String stdBest = nf.format(bestMap.sigma());
        System.out.println("Avg (std) time 'til best realization: " + avgBest + " (" + stdBest + ")");
        System.out.println("Max time 'til best realization: " + maxBest + " (" + maxBestStr +")");
        System.out.println("Max time 'til new best realization: " + maxNewBest + " (" + maxNewBestStr +")");
        String avgPacked = nf.format(totalPacked / rCount);
        System.out.println("Avg time 'til done packing: " + avgPacked);
        System.out.println("Max time 'til done packing: " + maxPacked + " (" + maxPackedStr +")");
        String avgStoppedOrDone = nf.format(allMap.mean());
        String stdStoppedOrDone = nf.format(allMap.sigma());
        System.out.println("Avg (std) time 'til stopped/done with realizations: " + avgStoppedOrDone + " (" + stdStoppedOrDone + ")");
        System.out.println("Max time 'til stopped/done with realizations: " + maxStoppedOrDone + " (" + maxStoppedOrDoneStr +")");
        if (statsfile != null) {
            Document doc = new Document();
            Element root = new Element("rstats");
            doc.setRootElement(root);
            Element counts = new Element("counts");
            root.addContent(counts);
            counts.setAttribute("realized", "" + rCount);
            counts.setAttribute("failed", "" + rBadCount);
            counts.setAttribute("exact", "" + rExactCount);
            counts.setAttribute("inexact", rInexact);
            counts.setAttribute("finished", "" + rDoneCount);
            Element overall = new Element("overall");
            root.addContent(overall);
            overall.setAttribute("avg-score", avgScore);
            overall.setAttribute("mean-reciprocal-rank", meanReciprocalRank);
            overall.setAttribute("residual-mrr", residualMRR);
            overall.setAttribute("avg-nodes", avgNodes);
            overall.setAttribute("avg-words", avgTokens);
            overall.setAttribute("min-words", "" + minTokens);
            overall.setAttribute("max-words", "" + maxTokens);
            Element rules = new Element("rules");
            root.addContent(rules);
            rules.setAttribute("avg-apps", avgRuleApps);
            Element edges = new Element("edges");
            root.addContent(edges);
            edges.setAttribute("avg", avgEdges);
            edges.setAttribute("avg-unpruned", avgUnprunedEdges);
            edges.setAttribute("avg-removed", avgRemoved);
            edges.setAttribute("avg-never-added", avgNeverAdded);
            Element newBest = new Element("new-best");
            root.addContent(newBest);
            newBest.setAttribute("total", "" + totalNewBest);
            newBest.setAttribute("avg", avgNewBest);
            Element times = new Element("times-summary");
            root.addContent(times);
            times.setAttribute("avg-lex", avgLex);
            times.setAttribute("std-lex", stdLex);
            times.setAttribute("avg-first", avgFirst);
            times.setAttribute("std-first", stdFirst);
            times.setAttribute("max-first", "" + maxFirst);
            times.setAttribute("avg-best", avgBest);
            times.setAttribute("std-best", stdBest);
            times.setAttribute("max-best", "" + maxBest);
            times.setAttribute("max-new-best", "" + maxNewBest);
            times.setAttribute("avg-packed", avgPacked);
            times.setAttribute("max-packed", "" + maxPacked);
            times.setAttribute("avg-stopped-or-done", avgStoppedOrDone);
            times.setAttribute("std-stopped-or-done", stdStoppedOrDone);
            times.setAttribute("max-stopped-or-done", "" + maxStoppedOrDone);
            Element strings = new Element("max-strings");
            root.addContent(strings);
            Element lex = new Element("lex");
            strings.addContent(lex);
            lex.addContent(maxLexStr);
            Element first = new Element("first");
            strings.addContent(first);
            first.addContent(maxFirstStr);
            Element best = new Element("best");
            strings.addContent(best);
            best.addContent(maxBestStr);
            Element newBest2 = new Element("new-best");
            strings.addContent(newBest2);
            newBest2.addContent(maxNewBestStr);
            Element packed = new Element("packed");
            strings.addContent(packed);
            packed.addContent(maxPackedStr);
            Element stoppedOrDone = new Element("stopped-or-done");
            strings.addContent(stoppedOrDone);
            stoppedOrDone.addContent(maxStoppedOrDoneStr);
            Element scores = new Element("scores");
            root.addContent(scores);
            for (int i = 0; i < bestActualScores.size(); i++) {
                Element score = new Element("score");
                scores.addContent(score);
                score.setAttribute("val", bestActualScores.get(i).toString());
                score.setAttribute("est", bestEstimatedScores.get(i).toString());
                score.setAttribute("rank", itemRanks.get(i).toString());
            }
            firstMap.saveTimes(root);
            bestMap.saveTimes(root);
            allMap.saveTimes(root);
            try {
                FileOutputStream out = new FileOutputStream(statsfile);
                grammar.serializeXml(doc, out);
                out.flush();
            }
            catch (IOException exc) {
                System.out.println("Unable to write stats to: " + statsfile + " (" + exc + ")");
            }
        }
    }
    
    // show outcome, with wrapping
    private static void showOutcome(String parseResult, String realizeResult, String starForBadSentence, String str) {
        showOutcome(parseResult, realizeResult, starForBadSentence, str, null);
    }
    
    // show outcome including best realization
    private static void showOutcome(String parseResult, String realizeResult, String starForBadSentence, 
                                    String str, String bestRealization) 
    {
        System.out.print(parseResult + "\t" + realizeResult + "\t");
        simpleWrap(starForBadSentence + str);
        if (bestRealization != null) {
            System.out.print("\t\t");
            simpleWrap("(best: " + bestRealization + ")");
        }
    }
    
    // does simple wrapping at TEXTWIDTH
    private static void simpleWrap(String str) {
        int TEXTWIDTH = 60;
        for (int i = 0; i <= (str.length()-1)/TEXTWIDTH; i++) {
            if (i != 0) {
                System.out.print("\t\t");
            }
            System.out.println(str.substring(i*TEXTWIDTH, Math.min(i*TEXTWIDTH + TEXTWIDTH, str.length())));
        }
    }
    
    // formats to three decimal places
    private static final NumberFormat nf = initNF();
    private static NumberFormat initNF() { 
        NumberFormat f = NumberFormat.getInstance();
        f.setMinimumIntegerDigits(1);
        f.setMinimumFractionDigits(1);
        f.setMaximumFractionDigits(2);
        return f;
    }
    
    /** Shows realizer settings for current test. */
    static void showRealizerSettings() {
        // get, show prefs
        Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
        boolean useIndexing = prefs.getBoolean(EdgeFactory.USE_INDEXING, true);
        boolean useChunks = prefs.getBoolean(EdgeFactory.USE_CHUNKS, true);
        boolean useLicensing = prefs.getBoolean(EdgeFactory.USE_FEATURE_LICENSING, true);
        boolean useCombos = prefs.getBoolean(opennlp.ccg.realize.Chart.USE_COMBOS, true);
        boolean usePacking = prefs.getBoolean(opennlp.ccg.realize.Chart.USE_PACKING, false);
        int timeLimit = prefs.getInt(opennlp.ccg.realize.Chart.TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
        double nbTimeLimit = prefs.getDouble(opennlp.ccg.realize.Chart.NEW_BEST_TIME_LIMIT, opennlp.ccg.realize.Chart.NO_TIME_LIMIT);
        int pruningVal = prefs.getInt(opennlp.ccg.realize.Chart.PRUNING_VALUE, opennlp.ccg.realize.Chart.NO_PRUNING);
        String msg = "Timing realization with index filtering " + ((useIndexing) ? "on" : "off") + ", "; 
        msg += "chunks " + ((useChunks) ? "on" : "off") + ", "; 
        msg += "licensing " + ((useLicensing) ? "on" : "off") + ", ";
        if (usePacking) msg += "packing on, ";
        else {
            msg += "combos " + ((useCombos) ? "on" : "off") + ", ";
            if (timeLimit == opennlp.ccg.realize.Chart.NO_TIME_LIMIT) msg += "no time limit, ";
            else msg += "a time limit of " + timeLimit + " ms, ";
            if (nbTimeLimit == opennlp.ccg.realize.Chart.NO_TIME_LIMIT) msg += "no new best time limit, ";
            else {
                msg += "a new best time limit of ";
                if (nbTimeLimit >= 1) msg += ((int)nbTimeLimit) + " ms, ";
                else msg += nbTimeLimit + " of first, ";
            }
        }
        msg += "and ";
        if (pruningVal == opennlp.ccg.realize.Chart.NO_PRUNING) msg += "no pruning";
        else msg += "a pruning value of " + pruningVal;
        System.out.println(msg);
        System.out.println();
    }
    

    /** 
     * Writes the target strings from the given testbed to the given textfile.
     */
    public void writeTargets(URL tbUrl, String textfile) throws IOException {
        writeTargets(tbUrl, textfile, false, false, false);
    }
    
    /** 
     * Writes the target strings with semantic class replacement 
     * from the given testbed to the given textfile. 
     */
    public void writeTargetsSC(URL tbUrl, String textfile) throws IOException {
        writeTargets(tbUrl, textfile, true, false, false);
    }
    
    /** 
     * Writes the target strings with all associated factors 
     * from the given testbed to the given textfile. 
     */
    public void writeTargetsF(URL tbUrl, String textfile) throws IOException {
        writeTargets(tbUrl, textfile, false, true, false);
    }
    
    /** 
     * Writes the target strings with all associated factors with semantic class replacement  
     * from the given testbed to the given textfile. 
     */
    public void writeTargetsFSC(URL tbUrl, String textfile) throws IOException {
        writeTargets(tbUrl, textfile, true, true, false);
    }
    
    // writes targets, optionally with sem class replacement or factors, 
    // and optionally reversing the words; ungrammatical options are filtered out 
    private void writeTargets(
        URL tbUrl, String filename, 
        boolean semClassReplacement, boolean withFactors, 
        boolean reverse
    ) throws IOException {
        // load testbed
        System.out.println("Loading testbed from URL: " + tbUrl);
        System.out.println();
        RegressionInfo tbInfo = new RegressionInfo(grammar, tbUrl.openStream());
        int numItems = tbInfo.numberOfItems();
        // open text file
        String option = "";
        if (withFactors) option = " with factors";
        if (semClassReplacement) option += " with semantic class replacement";
        if (reverse) option += ", reversed";
        System.out.println("Writing text file" + option + ": " + filename);
        PrintWriter tOut = new PrintWriter(new FileWriter(filename));
        HashSet<String> unique = new HashSet<String>(); 
        Tokenizer tokenizer = grammar.lexicon.tokenizer;
        // do each test item
        for (int i = 0; i < numItems; i++) {
            // check even/odd only
            if (i % 2 == 1 && evenOnly) continue;
            if (i % 2 == 0 && oddOnly) continue;
            RegressionInfo.TestItem testItem = tbInfo.getItem(i); 
        	// check grammatical
        	if (testItem.numOfParses == 0) continue;
            String s = testItem.sentence;
            // get parsed words if doing more than just text
            List<Word> words = null;
            if (semClassReplacement || withFactors) {
                // use pre-parsed full words if available
                if (testItem.fullWords != null) 
                    words = tokenizer.tokenize(testItem.fullWords, true);
                // otherwise parse
                else words = grammar.getParsedWords(s);
            }
            else words = tokenizer.tokenize(s);
            // reverse, if apropos
            if (reverse) {
                List<Word> tmp = words;
                words = new ArrayList<Word>(words.size());
                words.add(Word.createWord("<s>"));
                for (int j = tmp.size()-1; j >= 0; j--) {
                    Word w = tmp.get(j);
                    if (w.getForm() == "<s>" || w.getForm() == "</s>") continue; // skip <s> or </s>
                    words.add(w);
                }
                words.add(Word.createWord("</s>"));
            }
            // write str, add to unique set
            String str = (!withFactors)
                ? tokenizer.getOrthography(words, semClassReplacement)
                : tokenizer.format(words, semClassReplacement);
            tOut.println(str);
            unique.add(str);
            System.out.print("."); // indicate progress
        }
        tOut.close();
        System.out.println();
        System.out.println("Unique strings: " + unique.size());
        System.out.println();
    }
    
    
    /** Command-line routine for regression testing. */
    public static void main(String[] args) throws IOException { 

        String usage = "java opennlp.ccg.test.Regression \n" + 
                       "  (-noparsing) (-norealization) (-even|-odd) \n" + 
                       "  (-nullscorer) (-depthfirst) (-aanfilter (<excfile>)) \n" +
                       "  (-scorer <scorerclass>) \n" +
                       "  (-ngrampruningstrategy) \n" +
                       "  (-pruningstrategy <pruningstrategyclass>) \n" +
                       "  (-ngramorder N) (-lm|-lmsc <lmfile>) \n" + 
                       "  (-flm|-flmsc <flmfile>) \n" + 
                       "  (-text|-textsc|-textf|-textfsc <textfile>) (-reverse) \n" +
                       "  (-2apml <apmldir>) \n" + 
                       "  (-g <grammarfile>) (-s <statsfile>) (<regressionfile>)";
                       
        if (args.length > 0 && args[0].equals("-h")) {
            System.out.println("Usage: \n\n" + usage);
            System.exit(0);
        }
        
        // setup Regression tester
        Regression tester = new Regression();

        // args
        String grammarfile = "grammar.xml";
        String regressionfile = "testbed.xml";
        boolean depthFirst = false;
        boolean aanfilter = false;
        String excfile = null;
        String scorerClass = null;
        boolean ngrampruningstrategy = false;
        String pruningStrategyClass = null; 
        String lmfile = null;
        String flmfile = null;
        boolean useSemClasses = false;
        boolean withFactors = false;
        String textfile = null;
        boolean reverse = false;
        for (int i = 0; i < args.length; i++) {
            if (args[i].equals("-noparsing")) { tester.doParsing = false; continue; }
            if (args[i].equals("-norealization")) { tester.doRealization = false; continue; }
            if (args[i].equals("-even")) { tester.evenOnly = true; continue; }
            if (args[i].equals("-odd")) { tester.oddOnly = true; continue; }
            if (args[i].equals("-nullscorer")) { tester.scorer = SignScorer.nullScorer; continue; }
            if (args[i].equals("-depthfirst")) { depthFirst = true; continue; }
            if (args[i].equals("-aanfilter")) {
                aanfilter = true; 
                if (i < args.length-1 && args[i+1].charAt(0) != '-') excfile = args[++i]; 
                continue;
            }
            if (args[i].equals("-scorer")) { scorerClass = args[++i]; continue; }
            if (args[i].equals("-ngrampruningstrategy")) { ngrampruningstrategy = true; continue; }
            if (args[i].equals("-pruningstrategy")) { pruningStrategyClass = args[++i]; continue; }
            if (args[i].equals("-ngramorder")) { tester.ngramOrder = Integer.parseInt(args[++i]); continue; }
            if (args[i].equals("-lm")) { lmfile = args[++i]; continue; }
            if (args[i].equals("-lmsc")) { lmfile = args[++i]; useSemClasses = true; continue; }
            if (args[i].equals("-flm")) { flmfile = args[++i]; continue; }
            if (args[i].equals("-flmsc")) { flmfile = args[++i]; useSemClasses = true; continue; }
            if (args[i].equals("-text")) { textfile = args[++i]; continue; }
            if (args[i].equals("-textsc")) { textfile = args[++i]; useSemClasses = true; continue; }
            if (args[i].equals("-textf")) { textfile = args[++i]; withFactors = true; continue; }
            if (args[i].equals("-textfsc")) { textfile = args[++i]; useSemClasses = true; withFactors = true; continue; }
            if (args[i].equals("-reverse")) { reverse = true; continue; }
            if (args[i].equals("-2apml")) { tester.apmldir = args[++i]; continue; }
            if (args[i].equals("-g")) { grammarfile = args[++i]; continue; }
            if (args[i].equals("-s")) { tester.statsfile = args[++i]; continue; }
            regressionfile = args[i];
        }
        
        // load grammar
        URL grammarURL = new File(grammarfile).toURL();
        System.out.println("Loading grammar from URL: " + grammarURL);
        tester.grammar = new Grammar(grammarURL);
        System.out.println();
        
        // with -aanfilter (<excfile) option, instantiate AAnFilter
        AAnFilter aanFilter = null;
        if (aanfilter) {
            if (excfile != null) System.out.println("Loading a/an exceptions from file: " + excfile);
            aanFilter = (excfile != null) ? new AAnFilter(excfile) : new AAnFilter();
        }
        
        // instantiate scorer, if any
        if (scorerClass != null) {
            try {
                System.out.println("Instantiating sign scorer from class: " + scorerClass);
                SignScorer scorer = (SignScorer) Class.forName(scorerClass).newInstance();
                if (scorer instanceof NgramScorer) {
                    NgramScorer lmScorer = (NgramScorer) scorer;
                    if (aanfilter) lmScorer.addFilter(aanFilter);
                    tester.ngramOrder = lmScorer.getOrder();
                }
                tester.scorer = scorer;
                System.out.println();
            } catch (Exception exc) {
                throw (RuntimeException) new RuntimeException().initCause(exc);
            }
        }
        
        // with -lm|-lmsc options, load n-gram model
        if (lmfile != null) {
            int order = (tester.ngramOrder > 0) ? tester.ngramOrder : 3;
            String reversedStr = (reverse) ? "reversed " : "";
            System.out.println("Loading " + reversedStr + order + "-gram model from file: " + lmfile);
            NgramScorer lmScorer = new StandardNgramModel(order, lmfile, useSemClasses);
            if (reverse) lmScorer.setReverse(true);
            if (aanfilter) lmScorer.addFilter(aanFilter);
            tester.scorer = lmScorer;
            System.out.println();
        }

        // with -flm|-flmsc options, load factored n-gram model family
        if (flmfile != null) {
            String reversedStr = (reverse) ? "reversed " : "";
            System.out.println("Loading " + reversedStr + "factored n-gram model family from file: " + flmfile);
            NgramScorer flmScorer = new FactoredNgramModelFamily(flmfile, useSemClasses);
            if (reverse) flmScorer.setReverse(true);
            if (aanfilter) flmScorer.addFilter(aanFilter);
            tester.scorer = flmScorer;
            tester.ngramOrder = flmScorer.getOrder();
            System.out.println();
        }

        // with -text|-textsc|-textf|-textfsc options, just write text file and exit
        if (textfile != null) {
            URL tbUrl = new File(regressionfile).toURL();
            tester.writeTargets(tbUrl, textfile, useSemClasses, withFactors, reverse);
            System.exit(0);
        }
        
        // setup parse
        if (tester.doParsing) {
            tester.parser = new Parser(tester.grammar);
        }
        
        // setup realizer, show settings
        if (tester.doRealization) {
            tester.realizer = new Realizer(tester.grammar);
            tester.realizer.depthFirst = depthFirst;
            // instantiate pruning strategy, if any
            if (ngrampruningstrategy) {
                int order = (tester.ngramOrder > 0) ? tester.ngramOrder : 3;
                System.out.println("Instantiating n-gram diversity pruning strategy with order " + order);
                tester.realizer.pruningStrategy = new NgramDiversityPruningStrategy(order);
                System.out.println();
            }
            if (pruningStrategyClass != null) {
                try {
                    System.out.println("Instantiating pruning strategy from class: " + pruningStrategyClass);
                    tester.realizer.pruningStrategy = (PruningStrategy) Class.forName(pruningStrategyClass).newInstance();
                    System.out.println();
                } catch (Exception exc) {
                    throw (RuntimeException) new RuntimeException().initCause(exc);
                }
            }
            showRealizerSettings();
        }
        
        // ensure apmldir exists
        if (tester.apmldir != null) {
            File apmlDir = new File(tester.apmldir);
            if (!apmlDir.exists()) { apmlDir.mkdirs(); }
            System.out.println("Writing APML files to dir: " + tester.apmldir);
            System.out.println();
        }

        // run test
        URL regressionURL = new File(regressionfile).toURL();
        tester.runTest(regressionURL);
    }
}
