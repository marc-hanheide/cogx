///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-8 University of Edinburgh / Michael White
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

package opennlp.ccg.realize;

import opennlp.ccg.grammar.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.*;
import org.jdom.*;
import java.util.*;
import java.util.prefs.*;

/**
 * The realizer manages the realization process.
 * Realization options may be set for use across calls 
 * to the realizer.
 *
 * @author      Michael White
 * @version     $Revision: 1.19 $, $Date: 2008/01/08 20:41:57 $
 */
public class Realizer
{
    
    /** The grammar used for realization. */
    public final Grammar grammar; 
    
    /** Flag for whether to use depth-first search.  Defaults to false. */
    public boolean depthFirst = false; 
    
    // the chart used to realize a request
    private Chart chart = null;
    
   
    /** Constructor. */
    public Realizer(Grammar grammar) { 
        this.grammar = grammar;
    }
    
    /** Returns the chart used in the latest request, or null if none. */
    public Chart getChart() { return chart; }
    
    
    //-----------------------------------------------------------------
    // default options, for use when not given in realization request
    // nb: as the usual practice is to set these options once 
    //     for reuse across calls to the realizer, only a subset of 
    //     the options may be overridden in different calls to the 
    //     realize method
        
    /** Time limit in ms.  (Default is -1, or none.) */
    public int timeLimitMS = -1;
    
    /** Flag for whether to wait for a complete edge. (Default is false.) */
    public boolean waitForCompleteEdge = false;

    /** Sign scorer to use.  (Default is none.) */
    public opennlp.ccg.synsem.SignScorer signScorer = null;
    
    /** Pruning strategy to use. (Default is none.) */
    public PruningStrategy pruningStrategy = null;
    
    /** Hypertagger to use. (Default is none.) */
    public Hypertagger hypertagger = null;
    

    //-----------------------------------------------------------------
    // get LF from doc    
    
    /**
     * Retrieves an input LF from the given XML doc, processing any 
     * LF chunks along the way.
     */
    public static LF getLfFromDoc(Document doc) {
        Element rootElt = doc.getRootElement();
        Element lfElt = (rootElt.getName().equals("lf")) ? rootElt : rootElt.getChild("lf");
        return getLfFromElt(lfElt);
    }

    /**
     * Retrieves an input LF from the given XML element, processing any 
     * LF chunks along the way.
     */
    public static LF getLfFromElt(Element lfElt) {
        HyloHelper.processChunks(lfElt);
        LF lf = HyloHelper.getLF(lfElt);
        return lf;
    }

    
    //-----------------------------------------------------------------
    // realization routines    
    
    /**
     * Realizes the input LF, 
     * returning the best edge found (or null if none).
     */
    public Edge realize(LF lf) {
        return realize(lf, this.signScorer);
    }

    /**
     * Realizes the input LF relative to the given sign scorer, 
     * returning the best edge found (or null if none).
     */
    public Edge realize(LF lf, opennlp.ccg.synsem.SignScorer signScorer) {
        Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
        int timeLimitToUse = (timeLimitMS != -1) 
            ? timeLimitMS
            : prefs.getInt(Chart.TIME_LIMIT, Chart.NO_TIME_LIMIT);
        return realize(lf, signScorer, timeLimitToUse, waitForCompleteEdge);
    }
    
    /**
     * Realizes the input LF relative to given sign scorer, 
     * returning the best edge found (or null if none)
     * in the given time limit (in ms), potentially waiting 
     * longer for a complete edge according to the given flag.
     * If a hypertagger is employed, realization proceeds 
     * iteratively through the available beta-best values 
     * within the overall time limit.
     */
    public Edge realize(LF lf, opennlp.ccg.synsem.SignScorer signScorer, int timeLimitMS, boolean waitForCompleteEdge) {
        List<SatOp> preds = HyloHelper.flatten(lf);
        opennlp.ccg.synsem.SignScorer scorerToUse = (signScorer != null) 
            ? signScorer : opennlp.ccg.synsem.SignScorer.nullScorer;
        PruningStrategy strategyToUse = (pruningStrategy != null) 
            ? pruningStrategy : new NBestPruningStrategy();
        // realize iteratively with hypertagger, if present
        if (hypertagger != null) {
        	return realizeWithHypertagger(preds, scorerToUse, strategyToUse, timeLimitMS);
        }
        // otherwise make chart, set start time
        long startTime = System.currentTimeMillis(); 
        chart = new Chart(new EdgeFactory(grammar, preds, scorerToUse), strategyToUse);
        chart.startTime = startTime; 
        chart.depthFirst = depthFirst;
        // run request
        chart.initialize();
        chart.combine(timeLimitMS, waitForCompleteEdge);
        // return best edge
        return chart.bestEdge;
    }
    
    // iterate through beta-best values until a complete realization is found; 
    // otherwise return the best fragment
    private Edge realizeWithHypertagger(List<SatOp> preds, opennlp.ccg.synsem.SignScorer signScorer, PruningStrategy pruningStrategy, int timeLimitMS) {
        // get start time, calc half time limit
        long startTime = System.currentTimeMillis();
        int halfTimeLimitMS = timeLimitMS / 2;
    	// set supertagger in lexicon
    	grammar.lexicon.setSupertagger(hypertagger);
    	// reset beta
    	hypertagger.resetBeta();
        // loop until retval set
        Edge retval = null;
        while (retval == null) {
        	// instantiate chart and set start time
            chart = new Chart(new EdgeFactory(grammar, preds, signScorer, hypertagger), pruningStrategy);
            chart.startTime = startTime; 
        	// if we're not at the least restrictive beta value already,
            // and we haven't used up more than half the time limit,
        	// do realization in packing mode to see if a complete realization 
        	// can be found with this setting
            long currentTime = System.currentTimeMillis();
            int timeSoFar = (int) (currentTime - startTime);
        	if (hypertagger.hasMoreBetas() && timeSoFar < halfTimeLimitMS) {
                // override chart settings
                chart.usePacking = true; chart.collectCombos = false;
                chart.doUnpacking = false; chart.joinFragments = false;
                // run request
                chart.initialize();
//                System.out.println("initial edges: " + chart.edgeFactory.initialEdges.size()); // tmp XXX
                chart.combine(halfTimeLimitMS, false);
                // if complete, unpack and return best edge
                if (chart.bestEdge.complete()) {
                	chart.doUnpacking();
                	retval = chart.bestEdge;
                	// update end time
        	        long endTime = System.currentTimeMillis();
        	        chart.timeTilDone = (int) (endTime - startTime);
                }
                // otherwise progress to next beta setting
                else {
                	hypertagger.nextBeta();
                	continue;
                }
        	}
        	// otherwise do realization with current settings
        	else {
                // run request
                chart.initialize();
//                System.out.println("initial edges (final): " + chart.edgeFactory.initialEdges.size()); // tmp XXX
                chart.combine(timeLimitMS, waitForCompleteEdge);
                // return best edge
                retval = chart.bestEdge;
        	}
        }
    	// reset supertagger in lexicon
    	grammar.lexicon.setSupertagger(null);
        // return
    	return retval;
    }
}
