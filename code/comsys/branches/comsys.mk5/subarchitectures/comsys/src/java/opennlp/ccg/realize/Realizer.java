///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 University of Edinburgh (Michael White)
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
 *
 * @author      Michael White
 * @version     $Revision: 1.16 $, $Date: 2005/10/13 18:20:30 $
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
    
    /** Default time limit. */
    public int timeLimitMS = -1;
    
    /** Default flag for whether to wait for a complete edge. */
    public boolean waitForCompleteEdge = false;

    /** Default sign scorer. */
    public SignScorer signScorer = null;
    
    /** Default pruning strategy. */
    public PruningStrategy pruningStrategy = null;
    

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
    public Edge realize(LF lf, SignScorer signScorer) {
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
     */
    public Edge realize(LF lf, SignScorer signScorer, int timeLimitMS, boolean waitForCompleteEdge) {
        List<SatOp> preds = HyloHelper.flatten(lf);
        SignScorer scorerToUse = (signScorer != null) 
            ? signScorer : SignScorer.nullScorer;
        PruningStrategy strategyToUse = (pruningStrategy != null) 
            ? pruningStrategy : new NBestPruningStrategy();
        // make chart, set start time
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
}
