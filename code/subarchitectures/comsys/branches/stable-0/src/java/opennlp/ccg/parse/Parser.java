///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-7 Jason Baldridge, Gann Bierner and Michael White
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

package opennlp.ccg.parse;

import opennlp.ccg.lexicon.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.unify.*;
import opennlp.ccg.TextCCG;

import java.util.*;
import java.util.prefs.*;


/**
 * The parser is a CKY chart parser for CCG.
 *
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.10 $, $Date: 2007/12/21 05:13:37 $
 */
public class Parser 
{
    /** The lexicon used to create edges. */    
    public final Lexicon lexicon;
    
    /** The rules used to create edges. */
    public final RuleGroup rules;
    
	/** The sign scorer (or null if none). */
	protected SignScorer signScorer = null;
	
	/** The "n" for n-best pruning (or 0 if none). */
	protected int pruneVal = 0;

    // parse results
    protected ArrayList<Sign> result;

    /** Constructor. */
    public Parser(Grammar grammar) { 
        this.lexicon = grammar.lexicon;
        this.rules = grammar.rules;
    }
    
	/** Sets the sign scorer. */
	public void setSignScorer(SignScorer signScorer) { this.signScorer = signScorer; }
	
	/** Sets the n-best pruning val. */
	public void setPruneVal(int n) { pruneVal = n; }
	
    // create answer ArrayList
    public void createResult(Chart table, int size) throws ParseException {
        result = new ArrayList<Sign>();
        // unpack top
        EdgeHash unpacked = table.unpack(0, size-1);
        // add signs for unpacked edges
        for (Edge edge : unpacked.asEdgeSet()) result.add(edge.sign);
        // check non-empty
        if (result.size() == 0) {
            throw new ParseException("Unable to parse");
        }
    }
    
    // actual CKY parsing
    public void parse(Chart table, int size) throws ParseException {
    	// fill in chart

        for (int i=0; i<size; i++) table.insertCell(i,i);

        for (int j=1; j<size; j++) {
            for (int i=j-1; i>=0; i--) {
                for (int k=i; k<j; k++) {
                    table.insertCell(i,k, k+1,j, i,j);
                }      
                table.insertCell(i,j);
            }
        }
            
        Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
        boolean scoring = prefs.getBoolean(TextCCG.SCORING, true);
        if (scoring) {
        	table = score(table,size);
        }
        
        // extract results
        createResult(table, size);
    }

    
    public static Chart score (Chart table, int size) {
    	CategoryChartScorer scorer = new CategoryChartScorer();
    	scorer.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
    	table = scorer.score(table, size-1);
    	return table;
    }
    
    // initialize the Table
    protected Chart getInitializedTable(List<SignHash> entries) {
        Chart table = new Chart(entries.size(), rules);
        for (int i=0; i < entries.size(); i++) {
            SignHash wh = entries.get(i);
            for (Sign sign : wh.asSignSet()) {
                Category cat = sign.getCategory();
                UnifyControl.reindex(cat);
                table.insert(i, i, sign);
            }
        }
        return table;
    }

    public int getChartSize(Chart chart) {
    	return chart._size;
    }
    /**
     * Parses a string.
     *
     * @param s the string
     * @exception ParseException thrown if a parse can't be found for the
     *            entire string
     */
    public void parse(String s) throws ParseException {
    	long initialTime = System.currentTimeMillis();
        try {
        	// init
            UnifyControl.startUnifySequence();
            Sign.resetCatInterner(false);
            // tokenize
            List<Word> words = lexicon.tokenizer.tokenize(s);
            // get entries for each word
            List<SignHash> entries = new ArrayList<SignHash>(words.size());
            for (Word w : words) entries.add(lexicon.getSignsFromWord(w));
            // set up chart
            Chart table = getInitializedTable(entries);
            if (signScorer != null) table.setSignScorer(signScorer);
            if (pruneVal > 0) table.setPruneVal(pruneVal);
            // do parsing
            parse(table, entries.size());
            
        	// timing information
            Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
            boolean showTimingInfo = prefs.getBoolean(TextCCG.TIMING, false);
            if (showTimingInfo) {
            	showTimingInfo(initialTime);
            }
            
        } catch (LexException e) {
            throw new ParseException("Unable to retrieve lexical entries:\n\t"
                                     + e.toString());
        }
    }

    private void showTimingInfo(long initialTime) {
  	   long finalTime = System.currentTimeMillis();
 		float result = (finalTime-initialTime)/1000.0f;
 		log("Total parsing time (in seconds) = " + result);
 }
 
    private void log(String str) {
    	System.out.println("[CKY Parser] " + str);
    }
    
    /**
     * Returns the results of the parse.
     */
    public ArrayList<Sign> getResult() { return result; }
}

