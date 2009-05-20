///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003 Jason Baldridge, Gann Bierner and 
//                    University of Edinburgh (Michael White)
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

import java.util.*;

/**
 * The parser is a CKY chart parser for CCG.
 *
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.8 $, $Date: 2005/10/20 18:49:42 $
 */
public class Parser 
{
    /** The lexicon used to create edges. */    
    public final Lexicon lexicon;
    
    /** The rules used to create edges. */
    public final RuleGroup rules;
    
    // parse results
    private ArrayList<Sign> result;

    /** Constructor. */
    public Parser(Grammar grammar) { 
        this.lexicon = grammar.lexicon;
        this.rules = grammar.rules;
    }
    
    private void createResult(Chart table, int size) throws ParseException {
        // create answer ArrayLists to loop through.
        result = new ArrayList<Sign>();
        for (Iterator<Sign> e=table.get(0,size-1).asSignSet().iterator(); e.hasNext();) {
            Sign sign = e.next();
        	result.add(sign);
          }
        if (result.size() == 0) {
            throw new ParseException("Unable to parse");
        }
    }
    
    private void parse(Chart table, int size) throws ParseException {
        // actual CKY parsing
        for(int i=0; i<size; i++) table.insertCell(i,i);
        for(int j=1; j<size; j++) {
            for(int i=j-1; i>=0; i--) {
                for(int k=i; k<j; k++) {
                    table.insertCell(i,k, k+1,j, i,j);
                }
                table.insertCell(i,j);
            }
        }
        
        log("scoring activated!");
        CategoryChartScorer scorer = new CategoryChartScorer();
        scorer.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
        table = scorer.score(table, size-1);
        
        
        createResult(table, size);
    }

    private Chart getChart(int size, RuleGroup r) {
        return new Chart(size, r);
    }
    

    private Chart getInitializedTable(List entries) {
        // initialize the Table
        Chart table = getChart(entries.size(), rules);
        int i = 0;
        for (Iterator entryIt=entries.iterator(); entryIt.hasNext(); i++) {
            SignHash wh = (SignHash)entryIt.next();
            for(Iterator whI=wh.iterator(); whI.hasNext();) {
                Category cat = ((Sign)whI.next()).getCategory();
                //cat.setSpan(i, i);
                UnifyControl.reindex(cat);
            }
            table.set(i,i,wh);
        }
        return table;
    }

    /**
     * Parses a string.
     *
     * @param s the string
     * @exception ParseException thrown if a parse can't be found for the
     *            entire string
     */
    public void parse(String s) throws ParseException {
        try {
            UnifyControl.startUnifySequence();
            Sign.resetCatInterner(false);
            List<SignHash> entries = lexicon.getEntriesFromWords(s);
            
            // simply remove unrecognized entries, instead of throwing
            // an exception -- plison
            List<SignHash> newEntries = new ArrayList<SignHash>();
            for (Iterator<SignHash> i = entries.iterator(); i.hasNext();) {
            	SignHash hash = i.next();
            	if (hash.size() > 0) {
            		newEntries.add(hash);
            	}
            }
            
            if (newEntries.size() > 0) {
            	Chart table = getInitializedTable(newEntries);
            	parse(table, newEntries.size());
            }
            else {
            	throw new ParseException("No lexical entries have been recognized");
            }
            
        } catch (LexException e) {
            throw new ParseException("Unable to retrieve lexical entries:\n\t"
                                     + e.toString());
        }
    }

    private void log(String str) {
    	System.out.println("[CKY Parser] " + str);
    }
    
    /**
     * Returns the results of the parse.
     */
    public List<Sign> getResult() { return result; }
}

