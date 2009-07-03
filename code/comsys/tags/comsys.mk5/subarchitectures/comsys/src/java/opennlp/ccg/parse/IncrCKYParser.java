///////////////////////////////////////////////////////////////////////////////
// Original class: 
// opennlp.ccg.parse.Parser:
// Copyright (C) 2003 Jason Baldridge, Gann Bierner and 
//                    University of Edinburgh (Michael White)
// 
// Current implementation, based on original class: 
// org.cognitivesystems.comsys.processing.IncrementalParser: 
// Copyright (C) 2006 Geert-Jan M. Kruijff
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

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

// package org.cognitivesystems.comsys.processing;

package opennlp.ccg.parse;


//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------

import java.util.*;
import java.util.prefs.Preferences;

import opennlp.ccg.TextCCG;
import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.grammar.RuleGroup;
import opennlp.ccg.lexicon.LexException;
import opennlp.ccg.lexicon.Lexicon;
import opennlp.ccg.lexicon.Word;
import opennlp.ccg.synsem.*;
import opennlp.ccg.unify.UnifyControl;


//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/** The parser is an incremental CKY chart parser for CCG, based on 
	the original CKY chart parser from the OpenNLP/OpenCCG API. 
	<p>
	The adaptations concern the following aspects: 
	<ul>
	<li>  initUnification: initialize the parser
	<li>  stepIncrParser: incremental parsing step on a chart, from positions "start" to "end"; returns Chart object
	<li>  createResult: returns an ArrayList of the Signs in the chart
	<li>  parse: standard CKY parsing, returns an ArrayList of the signs in the chart
	</ul>
 
 
	@author		Geert-Jan M. Kruijff (incremental version)
	@author     Jason Baldridge
	@author     Gann Bierner
	@author     Michael White
	@version    061023 (Started 060926)
*/
public class IncrCKYParser 
	implements CCGParserInterface
{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

    /** The lexicon used to create edges. */    
    public final Lexicon lexicon;
    
    /** The rules used to create edges. */
    public final RuleGroup rules;
    
    // parse results
    private ArrayList<Sign> result;
   

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

    /** Constructor. */
    public IncrCKYParser(Grammar grammar) { 
        this.lexicon = grammar.lexicon;
        this.rules = grammar.rules;
		initUnification();
		
    }
    
	//=================================================================
	// ACCESSOR METHODS
	//=================================================================
	
	/** The method <i>createResult</i> returns an array list of Sign 
		structures for a given chart. The results are taken from the 
		top-most cell at the end of the chart. 
	
		@param  table	  The chart
		@return ArrayList List of Sign objects
		@exception ParseException Thrown if there are no results
	*/
	
    public ArrayList createResult(opennlp.ccg.parse.Chart table) throws ParseException {
    	return createResult(table,this.getChartSize(table)-1);
    } // end createResult 


	/** 
		The method <i>createResult</i> returns a list of signs at the topcell in the position (i.e. column) in the chart. 
		
		@param table    The chart
		@param position The column position (=string position) in the chart
		
		@return ArrayList list with signs in the top-most cell in the given position
	*/  
    
    // create answer ArrayList
    public ArrayList createResult(Chart table, int size) throws ParseException {
        result = new ArrayList<Sign>();
        // unpack top
        EdgeHash unpacked = table.unpack(0, size-1);
        // add signs for unpacked edges
        for (Edge edge : unpacked.asEdgeSet()) result.add(edge.sign);
        // check non-empty
        if (result.size() == 0) {
            throw new ParseException("Unable to parse");
        }
        return result;
    }
    
	/** Returns the size of the given chart */

	public int getChartSize (opennlp.ccg.parse.Chart table) { return table._size; }

    /**
     * Returns the results of the parse.
     */
    public List<Sign> getResult() { return result; }

	/** The method <i>initUnification</i> initializes unification control for the parser (unification control, category interner)
	*/ 

	public void initUnification ()  {  
        UnifyControl.startUnifySequence();
        Sign.resetCatInterner(false);
        
	} // end initIncrParser

	//=================================================================
	// COMPUTING METHODS
	//=================================================================
	
	/**	The method <i>parse</i> provides a standard CKY parsing method, 
		(from the original OpenCCG Parser class). 
		
		@param  table	  The chart 
		@param  size	  The size of the chart
		@return ArrayList The results, after parsing (ArrayList of Sign objects)
		@exception ParseException Thrown if a parsing error occurs.
	*/ 
	
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
        	table = Parser.score(table,size);
        }
        
        // extract results
        createResult(table, size);
    }
    
    
 	/** The method <i>stepIncrParser</i> parses on the chart from position 
		<i>start</i> to position <i>end</i>, connecting back to the 0th cell, 
		and returning the resulting chart. 

		@param  table The chart to be parsed on
		@param  size  The size of the chart
		@param  start The starting position to parse from (including, &lt; end &lt; size)
		@param  end   The end position to parse to (including, &lt; size)
		@return Chart The resulting chart
		@exception ParseException Thrown if we cannot find a parse, or if end or start are beyond chart size
	*/

	public opennlp.ccg.parse.Chart stepIncrParser (opennlp.ccg.parse.Chart table, int start, int end) 
	throws ParseException {
		int size = table._size;
		if (start >= size || end > size) { 
			throw new ParseException("Start ["+start+"] or end ["+end+"] out of bounds on given chart size ["+size+"]");
		} else { 
	//		System.out.println("StepIncrParse: for until i<"+end);

			table.insertCell(end-1,end-1);
			
			int j = end-1;
			for(int i=j-1; i>=0; i--) {
				for(int k=i; k<j; k++) {
					table.insertCell(i,k, k+1,j, i,j);
				}
				table.insertCell(i,j);
			} // end for over constituent arcs
			
		} // end for over word-level entries	
		
		return table;
	} // end stepIncrParser

	
	

 	/** The method <i>stepIncrParser</i> parses on the chart from position 
		<i>start</i> to position <i>end</i>, connecting back to the 0th cell, 
		and returning the resulting chart. 

		@param  table The chart to be parsed on
		@param  size  The size of the chart
		@param  start The starting position to parse from (including, &lt; end &lt; size)
		@param  end   The end position to parse to (including, &lt; size)
		@return Chart The resulting chart
		@exception ParseException Thrown if we cannot find a parse, or if end or start are beyond chart size
	*/

	public opennlp.ccg.parse.Chart stepIncrParser_recovery (opennlp.ccg.parse.Chart table, int start, int end) 
	throws ParseException {
		int size = table._size;
		if (start >= size || end > size) { 
			throw new ParseException("Start ["+start+"] or end ["+end+"] out of bounds on given chart size ["+size+"]");
		} else { 
	//		System.out.println("StepIncrParse: for until i<"+end);

			for (int i=0; i<end; i++) table.insertCell(i,i);

	        for (int j=1; j<end; j++) {
	            for (int i=j-1; i>=0; i--) {
	                for (int k=i; k<j; k++) {
	                    table.insertCell(i,k, k+1,j, i,j);
	                }      
	                table.insertCell(i,j);
	            }
	        }
			
		} // end for over word-level entries	
		
		return table;
	} // end stepIncrParser

	
	
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

    
    /**
     * Parses a string.
     *
     * @param s the string
     * @exception ParseException thrown if a parse can't be found for the
     *            entire string
     */
    public void parse(String s) throws ParseException {
        try {
        	long initialTime = System.currentTimeMillis();
            UnifyControl.startUnifySequence();
            Sign.resetCatInterner(false);
            // tokenize
            List<Word> words = lexicon.tokenizer.tokenize(s);
            // get entries for each word
            List<SignHash> entries = new ArrayList<SignHash>(words.size());
            for (Word w : words) entries.add(lexicon.getSignsFromWord(w));
            // set up chart
            Chart table = getInitializedTable(entries);
            
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
    
    
    
    
    
    /**
     * Parses a string.
     *
     * @param s the string
     * @exception ParseException thrown if a parse can't be found for the
     *            entire string
     */
    public void incrParse(String s) throws ParseException {
        try {
        	long initialTime = System.currentTimeMillis();
            UnifyControl.startUnifySequence();
            Sign.resetCatInterner(false);
            List entries = lexicon.getEntriesFromWords(s);
			// initialize the Table
			opennlp.ccg.parse.Chart table = new opennlp.ccg.parse.Chart(entries.size(), rules);
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
			int nbFailedIncrParses = 0;
	        Preferences prefs = Preferences.userNodeForPackage(TextCCG.class);
	        boolean scoring = prefs.getBoolean(TextCCG.SCORING, true);
	        
			for (int j = 1; j <= entries.size(); j++) {	
				try {
				table = stepIncrParser(table,0, j);
		        if (scoring) {
		        	table = Parser.score(table,entries.size());
		        }
				createResult(table, j);
				}
				catch (ParseException e) {	
				//	log("ParseException!!");
					nbFailedIncrParses++;
				}
				if (nbFailedIncrParses > 12) {
					throw new ParseException("Unable to parse, (6 failed incremental steps)");
				}
			}
           

	        
			
			
	   //     createResult(table);
			
//          timing information
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
     * For testing purposes
     * @param args
     */
    public static void main (String[] args) {
    	try {
    		IncrCKYParser parser = new IncrCKYParser(new Grammar("//home//plison//svn.cosy//development//comsys-devil//subarchitectures//comsys.mk4//grammars//openccg//moloko.v5//grammar.xml"));
    		parser.parse("get the ball and put it here");
    	}
    	catch (Exception e) {
    		e.printStackTrace();
    	}
    }

}

