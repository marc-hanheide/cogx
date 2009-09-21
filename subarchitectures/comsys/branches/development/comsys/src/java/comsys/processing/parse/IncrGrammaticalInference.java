// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
// =================================================================

package comsys.processing.parse;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------

import comsys.datastructs.comsysEssentials.*;
import comsys.arch.ComsysException;


// -----------------------------------------------------------------
// OPENCCG IMPORTS
// -----------------------------------------------------------------
import opennlp.ccg.parse.IncrFrontierFilter;
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.parse.ParseException;


// =================================================================
// DOCUMENTATION
// =================================================================

/**
 * The abstract class <b>IncrGrammaticalInference</b> defines methods 
 * further methods that any incremental parse process should provide. 
 *
 * @version 070817 (Started: 070817)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
*/

public abstract class IncrGrammaticalInference 
	implements GrammaticalInference
{

	// Frontier filter
	protected IncrFrontierFilter a_iffilter; 

	/** 
	* The method <i>incrParse</i> triggers the next incremental parse step for the 
	* given PhonString, and returns the resulting analyses. 
	*
	* @param	str				The PhonString object with the utterance to be parsed
	* @return	ParseResults	The resulting analyses
	* @throws	ComsysException	Thrown when a parsing error occurred	
	* @see		org.cognitivesystems.comsys.data.ParseResults
	*/ 

	public abstract ParseResults incrParse (PhonString str) throws ComsysException, ParseException; 	
	  
	/**
		The method <i>registerFrontierFilter</i> registers a filter 
		to determine when a sequence of one or more incremental parsing 
		steps has reached an "eligible" frontier, to count as an intermediate
		analysis. If the filter is empty, incremental parsing resorts to a 
		word-by-word processing; the filter makes it possible to do 
		constituent-by-constituent incremental parsing. 

		@param iff The frontier filter to be used when parsing incrementally
	*/

	public void registerFrontierFilter (IncrFrontierFilter iff) { 
		a_iffilter = iff;
	} // end registerFrontierFilter
	  
	  
	  
	  


}
