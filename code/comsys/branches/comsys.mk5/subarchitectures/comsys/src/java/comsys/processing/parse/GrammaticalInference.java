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
// CAST IMPORTS
// -----------------------------------------------------------------

import cast.core.CASTData;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------

import comsys.datastructs.comsysEssentials.*;
import comsys.arch.ComsysException;


// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.List;
import java.util.Properties;

// -----------------------------------------------------------------
// OPENCCG IMPORTS
// -----------------------------------------------------------------
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.parse.ParseException;


// =================================================================
// INTERFACE DOCUMENTATION
// =================================================================

/**
 * The interface <b>GrammaticalInference</b> defines methods that 
 * any parse process should provide. 
 *
 * @version 080906 (Started: 070810)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
*/

public interface GrammaticalInference { 

	/** 
	* The method <i>parse</i> parses the given PhonString, and returns the resulting analyses. 
	*
	* @param	str				The PhonString object with the utterance to be parsed
	* @return	ParseResults	The resulting analyses
	* @throws	ComsysException	Thrown when a parsing error occurred	
	* @see		org.cognitivesystems.comsys.data.ParseResults
	*/ 

	public ParseResults parse (PhonString str) throws ComsysException, ParseException; 
	
	/**
	* The method <i>registerGrammarAccess</i> registers an object implementing <b>GrammarAccess</b> with the grammatical inference engine, 
	* so that it is able to obtain grammatical information for words (etc.).  
	* 
	* @param	lg	A class implementing the GrammarAccess interface, to provide access to the grammar
	* @throws	ComsysException Thrown when there is a problem connecting to the grammar
	*/	
	
	public void registerGrammarAccess(GrammarAccess lg) throws ComsysException; 
	
	
	/**
		The method <i>setLogLevel</i> sets the level (and the type) of the output, defined as an integer variable. It is up to the 
		user to define the meaning of any of these output levels ... 
	*/ 
	
	public void setLogLevel (int l); 
	
	/**
		The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	*/ 

	public void configure (Properties props); 


} // end interface