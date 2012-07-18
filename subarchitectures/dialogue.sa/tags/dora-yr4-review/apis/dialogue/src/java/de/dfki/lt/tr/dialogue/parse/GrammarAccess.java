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
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
// IMPORTS

// Java
import java.io.IOException;
import java.util.List;
import java.util.Properties;

// Dialogue API
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

/**
 * The interface <tt>GrammarAccess</tt> defines methods that 
 * any module implementing access to a grammar should provide. 
 *
 * @version 070810 
 * @since 	070810
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
*/

public interface GrammarAccess {

	/**
	 * Set the word position of the word related to the proposition
	 */
	public void setWordPosition(int stringPos);
	
	
	/**
	 * Set the utterance increment
	 */
	public void setUtteranceIncrement(int utteranceIncrement);
	
	/** 
	* The method <i>getLexicalEntries</i> provides the most basic type of access, 
	* namely retrieving grammar information about individual words. 
	* 
	* @param	word		A word to provide data about
	* @return	GrammarData	Information in the grammar about the word
	* @throws	DialogueMissingValueException Thrown when there is a problem accessing the grammar
	*/ 
	public GrammarData getLexicalEntries (String word) throws DialogueMissingValueException; 

	/** 
	* The method <i>setGrammar</i> provides the module with information about what grammar to use. 
	* 
	* @param	fileName		The name of the grammar file 
	* @throws	Thrown when there is a problem accessing the grammar
	*/ 
	
	public void setGrammar (String fileName) throws DialogueMissingValueException, IOException; 


	/**
		The method <i>configure</i> takes a Properties list, to set any relevant parameters. 
	*/ 

	public void configure (Properties props); 


} // end interface
