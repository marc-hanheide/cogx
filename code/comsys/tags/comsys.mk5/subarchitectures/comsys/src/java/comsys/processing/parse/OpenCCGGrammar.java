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

import opennlp.ccg.grammar.Grammar;

import comsys.processing.parse.GrammarInterface; 

/**
	The class <b>OpenCCGGrammar</b> provides a wrapper around the
	OpenCCG framework-specific representation of a grammar. 
	
	@version 070820
	@since	 070820
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 





public class OpenCCGGrammar implements GrammarInterface {

	// =================================================================
	// GLOBAL DATA STRUCTURES
	// =================================================================

	public opennlp.ccg.grammar.Grammar ccggrammar = null;

	// =================================================================
	// CONSTRUCTOR
	// =================================================================

	public OpenCCGGrammar (opennlp.ccg.grammar.Grammar g) { 
		ccggrammar = g;
	} // end constructor

	public Grammar getGrammar () { return ccggrammar; }
	

}
