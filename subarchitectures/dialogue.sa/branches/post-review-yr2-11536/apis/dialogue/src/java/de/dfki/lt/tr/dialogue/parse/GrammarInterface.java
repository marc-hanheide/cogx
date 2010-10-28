// =================================================================
// Copyright (C) 2010 Geert-Jan M. Kruijff (gj@dfki.de)
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

/**
	The interface <tt>GrammarInterface</tt> provides a level of abstraction to 
	wrap around any framework-specific representation of a grammar. 
	This interface is used together with the other abstract interface   
	<tt>GrammarAccess</tt>: A class implementing the <tt>GrammarInterface</tt> 
	provides the handling of the framework-specific grammar, called by a class
	implementing <tt>GrammarAccess</tt> to define how and when that grammar is
	accessed. 
	
	@see 	 de.dfki.lt.tr.dialogue.parse.GrammarAccess
	@version 100607
	@since	 070820
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public interface GrammarInterface {

	
	
} // end interface
