//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package comsys.processing.uttplan;

//=================================================================
// IMPORTS
//=================================================================

import comsys.datastructs.lf.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/**
 The interface <b>GREHandler</b> specifies an interface between a
 content planner, and a class handling the generation of referring
 expressions. 
 
 @started 080822
 @version 080822
 @author  Geert-Jan Kruijff (gj@dfki.de)
*/ 

public interface GREHandler {
	
	/**
	 The method <i>produceRFX</i> takes a referent (specifiable as an Object, to leave things flexible), 
	 and returns a logical form specifying an expression for referring to that referent. 
	
	 @param	referent	 The referent to which an expression should refer
	 @return String		 The content for the expression to refer to the referent
	*/ 
	
	public String produceRFX (Object referent); 
	
} // end interface
