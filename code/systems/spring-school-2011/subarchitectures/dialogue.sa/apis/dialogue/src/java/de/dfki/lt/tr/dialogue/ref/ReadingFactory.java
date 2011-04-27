// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.dialogue.ref;

//=================================================================
// IMPORTS

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.*;



/** 
 * The interface specifies what methods a reading factory needs to specify. 
 * The sort provides the sort of the root of the logical form on which this
 * factory can operate. 
 * 
 * @author	Geert-Jan Kruijff
 * @email	gj@dfki.de
 * @started	090921
 * @version	100608
 */ 


public interface ReadingFactory {

	public	ReadingFactoryResults constructReadings (LogicalForm lf); 
	
	public  String getSort(); 
	
	
} // end interface
