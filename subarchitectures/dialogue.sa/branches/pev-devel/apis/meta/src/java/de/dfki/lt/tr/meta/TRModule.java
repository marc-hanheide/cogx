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
package de.dfki.lt.tr.meta;

//=================================================================
//IMPORTS

// Java
import java.util.Properties;
import javax.xml.datatype.DatatypeConfigurationException;

// SDL
import de.dfki.lt.sdl.*;

/**
 * The class <tt>TRModule</tt> provides an abstract definition of a module 
 * (or process) conform the System Definition Language IModule interface. 
 * This functionality is defined at one (abstract) level above the actual 
 * modules which provide the functionality within the APIs. This allows for 
 * a flexible exchange of meta-functionality without having to touch the 
 * lower API module definitions. 
 * 
 * @author 	Geert-Jan M. Kruijff
 * @since	100607 Java 1.6
 * @version	100607
 */


public abstract class TRModule 
extends Modules
{

	
	
	abstract public void configure (Properties properties)
	throws DatatypeConfigurationException; 
	

} // end class
