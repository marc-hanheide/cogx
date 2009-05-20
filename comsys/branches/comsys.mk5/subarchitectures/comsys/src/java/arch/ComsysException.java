//=================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
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

package comsys.arch;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------


//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/**

The class <b>ComsysException</b> implements an exception class for 
comsys processing exceptions, in the comsys package. 

@version 061024 (started 061024) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/

public class ComsysException extends Exception { 

	/** some message */
	protected String m;

	/**
	 * Class constructor
	 * 
	 * @param s the error message
	 */
	public ComsysException(String s) {
		m = s;
	}

	public String getMessage () { 
		return m;
	}

	public String toString() {
		return m;
	}

} // end class

