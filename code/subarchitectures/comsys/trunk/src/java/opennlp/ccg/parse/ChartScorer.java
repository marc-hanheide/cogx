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

package opennlp.ccg.parse;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// OPENCCG
//-----------------------------------------------------------------

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================


/**
	The interface <b>ChartScorer</b> specifies the function(s) 
	required for classes that implement scoring functions over
	entire charts. 

	@author Geert-Jan M. Kruijff (gj@dfki.de)
	@version 070914 
	@started 070914
*/ 

public interface ChartScorer { 

	/** Returns a Chart, based on a scoring over the Signs in the top-most SignHash given the string position. */ 

	public Chart score (Chart chart, int stringPos); 

	/** Sets the beam width function to be used in controlling the number of prefered analyses included in the resulting top-most SignHash */

	public void setBeamWidthFunction (BeamWidthFunction bf); 

} // end interface