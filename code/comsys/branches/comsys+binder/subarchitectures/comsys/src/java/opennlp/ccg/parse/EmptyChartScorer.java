//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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
// JAVA
//-----------------------------------------------------------------
import java.util.Iterator;
import java.util.Vector;

//-----------------------------------------------------------------
// OPENCCG
//-----------------------------------------------------------------
import opennlp.ccg.synsem.*;

/**
	The class <b>EmptyChartScorer</b> implements an empty shell for 
	a chart scorer. The class returns the original chart when scoring. 
	
	@version 070924
	@started 070924
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public class EmptyChartScorer 
	implements ChartScorer
{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================
		
	/** The beam width function */ 
	BeamWidthFunction beamWidthFunction; 

	/** Logging boolean */ 
	boolean logging;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public EmptyChartScorer () { 
		init();
	} // end constr


	protected void init () { 
		beamWidthFunction = null; 
		logging = false; 
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Sets the beam width function to be used in controlling the number of prefered analyses included in the resulting top-most SignHash */

	public void setBeamWidthFunction (BeamWidthFunction bf) { 
		beamWidthFunction = bf;
	} // end setBeamWidthFunction

	/** Sets whether the method should be outputting log statements */
	
	public void setLogging(boolean l) { logging = l; }

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/** The method <i>score</i> returns the original, unscored chart. 
	
		@param chart The chart to be scored / pruned
		@param stringPos The current string position
		@return Chart The pruned chart
	*/ 

	public Chart score (Chart chart, int stringPos) { 
		return chart; 
	} // end score


	//=================================================================
	// LOG METHODS
	//=================================================================

	public void log (String msg) { 
		if (logging) System.out.println("[CatChartScorer] "+msg); 
	} // end log


} // end class
