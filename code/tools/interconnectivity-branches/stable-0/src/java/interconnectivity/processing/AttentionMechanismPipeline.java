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

package interconnectivity.processing;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// CAST
//-----------------------------------------------------------------
import cast.core.CASTData;

//-----------------------------------------------------------------
// JAVA
//-----------------------------------------------------------------
import java.util.Iterator;
import java.util.Vector; 

//=================================================================
// DOCUMENTATION
//=================================================================

/**
	The class <b>AttentionMechanismPipeline</b> implements a pipeline
	of attention mechanisms. Individual mechanisms can be registered,	  
	and when the <i>score</i> method is called on the pipeline, the
	mechanisms are called one after the other. (When adding a mechanism
	to the pipeline, it is checked whether the added mechanism operates
	on the same CASTData type as returned by the preceding mechanism.)
	
	@version 070814
	@since	 070814
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/

public class AttentionMechanismPipeline 
	implements AttentionMechanism
{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	Vector<AttentionMechanism> pipeline;

	//=================================================================
	// CONSTRUCTORS
	//=================================================================

	public AttentionMechanismPipeline () { 
		init();
	} // end constructor

	private void init () { 
		pipeline = new Vector<AttentionMechanism>(); 
	} // end init

	//=================================================================
	// ACCESS METHODS
	//=================================================================
	
	/**
		The method <i>add</i> adds an attention mechanism to the pipeline. 
	
		@param am The attention mechanism to be added
		@throws Exception If a mechanism is added which operates on a different CASTData type than the preceding mechanism does 
	*/ 
	
	public void add (AttentionMechanism am) 
		throws Exception 
	{ 
		if (pipeline.size() < 1) { 
			pipeline.addElement(am);
		} else { 
			String lastCDType = ((AttentionMechanism)pipeline.lastElement()).getDataType();
			if (lastCDType.equals(am.getDataType())) { 
				pipeline.addElement(am);
			} else { 
				throw new Exception("Type inconsistency when adding attention mechanism to pipeline: adding ["+am.getDataType()+"] while preceding type is ["+lastCDType+"].");
			} // end if..else type check for CASTData 
		} // 
	} // end add

	/**
		The method <i>getCASTDataType</i> returns the CASTData type 
		the pipeline operates on, as indicated by the last element 
		of the pipeline. 
	
		@return String The type of CASTData operated on (<tt>null</tt> if empty pipeline)
		@since  070814
	*/ 

	public String getDataType () { 
		if (pipeline.size() < 1) {
			return null; 
		} else { 
			return ((AttentionMechanism)pipeline.lastElement()).getDataType(); 
		} // end if..else check for type to return
	} // end getCASTDataType


	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/**
		The method <i>score</i> iterates over the attention mechanisms
		in the pipeline, applying the mechanism iteratively to the input
		data. Should an individual mechanism throw an exception, then is 
		just passed on. If there are no registered mechanisms, then the 
		score method returns the provided input data. 
		
		@param data	The input data to which the mechanisms are to be applied
		@return CASTData The scored, and possibly pruned, data
		@throws Exception Passed on if thrown by a called mechanism
	*/
	
	public Object score (Object source, CASTData scoring) 
		throws Exception
	{ 
		Object scoredInput = source;
		for (Iterator<AttentionMechanism> amsIter=pipeline.iterator();amsIter.hasNext(); ) { 
			AttentionMechanism am = amsIter.next();
			scoredInput = am.score(scoredInput,scoring);
		} // end for over attention mechanisms
		return (Object) scoredInput;
	} // end score


} // end 
