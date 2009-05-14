//=================================================================
//Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.
//
//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION 
//=================================================================

package org.cognitivesystems.comsys.processing.dialoguemovefactories;

//=================================================================
//IMPORTS
//=================================================================


//----------------------------------------------------------------
//COMSYS imports
//----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.util.datastructs.*;

import java.util.Iterator;
import java.util.List;

//=================================================================
//JAVADOC CLASS DOCUMENTATION
//=================================================================

/**
	The interface <b>EventNucleusFactory</b> describes the methods
	which any method implementing a factory for producing event 
	nuclei for a particular type of verbal predicate should implement. 
	
	@started 071026
	@version 071026
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public interface DialogueMoveFactory {


	public List<AbstractFeatureValue> getProperties();
	
	public String getLabel();

	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId);

} // end interface EventStructureFactory
