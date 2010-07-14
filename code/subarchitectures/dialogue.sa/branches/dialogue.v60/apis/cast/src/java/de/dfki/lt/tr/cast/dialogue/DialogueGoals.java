//=================================================================
// Copyright (C) 2006-2010 DFKI GmbH Talking Robots
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.cast.dialogue;
	
	/** 
	 * The class <tt>DialogueGoals</tt> specifies the encoding for the 
	 * different goals that the goal-driven processes in a dialogue processing
	 * subarchitecture can propose. 
	 *   
	 * @author  Geert-Jan M. Kruijff (gj@dfki.de) 
	 * @since 	060924
	 * @version	100608
	*/

public class DialogueGoals {

	// Speech recognition 
	public static final String ASR_TASK = "speech.rec";

    // Parsing
    public static final String INCREMENTAL_PARSING_STEP_TASK = "incrParsingStep";
    
    // Packing
    public static final String PACKEDLF_PROCESSING_TASK = "packedLFprocessingStep";
    
    // Parse selection 
    public static final String PARSESELECTION_TASK = "parseselection";

    // Referential readings determination
    public static final String REFERENTIALREADINGS_TASK = "referentialReadings";			
	
    // Discourse Referents Binding
    public static final String DISCREFBINDING_TASK = "discRefBinding";	
 
    // Event Structure Interpretation
    public static final String EVENSTRUCTUREINTERPRETATION_TASK = "eventStructureInt";		

    // Dialogue Move Interpretation
    public static final String DIALOGUEMOVEINTERPRETATION_TASK = "dialogueMoveInt";		
	   		         
	// Dialogue Production
	public static final String DIALPLAN_TASK = "dialPlan";

	// Content planning
	public static final String CONTENTPLANNING_TASK = "contentPlanning";		
	
	// Realization 
	public static final String REALIZATION_TASK = "realization";	
	
	// Speech synthesis
	public static final String SPEECHSYNTHESIS_TASK = "speechSynth";

	// Intention recognition
	public static final String INTENTION_RECOGNITION_TASK = "intentionRecognition";

	// Intention realisation
	public static final String INTENTION_REALISATION_TASK = "intentionRealisation";

} // end class
