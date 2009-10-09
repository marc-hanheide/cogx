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
// CLASS DOCUMENTATION 
//=================================================================

/** The class <b>ComsysGoals</b> specifies the encoding for the 
	different goals that the goal-driven processes in the comsys
	subarchitecture can propose. Below, the goals have been sorted
	by the kind(s) of goal-driven processes that propose them.  


	@version 070907 (started 060924) 
	@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/


public class ComsysGoals {

	// Speech recognition 
	public static final String ASR_TASK = "speech.rec";

    // Parsing
    public static final String INCREMENTAL_PARSING_STEP_TASK = "incrParsingStep";
    
    // Packing
    public static final String PACKEDLF_PROCESSING_TASK = "packedLFprocessingStep";
    
    // Referential readings determination
    public static final String REFERENTIALREADINGS_TASK = "referentialReadings";			

	// Referential readings determination
    public static final String REFERENTIALBINDINGS_TASK = "referentialBindings";	
    
    // Discourse Referents Binding
    public static final String DISCREFBINDING_TASK = "discRefBinding";	
 
    // Event Structure Interpretation
    public static final String EVENSTRUCTUREINTERPRETATION_TASK = "eventStructureInt";		

    // Dialogue Move Interpretation
    public static final String DIALOGUEMOVEINTERPRETATION_TASK = "dialogueMoveInt";		
	   		         
	// Dialogue Production
	public static final String DIALPLAN_TASK = "dialPlan";

	public static final String CONTENTPLANNING_TASK = "contentPlanning";		
	
	public static final String REALIZATION_TASK = "realization";	
	
	public static final String SPEECHSYNTHESIS_TASK = "speechSynth";
	
	public static final String PARSESELECTION_TASK = "parseselection";

	public static final String CCA_BM_TASK = "ccaBM";
	public static final String CCA_SENSE_TASK = "ccaSense";
	public static final String CCA_CLARIFICATION_REQUEST_TASK = "ccaClarif";
	public static final String CCA_UNDERSTAND_TASK = "ccaUnderstand";
	public static final String CCA_TACIT_ACTION_TASK = "ccaTacitAction";
	public static final String CCA_WAS_VERIFIED_TASK = "ccaWasVerified";
	
} // end class
