package org.cognitivesystems.comsys.processing.dialoguemovefactories;

import java.util.TreeMap;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedNominal;
import org.cognitivesystems.comsys.util.datastructs.*;
import java.util.ArrayList;

/**
 * Dialogue Move factory for assertive utterances
 * @author plison
 *
 */
public class QuestionYN extends AbstractDialogueMoveFactory implements DialogueMoveFactory  {

	public QuestionYN () {
		super();
		localInit();
	}
	

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_avm = new ArrayList<AbstractFeatureValue>();
		_avm.add(new FeatureValue("mood", "int"));
		_avm.add(new FeatureValue("sort", "ascription"));
		_avm.add(new FeatureValue("wh-restrictor", "no"));
		_label = "QUESTION-YN";
	} // end 
	
	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId) {
		DialogueMove dm = new DialogueMove();
		dm.mType = MoveType.QUESTION_YN ;
		dm.SDRSFormulaId1 = SDRSFormulaId1 ;
		dm.SDRSFormulaId2 = SDRSFormulaId2 ;	
		return dm;	
	}
}
