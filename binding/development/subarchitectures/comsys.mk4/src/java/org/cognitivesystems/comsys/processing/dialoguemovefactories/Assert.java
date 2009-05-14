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
public class Assert extends AbstractDialogueMoveFactory implements DialogueMoveFactory  {

	public Assert () {
		super();
		localInit();
	}
	

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_avm = new ArrayList<AbstractFeatureValue>();
		_avm.add(new FeatureValue("mood", "ind"));
		_avm.add(new FeatureValue("sort", "ascription"));
		_avm.add(new FeatureValue("tense", "pres"));
		_label = "ASSERT";
	} // end 
	
	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId) {
		DialogueMove dm = new DialogueMove();
		dm.mType = MoveType.ASSERT ;
		dm.SDRSFormulaId1 = SDRSFormulaId1 ;
		dm.SDRSFormulaId2 = SDRSFormulaId2 ;	
		return dm;	
	}
}
