package org.cognitivesystems.comsys.processing.dialoguemovefactories;

import java.util.TreeMap;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedNominal;
import org.cognitivesystems.comsys.util.datastructs.*;
import java.util.ArrayList;
import java.util.Vector;

/**
 * Dialogue Move factory for assertive utterances
 * @author plison
 *
 */
public class ActionDirective extends AbstractDialogueMoveFactory implements DialogueMoveFactory  {

	public ActionDirective () {
		super();
		localInit();
	}
	

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_avm = new ArrayList<AbstractFeatureValue>();
		_avm.add(new FeatureValue("mood", "imp"));
	//	Vector<String> values = new Vector<String>();
	//	values.add("action-motion");
	//	values.add("action-non-motion");
		_avm.add(new FeatureValue("sort", "action-non-motion"));
	//	_avm.add(new FeatureValue("tense", "pres"));
		_label = "ACTION-DIRECTIVE";
	} // end 
	
	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId) {
		DialogueMove dm = new DialogueMove();
		dm.mType = MoveType.ACTION_DIRECTIVE ;
		dm.SDRSFormulaId1 = SDRSFormulaId1 ;
		dm.SDRSFormulaId2 = SDRSFormulaId2 ;	
		return dm;	
	}
}
