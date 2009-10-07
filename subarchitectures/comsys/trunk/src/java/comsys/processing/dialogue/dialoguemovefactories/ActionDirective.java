package comsys.processing.dialogue.dialoguemovefactories;

import comsys.processing.dialogue.DialogueMoveFactory;
import comsys.datastructs.comsysEssentials.*;
import comsys.datastructs.lf.*;

import java.util.TreeMap;
import comsys.utils.datastructs.*;
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
		dm.mType = MoveType.ACTIONDIRECTIVE ;
		dm.SDRSFormulaId1 = SDRSFormulaId1 ;
		dm.SDRSFormulaId2 = SDRSFormulaId2 ;	
		return dm;	
	}
}
