package comsys.processing.dialogue.dialoguemovefactories;

import java.util.TreeMap;
import comsys.processing.dialogue.DialogueMoveFactory;
import comsys.datastructs.comsysEssentials.*;
import comsys.datastructs.lf.*;
import comsys.utils.datastructs.*;
import java.util.ArrayList;

/**
 * Dialogue Move factory for assertive utterances
 * @author plison
 *
 */
public class QuestionW extends AbstractDialogueMoveFactory implements DialogueMoveFactory  {

	public QuestionW () {
		super();
		localInit();
	}
	

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_avm = new ArrayList<AbstractFeatureValue>();
		_avm.add(new FeatureValue("mood", "int"));
		_avm.add(new FeatureValue("sort", "ascription"));
		_avm.add(new FeatureValue("wh-restrictor", "yes"));
		_label = "QUESTION-W";
	} // end 
	
	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId) {
		DialogueMove dm = new DialogueMove();
		dm.mType = MoveType.QUESTIONW ;
		dm.SDRSFormulaId1 = SDRSFormulaId1 ;
		dm.SDRSFormulaId2 = SDRSFormulaId2 ;	
		return dm;	
	}
}
