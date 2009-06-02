package comsys.processing.dialogue.dialoguemovefactories;

import comsys.utils.datastructs.*;
import java.util.List;

public abstract class AbstractDialogueMoveFactory {

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	
	/** List of attribute values specifying the necessary properties of the
	 * the dialoge move (ie. "mood: interrogative", "sort: action-motion", etc.)
	 */
	protected List<AbstractFeatureValue> _avm;
	
	protected String _label;
	
	public List<AbstractFeatureValue> getProperties() { return _avm; } 
	
	public AbstractDialogueMoveFactory () { 
		init();
	} // end constructor
	
	public String getLabel() { return _label; }

	protected void init () { 
	} // end 

}
