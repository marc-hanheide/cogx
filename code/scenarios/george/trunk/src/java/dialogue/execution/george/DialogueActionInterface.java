package dialogue.execution.george;

import dialogue.execution.AbstractDialogueActionInterface;
import eu.cogx.beliefs.slice.MergedBelief;

/**
 * Receives actions from the execution system and interfaces with the rest of
 * the dialogue system. This contains any George-specific code.
 * 
 * @author nah
 * 
 */
public class DialogueActionInterface extends
		AbstractDialogueActionInterface<MergedBelief> {

	public DialogueActionInterface() {
		super(MergedBelief.class);
	}

}
