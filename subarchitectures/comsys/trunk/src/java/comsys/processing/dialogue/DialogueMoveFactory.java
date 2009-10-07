//
//  DialogueMoveFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 6/12/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.dialogue;

import java.util.Iterator;
import java.util.List;
import comsys.utils.datastructs.*;
import comsys.datastructs.comsysEssentials.*;

public interface DialogueMoveFactory {

	public List<AbstractFeatureValue> getProperties();
	
	public String getLabel();
	
	public DialogueMove produceDialogueMove (String SDRSFormulaId1, String SDRSFormulaId2, String dialogueMoveId);
	
}
