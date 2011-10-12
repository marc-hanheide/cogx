package de.dfki.lt.tr.cast.dialogue.util;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.LFUtils;

public class VerbalisationUtils {

	public static final String DIALOGUE_SA = "dialogue";

	public static void verbaliseString(WorkingMemoryWriterComponent component, String text) {
		try {
			WorkingMemoryAddress wma = new WorkingMemoryAddress(component.newDataID(), DIALOGUE_SA);
			ContentPlanningGoal cpg = cannedContentPlanningGoal(wma.id, text);
			component.addToWorkingMemory(wma, cpg);
		}
		catch (SubarchitectureComponentException ex) {
			component.logException(ex);
		}
	}

	public static ContentPlanningGoal cannedContentPlanningGoal(String id, String text) {
		LogicalForm lf = LFUtils.convertFromString("@d:dvp(<CannedText>" + text.replace(" ", "_") + ")");
		return new ContentPlanningGoal(id, lf, null);
	}

}
