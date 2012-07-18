package de.dfki.lt.tr.cast.dialogue.util;

import cast.architecture.WorkingMemoryWriterComponent;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;

// FIXME: this is a bad bad fix reversing code dependencies
/**
 * 
 * @deprecated  use de.dfki.lt.tr.dialogue.util.VerbalisationUtils instead
 */
@Deprecated
public class VerbalisationUtils {

        /**
         * 
         * @deprecated  use de.dfki.lt.tr.dialogue.util.VerbalisationUtils instead
         */
        @Deprecated
	public static final String DIALOGUE_SA = de.dfki.lt.tr.dialogue.util.VerbalisationUtils.DIALOGUE_SA;

        /**
         * 
         * @deprecated  use de.dfki.lt.tr.dialogue.util.VerbalisationUtils instead
         */
        @Deprecated
	public static void verbaliseString(WorkingMemoryWriterComponent component, String text) {
            de.dfki.lt.tr.dialogue.util.VerbalisationUtils.verbaliseString(component, text);
	}

        /**
         * 
         * @deprecated  use de.dfki.lt.tr.dialogue.util.VerbalisationUtils instead
         */
        @Deprecated
	public static ContentPlanningGoal cannedContentPlanningGoal(String id, String text) {
            return de.dfki.lt.tr.dialogue.util.VerbalisationUtils.cannedContentPlanningGoal(id, text);
	}

}
