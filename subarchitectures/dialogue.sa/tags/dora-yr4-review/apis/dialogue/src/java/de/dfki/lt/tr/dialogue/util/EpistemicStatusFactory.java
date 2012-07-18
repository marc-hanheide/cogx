package de.dfki.lt.tr.dialogue.util;

import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import java.util.LinkedList;

public abstract class EpistemicStatusFactory {

	public static PrivateEpistemicStatus newPrivateEpistemicStatus(String agent) {
		return new PrivateEpistemicStatus(agent);
	}

	public static AttributedEpistemicStatus newAttributedEpistemicStatus(String ag1, String ag2) {
		AttributedEpistemicStatus epst = new AttributedEpistemicStatus(ag1, new LinkedList<String>());
		epst.attribagents.add(ag2);
		return epst;
	}

	public static SharedEpistemicStatus newSharedEpistemicStatus(String ag1, String ag2) {
		SharedEpistemicStatus epst = new SharedEpistemicStatus(new LinkedList<String>());
		epst.cgagents.add(ag1);
		epst.cgagents.add(ag2);
		return epst;
	}

}
