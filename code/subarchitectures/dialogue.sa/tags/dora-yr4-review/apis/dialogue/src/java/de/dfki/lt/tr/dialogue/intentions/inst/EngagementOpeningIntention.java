package de.dfki.lt.tr.dialogue.intentions.inst;

import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.AbstractRichIntention;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class EngagementOpeningIntention extends AbstractRichIntention {

	public EngagementOpeningIntention() {
		super();
	}

	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<EngagementOpeningIntention> {

		public static final EngagementOpeningIntention.Transcoder INSTANCE = new EngagementOpeningIntention.Transcoder();

		@Override
		protected void encodeContent(EngagementOpeningIntention rich, BaseIntention poor) {
			poor.stringContent.put(SKEY_TYPE, "engagement-opening");
		}

		@Override
		public EngagementOpeningIntention tryDecode(BaseIntention poor) {
                    try {
                            DecodingUtils.stringCheck(poor, SKEY_TYPE, "engagement-opening");
                            return new EngagementOpeningIntention();
                    }
                    catch (DecodingException ex) {
                            return null;
                    }
                }
	}
}
