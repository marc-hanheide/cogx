package de.dfki.lt.tr.dialogue.intentions.inst;

import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractRichIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class QuestionIntention extends AbstractRichIntention {

	public QuestionIntention() {
	}

	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<QuestionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		@Override
		protected void encodeContent(QuestionIntention rich, BaseIntention poor) {
			poor.stringContent.put(SKEY_TYPE, "question");
		}

		@Override
		public QuestionIntention tryDecode(BaseIntention poor) {
			try {
				DecodingUtils.stringCheck(poor, SKEY_TYPE, "question");
				return new QuestionIntention();
			}
			catch (DecodingException ex) {
				return null;
			}
		}
	}
	
}
