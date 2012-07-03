package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class OpenFeatureQuestionIntention extends FeatureQuestionIntention {

	public OpenFeatureQuestionIntention(WorkingMemoryAddress entity, String feature) {
		super(entity, feature);
	}


	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<OpenFeatureQuestionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		@Override
		protected void encodeContent(OpenFeatureQuestionIntention rich, BaseIntention poor) {
			FeatureQuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBTYPE, "open");
		}

		@Override
		public OpenFeatureQuestionIntention tryDecode(BaseIntention poor) {
			try {
				FeatureQuestionIntention qint = FeatureQuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (qint != null) {
					DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "open");
					return new OpenFeatureQuestionIntention(qint.getEntity(), qint.getFeatureName());
				}
				else {
					return null;
				}
			}
			catch (DecodingException ex) {
				return null;
			}
		}
	}
	
}
