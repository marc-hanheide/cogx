package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class FeatureQuestionIntention extends QuestionIntention {

	protected final WorkingMemoryAddress entity;
	protected final String feature;
	
	public FeatureQuestionIntention(WorkingMemoryAddress entity, String feature) {
		super();
		this.entity = entity;
		this.feature = feature;
	}

	public WorkingMemoryAddress getEntity() {
		return entity;
	}
	
	public String getFeatureName() {
		return feature;
	}


	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<FeatureQuestionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		public final String SKEY_FEATURE = "feature";

		@Override
		protected void encodeContent(FeatureQuestionIntention rich, BaseIntention poor) {
			QuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_FEATURE, rich.getFeatureName());
			poor.addressContent.put(PKEY_ABOUT, rich.getEntity());
		}

		@Override
		public FeatureQuestionIntention tryDecode(BaseIntention poor) {
			try {
				QuestionIntention qint = QuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (qint != null) {
					String feature = DecodingUtils.stringGet(poor, SKEY_FEATURE);
					WorkingMemoryAddress about = DecodingUtils.addressGet(poor, PKEY_ABOUT);
					return new FeatureQuestionIntention(about, feature);
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
