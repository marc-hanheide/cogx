package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class PolarFeatureQuestionIntention extends FeatureQuestionIntention {

	protected final String hypo;
	
	public PolarFeatureQuestionIntention(WorkingMemoryAddress entity, String feature, String hypo) {
		super(entity, feature);
		this.hypo = hypo;
	}

	public String getHypothesis() {
		return hypo;
	}


	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<PolarFeatureQuestionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		public final String SKEY_HYPOTHESIS = "hypothesis";

		@Override
		protected void encodeContent(PolarFeatureQuestionIntention rich, BaseIntention poor) {
			FeatureQuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBTYPE, "polar");
			poor.stringContent.put(SKEY_HYPOTHESIS, rich.getHypothesis());
		}

		@Override
		public PolarFeatureQuestionIntention tryDecode(BaseIntention poor) {
			try {
				FeatureQuestionIntention qint = FeatureQuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (qint != null) {
					DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "polar");
					String hypo = DecodingUtils.stringGet(poor, SKEY_HYPOTHESIS);
					return new PolarFeatureQuestionIntention(qint.getEntity(), qint.getFeatureName(), hypo);
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
