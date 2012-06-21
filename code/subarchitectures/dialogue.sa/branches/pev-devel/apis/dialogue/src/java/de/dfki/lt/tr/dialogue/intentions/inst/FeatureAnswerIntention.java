package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.Certainty;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class FeatureAnswerIntention extends FeatureAscriptionIntention {

	protected final Certainty certainty;
	protected final WorkingMemoryAddress answerTo;
	
	public FeatureAnswerIntention(WorkingMemoryAddress entity, String feature, String value, boolean positivePolarity, Certainty certainty, WorkingMemoryAddress answerTo) {
		super(entity, feature, value, positivePolarity);
		this.certainty = certainty;
		this.answerTo = answerTo;
	}

	public Certainty getCertainty() {
		return certainty;
	}
	
	public WorkingMemoryAddress getQuestionAddress() {
		return answerTo;
	}


	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<FeatureAnswerIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		public static final String SKEY_CERTAINTY = "certainty";

		@Override
		protected void encodeContent(FeatureAnswerIntention rich, BaseIntention poor) {
			AssertionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBSUBTYPE, "answer");
			poor.stringContent.put(SKEY_CERTAINTY, rich.getCertainty().toNiceString());
			poor.addressContent.put(PKEY_ANSWER_TO, rich.getQuestionAddress());
		}

		@Override
		public FeatureAnswerIntention tryDecode(BaseIntention poor) {
			try {
				FeatureAscriptionIntention supint = FeatureAscriptionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (supint != null) {
					DecodingUtils.stringCheck(poor, SKEY_SUBSUBTYPE, "answer");
					Certainty certainty = DecodingUtils.stringTransform(poor, SKEY_CERTAINTY, Certainty.Transformer.INSTANCE);
					WorkingMemoryAddress answerTo = DecodingUtils.addressGet(poor, PKEY_ANSWER_TO);

					return new FeatureAnswerIntention(
						supint.getEntity(),
						supint.getFeatureName(),
						supint.getFeatureValue(),
						supint.isPositive(),
						certainty,
						answerTo);
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
