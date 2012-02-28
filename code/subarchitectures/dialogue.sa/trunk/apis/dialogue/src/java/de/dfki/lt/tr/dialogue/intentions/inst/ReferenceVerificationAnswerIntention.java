package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractRichIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.PolarityTransformer;

public class ReferenceVerificationAnswerIntention extends AbstractRichIntention {

	protected final WorkingMemoryAddress entity;
	protected final WorkingMemoryAddress verificationOf;
	protected final WorkingMemoryAddress answerTo;
	protected final boolean affirmative;
	
	public ReferenceVerificationAnswerIntention(WorkingMemoryAddress entity, WorkingMemoryAddress verificationOf, WorkingMemoryAddress answerTo, boolean affirmative) {
		this.entity = entity;
		this.verificationOf = verificationOf;
		this.answerTo = answerTo;
		this.affirmative = affirmative;
	}

	public WorkingMemoryAddress getEntity() {
		return entity;
	}

	public WorkingMemoryAddress getVerificationObjectAddress() {
		return verificationOf;
	}

	public WorkingMemoryAddress getQuestionAddress() {
		return answerTo;
	}

	public boolean isAffirmative() {
		return affirmative;
	}
	
	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<ReferenceVerificationAnswerIntention> {

		public static final Transcoder INSTANCE = new Transcoder();
		
		public static final String SKEY_POLARITY = "polarity";

		@Override
		protected void encodeContent(ReferenceVerificationAnswerIntention rich, BaseIntention poor) {
			poor.stringContent.put(SKEY_TYPE, "verification");
			poor.addressContent.put(PKEY_ABOUT, rich.getEntity());
		}

		@Override
		public ReferenceVerificationAnswerIntention tryDecode(BaseIntention poor) {
			try {
				DecodingUtils.stringCheck(poor, SKEY_TYPE, "assertion");
				boolean polarity = DecodingUtils.stringTransform(poor, SKEY_POLARITY, PolarityTransformer.INSTANCE);

				WorkingMemoryAddress about = DecodingUtils.addressGet(poor, PKEY_ABOUT);
				WorkingMemoryAddress verificationOf = DecodingUtils.addressGet(poor, PKEY_VERIFICATION_OF);
				WorkingMemoryAddress answerTo = DecodingUtils.addressGet(poor, PKEY_ANSWER_TO);

				return new ReferenceVerificationAnswerIntention(about, verificationOf, answerTo, polarity);

			}
			catch (DecodingException ex) {
				return null;
			}
		}
	}
	
}
