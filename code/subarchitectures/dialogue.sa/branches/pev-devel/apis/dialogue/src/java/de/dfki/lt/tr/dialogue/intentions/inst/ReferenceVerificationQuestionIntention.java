package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class ReferenceVerificationQuestionIntention extends QuestionIntention {

	protected final WorkingMemoryAddress verificationOf;
	
	public ReferenceVerificationQuestionIntention(WorkingMemoryAddress verificationOf) {
		super();
		this.verificationOf = verificationOf;
	}

	public WorkingMemoryAddress getVerifiedObjectAddress() {
		return verificationOf;
	}
	

	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<ReferenceVerificationQuestionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		@Override
		protected void encodeContent(ReferenceVerificationQuestionIntention rich, BaseIntention poor) {
			QuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBTYPE, "verification");
			poor.addressContent.put(PKEY_VERIFICATION_OF, rich.getVerifiedObjectAddress());
		}

		@Override
		public ReferenceVerificationQuestionIntention tryDecode(BaseIntention poor) {
			try {
				QuestionIntention qint = QuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (qint != null) {
					DecodingUtils.stringCheck(poor, SKEY_TYPE, "verification");
					WorkingMemoryAddress verificationOf = DecodingUtils.addressGet(poor, PKEY_VERIFICATION_OF);
					return new ReferenceVerificationQuestionIntention(verificationOf);
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
