package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.CASTEffect;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.PolarityTransformer;
import de.dfki.lt.tr.dialogue.intentions.VerifiedBeliefUpdateEffect;
import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import java.util.HashMap;
import java.util.Map;

public class HypothesisGenerationVerificationAnswerIntention extends AssertionIntention {

        protected final WorkingMemoryAddress question;
	protected final boolean positivePolarity;
	
	public HypothesisGenerationVerificationAnswerIntention(WorkingMemoryAddress entity, WorkingMemoryAddress question, boolean positivePolarity) {
		super(entity);
                this.question = question;
		this.positivePolarity = positivePolarity;
	}

        public WorkingMemoryAddress getQuestionAddress() {
            return question;
        }
        
	public boolean isPositive() {
		return positivePolarity;
	}
	
	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<HypothesisGenerationVerificationAnswerIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		public final String SKEY_ASSERTED_POLARITY = "asserted-polarity";

		@Override
		protected void encodeContent(HypothesisGenerationVerificationAnswerIntention rich, BaseIntention poor) {
			AssertionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBTYPE, "hgv-answer");
			poor.stringContent.put(SKEY_ASSERTED_POLARITY, rich.isPositive() ? "pos" : "neg");
                        poor.addressContent.put(PKEY_ANSWER_TO, rich.getQuestionAddress());
                }

		@Override
		public HypothesisGenerationVerificationAnswerIntention tryDecode(BaseIntention poor) {
			try {
				AssertionIntention asint = AssertionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (asint != null) {
					DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "hgv-answer");
                                        WorkingMemoryAddress question = DecodingUtils.addressGet(poor, PKEY_ANSWER_TO);
					boolean polarity = DecodingUtils.stringTransform(poor, SKEY_ASSERTED_POLARITY, PolarityTransformer.INSTANCE);
					
					return new HypothesisGenerationVerificationAnswerIntention(asint.getEntity(), question, polarity);
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
