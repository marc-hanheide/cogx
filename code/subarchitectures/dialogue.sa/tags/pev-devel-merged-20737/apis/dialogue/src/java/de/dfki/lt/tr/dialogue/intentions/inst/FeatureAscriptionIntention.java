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

public class FeatureAscriptionIntention extends AssertionIntention {

	protected final String feature;
	protected final String value;
	protected final boolean positivePolarity;
	
	public FeatureAscriptionIntention(WorkingMemoryAddress entity, String feature, String value, boolean positivePolarity) {
		super(entity);
		this.feature = feature;
		this.value = value;
		this.positivePolarity = positivePolarity;
	}

	public String getFeatureName() {
		return feature;
	}
	
	public String getFeatureValue() {
		return value;
	}
	
	public boolean isPositive() {
		return positivePolarity;
	}
	
	@Override
	public CASTEffect getOnSuccessEffect() {
		Map<String, dFormula> updates = new HashMap<String, dFormula>();
		updates.put(getFeatureName(), BeliefFormulaFactory.maybeNegatedElementaryFormula(getFeatureValue(), !isPositive()));
		return new VerifiedBeliefUpdateEffect(getEntity(), updates);
	}
	
	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<FeatureAscriptionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		public final String SKEY_ASSERTED_FEATURE = "asserted-feature";
		public final String SKEY_ASSERTED_VALUE = "asserted-value";
		public final String SKEY_ASSERTED_POLARITY = "asserted-polarity";

		@Override
		protected void encodeContent(FeatureAscriptionIntention rich, BaseIntention poor) {
			AssertionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
			poor.stringContent.put(SKEY_SUBTYPE, "ascription");
			poor.stringContent.put(SKEY_ASSERTED_FEATURE, rich.getFeatureName());
			poor.stringContent.put(SKEY_ASSERTED_VALUE, rich.getFeatureValue());
			poor.stringContent.put(SKEY_ASSERTED_POLARITY, rich.isPositive() ? "pos" : "neg");
		}

		@Override
		public FeatureAscriptionIntention tryDecode(BaseIntention poor) {
			try {
				AssertionIntention asint = AssertionIntention.Transcoder.INSTANCE.tryDecode(poor);
				if (asint != null) {
					DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "ascription");
					String feature = DecodingUtils.stringGet(poor, SKEY_ASSERTED_FEATURE);
					String value = DecodingUtils.stringGet(poor, SKEY_ASSERTED_VALUE);
					boolean polarity = DecodingUtils.stringTransform(poor, SKEY_ASSERTED_POLARITY, PolarityTransformer.INSTANCE);
					
					return new FeatureAscriptionIntention(asint.getEntity(), feature, value, polarity);
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
