package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractRichIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;

public class AssertionIntention extends AbstractRichIntention {

	protected final WorkingMemoryAddress entity;
	
	public AssertionIntention(WorkingMemoryAddress entity) {
		this.entity = entity;
	}

	public WorkingMemoryAddress getEntity() {
		return entity;
	}
	
	public static class Transcoder
			extends AbstractBaseIntentionTranscoder<AssertionIntention> {

		public static final Transcoder INSTANCE = new Transcoder();

		@Override
		protected void encodeContent(AssertionIntention rich, BaseIntention poor) {
			poor.stringContent.put(SKEY_TYPE, "assertion");
			poor.addressContent.put(PKEY_ABOUT, rich.getEntity());
		}

		@Override
		public AssertionIntention tryDecode(BaseIntention poor) {
			try {
				DecodingUtils.stringCheck(poor, SKEY_TYPE, "assertion");
				WorkingMemoryAddress about = DecodingUtils.addressGet(poor, PKEY_ABOUT);

				return new AssertionIntention(about);

			}
			catch (DecodingException ex) {
				return null;
			}
		}
	}
	
}
