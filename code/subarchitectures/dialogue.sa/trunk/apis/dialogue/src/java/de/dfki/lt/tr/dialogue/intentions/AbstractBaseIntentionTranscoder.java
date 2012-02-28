package de.dfki.lt.tr.dialogue.intentions;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import java.util.HashMap;
import java.util.Map;

public abstract class AbstractBaseIntentionTranscoder<T extends RichIntention>
		implements Transcoder<BaseIntention, T> {

	public final String SKEY_TYPE = "type";
	public final String SKEY_SUBTYPE = "subtype";
	public final String SKEY_SUBSUBTYPE = "subsubtype";
	
	public final String PKEY_ABOUT = "about";
	public final String PKEY_ANSWER_TO = "answer-to";
	public final String PKEY_VERIFICATION_OF = "verification-of";
		

	@Override
	public BaseIntention tryEncode(T obj) {
		Map<String, String> stringContent = new HashMap<String, String>();
		Map<String, WorkingMemoryAddress> addressContent = new HashMap<String, WorkingMemoryAddress>();
		
		BaseIntention iint = new BaseIntention(stringContent, addressContent);
		
		encodeContent(obj, iint);
		
		return iint;
	}
	
	protected abstract void encodeContent(T obj, BaseIntention bint);
	
}
