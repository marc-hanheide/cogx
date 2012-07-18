package de.dfki.lt.tr.dialogue.intentions;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

// NOTE: f***ing Java and her invariant collections render this class useless...
public class FirstMatchingDecoderCascade<Poor, Rich> implements Transcoder<Poor, Rich> {

	private final List<Transcoder<Poor, Rich>> transcoders;
	
	public FirstMatchingDecoderCascade(Collection<Transcoder<Poor, Rich>> ts) {
		transcoders = new LinkedList<Transcoder<Poor, Rich>>();
		transcoders.addAll(ts);
	}
	
	/**
	 * @return first matching, else null if none found
	 */
	@Override
	public Rich tryDecode(Poor obj) {
		Rich result = null;

		Iterator<Transcoder<Poor, Rich>> it = transcoders.iterator();
		while (it.hasNext() && result == null) {
			Transcoder<Poor, Rich> transcoder = it.next();
			result = transcoder.tryDecode(obj);
		}

		return result;
	}

	@Override
	public Poor tryEncode(Rich obj) {
		Poor result = null;
		
		Iterator<Transcoder<Poor, Rich>> it = transcoders.iterator();
		while (it.hasNext() && result == null) {
			Transcoder<Poor, Rich> transcoder = it.next();
			result = transcoder.tryEncode(obj);
		}

		return result;
	}
	
}
