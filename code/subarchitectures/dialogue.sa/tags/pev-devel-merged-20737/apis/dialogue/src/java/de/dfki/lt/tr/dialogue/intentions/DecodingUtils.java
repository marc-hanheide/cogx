package de.dfki.lt.tr.dialogue.intentions;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;

public abstract class DecodingUtils {

	public static String stringGet(BaseIntention iint, String key)
			throws DecodingException {

		String v = iint.stringContent.get(key);

		if (v == null) throw new DecodingException();
		
		return v;
	}

	public static void stringCheck(BaseIntention iint, String key, String value)
			throws DecodingException {
		
		String v = stringGet(iint, key);
		if (!v.equals(value)) throw new DecodingException();		
	}

	public static <T> T stringTransform(BaseIntention iint, String key, StringTransformer<T> tf)
			throws DecodingException {
	
		String v = stringGet(iint, key);
		return tf.transform(v);
	}

	public static WorkingMemoryAddress addressGet(BaseIntention iint, String key)
			throws DecodingException {
		WorkingMemoryAddress wma = iint.addressContent.get(key);

		if (wma == null) throw new DecodingException();
		
		return wma;
	}

	public static interface StringTransformer<T> {
		public T transform(String s) throws DecodingException;
	}

}
