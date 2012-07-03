package de.dfki.lt.tr.dialogue.intentions;

import de.dfki.lt.tr.dialogue.intentions.DecodingUtils.StringTransformer;

public class PolarityTransformer implements StringTransformer<Boolean> {

	public static final PolarityTransformer INSTANCE = new PolarityTransformer();

	private PolarityTransformer() {
	}
		
	@Override
	public Boolean transform(String s) throws DecodingException {
		if (s.equals("pos")) {
			return true;
		}
		else if (s.equals("neg")) {
			return false;
		}
		else {
			throw new DecodingException();
		}
	}

}
