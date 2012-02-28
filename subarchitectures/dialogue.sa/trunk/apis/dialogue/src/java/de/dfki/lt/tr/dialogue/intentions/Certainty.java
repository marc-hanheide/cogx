package de.dfki.lt.tr.dialogue.intentions;

import de.dfki.lt.tr.dialogue.intentions.DecodingUtils.StringTransformer;

public enum Certainty {
	
	HIGH,
	LOW;

	public String toNiceString() {
		switch (this) {
			case HIGH: return "high";
			case LOW: return "low";
			default: return this.toString();
		}
	}

	public static class Transformer implements StringTransformer<Certainty> {

		public static final Transformer INSTANCE = new Transformer();

		@Override
		public Certainty transform(String s) throws DecodingException {
			if (s.equals("high")) {
				return HIGH;
			}
			else if (s.equals("low")) {
				return LOW;
			}
			else {
				throw new DecodingException();
			}
		}

}

}
