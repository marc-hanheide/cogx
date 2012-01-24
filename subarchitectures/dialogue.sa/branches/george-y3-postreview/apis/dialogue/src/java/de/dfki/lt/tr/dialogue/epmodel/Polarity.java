package de.dfki.lt.tr.dialogue.epmodel;

public enum Polarity {

	POS,
	NEG;

	public static Polarity fromBoolean(boolean isPositive) {
		return isPositive ? POS : NEG;
	}

	public static boolean isPositive(Polarity pol) {
		return pol == POS;
	}

}
