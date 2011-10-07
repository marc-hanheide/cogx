package de.dfki.lt.tr.dialogue.util;

public class BasicNominalRemapper
implements NominalRemapper {

	private final int index;

	public BasicNominalRemapper(int index) {
		this.index = index;
	}

	@Override
	public String remap(String nom) {
		return "n" + index + ":" + nom;
	}

	@Override
	public String unremap(String nom) {
		String[] sub = nom.split(":");
		assert sub.length == 2;
		return sub[1];
	}

}
