package de.dfki.lt.tr.dialogue.util;

public class NoOpNominalRemapper
implements NominalRemapper {

	public static final NoOpNominalRemapper INSTANCE = new NoOpNominalRemapper();

	@Override
	public String remap(String nom) {
		return nom;
	}

	@Override
	public String unremap(String nom) {
		return nom;
	}
	
}
