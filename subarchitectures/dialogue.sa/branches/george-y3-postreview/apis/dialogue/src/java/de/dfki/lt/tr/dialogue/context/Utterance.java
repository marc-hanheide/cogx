package de.dfki.lt.tr.dialogue.context;

public class Utterance {

	private final String id;

	public Utterance(String id) {
		if (id == null) {
			throw new NullPointerException("id null in utterance initialisation");
		}
		this.id = id;
	}

	public final String getId() {
		return id;
	}

	public static class Builder {

		public Utterance build(String id) {
			return new Utterance(id);
		}

	}

}
