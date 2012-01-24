package de.dfki.lt.tr.dialogue.context;

import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import java.util.HashMap;
import java.util.Map;

public class Context {

	private final IdentifierGenerator<String> idGen;
	private final Map<String, Utterance> utts;

	public Context(IdentifierGenerator<String> idGen) {
		if (idGen == null) {
			throw new NullPointerException("id generator null in context init");
		}
		this.idGen = idGen;
		utts = new HashMap<String, Utterance>();
	}

	public String addNewUtterance(Utterance.Builder u) {
		String id = idGen.newIdentifier();
		assert utts.put(id, u.build(id)) == null;
		return id;
	}

}
