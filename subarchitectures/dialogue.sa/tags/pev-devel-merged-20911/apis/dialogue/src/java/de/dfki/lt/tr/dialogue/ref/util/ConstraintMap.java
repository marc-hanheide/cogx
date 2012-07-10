package de.dfki.lt.tr.dialogue.ref.util;

import de.dfki.lt.tr.dialogue.ref.Constraint;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class ConstraintMap {

	private final Map<String, String> content;

	protected ConstraintMap(Map<String, String> content) {
		this.content = content;
	}

	public ConstraintMap() {
		this(new HashMap<String, String>());
	}

	public Map<String, String> contentMap() {
		return content;
	}

	public List<Constraint> toConstraintList() {
		List<Constraint> result = new ArrayList<Constraint>();

		for (Entry<String, String> entry : content.entrySet()) {
			result.add(new Constraint(entry.getKey(), entry.getValue()));
		}

		return result;
	}

	public static ConstraintMap fromConstraintList(List<Constraint> constraints) {
		Map<String, String> map = new HashMap<String, String>();

		for (Constraint c : constraints) {
			String oldValue = map.put(c.feature, c.value);
			assert oldValue == null;
		}

		return new ConstraintMap(map);
	}

}
