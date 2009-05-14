package planning.util;

import java.util.Comparator;

import Planner.Fact;

public class FactComparator implements Comparator<Fact> {

	public int compare(Fact _f1, Fact _f2) {
		if (_f1 == _f2) {
			return 0;
		}

		// length of args
		int comparison = _f1.arguments.length - _f2.arguments.length;
		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		// name
		comparison = _f1.name.compareToIgnoreCase(_f2.name);
		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		// value
		comparison = _f1.value.compareToIgnoreCase(_f2.value);
		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		// each arg
		for (int i = 0; i < _f1.arguments.length; ++i) {
			comparison = _f1.arguments[i].compareToIgnoreCase(_f2.arguments[i]);
			if (comparison < 0) {
				return -1;
			}
			if (comparison > 0) {
				return 1;
			}
		}

		// modality
		if (_f1.modality.value() < _f2.modality.value()) {
			return -1;
		}
		if (_f1.modality.value() > _f2.modality.value()) {
			return 1;
		}

		comparison = _f1.agent.compareToIgnoreCase(_f2.agent);

		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		return 0;

	}

}
