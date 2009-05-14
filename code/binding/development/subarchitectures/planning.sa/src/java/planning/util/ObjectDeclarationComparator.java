package planning.util;

import java.util.Comparator;

import Planner.ObjectDeclaration;

public class ObjectDeclarationComparator
		implements
			Comparator<ObjectDeclaration> {

	public int compare(ObjectDeclaration _o1, ObjectDeclaration _o2) {

		// same object
		if (_o1 == _o2) {
			return 0;
		}

		// change id
		int comparison = _o1.name.compareToIgnoreCase(_o2.name);
		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		comparison = _o1.type.compareToIgnoreCase(_o2.type);

		if (comparison < 0) {
			return -1;
		}
		if (comparison > 0) {
			return 1;
		}

		return 0;

	}

}
