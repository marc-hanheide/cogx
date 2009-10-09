package motivation.components.filters;

import motivation.slice.Motive;

interface MotiveFilter {
	boolean shouldBeSurfaced(Motive motive);

	boolean shouldBeUnsurfaced(Motive motive);

	void setManager(MotiveFilterManager motiveFilterManager);
	
}
