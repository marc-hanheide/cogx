package motivation.components.filters;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;

interface MotiveFilter {
	MotivePriority shouldBeSurfaced(Motive motive);

	boolean shouldBeUnsurfaced(Motive motive);

	void setManager(MotiveFilterManager motiveFilterManager);
	
}
