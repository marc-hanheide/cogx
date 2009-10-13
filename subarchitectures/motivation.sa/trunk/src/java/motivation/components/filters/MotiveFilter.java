package motivation.components.filters;

import cast.cdl.WorkingMemoryChange;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;

interface MotiveFilter {
	MotivePriority shouldBeSurfaced(Motive motive, WorkingMemoryChange wmc);

	boolean shouldBeUnsurfaced(Motive motive, WorkingMemoryChange wmc);

	void setManager(MotiveFilterManager motiveFilterManager);
	
}
