package motivation.components.filters;

import cast.cdl.WorkingMemoryChange;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;

interface MotiveFilter {
	MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc);

	void setManager(MotiveFilterManager motiveFilterManager);

	void start();
	
}
