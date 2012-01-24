package motivation.components.filters;

import java.util.Map;

import cast.cdl.WorkingMemoryChange;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;

interface MotiveFilter {
	MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc);

	void setManager(MotiveFilterManager motiveFilterManager);

	void start();

	void configure(Map<String, String> arg0);
	
}
