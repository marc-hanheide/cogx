package navigation;

import java.util.Vector;


import exploration.PathRun;
import exploration.PathTimes;

/**
 * class that when given a PathTimes object, will return the average of all it's runs
 * @author kenslaptop
 *
 */
public class NaivePathSelector extends PathSelector {

	@Override
	public long computeTime(PathTimes pathTime) {
		
		Vector<PathRun> runs = pathTime.getRuns();
		if(runs.size()==0){
			return 10^10;
		}
		int count = 0;
		for (PathRun run : runs) {
			count += run.timeTaken();
		}
		count /= runs.size();
		return count;
	}

}
