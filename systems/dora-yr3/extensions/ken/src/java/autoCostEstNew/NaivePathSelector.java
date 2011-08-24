package autoCostEstNew;

import java.util.Vector;

public class NaivePathSelector extends PathSelector {

	@Override
	public long computeTime(PathTimes pathTime) {
		Vector<PathRun> runs = pathTime.getRuns();
		int count = 0;
		for (PathRun run : runs) {
			count += run.timeTaken();
		}
		count /= runs.size();
		return count;
	}

}
