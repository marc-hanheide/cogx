package navigation;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

/**
 * class that when given a PathTimes object, will return the average of all it's
 * runs
 * 
 * @author kenslaptop
 * 
 */
public class NaivePathSelector extends PathSelector {

	@Override
	public Vector<Path> generateGraph(int time, boolean includeDay, int day) {
		Vector<PathTimes> pathTimes = load();
		//System.out.println("path times are "+pathTimes);
		Vector<Path> returnList = new Vector<Path>();
		for (PathTimes pathTime : pathTimes) {
			Vector<PathRun> runs = pathTime.getRuns();
			if (runs.size() == 0) {

				returnList.add(new Path(pathTime.getA(), pathTime.getB(),
						10 ^ 10));
				continue;
			}
			int count = 0;
			for (PathRun run : runs) {
				count += run.timeTaken();
			}
			count /= runs.size();
			returnList.add(new Path(pathTime.getA(), pathTime.getB(), count));
		}
		return returnList;
	}

	public Vector<PathTimes> load() {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}
		return pathTimes;

	}

}
