package navigation;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.ArrayList;
import java.util.Date;
import java.util.Vector;

import clustering.ProbGenerator;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

/**
 * class that when given a PathTimes object, will return the average of all it's
 * runs
 * 
 * @author kenslaptop
 * 
 */
public class IntelligentPathSelector extends PathSelector {

	Vector<PathTimes> pathTimes;

	@Override
	public Vector<Path> generateGraph(int time, boolean includeDay, int day) {
		Vector<Path> paths = new Vector<Path>();
		int n = load().size();
		for (int i = 0; i < n; i++) {
			System.out.println("position is " + i);
			ProbGenerator g = new ProbGenerator(i, false, false,includeDay, day);
			ArrayList<Integer> means = g.getMeans();
			
			try {
				int cost = g.getEstimate(time);
				System.out.println("cost is " + cost);
			paths.add(new Path(pathTimes.get(i).getA(),
					pathTimes.get(i).getB(), cost));
			} catch (NullPointerException e) {
				paths.add(new Path(pathTimes.get(i).getA(),
						pathTimes.get(i).getB(), 100000));
			}
			// System.out.println("pos is "+pos);
			// double[] times = g.getProbs(pos - 1);
			//			
			// System.out.println("end of times values");
			// pos = 0;
			// double highest = 0;
			// for (int j = 0; j < times.length; j++) {
			//				
			// if (times[j] > highest && times[j]!=Double.NaN) {
			// highest = times[j];
			// pos = i;
			// }
			// }

			

		}

		return paths;
	}

	public Vector<PathTimes> load() {
		pathTimes = new Vector<PathTimes>();
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
