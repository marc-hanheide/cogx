package gA;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import org.jaxen.function.ext.EvaluateFunction;

import exploration.PathTimes;
import exploration.PathTimesWrapper;
import exploration.TourFinder;

public class BlockedGeneticAlgorithm {

	private Vector<Vector<PathTimes>> paths;
	private Vector<PathTimes> pT;
	private Vector<PathTimes> best;
	private Vector<Vector<PathTimes>> temp;
	// private Vector<PathTimes> complete;
	private int startPos;

	public static void main(String[] args) {

		String[] files = { "howarth2.txt", "timings.txt", "timings1.txt", "timings2.txt",
				"timings3.txt" };
		for (int i = 0; i < files.length; i++) {
			Vector<PathTimes> pathTimes = new Vector<PathTimes>();
			try {
				ObjectInputStream in = new ObjectInputStream(
						new BufferedInputStream(new FileInputStream(files[i])));

				pathTimes = ((PathTimesWrapper) (in.readObject()))
						.getPathTimes();

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

			// System.out.println("path times is");
			// System.out.println(pathTimes);
			for (int j = 0; j < 5; j++) {
				int startPos = j * 10;

				int standard = TourFinder.evaluatePath(pathTimes, pathTimes,
						startPos);
				// System.out.println("standard "+ );
				long time = System.currentTimeMillis();
				Vector<PathTimes> simple = TourFinder.generateSimplePath(
						pathTimes, startPos);
				long simpleTime = time - System.currentTimeMillis();
				int simpleT = TourFinder.evaluateIncompletePath(simple,
						pathTimes, startPos);
				int[] gaT = new int[10];
				long[] gaTime = new long[10];
				for (int k = 0; k < 10; k++) {
					time = System.currentTimeMillis();
					BlockedGeneticAlgorithm gA = new BlockedGeneticAlgorithm(
							pathTimes, new Vector<PathTimes>(),
							new Vector<PathTimes>(), 200, k * 10, startPos);
					gaTime[k] = System.currentTimeMillis() - time;
					Vector<PathTimes> path = gA.getBest();

					gaT[k] = TourFinder.evaluateIncompletePath(path, pathTimes,
							startPos);
				}

				System.out.println("For " + files[i] + " and start position "
						+ startPos);
				System.out.println("standard is " + standard);
				System.out.println("simple is " + simpleT + " and took "
						+ simpleTime);
				for (int k = 0; k < gaT.length; k++) {
					System.out.println("Ga of size " + k * 10 + " gave "
							+ gaT[k] + " and took " + gaTime[k]);
				}
			}

		}
	}

	/**
	 * assumes all the blocks have been removed and that this is a completly
	 * connected graph
	 * 
	 * @param pathTimes
	 * @param travelled
	 * @param blocked
	 * @param popSize
	 * @param noOfGenerations
	 * @param startPos
	 */
	public BlockedGeneticAlgorithm(Vector<PathTimes> pathTimes,
			Vector<PathTimes> travelled, Vector<PathTimes> blocked,
			int popSize, int noOfGenerations, int startPos) {
		//System.out.println("in GA pathTimes is ");
		//PathTimes.printList(pathTimes);
	//	System.out.println("travelled is ");
		//PathTimes.printList(travelled);
		this.startPos = startPos;

		Vector<PathTimes> complete = TourFinder.copy(pathTimes);
		for (PathTimes bl : blocked) {
			complete.remove(blocked);

		}

		pT = complete;

		paths = new Vector<Vector<PathTimes>>();
		paths.add(TourFinder.generateSimplePath(TourFinder.copy(pT), blocked,
				travelled, startPos));
		//System.out.println("starting path is ");

		for (PathTimes path : travelled) {
			paths.get(0).remove(path);

		}
	//	PathTimes.printList(paths.get(0));

		// paths.add(TourFinder.generateSimplePath(paths.remove(0), startPos));
		init(popSize);

		int gen = 0;
		//System.out.println("GA created");
		// if (pT.size() > 3) {
		if (pathTimes.size() == 0) {
			System.out.println("there are no path times");
			return;
		}
		while (gen < noOfGenerations) {
			//System.out.println("gen is " + gen);

			// System.out.println("best is "
			// + TourFinder.evaluateIncompletePath(getBest(), complete,
			// startPos));

			// looks like here is our problem?

			selection();
			// System.out.println("post sel");
			// System.out.println("selection done");
			reproduction();
			// System.out.println("post re");
			// System.out.println("reproduction done");
			gen++;
			// }
			// }else{System.out.println( );
		}

	//	System.out.println("GA finished");

	}

	private void init(int popSize) {

		for (int i = 0; i < popSize; i++) {
			paths.add(TourFinder.copy(paths.get(0)));
		}
	}

	private Vector<PathTimes> randomPath() {
		Vector<PathTimes> temp = TourFinder.copy(pT);
		Vector<PathTimes> returnPath = new Vector<PathTimes>();
		while (!temp.isEmpty()) {
			returnPath.add(temp.remove(getRandomPos(temp.size())));
		}
		return returnPath;
	}

	private int getRandomPos(int length) {
		return (int) (Math.random() * length);

	}

	public Vector<Vector<PathTimes>> crossOver(Vector<PathTimes> mama,
			Vector<PathTimes> papa) {

		Vector<Vector<PathTimes>> returnV = new Vector<Vector<PathTimes>>();
		returnV.add(papa);
		returnV.add(mama);
		return returnV;
	}

	private void selection() {
		// System.out.println("path size " + paths.size());
		Vector<Double> values = new Vector<Double>();
		double tot = 0;
		int total = 0;
		// System.out.println("eval");
		for (Vector<PathTimes> path : paths) {
			int val = TourFinder.evaluateIncompletePath(path, pT, startPos);
			total += val;
			double value = 1 / (double) val;
			values.add(value);
			tot += value;
		}
		// System.out.println("post eval");
		// System.out.println("tot is "+tot);
		// System.out.println("average fitness is " + (tot / paths.size()));
		int gen = paths.size();
		temp = new Vector<Vector<PathTimes>>();

		while (temp.size() < gen / 2) {

			int pos = getRandomPos(values.size());
			double v = values.get(pos);
			double value = v / tot;
			if (value > Math.random()) {
				temp.add(paths.remove(pos));
				values.remove(pos);
			}

		}
		// System.out.println("end of selection, path size is " + paths.size());
	}

	private void reproduction() {
		paths.clear();

		while (!temp.isEmpty()) {
			Vector<Vector<PathTimes>> child = crossOver(temp
					.remove(getRandomPos(temp.size())), temp
					.remove(getRandomPos(temp.size())));
			paths.add(child.get(0));
			paths.add(mutate(child.get(0)));
			paths.add(child.get(1));
			paths.add(mutate(child.get(1)));
		}
		// System.out.println("path length " + paths.size());
	}

	public Vector<PathTimes> mutate(Vector<PathTimes> normal) {
		Vector<PathTimes> mutant = TourFinder.copy(normal);

		while (Math.random() < Math.random()) {

			PathTimes t = mutant.remove(getRandomPos(mutant.size()));
			mutant.insertElementAt(t, getRandomPos(mutant.size()));
		}

		return mutant;

	}

	public Vector<PathTimes> getBest() {

		findBest();

		return best;
	}

	public void findBest() {

		int bestV = Integer.MAX_VALUE;

		// if(paths.get(0).size()==1){
		// best = paths.get(0);
		// return;
		// }
		for (Vector<PathTimes> path : paths) {
			// System.out.println("current path looked at is size "+path.size());
			int current = TourFinder.evaluatePath(path, pT, startPos);

			if (current < bestV) {
				best = path;
				bestV = current;
			}
		}
		//System.out.println("best from GA is " + bestV);
		//System.out.println("route is ");
		//PathTimes.printList(best);
	}

}
