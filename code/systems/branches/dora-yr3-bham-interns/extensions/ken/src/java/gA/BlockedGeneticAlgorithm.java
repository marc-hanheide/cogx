package gA;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import exploration.PathTimes;
import exploration.PathTimesWrapper;
import exploration.TourFinder;

public class BlockedGeneticAlgorithm {

	private Vector<Vector<PathTimes>> paths;
	private Vector<PathTimes> pT;
	private Vector<PathTimes> best;
	private Vector<Vector<PathTimes>> temp;
	private int startPos;

	public static void main(String[] args) {

		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings2.txt")));

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
		System.out.println("begining value "
				+ TourFinder.evaluatePath(pathTimes, 0));
		//TourFinder tF = new TourFinder(pathTimes, 0);
	//	Vector<PathTimes> pT = tF.generateSimplePath(pathTimes, 0);
//		Vector<PathTimes> blocked =new Vector<PathTimes>();
//		blocked.add(pT.get(4));
		System.out.println(pathTimes.remove(4));
		Vector<PathTimes> travelled = new Vector<PathTimes>();
		travelled.add(pathTimes.get(0));
		travelled.add(pathTimes.get(1));
		travelled.add(pathTimes.get(2));
		travelled.add(pathTimes.get(3));
		travelled.add(pathTimes.get(4));
		BlockedGeneticAlgorithm gA = new BlockedGeneticAlgorithm(pathTimes,travelled, 200, 60, 4);
		Vector<PathTimes> path = gA.getBest();
		System.out.println("cost is " + TourFinder.evaluatePath(path, 4));
		for (PathTimes route : path) {
			System.out.println("A " + route.getA());
			System.out.println("B " + route.getB());
		}
	}

	/**
	 * assumes all the blocks have been removed and that this is a completly
	 * connected graph
	 * 
	 * @param pathTimes
	 * @param travelled
	 * @param popSize
	 * @param noOfGenerations
	 * @param startPos
	 */
	public BlockedGeneticAlgorithm(Vector<PathTimes> pathTimes,
			Vector<PathTimes> travelled, int popSize, int noOfGenerations,
			int startPos) {

		this.startPos = startPos;

		Vector<PathTimes> complete = TourFinder.copy(pathTimes);

		for (PathTimes path : travelled) {
			pathTimes.remove(path);
		}
		pT = complete;
		paths = new Vector<Vector<PathTimes>>();
		paths.add(pT);

		init(popSize);

		int gen = 0;
		System.out.println("created");
		while (gen < noOfGenerations) {
			System.out.println("gen is " + gen);
			
			System.out.println("best is "
					+ TourFinder.evaluateIncompletePath(getBest(), complete,
							startPos));
			
			selection();
			// System.out.println("selection done");
			reproduction();
			// System.out.println("reproduction done");
			gen++;
		}

	}

	private void init(int popSize) {

		for (int i = 0; i < popSize; i++) {
			paths.add(TourFinder.copy(pT));
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
		System.out.println("path size " + paths.size());
		Vector<Double> values = new Vector<Double>();
		double tot = 0;
		int total = 0;

		for (Vector<PathTimes> path : paths) {
			int val = TourFinder.evaluateIncompletePath(path, pT, startPos);
			total += val;
			double value = 1 / (double) val;
			values.add(value);
			tot += value;
		}
		System.out.println("average fitness is " + (tot / paths.size()));
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
		System.out.println("end of selection, path size is " + paths.size());
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

		for (Vector<PathTimes> path : paths) {
			int current = TourFinder.evaluatePath(path, startPos);

			if (current < bestV) {
				best = path;
				bestV = current;
			}
		}
		System.out.println("best from GA is " + bestV);
	}
	


}
