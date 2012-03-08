package clustering;

import java.util.ArrayList;

import weka.core.Instance;

public class ProbGenerator {

	private Cluster c;
	private ArrayList<ArrayList<Instance>> instances;
	private ArrayList<Instance> complete;
	private ArrayList<Integer> bounds;
	private boolean includeDay;

	private double[][] joint;
	private double[] hypothesis;
	private double[] timeOfDay;

	private double[][][] jointDay;
	private double[] day;
	private double[][] timeAndDay;

	private double minEntropy = 0.5;

	private boolean messy;

	// deciding a time has had enough data
	// collection

	public static void main(String[] args) {
		long time = System.currentTimeMillis();
		for (int i = 15; i < 16; i++) {

			ProbGenerator g = new ProbGenerator(i, true, true, false,1);

			ArrayList<Integer> toInv = g.getTimesToInvestigate();
			for (int j = 0; j < 7; j++) {
				System.out.println("day " + j + " " + toInv.get(j));
			}
			g.minEntropy = 0.6;
			toInv = g.getTimesToInvestigate();
			for (int j = 0; j < 7; j++) {
				System.out.println("day " + j + " " + toInv.get(j));
			}
			System.out.println("paths under 1");
			System.out.println(g.getPathsLessThan(1));
			System.out.println("paths under 2");
			System.out.println(g.getPathsLessThan(2));
			System.out.println("paths under 3");
			System.out.println(g.getPathsLessThan(3));
			// System.out.println("At path " + i + " to investigate is "
			// + g.getTimesToInvestigateDay());
			// g.getEntropies();
		}
		System.out.println(System.currentTimeMillis() - time);
	}

	public ArrayList<Integer> getTimesToInvestigate() {
		double[] max = getEntropies();
		ArrayList<Integer> startPositions = new ArrayList<Integer>();

		for (int i = 1; i < max.length; i++) {

			if (max[i] > minEntropy) {

				startPositions.add(bounds.get(i - 1));
			}
		}

		return startPositions;

	}

//	public ArrayList<ArrayList<Integer>> getTimesToInvestigateDay() {
//		double[][] max = getEntropiesDay();
//		ArrayList<ArrayList<Integer>> startPositions = new ArrayList<ArrayList<Integer>>();
//		for (int j = 0; j < max.length; j++) {
//			startPositions.add(new ArrayList<Integer>());
//			for (int i = 1; i < max[0].length; i++) {
//				// System.out.println("entropy at point " +i + " + "+
//				// j+" is "+max[j][i]);
//				if (max[j][i] > minEntropy) {
//					// System.out.println("added");
//					startPositions.get(j).add(bounds.get(i - 1));
//
//				}
//			}
//		}
//
//		return startPositions;
//
//	}

//	public ArrayList<Integer> getPathsLessThanDay(int n, int day) {
//		ArrayList<Integer> startPositions = new ArrayList<Integer>();
//		double runs[] = timeAndDay[day];
//		for (int i = 1; i < runs.length; i++) {
//			if (runs[i] * complete.size() < n) {
//				startPositions.add(bounds.get(i - 1));
//			}
//		}
//		return startPositions;
//
//	}

	public ArrayList<Integer> getPathsLessThan(int n) {

		double runs[] = getRuns();
		ArrayList<Integer> startPositions = new ArrayList<Integer>();
		// ArrayList<Integer> endPositions = new ArrayList<Integer>();
		for (int i = 1; i < runs.length; i++) {
			if (runs[i] < n) {
				startPositions.add(bounds.get(i - 1));
				// endPositions.add(bounds.get(i));
			}

		}
		// ArrayList<ArrayList<Integer>> list = new
		// ArrayList<ArrayList<Integer>>();
		// list.add(startPositions);
		// list.add(endPositions);
		return startPositions;

		// look at the probability for each path occuring on each time, times by
		// the number of runs done. profit

	}

	public ProbGenerator(int n, boolean printGraph, boolean messy,
			boolean includeDay, int day) {
		this.includeDay = includeDay;
		c = new Cluster(n, includeDay, printGraph, messy,day);
		this.messy = messy;
		instances = c.retrieveInstances();
		complete = unsort(instances);
		if (messy) {
			System.out.println(instances);
		}
		bounds = getBounds(true);
		if (messy) {
			System.out.println("bounds positions " + bounds);
		}
		ArrayList<ArrayList<Integer>> bounded = sortIntoBounds(bounds);
		prob(bounded);

	}

	/**
	 * will sort the complete collection into which of the provided bounds they
	 * fall into
	 * 
	 * @param bounds
	 * @return
	 */
	private ArrayList<ArrayList<Integer>> sortIntoBounds(
			ArrayList<Integer> bounds) {
		ArrayList<ArrayList<Integer>> bounded = new ArrayList<ArrayList<Integer>>();

		for (int i = 0; i < bounds.size(); i++) {
			bounded.add(new ArrayList<Integer>());
		}
		for (Instance i : complete) {
			int val = (int) i.value(0);
			
			int pos = 0;
			while (pos < bounded.size()) {
				if (val < bounds.get(pos)) {
					// System.out.println(val + " is in bound " + pos);
					break;
				}
				pos++;
			}

			// adds the cluster index to the list associated with that
			// executions time
			// System.out.println("clustered into "+c.getCluster(i));
			bounded.get(pos).add(c.getCluster(i));
		}

		return bounded;
	}

	private ArrayList<Instance> unsort(ArrayList<ArrayList<Instance>> sorted) {

		ArrayList<Instance> unsorted = new ArrayList<Instance>();
		if (sorted != null) {
			for (ArrayList<Instance> list : sorted) {
				for (Instance i : list) {
					unsorted.add(i);
				}
			}
		}

		return unsorted;
	}

	private void setUpTimeOfDay(ArrayList<ArrayList<Integer>> bounded) {

		timeOfDay = new double[bounded.size()];
		int sum = 0;
		for (int i = 0; i < bounded.size(); i++) {
			timeOfDay[i] = (double) bounded.get(i).size();
			sum += bounded.get(i).size();
		}
		for (int i = 0; i < timeOfDay.length; i++) {
			timeOfDay[i] /= sum;
		}

	}

	private ArrayList<ArrayList<Integer>> setUpHypothesis(
			ArrayList<ArrayList<Integer>> bounded) {
		int largest = largestClusterNum(bounded);
		if (messy) {
			System.out.println("largest gives " + largest);
		}
		hypothesis = new double[largestClusterNum(bounded) + 1];
		if (messy) {
			System.out.println("moo" + hypothesis.length);
		}
		for (int i = 0; i < bounded.size(); i++) {
			for (Integer hyp : bounded.get(i)) {
				hypothesis[hyp]++;
			}
		}
		int sum = 0;
		for (int i = 0; i < hypothesis.length; i++) {
			sum += hypothesis[i];
		}
		for (int i = 0; i < hypothesis.length; i++) {
			if (messy) {
				System.out.println("before division size is " + hypothesis[i]);
			}
			hypothesis[i] /= sum;
			System.out.println("hyp " + i + hypothesis[i]);
		}

		// now we need to compute the joint probability table
		ArrayList<ArrayList<Integer>> hyp = sortIntoHyp(bounded);
		if (messy) {
			System.out.println(hyp);
		}
		return hyp;
	}

	/**
	 * generates the probability matrix for each cluster using the arrtibute
	 * position specified
	 * 
	 * @param n
	 * @return
	 */
	public void prob(ArrayList<ArrayList<Integer>> bounded) {
		if (messy) {
			System.out.println("bounded " + bounded);
		}
		setUpTimeOfDay(bounded);
		ArrayList<ArrayList<Integer>> hyp = setUpHypothesis(bounded);
	//
			// this bit is where the money is
			// the first index will indicate the hypothesis we are looking at
			// while the second is the time of day we are looking at
			joint = new double[hypothesis.length][timeOfDay.length];
			for (int i = 0; i < hyp.size(); i++) {
				for (Integer in : hyp.get(i)) {
					joint[i][in]++;
				}
			}
			// now normalize each
			for (int i = 0; i < joint.length; i++) {// for each hypothesis
				int sumed = 0;
				for (int j = 0; j < joint[i].length; j++) {
					sumed += joint[i][j];
				}
				if (sumed != 0) {
					for (int j = 0; j < joint[i].length; j++) {
						joint[i][j] /= sumed;
					}
				}
			//}
//		} else {
//
//			setUpDay();
//			jointDay = new double[day.length][hypothesis.length][timeOfDay.length];
//			timeAndDay = new double[day.length][timeOfDay.length];
//			int total = 0;
//			for (Instance ins : complete) {
//				// we need to find where in this lovely 3d space we are
//				int day = (int) ins.value(0);
//				int hypo = c.getCluster(ins);
//				int time = (int) ins.value(1);
//
//				int pos = 0;
//				while (pos < bounded.size()) {
//					if (time < bounds.get(pos)) {
//						// System.out.println(val + " is in bound " + pos);
//						break;
//					}
//					pos++;
//				}
//				total++;
//				timeAndDay[day][pos]++;
//				jointDay[day][hypo][pos]++;
//			}
//			for (int i = 0; i < day.length; i++) {
//				for (int j = 0; j < hypothesis.length; j++) {
//					for (int k = 0; k < timeOfDay.length; k++) {
//						if (jointDay[i][j][k] != 0) {
//
//							jointDay[i][j][k] /= total;
//							System.out.println("point at " + i + " " + j + " "
//									+ k + " is " + jointDay[i][j][k]);
//						}
//					}
//				}
//			}
//			for (int i = 0; i < day.length; i++) {
//				for (int j = 0; j < timeOfDay.length; j++) {
//					if (timeAndDay[i][j] != 0) {
//						timeAndDay[i][j] /= total;
//						// System.out.println("time and day is "+timeAndDay[i][j]);
//					}
//				}
//			}
		}
		System.out.println("prob fin");
	}

	private void setUpDay() {
		day = new double[7];
		for (Instance ins : complete) {
			day[(int) ins.value(0)]++;
		}
		for (int i = 0; i < day.length; i++) {

			day[i] /= complete.size();
			System.out.println("day chance is " + day[i]);
		}

	}

	/**
	 * returns how many clusters there are (by looking for the largest)
	 * 
	 * @param bounded
	 * @return
	 */
	private int largestClusterNum(ArrayList<ArrayList<Integer>> bounded) {
		int max = 0;
		for (ArrayList<Integer> arr : bounded) {
			for (Integer i : arr) {
				if (i > max) {
					max = i;
				}
			}
		}
		return max;
	}

	/**
	 * will look at the complete list of instances and decide what times of day
	 * they can be discretized into
	 */
	public ArrayList<Integer> getBounds(boolean simple) {
		if (bounds == null) {
			if (messy) {
				System.out.println("creating bounds");
			}
			bounds = new ArrayList<Integer>();
			if (simple) {
				for (int i = 0; i < 24 * 60; i += 10) {
					bounds.add(new Integer(i));
				}

			}
		}
		return bounds;
	}

	/**
	 * sorts the list into which hypothesis they fall into
	 * 
	 * @param list
	 * @return
	 */
	private ArrayList<ArrayList<Integer>> sortIntoHyp(
			ArrayList<ArrayList<Integer>> list) {
		ArrayList<ArrayList<Integer>> hyp = new ArrayList<ArrayList<Integer>>();
		int n = largestClusterNum(list);
		for (int i = 0; i <= n; i++) {
			hyp.add(new ArrayList<Integer>());
		}
		for (int i = 0; i < list.size(); i++) {
			for (Integer in : list.get(i)) {
				hyp.get(in).add(i);
			}
		}
		return hyp;
	}

	public double[] getProbs(int time, int thisDay) {
		double[] probs = new double[jointDay[0].length];
		time /= getBounds(true).get(1);
		time += 1;
		for (int i = 0; i < probs.length; i++) {
			probs[i] = jointDay[thisDay][i][time] * hypothesis[i]
					/ timeAndDay[thisDay][time] / day[thisDay];
			// (timeOfDay[time]*day[thisDay]);
		}
		return probs;
	}

	public double[] getProbs(int time) {
		double[] probs = new double[joint.length];
		time /= getBounds(true).get(1);
		time += 1;
		// System.out.println(joint);
		// System.out.println(hypothesis);
		// System.out.println("time of day "+timeOfDay[67]);
		for (int i = 0; i < probs.length; i++) {
			probs[i] = joint[i][time] * hypothesis[i] / timeOfDay[time];
			// System.out.println(joint[i][time]);
			// System.out.println(hypothesis[i]);
			// System.out.println(timeOfDay[time]);
			// System.out.println("prob" + probs[i]);
		}

		return probs;
	}

	/**
	 * returns the highest probability for each time (can use getBounds to query
	 * where each comes from)
	 * 
	 * a result of Nan means we need to investigate this time
	 * 
	 * a value between 1 and 0 means that one of the hypothesis is likely to be
	 * selected for this time
	 * 
	 * a value of exactly 1 means we should check out that time incase we only
	 * have a few readings for that
	 * 
	 * 0 is probably an error
	 * 
	 * @return
	 */
	public double[] getHighest() {
		System.out.println();
		double[] maxProbs = new double[bounds.size()];
		for (int i = 0; i < bounds.size(); i++) {
			double[] arr = getProbs(i*bounds.get(1));
			double max = arr[0];
			for (double d : arr) {
				if (d > max) {
					max = d;
				}
			}
			maxProbs[i] = max;
		}
		return maxProbs;
	}

	/**
	 * return the entropy of each time for this path If the path has no
	 * crossings, the entropy is interpreted as 1
	 * 
	 * @return
	 */
	public double[][] getEntropiesDay() {
		System.out.println("in entropies");
		double[][] entropies = new double[7][bounds.size()];
		for (int j = 0; j < 7; j++) {
			for (int i = 0; i < bounds.size(); i++) {
				double sum = 0;
				double[] arr = getProbs(i, j);

				for (double d : arr) {
					// System.out.println("d is " + d + " at " + i + " " + j);
					if (d > 0) {

						sum -= d * Math.log(d);
					}
				}
				if (timeAndDay[j][i] > 0) {
					entropies[j][i] = sum;
					// System.out.println("sum is "+sum + " at point "+ i+
					// ", "+j);
				} else {
					entropies[j][i] = Double.MAX_VALUE;
				}
			}
		}
		return entropies;
	}

	/**
	 * return the entropy of each time for this path If the path has no
	 * crossings, the entropy is interpreted as 1
	 * 
	 * @return
	 */
	public double[] getEntropies() {
		System.out.println("in entropies");
		double[] entropies = new double[bounds.size()];
		for (int i = 0; i < bounds.size()-1; i++) {
			double sum = 0;
			double[] arr = getProbs(i*10);
			System.out.println(" i is " + i);
			// System.out.println("size of arr "+ arr.length);
			for (double d : arr) {
				
				if (d > 0) {

					sum -= d * Math.log(d);
				}
				System.out.println("in prob " + d);
				System.out.println("in prob sum is " + sum);
			}
			if (timeOfDay[i] > 0) {
				entropies[i] = sum;
			} else {
				entropies[i] = Double.MAX_VALUE;
			}
		}
		return entropies;
	}

	public double[] getRuns() {
		double[] runs = new double[bounds.size()];
		for (int i = 0; i < bounds.size(); i++) {
			runs[i] = timeOfDay[i] * complete.size();
		}
		return runs;
	}

	public ArrayList<Integer> getMeans() {
		return c.getMeans();
	}

	
	public int[] getEstimates(){
		int[] est = new int[bounds.size()];
		for(int i =0;i<est.length-1;i++){
			est[i]=getEstimate(i*bounds.get(1));
		}
		return est;
	}
	
	public int getEstimate(int time) {
		double[] probs = getProbs(time);
		if (Double.isNaN(probs[0])) {
			double av = 0;
			for (int i = 0; i < hypothesis.length; i++) {
				try{
				av += hypothesis[i] * getMeans().get(i);
				}catch(NullPointerException e){
					return 0;
				}
			}

			return (int) av;
		}

		int sum = 0;
		for (int i = 0; i < probs.length; i++) {
			sum += probs[i] * getMeans().get(i);
		}
		return sum;
	}

	public int getEstimateDay(int time, int day) {
		double[] probs = getProbs(time, day);
		if (Double.isNaN(probs[0])) {
			double av = 0;
			for (int i = 0; i < hypothesis.length; i++) {
				av += hypothesis[i] * getMeans().get(i);
			}

			return (int) av;
		}

		int sum = 0;
		for (int i = 0; i < probs.length; i++) {
			sum += probs[i] * getMeans().get(i);
		}
		return sum;
	}

}
