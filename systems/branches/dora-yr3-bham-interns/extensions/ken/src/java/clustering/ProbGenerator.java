package clustering;

import java.util.ArrayList;

import weka.core.Instance;

public class ProbGenerator {

	private Cluster c;
	private ArrayList<ArrayList<Instance>> instances;
	private ArrayList<Instance> complete;
	private ArrayList<Integer> bounds;

	private double[][] joint;
	private double[] hypothesis;
	private double[] timeOfDay;

	private int minPath = 2; // the minimum number of paths before we start
	private double minEntropy = 0.5;

	private boolean messy;

	// deciding a time has had enough data
	// collection

	public static void main(String[] args) {
		long time = System.currentTimeMillis();
		for (int i = 0; i < 83; i++) {

			ProbGenerator g = new ProbGenerator(i, false, false);
			// ArrayList<Integer> bounds = g.getBounds(true);
			// double[] max = g.getEntropies();
			// ArrayList<Integer> startPositions = new ArrayList<Integer>();
			// ArrayList<Integer> endPositions = new ArrayList<Integer>();
			//
			// for (int i = 0; i < max.length; i++) {
			// if (max[i] != 1) {
			// System.out.println(max[i] + " at point " + i);
			// startPositions.add(bounds.get(i - 1));
			// endPositions.add(bounds.get(i));
			// }
			// }
			//
			// System.out.println();
			// System.out.println(bounds);
			// for (int i = 0; i < startPositions.size(); i++) {
			// System.out.println("between " + startPositions.get(i) + " & "
			// + endPositions.get(i));
			// }

//			System.out.println("At point " + i + " need to look at "
//					+ g.getTimesToInvestigate().get(0).size() + " times ");
			System.out.println("At path "+i+" to investigate is "+g.getTimesToInvestigate().get(0));
			//g.getEntropies();
		}
		System.out.println(System.currentTimeMillis() - time);
	}

	public ArrayList<ArrayList<Integer>> getTimesToInvestigate() {
		double[] max = getEntropies();
		ArrayList<Integer> startPositions = new ArrayList<Integer>();
		ArrayList<Integer> endPositions = new ArrayList<Integer>();
		for (int i = 1; i < max.length; i++) {
			if (max[i] > minEntropy) {
				startPositions.add(bounds.get(i - 1));
				endPositions.add(bounds.get(i));
			}
		}
		ArrayList<ArrayList<Integer>> list = new ArrayList<ArrayList<Integer>>();
		list.add(startPositions);
		list.add(endPositions);
		return list;

	}

	public ProbGenerator(int n, boolean printGraph, boolean messy) {
		c = new Cluster(n, false, printGraph, messy);
		this.messy = messy;
		instances = c.retrieveInstances();
		complete = unsort(instances);
		if (messy) {
			System.out.println(instances);
		}
		bounds = getBounds(true);
		if (messy) {
			System.out.println("bounds positions "+bounds);
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

	/**
	 * generates the probability matrix for each cluster using the arrtibute
	 * position specified
	 * 
	 * @param n
	 * @return
	 */
	public void prob(ArrayList<ArrayList<Integer>> bounded) {
		if (messy) {
			System.out.println("bounded "+bounded);
		}
		timeOfDay = new double[bounded.size()];
		int sum = 0;
		for (int i = 0; i < bounded.size(); i++) {
			timeOfDay[i] = (double) bounded.get(i).size();
			sum += bounded.get(i).size();
		}
		for (int i = 0; i < timeOfDay.length; i++) {
			timeOfDay[i] /= sum;
		}

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
		sum = 0;
		for (int i = 0; i < hypothesis.length; i++) {
			sum += hypothesis[i];
		}
		for (int i = 0; i < hypothesis.length; i++) {
			if (messy) {
				System.out.println("before division size is " + hypothesis[i]);
			}
			hypothesis[i] /= sum;
		}
		// System.out.println("time of day");
		// for (int i = 0; i < timeOfDay.length; i++) {
		// System.out.println(timeOfDay[i]);
		// }
		// System.out.println("hypothesis");
		// System.out.println(hypothesis.length);
		// for (double h : hypothesis) {
		// System.out.println(h);
		// }

		// now we need to compute the joint probability table
		ArrayList<ArrayList<Integer>> hyp = sortIntoHyp(bounded);
		if (messy) {
			System.out.println(hyp);
		}
		// this bit is where the money is
		// the first index will indicate the hypothesis we are looking at
		// while the second is the time of day we are looking at
		joint = new double[hypothesis.length][timeOfDay.length];
		for (int i = 0; i < hyp.size(); i++) {
			for (Integer in : hyp.get(i)) {
				joint[i][in]++;
			}
		}

		// for (int i = 0; i < joint.length; i++) {
		// for (int j = 0; j < joint[0].length; j++) {
		// System.out.print(joint[i][j] + " ");
		// }
		// System.out.println();
		// }
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
		}
		// for (int i = 0; i < joint.length; i++) {
		// for (int j = 0; j < joint[0].length; j++) {
		// System.out.print(joint[i][j] + " ");
		// }
		// System.out.println();
		// }

	}

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

	public double[] getProbs(int time) {
		double[] probs = new double[joint.length];
		time/=getBounds(true).get(1);
		time+=1;
//				System.out.println(joint);
//		/System.out.println(hypothesis);
		//System.out.println("time of day "+timeOfDay[67]);
		for (int i = 0; i < probs.length; i++) {
			probs[i] = joint[i][time] * hypothesis[i] / timeOfDay[time];
		
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
			double[] arr = getProbs(i);
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
	public double[] getEntropies() {
		System.out.println();
		double[] entropies = new double[bounds.size()];
		for (int i = 0; i < bounds.size(); i++) {
			double sum = 0;
			double[] arr = getProbs(i);

			for (double d : arr) {
				if (d > 0) {
					System.out.println(d);
					sum -= d * Math.log(d);
				}
			}
			// System.out.println(timeOfDay[i]);
			if (timeOfDay[i] >= (1.0 / (double) complete.size()) * minPath) {
				// System.out.println("succeeded " +timeOfDay[i]);
				// System.out.println((1.0/(double)complete.size())*minPath);
				entropies[i] = sum;
			} else {
				// System.out.println("failed "+timeOfDay[i]);
				// System.out.println( (1.0/(double)complete.size())*minPath);

				entropies[i] = Double.MAX_VALUE;
			}

		}
		return entropies;
	}

	public ArrayList<Integer> getMeans() {
		return c.getMeans();
	}
	
	public int getEstimate(int time){
		double[] probs =getProbs(time);
		if(Double.isNaN(probs[0])){
			double av=0;
			for(int i=0;i<hypothesis.length;i++){
				av+=hypothesis[i]*getMeans().get(i);
			}
			
			return (int)av;
		}
//		for(int i=0;i<probs.length;i++){
//			if(probs[i]>highest){
//				highest = probs[i];
//				highestPos=i;
//			}
//		}
//		if(probs.length==2){
//			if(probs[0]==probs[1]){
//				return (getMeans().get(0)+getMeans().get(1))/2;
//			}
//		}
//		
//		return getMeans().get(highestPos);
		int sum =0;
		for(int i=0;i<probs.length;i++){
			sum+= probs[i]*getMeans().get(i);
		}
		return sum;
	}

}
