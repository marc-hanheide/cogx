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

	private boolean boundsCreated;
	
	public static void main(String[] args) {

		ProbGenerator g = new ProbGenerator(49);
		double[] probs = g.getProbs(58);
		for (int i = 0; i < probs.length; i++) {
			System.out.print(probs[i]);
		}
		System.out.println();
		probs = g.getProbs(62);
		for (int i = 0; i < probs.length; i++) {
			System.out.print(probs[i]);
		}
		System.out.println();
		probs = g.getProbs(10);
		for (int i = 0; i < probs.length; i++) {
			System.out.print(probs[i]);
		}
	double[] max = g.getHighest();
	for(double d: max){
		System.out.print( " "+ d);
	}

	}

	public ProbGenerator(int n) {
		c = new Cluster(n, false);
		boundsCreated=false;
		instances = c.retrieveInstances();
		complete = unsort(instances);
		System.out.println(instances);
		bounds = getBounds(true);
		System.out.println(bounds);
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
	public ArrayList<ArrayList<Integer>> sortIntoBounds(
			ArrayList<Integer> bounds) {
		ArrayList<ArrayList<Integer>> bounded = new ArrayList<ArrayList<Integer>>();

		for (int i = 0; i < bounds.size(); i++) {
			bounded.add(new ArrayList<Integer>());
		}
		System.out.println(bounds);
		for (Instance i : complete) {
			int val = (int) i.value(0);

			int pos = 0;
			while (pos < bounded.size()) {
				if (val < bounds.get(pos)) {
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

	public ArrayList<Instance> unsort(ArrayList<ArrayList<Instance>> sorted) {

		ArrayList<Instance> unsorted = new ArrayList<Instance>();
		for (ArrayList<Instance> list : sorted) {
			for (Instance i : list) {
				unsorted.add(i);
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
		System.out.println("bounded ");
		System.out.println(bounded);
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
		System.out.println("largest gives " + largest);
		hypothesis = new double[largestClusterNum(bounded) + 1];
		System.out.println("moo" + hypothesis.length);
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
			System.out.println("before division size is " + hypothesis[i]);
			hypothesis[i] /= sum;
		}
		System.out.println("time of day");
		for (int i = 0; i < timeOfDay.length; i++) {
			System.out.println(timeOfDay[i]);
		}
		System.out.println("hypothesis");
		System.out.println(hypothesis.length);
		for (double h : hypothesis) {
			System.out.println(h);
		}

		// now we need to compute the joint probability table
		ArrayList<ArrayList<Integer>> hyp = sortIntoHyp(bounded);
		System.out.println(hyp);
		// this bit is where the money is
		// the first index will indicate the hypothesis we are looking at
		// while the second is the time of day we are looking at
		joint = new double[hypothesis.length][timeOfDay.length];
		for (int i = 0; i < hyp.size(); i++) {
			for (Integer in : hyp.get(i)) {
				joint[i][in]++;
			}
		}

		for (int i = 0; i < joint.length; i++) {
			for (int j = 0; j < joint[0].length; j++) {
				System.out.print(joint[i][j] + " ");
			}
			System.out.println();
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
		}
		for (int i = 0; i < joint.length; i++) {
			for (int j = 0; j < joint[0].length; j++) {
				System.out.print(joint[i][j] + " ");
			}
			System.out.println();
		}

	}

	public int largestClusterNum(ArrayList<ArrayList<Integer>> bounded) {
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
		if (bounds==null) {
			System.out.println("creating bounds");
			bounds = new ArrayList<Integer>();
			if (simple) {
				for (int i = 0; i < 24 * 60; i += 10) {
					bounds.add(new Integer(i));
				}

			}
		}
		return bounds;
	}

	public ArrayList<ArrayList<Integer>> sortIntoHyp(
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
		for (int i = 0; i < probs.length; i++) {
			probs[i] = joint[i][time] * hypothesis[i] / timeOfDay[time];
		}
		return probs;
	}

	/**
	 * returns the highest probability for each time (can use getBounds to query where each comes from)
	 * 
	 * a result of Nan means we need to investigate this time
	 * 
	 * a value between 1 and 0 means that one of the hypothesis is likely to be selected for this time
	 *  
	 * a value of exactly 1 means we should check out that time incase we only have a few readings for that
	 * 
	 * 0 is probably an error
	 * 
	 * @return
	 */
	public double[] getHighest(){
		System.out.println();
		double[] maxProbs = new double[bounds.size()];
		for(int i=0;i<bounds.size();i++){
			double[] arr = getProbs(i);
			double max=arr[0];
			for(double d: arr){
				if(d>max){
					max=d;
				}
			}
			maxProbs[i]=max;
		}
		return maxProbs;
	}
	
}
