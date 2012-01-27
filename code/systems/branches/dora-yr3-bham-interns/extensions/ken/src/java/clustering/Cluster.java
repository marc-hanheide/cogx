package clustering;

import java.util.ArrayList;

import javax.swing.JFrame;

import weka.clusterers.HierarchicalClusterer;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.converters.ConverterUtils.DataSource;
import weka.gui.hierarchyvisualizer.HierarchyVisualizer;

public class Cluster {
	private Instances data;
	private int maxTime;

	
	private ArrayList<ArrayList<Instance>> instances;
	private boolean includeDay;
	private HierarchicalClusterer h1;
	
	public Cluster(int pathArrPos, boolean includeDay) {
		DataSource source;
		try {
			if (pathArrPos == -1) {
				source = new DataSource("data.arff");
			} else {
				PathCluster p = new PathCluster(pathArrPos, false, includeDay);
				this.includeDay=includeDay;
				source = new DataSource("temp.arff");
			}

			data = source.getDataSet();
			maxTime = largest();
			System.out.println("data is");
			System.out.println(data);
			System.out.println();
			System.out.println("top level 1");
			HierarchicalClusterer h = cluster(1);
			double startVar = variance(h);

			JFrame frame = new JFrame("entire system");
			HierarchyVisualizer v = new HierarchyVisualizer(h.toString());
			frame.add(v);
			frame.setSize(400, 400);
			frame.setVisible(true);

			double varStart = startVar;
			double prevVar = varStart;
			int n = data.size();
			ArrayList<Double> grads = new ArrayList<Double>();

			// n++;
			if (n > 50) {
				n = (int) Math.sqrt(n / 2);
			} else {
				n = 5;

			}
			System.out.println(n);
			for (int i = 2; i <= n; i++) {
				System.out.println();
				System.out.println("cluster number " + i);

				double var = variance(cluster(i));

				System.out.println("variance of this cluster number is " + var);
				double diff = prevVar - var;
				prevVar = var;
				grads.add(new Double(diff));
				System.out.println("gradient here is " + diff);
			}
			System.out.println("gradients are " + grads);
			double sum = 0;
			for (Double d : grads) {
				sum += d;
			}
			double av = sum / grads.size();
			System.out.println("average gradient was " + av);
			int clustNum = 1;
			for (int i = 0; i < grads.size(); i++) {
				if (grads.get(i) < av) {
					clustNum = i + 1;
					System.out.println("number of clusters is " + clustNum);
					break;
				}
			}
			System.out.println("clust num is " + clustNum);
		 h1 = cluster(clustNum);
			System.out.println(h1);
			String entire = h1.toString();
			int pos = 7;
			int prev = 0;
			String[] clusters = new String[clustNum];

			for (int i = 0; i < clustNum - 1; i++) {
				pos = entire.indexOf("Cluster", pos + 7);
				System.out.println("prev is " + prev + " pos is " + pos);
				if (pos == -1) {
					if (i == 0) {
						clusters[0] = entire;
					}
					break;

				}
				clusters[i] = entire.substring(prev, pos);

				prev = pos;

			}
			if (pos != -1) {
				clusters[clustNum - 1] = entire.substring(pos, entire.length());
			} else {

			}
			for (int i = 0; i < clustNum; i++) {

				JFrame frame1 = new JFrame("cluster number " + i);
				if (clusters[i] == null) {
					break;
				}
				HierarchyVisualizer v1 = new HierarchyVisualizer(clusters[i]);
				frame1.add(v1);
				frame1.setSize(400, 400);
				frame1.setVisible(true);
			}

		} catch (Exception e) {
			System.out.println(e);
			e.printStackTrace();
		}

	}

	public int largest() {
		int large = 0;
		for (Instance i : data) {
			if(includeDay){
			if (i.value(2) > large) {
				large = (int) i.value(2);
			}
			}else{
				if (i.value(1) > large) {
					large = (int) i.value(1);
				}
			}
		}
		return large;
	}

	public static void main(String[] args) {

		Cluster c = new Cluster(49, true);
		// 50 actually has somethign happen
	}

	public HierarchicalClusterer cluster(int cluster) {
		HierarchicalClusterer h = new HierarchicalClusterer();

		h.setNumClusters(cluster);
		try {
			h.buildClusterer(data);
		} catch (Exception e) {
			System.out.println("problem building clusterer from data");
			System.out.println(h);
			e.printStackTrace();
		}
		return h;
	}

	public double variance(HierarchicalClusterer cluster) {
		ArrayList<ArrayList<Instance>> sorted = getClusters(cluster);
		int n = 0;
		double var = 0;
		for (ArrayList<Instance> ins : sorted) {
			System.out.println(ins);
			double curVar = variance(ins);
			System.out.println("var is " + curVar);
			var += (curVar * ins.size());
			n += ins.size();
		}
		var /= n;
		return var;

	}

	public double variance(ArrayList<Instance> instances) {
		int max0 = 6;// max for days
		int max1 = 60 * 24;// max for minutes
		int max2 = maxTime;// max for timetaken

		double day = 0, min = 0, time = 0;// sums for averages

		for (Instance ins : instances) {
			if(includeDay){
			day += (ins.value(0) / max0);
			min += (ins.value(1) / max1);
			time += (ins.value(2) / max2);
			}else{
				min += (ins.value(0) / max1);
				time += (ins.value(1) / max2);
			}
			

		}
		if(includeDay){
		day /= instances.size();
		}
		min /= instances.size();
		time /= instances.size();
		if(includeDay){
		System.out.println("mean point is " + day * max0 + " , " + min * max1
				+ " , " + time * max2);
		}else{
			System.out.println("mean point is "  + min * max1
					+ " , " + time * max2);
		}

		// the vector we just calculated is our centre point

		// now we need to calculate each points distance from here
		double sum = 0;
		double sumSqr = 0;
		for (Instance ins : instances) {
			double d=0;
			double m=0;
			double t =0;
			if(includeDay){
			d = ins.value(0) / max0;
			m = ins.value(1) / max1;
			t = ins.value(2) / max2;
			d = d - day;
			}else{
				m = ins.value(0) / max1;
				t = ins.value(1) / max2;
			}
			
			
			m = m - min;
			t = t - time;
			double dist;
			if(includeDay){
			 dist = Math.pow(d * d + m * m + t * t, 0.5);
			}else{
				 dist = Math.pow(m * m + t * t, 0.5);
			}
			sum += dist;
			sumSqr += dist * dist;
		}

		double var = sumSqr / instances.size();
		return var;

	}

	public ArrayList<ArrayList<Instance>> getClusters(
			HierarchicalClusterer cluster) {

		int clusters = cluster.getNumClusters();
		ArrayList<ArrayList<Instance>> sorted = new ArrayList<ArrayList<Instance>>();
		for (int i = 0; i < clusters; i++) {
			sorted.add(new ArrayList<Instance>());

		}
		for (Instance d : data) {// sorts each piece of data into it's cluster
			try {
				sorted.get(cluster.clusterInstance(d)).add(d);
			} catch (Exception e) {
				System.out.println("error in clustering a particular instance");
				System.out.println("Instance " + d);
				System.out.println("cluster " + cluster);
				e.printStackTrace();
			}
		}
		instances=sorted;
		return sorted;
	}
	
	public ArrayList<ArrayList<Instance>> retrieveInstances(){
		return instances;
	}

	public int getCluster(Instance i){
	try {
		return 	h1.clusterInstance(i);
	} catch (Exception e) {
		System.out.println("instance "+ i + " could not be clustered");
		return -1;
		
	}
	}
	
	public double combine(double[] arr) {

		double sum = 1;
		for (double d : arr) {
			sum *= d;
		}
		return sum;
	}
	
}
