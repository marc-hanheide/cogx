package clustering;

import java.util.ArrayList;

import javax.swing.JFrame;

import weka.clusterers.HierarchicalClusterer;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.converters.ConverterUtils.DataSource;
import weka.gui.hierarchyvisualizer.HierarchyVisualizer;

public class CopyOfCluster {
	private Instances data;

	public CopyOfCluster() {
		DataSource source;
		try {
			source = new DataSource("temp.arff");
			data = source.getDataSet();
			System.out.println("data is");
			System.out.println(data);
			System.out.println();
			System.out.println("top level 1");
			HierarchicalClusterer h = cluster(1);
			double[] startVar = variance(h);
			
			 JFrame frame = new JFrame();
			 HierarchyVisualizer v = new HierarchyVisualizer(h.toString());
			 frame.add(v);
			 frame.setSize(400, 400);
			 frame.setVisible(true);
			
			
			double varStart = combine(startVar);
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
			for (int i = 2; i < n; i++) {
				System.out.println();
				System.out.println("top level " + i);
				// cluster(i);
				double[] curVar = variance(cluster(i));
				double[] per = percentify(startVar, curVar);
				double var = combine(per);
				// TODO process variances to find elbow
				System.out.println("current var is" + var);
				double diff = prevVar - var;
				prevVar = var;
				grads.add(new Double(diff));
				System.out.println("difference is " + diff);
			}
			System.out.println("gradients are " + grads);
			double sum = 0;
			for (Double d : grads) {
				sum += d;
			}
			double av =sum/grads.size();
			System.out.println("average gradient was "+av);
			for(int i=0;i<grads.size();i++){
				//if(i<av)
				
			}
			
		} catch (Exception e) {
			System.out.println(e);
			e.printStackTrace();
		}

	}

	public static void main(String[] args) {
		CopyOfCluster c = new CopyOfCluster();

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

	public double[] variance(HierarchicalClusterer cluster) {
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
		double[] totVar = new double[data.get(0).numAttributes()];
		for (ArrayList<Instance> i : sorted) {
			System.out.println(i);
			double[] var = variance(i);
			for (int j = 0; j < data.get(0).numAttributes(); j++) {
				System.out.print(var[j] + ", ");
				totVar[j] += var[j] * i.size();
			}

			System.out.println();
		}

		for (int i = 0; i < totVar.length; i++) {
			totVar[i] /= data.size();

		}

		System.out.println("total variances");
		for (double d : totVar) {
			System.out.print(d + " , ");
		}
		System.out.println();
		return totVar;
	}

	public double[] variance(ArrayList<Instance> cluster) {
		int n = cluster.size();
		double[] sum = new double[cluster.get(0).numAttributes()];
		double[] sumSqr = new double[cluster.get(0).numAttributes()];
		for (Instance i : cluster) {
			for (int j = 0; j < i.numAttributes(); j++) {
				sum[j] += i.value(j);
				sumSqr[j] += i.value(j) * i.value(j);
			}

		}
		double var[] = new double[cluster.get(0).numAttributes()];
		for (int i = 0; i < sum.length; i++) {
			if (n > 1) {
				double mean = sum[i] / n;
				var[i] = (sumSqr[i] - sum[i] * mean) / (n - 1);
			}
		}

		return var;
	}

	public double[] percentify(double[] start, double[] end) {
		double[] per = new double[start.length];
		System.out.println("percentages");
		for (int i = 0; i < start.length; i++) {
			if (end[i] != 0) {
				per[i] = end[i] / start[i];
			} else {
				per[i] = 0.01;

			}
			System.out.print(per[i] + " , ");
		}
		System.out.println();
		return per;
	}

	public double combine(double[] arr) {

		double sum = 1;
		for (double d : arr) {
			sum *= d;
		}
		return sum;
	}
}
// code for displaying shiz

// JFrame frame = new JFrame();
// HierarchyVisualizer v = new HierarchyVisualizer(h.toString());
// frame.add(v);
// frame.setSize(400, 400);
// frame.setVisible(true);
// System.out.println("data size is " + data.size());
// for (Instance i : data) {
// System.out.println(i);
// System.out.println("is in " + h.clusterInstance(i));
//
// }
