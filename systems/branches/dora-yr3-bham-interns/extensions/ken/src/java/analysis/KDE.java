package analysis;

import java.awt.geom.Point2D;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.ArrayList;
import java.util.Vector;

import displays.Graph;
import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class KDE {

	private Vector<PathTimes> pathTimes;
	private double bandwidth;
	private Kernel k;
	private int NO_OF_STEPS = 500;

	public static void main(String[] args) {
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
		KDE kde = new KDE(pathTimes, 0.5, new EpanechinikovKernel());
		// KDE kde = new KDE(pathTimes, 0.1, new GaussianKernel());
		int pos = 0;
		double[][] values = kde.runKDE(pos);

		// System.out.println("initial values ");
		// for (int i = 0; i < values[0].length; i++) {
		// System.out.println("at point " + values[0][i] + " value is "
		// + values[1][i]);
		// }

		 double[][] curve = curvify(values);
		 values[1] = kde.smooth(values[1]);
		double[] diffd1 = kde.differentiate(values[1]);
		// double[] diffd2 = kde.differentiate(diffd1);

		// for (int i = 0; i < diffd2.length; i++) {
		// System.out.println("at pos " + values[0][i] + " grad is "
		// + diffd2[i]);
		//
		// }

		double[] paths = new double[pathTimes.get(pos).getRuns().size()];
		for (int i = 0; i < pathTimes.get(pos).getRuns().size(); i++) {
			paths[i] = ((double) pathTimes.get(pos).getRun(i).timeTaken()) / 1000;
		}

		// System.out.println("paths" + paths.length);
		Graph gO = new Graph(paths, "raw data");
		// // System.out.println(values[1].length);
		Graph gV = new Graph(values[1], values[0], "kde values");
		 // System.out.println(diffd1.length);
	//	Graph gD1 = new Graph(diffd1, values[0], "diffd once");
		// // System.out.println(diffd2.length);
		// Graph gD = new Graph(diffd2, values[0], "double differentiated");
		// ArrayList aL = kde.findGradChange(values[1]);
		// System.out.println(aL.get(0));
		// System.out.println(aL.get(1));
		// for (Point2D p : points) {
		// System.out.println(p);
		// }
		//
		// ArrayList<Point2D> points = new ArrayList<Point2D>();
		// for (int i = 0; i < diffd2.length; i++) {
		//
		// if (diffd2[i] != 0) {
		// Point2D p = new Point2D.Double(diffd2[i], values[0][i]);
		// points.add(p);
		// System.out.println(p);
		// }
		//
		// }

		ArrayList<Double> zeroes = new ArrayList<Double>();
		System.out.println("looking at points where grad is 0");
		for (int i = 1; i < diffd1.length - 1; i++) {
			if (diffd1[i] == 0 && (diffd1[i + 1] != 0 || diffd1[i - 1] != 0)) {
				zeroes.add(values[0][i]);
			}
		}

		for (Double d : zeroes) {
			System.out.println(d);
		}
		int index = 0;

		ArrayList<Double> mids = new ArrayList<Double>();
		ArrayList<Double> widths = new ArrayList<Double>();
		if (zeroes.get(index) < 1) {
			System.out.println("cluster at 0, size " + zeroes.get(index));
			index = 1;
		
			mids.add(new Double(0));
			widths.add(new Double(zeroes.get(0)));
		}

		while (index < zeroes.size()) {
			System.out.print("cluster at "
					+ (zeroes.get(index) + zeroes.get(index + 1)) / 2);
			System.out.println(", size "
					+ (zeroes.get(index + 1) - zeroes.get(index)));
			mids
					.add(new Double(
							(zeroes.get(index) + zeroes.get(index + 1)) / 2));
			widths.add(new Double(zeroes.get(index + 1) - zeroes.get(index)));
			index += 2;

		}
		for (double d : paths) {

			for (int i = 0; i < mids.size(); i++) {
				if (d < (mids.get(i) + .5 * widths.get(i))) {
					System.out.println("path of " + d + " in cluster " + i);
					break;
				}
			}
		}

	}

	private double[] smooth(double[] diffd2) {
		double[] smooth = new double[diffd2.length];
		for (int i = 2; i < diffd2.length - 2; i++) {

			smooth[i] = (diffd2[i - 2] + diffd2[i - 1] + diffd2[i]
					+ diffd2[i + 1] + diffd2[i + 2]) / 5;
		}
		return smooth;
	}

	public static double[][] curvify(double[][] values) {

		ArrayList<Point2D> points = new ArrayList<Point2D>();
		for (int i = 1; i < values[0].length; i++) {
			if (values[1][i] != values[1][i - 1]) {

				points.add(new Point2D.Double(values[0][i - 1],
						values[1][i - 1]));
				points.add(new Point2D.Double(values[0][i], values[1][i]));
			}
		}
		if (points.get(0).getY() != 0) {// basically a hack for if the first
			// point is centred on 0
			points.add(0, new Point2D.Double(-points.get(0).getX(), points.get(
					0).getY()));
			points.add(0, new Point2D.Double(-points.get(2).getX(), points.get(
					2).getY()));
		}
		System.out.println("now looking through curvified");
		for (Point2D p : points) {
			System.out.println(p);
		}
		ArrayList<Point2D> mids = new ArrayList<Point2D>();
		if (points.get(0).getY() == 0) {
			mids.add(points.get(0));
		}
		for (int i = 1; i < points.size(); i++) {// find the mid values
			if (points.get(i - 1).getY() == points.get(i).getY()
					&& points.get(i).getY() != 0) {// same
				// level, so
				// we can
				// use the
				// midpoint
				mids.add(new Point2D.Double((points.get(i).getX() + points.get(
						i - 1).getX()) / 2, points.get(i).getY()));
			} else {
				if (points.get(i).getY() == 0) {
					mids.add(points.get(i));
				}
			}
		}
		System.out.println("mids");
		for (Point2D p : mids) {
			System.out.println(p);
		}

		double[][] retval = new double[2][values[0].length];

		double grad;
		double inc;

		double stepSize = values[0][1] - values[0][0];

		for (int i = 1; i < mids.size(); i++) {
			if (mids.get(i).getX() > 0) {// we're actually in the array's space
				if (mids.get(i - 1).getY() != mids.get(i).getY()) {// +ve or -ve
					// grad
					grad = (mids.get(i - 1).getX() - mids.get(i).getX())
							/ (mids.get(i - 1).getY() - mids.get(i).getY());
					inc = stepSize * grad;
					int start = (int) (mids.get(i - 1).getX() / stepSize);
					int end = (int) (mids.get(i).getX() / stepSize);
					double val = mids.get(i - 1).getY();
					for (int j = start; j < end; j++) {
						retval[0][j] = values[0][j];
						retval[1][j] = val;
						val += inc;
					}
				} else {// grad is 0 (which is a lie), entire reason I have to
					// do this hack

				}

			}

		}
		for (int i = 0; i < values[0].length; i++) {
			System.out.println(retval[1][i]);
		}
		return retval;
	}

	public KDE(Vector<PathTimes> pathTimes, double bandwidth, Kernel k) {
		this.pathTimes = pathTimes;
		this.bandwidth = bandwidth;
		this.k = k;
	}

	/**
	 * given a array position within pathTImes and a time, finds the kde of that
	 * position
	 * 
	 * @param pos
	 * @param time
	 * @return
	 */
	public double get(int pos, double time) {
		PathTimes path = pathTimes.get(pos);

		double sum = 0;

		for (PathRun p : path.getRuns()) {
			double kV = k
					.apply(((double) time - (((double) p.timeTaken()) / 1000))
							/ (bandwidth));
			if (kV == 1) {

			}
			sum += kV;
		}

		sum /= (path.getRuns().size() * bandwidth);
		return sum;
	}

	/**
	 * given the position in pathTimes, find the time of the largest pathRun
	 * 
	 * @param pos
	 * @return
	 */
	public double findLargest(int pos) {
		double largest = Double.MIN_VALUE;
		for (PathRun p : pathTimes.get(pos).getRuns()) {
			if (p.timeTaken() / 1000 > largest) {
				largest = (double) p.timeTaken() / 1000;
			}
		}
		return largest;
	}

	public double[][] runKDE(int pos) {
		PathTimes path = pathTimes.get(pos);
		// System.out.println("path is " + path.getA() + " & " + path.getB());
		double stepSize = findLargest(pos) / NO_OF_STEPS;
		double[][] values = new double[2][(int) (NO_OF_STEPS * 1.4)];
		System.out.println(" stepsize is " + stepSize);
		for (int i = 0; i < values[0].length; i++) {

			values[1][i] = get(pos, stepSize * (double) i);
			values[0][i] = stepSize * i;
			// System.out.println("i is " + i + " pos is " + stepSize * i);

		}

		return values;
	}

	/**
	 * given a curve as an array, will differentiate it
	 * 
	 * @param curve
	 * @return
	 */
	public double[] differentiate(double[] curve) {
		double[] diffd = new double[curve.length - 1];
		for (int i = 1; i < diffd.length; i++) {

			diffd[i - 1] = curve[i - 1] - curve[i];

		}
		return diffd;
	}

}
