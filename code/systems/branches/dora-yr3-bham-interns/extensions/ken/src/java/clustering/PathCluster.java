package clustering;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.PrintWriter;
import java.util.Date;
import java.util.Vector;

import javax.swing.JFrame;

import weka.clusterers.HierarchicalClusterer;
import weka.core.Instances;
import weka.core.converters.ConverterUtils.DataSource;
import weka.gui.hierarchyvisualizer.HierarchyVisualizer;
import displays.PathVisualization;
import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class PathCluster {

	private boolean hax = false;// this should be true if i want to use real
	// data, otherwise this is cheating

	private boolean printy;
	private boolean includeDay;
	Vector<PathTimes> pathTimes;
	PathTimes pT;

	public PathTimes load(int n) {

		pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream(
							PathVisualization.getVal())));

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
		// System.out.println("path times is " + pathTimes);
		 pT = pathTimes.get(n);
		if (printy) {
			System.out.println(pT);

		}
		return pT;
	}

	public PathCluster(int n, boolean printy, boolean includeDay, int day) {
		this.printy = printy;
		this.includeDay = includeDay;
		PathTimes pT = load(n);
		generateFile(pT, includeDay, day);
		if (printy) {
			prettyOutput(pT);
		}

	}
	
	public PathTimes getPath(){
		return pT;
		
	}

	private void prettyOutput(PathTimes pT) {
		if (printy) {

			System.out.println("Path found goes from " + pT.getA() + " & "
					+ pT.getB());
			DataSource source;
			try {
				source = new DataSource("temp.arff");
				Instances data = source.getDataSet();
				if(data.size()<3){
					System.out.println("that day/path is empty");
					return;
				}
				// System.out.println(data);
				HierarchicalClusterer h = new HierarchicalClusterer();
				h.buildClusterer(data);
				JFrame frame = new JFrame();
				if(h.toString()!=null){
				System.out.println("h is "+h.toString());
				
				HierarchyVisualizer v = new HierarchyVisualizer(h.toString());
				
				frame.add(v);
				frame.setSize(400, 400);
				frame.setTitle("original state");
				frame.setVisible(true);
				}
			} catch(NullPointerException e){
				System.out.println("null path");
				return;
			}catch (Exception e) {
				System.out.println(e);
				e.printStackTrace();
			}
		}

	}

	private void generateFile(PathTimes pT, boolean includeDay, int day) {
		try {
			PrintWriter out = new PrintWriter(new FileWriter("temp.arff"));
			out.println("% 1. Title: Path Relations");
			out.println("%");
			out.println("% 2. Sources");
			out.println("%      (a) Creator: K.W. Poyner");
			out.println("\n @RELATION cluster");
			out.println("\n");
			// if (includeDay) {
			// out.println("@ATTRIBUTE day NUMERIC");
			// }
			out.println("@ATTRIBUTE timeofday NUMERIC");
			out.println("@ATTRIBUTE timetaken NUMERIC");
			out.println("\n @DATA \n");
			for (PathRun r : pT.getRuns()) {
				Date time = r.timeStarted();
				int mins = time.getHours() * 60 + time.getMinutes();
				if (hax) {
					if (time.getDay() == 1) {
						out.println(time.getDay() + " , " + mins + " , "
								+ r.timeTaken());
						out.println(3 + " , " + mins + " , " + r.timeTaken());
						out.println(4 + " , " + mins + " , " + r.timeTaken());
					} else {
						out.println(time.getDay() + " , " + mins + " , "
								+ r.timeTaken());
						out.println(0 + " , " + mins + " , " + r.timeTaken());
						out.println(2 + " , " + mins + " , " + r.timeTaken());
					}
				}
				if (includeDay) {
					if (time.getDay() == day) {
						out.println(mins + " , " + r.timeTaken());
					}
				} else {
					out.println(mins + " , " + r.timeTaken());
				}

			}
			out.close();
		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		}

	}

	public static void main(String[] args) {

		PathCluster p = new PathCluster(4, true, false, 5);

	}

}
