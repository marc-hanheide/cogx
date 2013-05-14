package displays;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Date;
import java.util.Vector;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class PathVisualization extends JFrame {

	JFreeChart chart;

	public static String[] values = { "timings.txt","howarth3.txt", "howarth2.txt","timings1.txt","timings2.txt","timings3.txt"};
	static String value = values[0];
	ChartPanel panel;
	private float data[][];
	int start;
	int end;

	public PathVisualization(int n, boolean includeDay) {
	//	super("Path Traversals For Edge " + n);
		super();
		setSize(800, 600);
		load(n, includeDay);
		setTitle("Path Traversals between " + start + " & " + end);
		// panel = new ChartPanel(chart);
		// this.add(panel);
		setVisible(true);
	}

	public void load(int n, boolean includeDay) {
		// XYSeries pathTimes = new XYSeries("path times");

		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream(
							PathVisualization.value)));
			PathTimes path = ((PathTimesWrapper) (in.readObject()))
					.getPathTimes().get(n);
			Vector<PathRun> pT = path.getRuns();
			start = path.getA();
			end = path.getB();
			data = new float[2][pT.size()];
			XYSeriesCollection xySeriesCollection;
			if (includeDay) {
				xySeriesCollection = new XYSeriesCollection();
				XYSeries[] series = new XYSeries[7];
				for (int i = 0; i < 7; i++) {
					series[i] = new XYSeries("day " + i);
				}

				for (int i = 0; i < pT.size(); i++) {
					data[1][i] = pT.get(i).timeTaken();
					Date time = pT.get(i).timeStarted();
					data[0][i] = time.getHours() * 60 + time.getMinutes();
					int day = time.getDay();
					series[day].add(data[0][i], data[1][i]);

				}
				for (int i = 0; i < 7; i++) {
					xySeriesCollection.addSeries(series[i]);
				}
			} else {
				xySeriesCollection = new XYSeriesCollection();
				XYSeries series = new XYSeries("all days");
				for (int i = 0; i < pT.size(); i++) {
					data[1][i] = pT.get(i).timeTaken();
					Date time = pT.get(i).timeStarted();
					data[0][i] = time.getHours() * 60 + time.getMinutes();
					series.add(data[0][i], data[1][i]);

				}
				xySeriesCollection.addSeries(series);
			}
			chart = ChartFactory.createScatterPlot("Path Traversals between "
					+ start + " & " + end, "Time Recorded", "Time Taken (ms)",
					xySeriesCollection, PlotOrientation.VERTICAL, true, true,
					false);

			final ChartPanel panel = new ChartPanel(chart, true);
			panel.setPreferredSize(new java.awt.Dimension(500, 270));

			setContentPane(panel);
			in.close();
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		} catch (ArrayIndexOutOfBoundsException e){
			System.out.println("not a valid path number");
		}

	}

	public static String getVal() {
		return value;
	}

}
